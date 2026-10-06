#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把「本次 launch 的会话是谁」挂到 **ROS 图**上（2026-10-06 存档事故后新增）。

为什么需要它（事故原话）：用户给自己那套 `map_name:=RMUC2026_v2` 的栈跑
`map_archive.sh save`，但 `<map_dir>/.session.yaml` 已被**并发的自动化测试栈**
（`map_name:=RMUC2026_dropab_ab_g_long`）覆盖 ⇒ 31.89 MB 位姿图被写到了别人那套会话的名字上。
根因：名字来自一个"谁后启动谁覆盖"的**可变文件**，而且当时唯一的活性检查是
`kill -0 <session_pid>`（pid 会被回收、也可能只是包装进程 ⇒ 两头都错）。

本节点是修法之一：**会话身份由活着的栈自己在 ROS 图上广播**，客观、可被 `save` 当场核对：

  · 常驻（latched, transient_local）话题 `std_msgs/String` `/map_session/info`
      —— 任何时刻新加入的读者立刻能收到当前会话；
  · 服务 `std_srvs/Trigger` `/map_session/query`（message = 同一份 JSON 负载）
      —— 方便 `ros2 service call` 手工看一眼。
  负载（JSON，schema=rm_nav_bringup/map_session@1）：
      map_name / world / archive_base / map_start_pose / resumed / autocontinue /
      allow_world_mismatch / started_at / session_id / launch_pid /
      topic / service / mapper_node / mapper_nodes / mapper_count /
      serialize_service / serialize_present / helper_nodes / helper_count /
      verified / graph_at / node_count
  · `verified` = **广播那一刻图上确实有且只有一个 `mapper_node`（默认 /slam_toolbox）
    并且它提供 `serialize_service`（默认 /slam_toolbox/serialize_map）** ——
    也就是"这套栈真的在 mapping 模式、而且映射器唯一"。`save` 拿不到 verified=True 就拒绝、
    不猜名字（见 `map_asset_guard.py::resolve_session`）。

它**只是发布者**：不发 /map、不发 TF、不订阅任何东西，不改变既有节点集/时序；
`~` 侧也没有任何写操作。跑不起来也只是少了一条证据（`save` 会退到"活 launch 进程命令行"
与"`.session.yaml` + 活性证明"两条路），绝不会挡住建图。

直接手工跑（一般由 launch 起，不用手工）：
    ros2 run rm_nav_bringup map_session_announcer.py --ros-args -p map_name:=X -p world:=RMUC2026
"""
import json
import os
import socket
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger

SESSION_SCHEMA = 'rm_nav_bringup/map_session@1'
DEFAULT_TOPIC = '/map_session/info'
DEFAULT_SERVICE = '/map_session/query'
DEFAULT_MAPPER = 'slam_toolbox'
DEFAULT_SERIALIZE_SERVICE = '/slam_toolbox/serialize_map'


def _floats3(text):
    """launch 传进来的 map_start_pose 可能是字符串 "[0.0, 0.0, 0.0]"、也可能是 double 数组。"""
    if text is None:
        return None
    if isinstance(text, (list, tuple)):
        vals = text
    else:
        import ast
        try:
            vals = ast.literal_eval(str(text).strip())
        except Exception:
            return None
        if not isinstance(vals, (list, tuple)):
            return None
    try:
        return [float(v) for v in vals]
    except Exception:
        return None


class MapSessionAnnouncer(Node):
    def __init__(self):
        super().__init__('map_session')
        self.declare_parameter('map_name', '')
        self.declare_parameter('world', '')
        self.declare_parameter('archive_base', '')
        self.declare_parameter('map_start_pose', '')
        self.declare_parameter('resumed', False)
        self.declare_parameter('autocontinue', True)
        self.declare_parameter('allow_world_mismatch', False)
        self.declare_parameter('started_at', '')
        self.declare_parameter('session_id', '')
        self.declare_parameter('launch_pid', 0)
        self.declare_parameter('mapper_node', DEFAULT_MAPPER)
        self.declare_parameter('serialize_service', DEFAULT_SERIALIZE_SERVICE)
        self.declare_parameter('topic', DEFAULT_TOPIC)
        self.declare_parameter('service', DEFAULT_SERVICE)
        self.declare_parameter('publish_period', 2.0)

        g = self.get_parameter
        self.map_name = str(g('map_name').value or '')
        self.world = str(g('world').value or '')
        self.archive_base = str(g('archive_base').value or '')
        self.map_start_pose = _floats3(g('map_start_pose').value)
        self.resumed = bool(g('resumed').value)
        self.autocontinue = bool(g('autocontinue').value)
        self.allow_world_mismatch = bool(g('allow_world_mismatch').value)
        self.started_at = str(g('started_at').value or '')
        self.session_id = str(g('session_id').value or '') or uuid.uuid4().hex[:12]
        self.launch_pid = int(g('launch_pid').value or 0)
        self.mapper_node = str(g('mapper_node').value or DEFAULT_MAPPER).lstrip('/')
        self.serialize_service = str(g('serialize_service').value or DEFAULT_SERIALIZE_SERVICE)
        topic = str(g('topic').value or DEFAULT_TOPIC)
        service = str(g('service').value or DEFAULT_SERVICE)
        period = max(0.5, float(g('publish_period').value or 2.0))

        # latched：晚来的读者（另一个终端里的 map_archive.sh save）也能立刻收到
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, topic, qos)
        self.srv = self.create_service(Trigger, service, self._on_query)
        self.timer = self.create_timer(period, self._tick)
        node_count, verified = self._graph_facts()
        self.get_logger().info(
            '[map_session] 会话播报：map_name=%s world=%s session_id=%s（话题 %s｜服务 %s；'
            '图上 %s ×%s、serialize_map=%s ⇒ verified=%s）'
            % (self.map_name or '（空）', self.world or '（空）', self.session_id, topic, service,
               self.mapper_node, node_count, self._serialize_present, verified))
        self._tick()

    # ------------------------------------------------------------------ 图事实
    def _graph_facts(self):
        """(图上有几个同名映射器, 该映射器是否唯一且提供 serialize 服务)。读的是**实时图**。"""
        try:
            endpoints = []
            for name, ns in self.get_node_names_and_namespaces():
                endpoints.append(('/' + name) if ns in ('', '/') else (ns.rstrip('/') + '/' + name))
            mappers = [n for n in endpoints if os.path.basename(n) == self.mapper_node]
            helpers = [n for n in endpoints if os.path.basename(n) == self.get_name()]
            self._node_count = len(endpoints)
            self._mappers = mappers
            self._helpers = helpers
        except Exception as e:                                    # pragma: no cover
            self._node_count, self._mappers, self._helpers = -1, [], []
            self.get_logger().warn('[map_session] 读节点表失败（不影响建图）：%s' % e)
        try:
            self._serialize_present = self.serialize_service in \
                [n for n, _t in self.get_service_names_and_types()]
        except Exception:                                         # pragma: no cover
            self._serialize_present = False
        verified = (len(self._mappers) == 1 and self._helpers == ['/' + self.get_name()]
                    and self._serialize_present)
        return len(self._mappers), verified

    def payload(self):
        mapper_count, verified = self._graph_facts()
        return {
            'schema': SESSION_SCHEMA,
            'map_name': self.map_name,
            'world': self.world,
            'archive_base': self.archive_base,
            'map_start_pose': self.map_start_pose,
            'resumed': self.resumed,
            'autocontinue': self.autocontinue,
            'allow_world_mismatch': self.allow_world_mismatch,
            'started_at': self.started_at,
            'session_id': self.session_id,
            'launch_pid': self.launch_pid,
            'announcer_pid': os.getpid(),
            'host': socket.gethostname(),
            'topic': self.pub.topic_name,
            'service': self.srv.srv_name,
            'mapper_node': self.mapper_node,
            'mapper_nodes': list(getattr(self, '_mappers', [])),
            'mapper_count': mapper_count,
            'helper_nodes': list(getattr(self, '_helpers', [])),
            'helper_count': len(getattr(self, '_helpers', [])),
            'serialize_service': self.serialize_service,
            'serialize_present': bool(getattr(self, '_serialize_present', False)),
            'verified': bool(verified),
            'graph_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'node_count': int(getattr(self, '_node_count', -1)),
        }

    # ------------------------------------------------------------------ 发布 / 应答
    def _tick(self):
        msg = String()
        msg.data = json.dumps(self.payload(), ensure_ascii=False)
        self.pub.publish(msg)

    def _on_query(self, _req, res):
        p = self.payload()
        res.success = bool(p['verified'])
        res.message = json.dumps(p, ensure_ascii=False)
        return res


def main(argv=None):
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = None
    try:
        node = MapSessionAnnouncer()
        rclpy.spin(node)
    except KeyboardInterrupt:                                     # pragma: no cover
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
