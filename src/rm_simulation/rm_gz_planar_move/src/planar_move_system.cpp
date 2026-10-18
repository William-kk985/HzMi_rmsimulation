// =============================================================================
// planar_move_system.cpp —— `sim_backend:=gz` 的**底盘驱动**（我们自己的代码）
// =============================================================================
// 一句话：这是 Classic 的 `libgazebo_ros_planar_move.so` 在 Gazebo Fortress 上的
// **对应实现** —— `/cmd_vel_chassis` 进 → 车体按同一套语义动。
//
// 为什么不用 gz 自带的系统插件（都试过，逐条否掉）：
//   · `DiffDrive` / `JointController`：需要**轮子关节 + 轮子控制器**。本仓 robot11 的
//     URDF 里 `j2..j11` 是云台/炮管，**没有轮子关节**，`.xacro` 自己写着"不加轮子
//     控制器" ⇒ 无对象可驱动（Classic 侧也是同一个原因才用 planar_move）。
//   · `VelocityControl`：它对每个 link 写 `LinearVelocityCmd/AngularVelocityCmd`，物理
//     引擎再积分。这条正是 Classic 上被实测否掉的形态：多刚体 + 接触约束每步把速度
//     "掰回来"，角速度执行率被吃到 1% 量级（docs/tilted_lidar_fidelity.md §M.3）；
//     而且本仓 gz 侧的模型是 `<static>`（robot11 的 SDF 由 URDF 派生、只保留
//     livox_frame/imu_link 两个 fixed 子 link），改动态就要重建整套关节/惯量/接触，
//     收益是"更物理"，代价是把 Classic 已修好的那件事再坏一次。
//   · 上游 `libgazebo_ros_planar_move.so` 本身：Classic 专有（gazebo/physics），
//     Fortress 加载不了。
//
// ⚠️ **写 gz 组件的时机必须是 `Update` 阶段，不能是 `PreUpdate`**（实测踩过）：
//   gz-sim 的一步是 `PreUpdate(所有系统) → Update(所有系统) → PostUpdate(所有系统)`，
//   而 **物理系统在 `Update` 里"步进 + 把 link/sensor 的
//   WorldPose/AngularVelocity/LinearAcceleration 从物理引擎写回"**（gz-sim6 的
//   `Physics::Update` → `PhysicsPrivate::UpdateSim`，src/systems/physics/Physics.cc:841）。
//   在 PreUpdate 里写 AngularVelocity，物理系统随后把它覆盖成静止车的 0
//   ⇒ `/livox/imu` 的陀螺恒 0（实测：命令 wz=0.5 rad/s，IMU 均值 0.0000）⇒ LIO 认为车没转。
//   模型插件是**后加载**的，所以在本插件的 `Update` 里写 = "物理之后、PostUpdate 之前"。
//
// 所以这里做的是**运动学底盘**（与 Classic 的 planar_move 同语义、同话题、同单位）：
//   订阅 gz transport 的 `gz.msgs.Twist`（由 `ros_gz_bridge` 从 ROS `/cmd_vel_chassis`
//   桥过来）→ 每个物理步按 (vx, vy, wz) 积分**模型位姿** → 直接写回 gz 的 ECM：
//     · `components::Pose`（model，世界系）      ← 射线插件用 `gz::sim::worldPose()`
//                                                  沿父链算位姿，读的就是它
//     · `components::WorldPose`（每个 link）      ← 给读 WorldPose 的消费者
//     · `components::WorldPose` + `AngularVelocity`（IMU 传感器实体）
//                                                ← ignition-gazebo-imu-system 读这两者；
//                                                  不写的话陀螺恒 0，LIO 会认为车没转
//   并把 `/odom_ground_truth`（`nav_msgs/Odometry`，世界位姿真值）在**本进程内**用
//   rclcpp 直发 —— 与 Classic 的 planar_move 同名同语义（`docs/fortress_sim_backend.md`
//   §2.9-9 之前记的是"gz 侧缺这个话题"）。
//
// **诚实的边界**（写进 docs 的未实现/未验证清单）：
//   1. 运动学、**没有碰撞响应**：车会穿墙。Classic 的 planar_move 是"设速度让物理积分"，
//      撞墙会被挡住；这一条 gz 侧**不等价**（要等价就得把模型改成动态 + 轮子/接触，
//      见上）。
//   2. **权威 100%**（命令 1.0 rad/s 就是 1.0 rad/s），Classic 侧因为轮子对地摩擦只
//      执行 ~5–16%（§M.7 实测）。同轮 A/B 里两个数都测、都报，不做"看起来一样"的假账。
//   3. 加速度计不反映向心/切向加速度（只保留 gz 物理给的重力项）；陀螺反映命令角速度。
//   4. 平面运动（z 恒定、roll/pitch 恒定），不支持差速/阿克曼/悬挂。
//
// SDF 用法（模型级插件）：
//   <plugin filename="librm_gz_planar_move.so" name="rm_gz::PlanarMove">
//     <link_name>base_link</link_name>          <!-- 位姿基准 link（默认 base_link） -->
//     <topic>/cmd_vel_chassis</topic>           <!-- gz 侧 Twist 话题（默认同名） -->
//     <odom_topic>/odom_ground_truth</odom_topic>
//     <odom_frame>odom</odom_frame>
//     <base_frame>base_link</base_frame>
//     <odom_rate>50</odom_rate>                 <!-- 真值里程计发布率（Hz，仿真时间） -->
//     <publish_odom>true</publish_odom>
//     <command_timeout>0.0</command_timeout>    <!-- 0 = 永不过期（= planar_move 语义） -->
//     <max_vx>3.0</max_vx> <max_wz>4.0</max_wz> <!-- 安全钳位 -->
//   </plugin>
//
// 作者：HzMi。许可：Apache-2.0（与本仓一致）。**不包含任何第三方源码。**
// =============================================================================
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/ParentEntity.hh>
// Fortress（gz-sim6）把 components::WorldPose 定义在 Pose.hh 里，没有单独的
// WorldPose.hh（gz-sim7+ 才拆出去）；所以这里只 include Pose.hh。
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Fortress = ignition-gazebo6 / ignition-common4 只定义 ign* 拼法的控制台宏；
// gz* 拼法是 gz-common5+ 才有的（同一坑见 tools/scripts/gzsim/patches/0002）。
#ifndef gzerr
  #define gzerr ignerr
  #define gzwarn ignwarn
  #define gzmsg ignmsg
  #define gzdbg igndbg
#endif

namespace rm_gz
{
  namespace
  {
    template <typename T>
    T SdfValue(const std::shared_ptr<const sdf::Element> &_sdf,
               const std::string &_key, const T &_default)
    {
      if (!_sdf || !_sdf->HasElement(_key))
        return _default;
      return _sdf->Get<T>(_key);
    }
  }  // namespace

  class PlanarMove : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemUpdate
  {
  public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override
    {
      gz::sim::Model model(_entity);
      if (!model.Valid(_ecm))
      {
        gzerr << "[RM_PLANAR] 必须挂在 model 上（当前实体 " << _entity << "）。\n";
        return;
      }
      this->modelEntity = _entity;
      this->modelName = model.Name(_ecm);

      const std::string linkName = SdfValue<std::string>(_sdf, "link_name", "base_link");
      this->linkEntity = model.LinkByName(_ecm, linkName);
      if (this->linkEntity == gz::sim::kNullEntity)
      {
        gzerr << "[RM_PLANAR] 找不到 link [" << linkName << "]，无法确定位姿基准。\n";
        return;
      }
      this->topic = SdfValue<std::string>(_sdf, "topic", "/cmd_vel_chassis");
      this->odomTopic = SdfValue<std::string>(_sdf, "odom_topic", "/odom_ground_truth");
      this->odomFrame = SdfValue<std::string>(_sdf, "odom_frame", "odom");
      this->baseFrame = SdfValue<std::string>(_sdf, "base_frame", linkName);
      this->odomRate = std::max(0.0, SdfValue<double>(_sdf, "odom_rate", 50.0));
      this->publishOdom = SdfValue<bool>(_sdf, "publish_odom", true);
      this->commandTimeout = std::max(0.0, SdfValue<double>(_sdf, "command_timeout", 0.0));
      this->maxVx = std::max(0.0, SdfValue<double>(_sdf, "max_vx", 5.0));
      this->maxVy = std::max(0.0, SdfValue<double>(_sdf, "max_vy", 5.0));
      this->maxWz = std::max(0.0, SdfValue<double>(_sdf, "max_wz", 6.0));

      // 初始位姿：模型在世界的位姿（= 世界文件 <include><pose> 给的出生点）
      this->pose = gz::sim::worldPose(_entity, _ecm);
      this->startPose = this->pose;

      // 每个 link 相对 model 的固定偏移（运动学底盘把它们整体搬走）
      this->links.clear();
      const auto linkEntities = model.Links(_ecm);
      for (const auto &le : linkEntities)
      {
        const auto linkPose = gz::sim::worldPose(le, _ecm);
        this->links.push_back({le, this->pose.Inverse() * linkPose});
      }

      this->gzNode.Subscribe(this->topic, &PlanarMove::OnCmdVel, this);

      // 真值里程计：与 Classic 的 planar_move 同名同语义（nav_msgs/Odometry）。
      // 用本进程内的 rclcpp 直发（与 mid360 插件同一个做法：不走 bridge）。
      if (this->publishOdom)
      {
        if (!rclcpp::ok())
        {
          int argc = 0;
          char **argv = nullptr;
          rclcpp::init(argc, argv);
        }
        if (!this->node)
        {
          this->node = std::make_shared<rclcpp::Node>(
            "rm_gz_planar_move_" + this->modelName);
          this->odomPub = this->node->create_publisher<nav_msgs::msg::Odometry>(
            this->odomTopic, rclcpp::QoS(rclcpp::KeepLast(50)));
        }
      }

      this->configured = true;
      gzmsg << "[RM_PLANAR] " << this->modelName << " 运动学底盘就绪：link=" << linkName
            << " 命令话题(gz)=" << this->topic
            << " 真值里程计=" << (this->publishOdom ? this->odomTopic : std::string("off"))
            << " odom_rate=" << this->odomRate << " Hz"
            << " 出生位姿=" << this->startPose.Pos() << " yaw="
            << this->startPose.Rot().Yaw() << " rad\n";
    }

    // ★ 在 `Update` 阶段写 ECM（理由见文件头：物理系统在 Update 里覆盖速度/位姿组件）
    void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override
    {
      if (!this->configured || _info.paused)
        return;

      const double dt = std::chrono::duration<double>(_info.dt).count();
      if (!(dt > 0.0))
        return;
      this->lastSimTime = _info.simTime;

      // 命令超时（默认 0 = 永不过期，与 planar_move 一致：nav2 会持续发）
      double vx = this->cmdVx, vy = this->cmdVy, wz = this->cmdWz;
      if (this->commandTimeout > 0.0)
      {
        const double age = std::chrono::duration<double>(
          _info.simTime - this->lastCmdTime).count();
        if (age > this->commandTimeout)
          vx = vy = wz = 0.0;
      }
      vx = std::clamp(vx, -this->maxVx, this->maxVx);
      vy = std::clamp(vy, -this->maxVy, this->maxVy);
      wz = std::clamp(wz, -this->maxWz, this->maxWz);

      if (vx != 0.0 || vy != 0.0 || wz != 0.0 || !this->firstStepDone)
      {
        this->StepKinematics(_info, _ecm, vx, vy, wz, dt);
        this->firstStepDone = true;
      }

      // 真值里程计（仿真时间节拍）
      if (this->publishOdom && this->odomPub && this->odomRate > 0.0)
      {
        const double period = 1.0 / this->odomRate;
        const double now = std::chrono::duration<double>(_info.simTime).count();
        if (now >= this->nextOdomTime)
        {
          this->nextOdomTime = now + period;
          this->PublishOdom(_info, vx, vy, wz);
        }
      }
    }

  private:
    void StepKinematics(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm,
                        double vx, double vy, double wz, double dt)
    {
      // 平面运动学：先在**当前车体系**里平移，再按 wz 转（planar_move 的语义）
      const gz::math::Pose3d cur = this->pose;
      const double yaw0 = cur.Rot().Yaw();
      const double yaw1 = yaw0 + wz * dt;
      const gz::math::Vector3d dp(
        (std::cos(yaw0) * vx - std::sin(yaw0) * vy) * dt,
        (std::sin(yaw0) * vx + std::cos(yaw0) * vy) * dt,
        0.0);
      this->pose = gz::math::Pose3d(cur.Pos() + dp,
                                    gz::math::Quaterniond(0.0, 0.0, yaw1));
      this->pose.Rot().Normalize();

      // ① model 的 Pose（世界系）：射线插件 `gz::sim::worldPose()` 沿父链读的就是它
      auto modelPose = _ecm.Component<gz::sim::components::Pose>(this->modelEntity);
      if (modelPose)
        modelPose->SetData(this->pose, this->poseEql);
      else
        _ecm.CreateComponent(this->modelEntity,
                             gz::sim::components::Pose(this->pose));

      // ② 每个 link 的 WorldPose（`<static>` 模型不会被物理更新 ⇒ 这里说了算）
      for (const auto &l : this->links)
      {
        const auto wp = this->pose * l.relPose;
        auto comp = _ecm.Component<gz::sim::components::WorldPose>(l.entity);
        if (comp)
          comp->SetData(wp, this->poseEql);
        else
          _ecm.CreateComponent(l.entity, gz::sim::components::WorldPose(wp));

        // 车体系角速度/线速度：IMU 与 velocity 类消费者读它们
        auto av = _ecm.Component<gz::sim::components::AngularVelocity>(l.entity);
        if (av)
          av->SetData(gz::math::Vector3d(0.0, 0.0, wz), this->vecEql);
        else
          _ecm.CreateComponent(l.entity,
            gz::sim::components::AngularVelocity(gz::math::Vector3d(0.0, 0.0, wz)));
        auto lv = _ecm.Component<gz::sim::components::LinearVelocity>(l.entity);
        if (lv)
          lv->SetData(gz::math::Vector3d(vx, vy, 0.0), this->vecEql);
        else
          _ecm.CreateComponent(l.entity,
            gz::sim::components::LinearVelocity(gz::math::Vector3d(vx, vy, 0.0)));

        // ③ 挂在这个 link 上的传感器（IMU 传感器实体）：ignition-gazebo-imu-system
        //    读的是**传感器实体**的 WorldPose + AngularVelocity。
        _ecm.Each<gz::sim::components::Imu, gz::sim::components::ParentEntity>(
          [&](const gz::sim::Entity &_e,
              const gz::sim::components::Imu *,
              const gz::sim::components::ParentEntity *_parent) -> bool
          {
            if (_parent->Data() != l.entity)
              return true;
            gz::math::Pose3d relSensor;
            const auto sp = _ecm.Component<gz::sim::components::Pose>(_e);
            if (sp)
              relSensor = sp->Data();
            const gz::math::Pose3d sensorWorld = wp * relSensor;
            auto sw = _ecm.Component<gz::sim::components::WorldPose>(_e);
            if (sw)
              sw->SetData(sensorWorld, this->poseEql);
            else
              _ecm.CreateComponent(_e, gz::sim::components::WorldPose(sensorWorld));
            auto sav = _ecm.Component<gz::sim::components::AngularVelocity>(_e);
            if (sav)
              sav->SetData(gz::math::Vector3d(0.0, 0.0, wz), this->vecEql);
            else
              _ecm.CreateComponent(_e,
                gz::sim::components::AngularVelocity(gz::math::Vector3d(0.0, 0.0, wz)));
            return true;
          });
      }
      (void)_info;
    }

    void OnCmdVel(const gz::msgs::Twist &_msg)
    {
      this->cmdVx = _msg.linear().x();
      this->cmdVy = _msg.linear().y();
      this->cmdWz = _msg.angular().z();
      this->lastCmdTime = this->lastSimTime;
      this->nCmd++;
    }

    void PublishOdom(const gz::sim::UpdateInfo &_info, double vx, double vy, double wz)
    {
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        _info.simTime).count();
      nav_msgs::msg::Odometry odom;
      odom.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
      odom.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
      odom.header.frame_id = this->odomFrame;
      odom.child_frame_id = this->baseFrame;
      odom.pose.pose.position.x = this->pose.Pos().X();
      odom.pose.pose.position.y = this->pose.Pos().Y();
      odom.pose.pose.position.z = this->pose.Pos().Z();
      odom.pose.pose.orientation.x = this->pose.Rot().X();
      odom.pose.pose.orientation.y = this->pose.Rot().Y();
      odom.pose.pose.orientation.z = this->pose.Rot().Z();
      odom.pose.pose.orientation.w = this->pose.Rot().W();
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = wz;
      // 真值：协方差给 0（"这是真值，不是估计"）——与 Classic 的 planar_move 一致
      this->odomPub->publish(odom);
    }

    struct LinkRel
    {
      gz::sim::Entity entity;
      gz::math::Pose3d relPose;
    };

    struct PoseEql
    {
      bool operator()(const gz::math::Pose3d &_a, const gz::math::Pose3d &_b) const
      {
        return _a == _b;
      }
    };
    struct VecEql
    {
      bool operator()(const gz::math::Vector3d &_a, const gz::math::Vector3d &_b) const
      {
        return _a == _b;
      }
    };

    bool configured{false};
    bool firstStepDone{false};
    gz::sim::Entity modelEntity{gz::sim::kNullEntity};
    gz::sim::Entity linkEntity{gz::sim::kNullEntity};
    std::string modelName;
    std::string topic{"/cmd_vel_chassis"};
    std::string odomTopic{"/odom_ground_truth"};
    std::string odomFrame{"odom"};
    std::string baseFrame{"base_link"};
    double odomRate{50.0};
    bool publishOdom{true};
    double commandTimeout{0.0};
    double maxVx{5.0}, maxVy{5.0}, maxWz{6.0};
    double nextOdomTime{0.0};
    std::atomic<double> cmdVx{0.0}, cmdVy{0.0}, cmdWz{0.0};
    std::chrono::steady_clock::duration lastCmdTime{0};
    std::chrono::steady_clock::duration lastSimTime{0};
    std::uint64_t nCmd{0};
    gz::math::Pose3d pose;
    gz::math::Pose3d startPose;
    std::vector<LinkRel> links;
    PoseEql poseEql;
    VecEql vecEql;
    gz::transport::Node gzNode;
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
  };
}  // namespace rm_gz

IGNITION_ADD_PLUGIN(rm_gz::PlanarMove, gz::sim::System,
                    gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)
IGNITION_ADD_PLUGIN_ALIAS(rm_gz::PlanarMove, "rm_gz::PlanarMove")
