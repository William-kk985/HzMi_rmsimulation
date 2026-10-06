#!/usr/bin/env bash
# =============================================================================
# map_archive.sh —— 「在上次基础上继续建图」的存档/查询工具（2D 位姿图 + 3D 先验）
#
# 它做三件事（子命令）：
#   save [--name X]   把**正在跑的** slam_toolbox 的位姿图存到
#                     src/rm_nav_bringup/map/<name>.{posegraph,data}（同名覆盖），
#                     等两个文件都真的出现，然后刷新 sidecar <name>.meta.yaml；
#                     若 3D 累加器（cloud_accumulator:=True）在跑，顺带存 PCD/<name>.pcd。
#   info [--name X]   打印某份存档的 sidecar（world/出生点/版本）+ 各文件大小与时间戳。
#   list              列出 map/（位姿图）与 PCD/（点云）下所有存档及其 world 摘要。
#   adopt --name X --world W   给**旧版**（没有 sidecar 的）存档补一份 sidecar —— 等于人工担保它属于 W。
#   dirs              打印"launch 与脚本实际读写的那两个目录"（绕开 install/share 的软链陷阱）。
#
# 为什么必须有 sidecar + 守卫：本工程 `map` 系 = **出生点相对系**，换 world 就是换原点。
#   把 A 场地的位姿图/点云加载到 B 场地 = 两场比赛的地图叠在一起（假墙/回环错配/定位全废）。
#   判定与报错文本与 launch / cloud_accumulator 共用同一份：
#       src/rm_nav_bringup/scripts/map_asset_guard.py
#
# 名字从哪来（按优先级）：
#   ① --name X       ② 环境变量 $MAP_NAME
#   ③ <map_dir>/.session.yaml（**推荐**：launch 每次都会写，见 bringup_sim.launch.py）
#   ④ 正在跑的 /cloud_accumulator 的 map_name 参数
#   ⑤ 正在跑的 /slam_toolbox 的 map_file_name（续建时才有值）
#   ⑥ 都没有 ⇒ 报错让你显式给 --name（**故意不猜**：存错名字比报错危险得多）
#
# 典型用法（终端 B，栈在终端 A 跑着）：
#   tools/scripts/mapping/map_archive.sh save                 # 存档名 = 本次 launch 的 map_name
#   tools/scripts/mapping/map_archive.sh save --name RMUC2026_home
#   tools/scripts/mapping/map_archive.sh info --name RMUC2026_home
#   tools/scripts/mapping/map_archive.sh list
#
# 退出码：0 成功；2 用法/环境问题；3 **守卫拒绝**（场地不一致，见打印出来的四条出路）；4 落盘超时。
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
GUARD="$WS/src/rm_nav_bringup/scripts/map_asset_guard.py"
NAME=""; WORLD=""; TIMEOUT=60; ALLOW=0; WITH_CLOUD=1; RESOLUTION=""

usage() { sed -n '2,45p' "${BASH_SOURCE[0]}"; }

py_guard() { python3 "$GUARD" "$@"; }

# ---- 目录（走守卫模块，与 launch/节点**同一份**判定：symlink-install 下取源码树目录）
MAP_DIR="$(py_guard dirs 2>/dev/null | sed -n 's/^map_dir=//p')"
PCD_DIR="$(py_guard dirs 2>/dev/null | sed -n 's/^pcd_dir=//p')"
[ -n "$MAP_DIR" ] || { echo "[map_archive] ❌ 拿不到 map 目录（先 source install/setup.bash？）" >&2; exit 2; }

# ---- 会话状态（launch 写的；用于默认名字/场地）
SESSION="$MAP_DIR/.session.yaml"
session_get() {   # session_get <key>
  [ -f "$SESSION" ] || return 0
  python3 - "$SESSION" "$1" <<'PY'
import sys, yaml
try:
    d = yaml.safe_load(open(sys.argv[1])) or {}
except Exception:
    sys.exit(0)
v = d.get(sys.argv[2])
print('' if v is None else v)
PY
}

ros_param() {     # ros_param <node> <param>  —— 拿不到就空
  timeout 8 ros2 param get "$1" "$2" 2>/dev/null | sed -n 's/^String value is: //p'
}

resolve_name() {
  [ -n "$NAME" ] && return 0
  [ -n "${MAP_NAME:-}" ] && { NAME="$MAP_NAME"; echo "[map_archive] 名字来自 \$MAP_NAME=$NAME"; return 0; }
  local v; v="$(session_get map_name)"
  if [ -n "$v" ]; then
    # 会话状态可能是"上一次跑完留下的"：pid 不在 ⇒ 至少提醒一句（不拦，用户仍可 --name 覆盖）
    local spid; spid="$(session_get session_pid)"
    if [ -n "$spid" ] && ! kill -0 "$spid" 2>/dev/null; then
      echo "[map_archive] ⚠️ 会话状态里的 launch pid=$spid 已经不在了 ⇒ 下面这个名字可能来自**上一次**会话；若不对请显式 --name X" >&2
    fi
    NAME="$v"; echo "[map_archive] 名字来自本次会话状态 $SESSION：map_name=$NAME"; return 0
  fi
  v="$(ros_param /cloud_accumulator map_name)"
  if [ -n "$v" ]; then NAME="$v"; echo "[map_archive] 名字来自运行中的 /cloud_accumulator：map_name=$NAME"; return 0; fi
  v="$(ros_param /slam_toolbox map_file_name)"
  if [ -n "$v" ]; then NAME="$(basename "$v")"; echo "[map_archive] 名字来自运行中的 /slam_toolbox：map_file_name=$v"; return 0; fi
  return 1
}

resolve_world() {
  [ -n "$WORLD" ] && return 0
  [ -n "${RM_WORLD:-}" ] && { WORLD="$RM_WORLD"; echo "[map_archive] 场地来自 \$RM_WORLD=$WORLD"; return 0; }
  local v; v="$(session_get world)"
  if [ -n "$v" ]; then WORLD="$v"; echo "[map_archive] 场地来自本次会话状态：world=$WORLD"; return 0; fi
  if [ -n "$NAME" ] && [ -f "$MAP_DIR/$NAME.meta.yaml" ]; then
    v="$(python3 - "$MAP_DIR/$NAME.meta.yaml" <<'PY'
import sys, yaml
try:
    d = yaml.safe_load(open(sys.argv[1])) or {}
except Exception:
    sys.exit(0)
print(d.get('world') or '')
PY
)"
    if [ -n "$v" ]; then WORLD="$v"; echo "[map_archive] 场地来自存档自身的 sidecar：world=$WORLD"; return 0; fi
  fi
  return 1
}

mapper_resolution() {
  local v; v="$(timeout 8 ros2 param get /slam_toolbox resolution 2>/dev/null | sed -n 's/^Double value is: //p')"
  [ -n "$v" ] && echo "$v" || echo "0.05"
}

wait_for_files() {   # wait_for_files <base> <timeout>
  local base="$1" to="$2" t=0
  while [ "$t" -lt "$to" ]; do
    if [ -s "$base.posegraph" ] && [ -s "$base.data" ]; then return 0; fi
    sleep 1; t=$((t + 1))
  done
  return 1
}

human() { python3 -c "import os,sys;p=sys.argv[1];print('%.2f MB'%(os.path.getsize(p)/1048576.0) if os.path.isfile(p) else '缺失')" "$1"; }

# ============================================================ save
cmd_save() {
  resolve_name || { echo "[map_archive] ❌ 无法确定存档名：请显式 --name X（或 --name / \$MAP_NAME；也可给 launch 传 map_name:=X）" >&2; exit 2; }
  resolve_world || { echo "[map_archive] ❌ 无法确定 world：请显式 --world W（例如 --world RMUC2026）" >&2; exit 2; }
  local base="$MAP_DIR/$NAME"
  echo "[map_archive] 存档: name=$NAME world=$WORLD"
  echo "[map_archive]   map_dir=$MAP_DIR"
  echo "[map_archive]   base=$base"

  # ---- 起手就必须能看见服务，否则后面等 60 s 是白等
  if ! timeout 10 ros2 service list 2>/dev/null | grep -qx '/slam_toolbox/serialize_map'; then
    echo "[map_archive] ❌ 没有 /slam_toolbox/serialize_map 服务 —— 只有 'mode:=mapping|slam_nav + mapper:=slam_toolbox' 才在。" >&2
    echo "              先起栈：ros2 launch rm_nav_bringup bringup_sim.launch.py world:=$WORLD mode:=mapping lio:=fastlio mapper:=slam_toolbox" >&2
    exit 2
  fi

  # ---- 守卫：**先查再写**（拒绝就把存档原样留着，绝不覆盖别场地的图）
  RESOLUTION="$(mapper_resolution)"
  local check_args=(check --kind posegraph --name "$NAME" --world "$WORLD" --resolution "$RESOLUTION")
  [ "$ALLOW" = 1 ] && check_args+=(--allow-world-mismatch)
  set +e
  py_guard "${check_args[@]}"
  local rc=$?
  set -e
  if [ "$rc" = 3 ]; then
    echo "[map_archive] ❌ 存档被守卫拒绝（退出码 3）：**没有写入任何文件**。" >&2
    exit 3
  elif [ "$rc" != 0 ]; then
    echo "[map_archive] ❌ 守卫检查本身失败（退出码 $rc）" >&2; exit 2
  fi

  # ---- 记录写之前的指纹（用于确认"真的被重写了"，而不是"本来就在那儿"）
  local pre_pg pre_data
  pre_pg="$(stat -c '%s %Y' "$base.posegraph" 2>/dev/null || echo "none")"
  pre_data="$(stat -c '%s %Y' "$base.data" 2>/dev/null || echo "none")"

  echo "[map_archive] 调 /slam_toolbox/serialize_map → $base"
  set +e
  timeout 120 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
      "{filename: '$base'}" 2>&1 | tee /tmp/map_archive_serialize.$$.log
  local src=${PIPESTATUS[0]}
  set -e
  if [ "$src" != 0 ]; then
    echo "[map_archive] ❌ serialize_map 服务调用失败（退出码 $src，见 /tmp/map_archive_serialize.$$.log）" >&2; exit 4
  fi
  if grep -q 'result=255' /tmp/map_archive_serialize.$$.log 2>/dev/null; then
    echo "[map_archive] ❌ slam_toolbox 回报 RESULT_FAILED_TO_WRITE_FILE（磁盘满/权限？）" >&2; exit 4
  fi

  echo "[map_archive] 等 $base.{posegraph,data} 落盘（最多 ${TIMEOUT}s）…"
  if ! wait_for_files "$base" "$TIMEOUT"; then
    echo "[map_archive] ❌ ${TIMEOUT}s 内没等到两个文件都出现：$base.posegraph / $base.data" >&2
    ls -l "$base".* 2>&1 | sed 's/^/    /' >&2
    exit 4
  fi

  # ---- 写/刷新 sidecar（world + 出生点 + 版本；这是下次续建能"拒绝错场地"的唯一依据）
  local spawn_args=()
  local sp; sp="$(session_get map_start_pose)"
  [ -n "$sp" ] && spawn_args+=(--map-start-pose "$sp")
  py_guard write-manifest --kind posegraph --name "$NAME" --world "$WORLD" \
      --resolution "$RESOLUTION" "${spawn_args[@]}" || {
        echo "[map_archive] ❌ sidecar 写入失败" >&2; exit 4; }

  # ---- 3D 先验（累加器在跑就顺带存；它自己也会过守卫）
  local cloud_note="（没有 /cloud_accumulator —— 本次未存 3D 先验；要它请加 cloud_accumulator:=True）"
  if [ "$WITH_CLOUD" = 1 ] && timeout 10 ros2 service list 2>/dev/null | grep -qx '/cloud_accumulator/save'; then
    echo "[map_archive] 顺带存 3D 先验：/cloud_accumulator/save"
    if timeout 300 ros2 service call /cloud_accumulator/save std_srvs/srv/Trigger 2>&1 | tee /tmp/map_archive_cloud.$$.log | tail -3; then :; fi
    if grep -q 'success=True' /tmp/map_archive_cloud.$$.log 2>/dev/null; then
      cloud_note="✅ PCD/$NAME.pcd 已更新"
    else
      cloud_note="⚠️ /cloud_accumulator/save 未成功（见上面的 message；守卫拒绝时会明确说明）"
    fi
  fi

  echo
  echo "[map_archive] ✅ 存档完成（同名覆盖）"
  echo "  位姿图 : $base.posegraph  $(human "$base.posegraph")"
  echo "  数据集 : $base.data       $(human "$base.data")"
  echo "  sidecar: $base.meta.yaml  $(human "$base.meta.yaml")"
  echo "  3D 先验: $cloud_note"
  echo "  写前指纹: posegraph=$pre_pg data=$pre_data"
  echo "  写后指纹: posegraph=$(stat -c '%s %Y' "$base.posegraph") data=$(stat -c '%s %Y' "$base.data")"
  echo "  下次续建（同一个命令就行，launch 会自动反序列化并接着建）："
  echo "      ros2 launch rm_nav_bringup bringup_sim.launch.py world:=$WORLD mode:=mapping \\"
  echo "          lio:=fastlio mapper:=slam_toolbox map_name:=$NAME"
  rm -f /tmp/map_archive_serialize.$$.log /tmp/map_archive_cloud.$$.log
}

# ============================================================ info / list / adopt / dirs
cmd_info() {
  resolve_name || { echo "[map_archive] ❌ 无法确定存档名：请显式 --name X" >&2; exit 2; }
  echo "[map_archive] 目录: map=$MAP_DIR  PCD=$PCD_DIR"
  if [ -f "$SESSION" ]; then
    echo "[map_archive] 本次会话（$SESSION）:"
    sed 's/^/    /' "$SESSION"
  fi
  echo
  py_guard show --kind posegraph --name "$NAME"
  echo
  py_guard show --kind pcd --name "$NAME"
  echo
  echo "[map_archive] 若上面 world 与你要跑的 world 不一致 ⇒ 续建会被**拒绝**（这是设计）："
  echo "    换名字 / 删改名 / map_autocontinue:=False / 显式 map_allow_world_mismatch:=True"
}

cmd_list() { py_guard list "${@}"; }

cmd_adopt() {
  [ -n "$NAME" ] || { echo "[map_archive] ❌ adopt 必须给 --name（要补 sidecar 的存档基名）" >&2; exit 2; }
  [ -n "$WORLD" ] || { echo "[map_archive] ❌ adopt 必须给 --world（你担保它属于哪个场地）" >&2; exit 2; }
  py_guard adopt --kind posegraph --name "$NAME" --world "$WORLD" || exit $?
  # 旧版存档常常还有同名 PCD（PCD/<name>.pcd）；存在就一并补
  if [ -f "$PCD_DIR/$NAME.pcd" ]; then
    py_guard adopt --kind pcd --name "$NAME" --world "$WORLD" || true
  fi
}

cmd_dirs() { py_guard dirs; [ -f "$SESSION" ] && { echo "session=$SESSION"; sed 's/^/  /' "$SESSION"; }; }

CMD="${1:-}"
[ $# -gt 0 ] && shift
while [ $# -gt 0 ]; do
  case "$1" in
    --name) NAME="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    --allow-world-mismatch) ALLOW=1; shift;;
    --no-cloud) WITH_CLOUD=0; shift;;
    --json) LIST_JSON=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "[map_archive] 未知参数：$1" >&2; usage; exit 2;;
  esac
done

case "$CMD" in
  save) cmd_save;;
  info) cmd_info;;
  list) cmd_list ${LIST_JSON:+--json};;
  adopt) cmd_adopt;;
  dirs) cmd_dirs;;
  ''|-h|--help|help) usage; exit 0;;
  *) echo "[map_archive] 未知子命令：$CMD" >&2; usage; exit 2;;
esac
