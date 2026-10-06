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
# ★ 2026-10-06（事故当天追加）：**覆盖前备份** + **恢复**
#   save 一直是"同名覆盖"。当天 3D 先验被一段 LIO 退化（ATE max 1.15 m）污染后，
#   一次 save 就把好存档换成了坏存档（见 docs/continue_mapping.md §9 事故复盘）。
#   现在 save 在写之前，把**既有**的 map/<name>.{posegraph,data,meta.yaml} 各复制一份
#   `<文件>.prev-<YYYYmmdd-HHMMSS>`（同代同时间戳，默认留最新 3 代，更老的自动删）；
#   PCD 那一侧由 cloud_accumulator 的 ~/save 自己备份（谁写谁备份，避免双重备份/两套命名）。
#   backup --name X [--kind posegraph|pcd|all]   手工备份一代（save 内部调的是同一条 guard 命令）
#   backups --name X            看这份存档现存的备份代
#   restore --name X [--from T] 回滚（T = 时间戳 或 某个 *.prev-* 路径；默认最新一代）；
#                               回滚前会把"当前文件"也留一代 ⇒ restore 本身可逆（退出码 5 = 没有这一代）
#
# 为什么必须有 sidecar + 守卫：本工程 `map` 系 = **出生点相对系**，换 world 就是换原点。
#   把 A 场地的位姿图/点云加载到 B 场地 = 两场比赛的地图叠在一起（假墙/回环错配/定位全废）。
#   判定与报错文本与 launch / cloud_accumulator 共用同一份：
#       src/rm_nav_bringup/scripts/map_asset_guard.py
#   ⚠️ 注意：守卫只管"world/spawn **对不对**"，管不了"图**本身歪了**"（当天事故就是后者）
#      ⇒ 续建后另有一次一致性检查（launch 参数 map_resume_check*，见 docs/continue_mapping.md §8）。
#
# 名字从哪来（**2026-10-06 事故后重写：不再"读文件就用"**）：
#   ① --name X        —— 人担保；打印为"显式"。若 X ≠ 活栈会话名 ⇒ 仍拒绝（见 ④），
#                        除非显式 --allow-cross-session（那时等于"另存一份"）
#   ② **ROS 图上的活会话** —— launch 起的播报器节点 /map_session（latched 话题
#                        /map_session/info + 服务 /map_session/query，内容含
#                        map_name/world/archive_base/map_start_pose/started_at/session_id）
#                        ⇒ 名字来自**正在跑的这套栈本身**，任何后启动的 launch 都覆盖不了它
#   ③ 活着的 bringup_sim.launch.py 进程的命令行（map_name:= / world:=）；看不到进程时
#                        （组合 launch / 别的 PID namespace）退一步用**活映射器自报的存档基名**
#                        （slam_toolbox 的 map_file_name —— 续建时才有值）
#   ④ <map_dir>/.session.yaml **+ 把它钉在活栈上的证明**（活 launch cmdline 对得上 /
#                        slam_toolbox 的 map_file_name 基名对得上 / 记录的 pid 活着且
#                        cmdline 与起始时刻都对得上 —— pid **单独不算证据**）
#   ⑤ 都不成立 ⇒ **拒绝**（退出码 3，列出查了什么、给三条出路），**绝不猜名字**
#   为什么不用 `kill -0 <session_pid>` 当活性检查（旧版的 bug）：
#     · 假阳性：pid 会被回收 ⇒ 早已结束的会话"看起来还活着"；
#     · 假阴性：记录的常是**包装进程**（`setsid ros2 launch … &` 的 $!、timeout、外层脚本）
#       或 launch 主进程先走而子节点还活着（本仓库 §5.2 记过 setsid/$! 这个坑）。
#   ⚠️ 多会话（同时两套栈）⇒ 直接拒绝并列出：两个 /slam_toolbox、两个 /map_session、
#      两个 mapping 形态的 bringup_sim.launch.py、或两份 session_id 不同的播报。
#      两套栈想同时存在就用不同 ROS_DOMAIN_ID（各自的 save 只看得到自己那一套）。
#
# 典型用法（终端 B，栈在终端 A 跑着）：
#   tools/scripts/mapping/map_archive.sh save                 # 存档名 = 本次 launch 的 map_name
#   tools/scripts/mapping/map_archive.sh save --check         # 只看"会用哪个名字+证据链"，不写盘
#   tools/scripts/mapping/map_archive.sh save --name RMUC2026_home
#   tools/scripts/mapping/map_archive.sh info --name RMUC2026_home
#   tools/scripts/mapping/map_archive.sh list
#   tools/scripts/mapping/map_archive.sh backups --name RMUC2026_home      # 看有几代备份
#   tools/scripts/mapping/map_archive.sh restore --name RMUC2026_home      # 回滚到最新一代
#
# 退出码：0 成功；2 用法/环境问题；3 **拒绝**（场地不一致 或 会话名不确认/多会话，见打印出来的出路）；
#         4 落盘超时；5 restore/backups 找不到可用的备份代。
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
GUARD="$WS/src/rm_nav_bringup/scripts/map_asset_guard.py"
NAME=""; WORLD=""; TIMEOUT=60; ALLOW=0; WITH_CLOUD=1; RESOLUTION=""
FROM=""; DRY_RUN=0; KEEP=3; LIST_JSON=0; KIND="all"; ALLOW_XSESSION=0
SESSION=""; SESSION_SOURCE=""; SESSION_WORLD=""; SESSION_SPAWN=""; SESSION_ID=""
SESSION_ARCHIVE_BASE=""; SESSION_STARTED=""; SESSION_NAME_RESOLVED=""

usage() { sed -n '2,/^set -u$/p' "${BASH_SOURCE[0]}" | sed '$d'; }

py_guard() { python3 "$GUARD" "$@"; }

# ---- 目录（走守卫模块，与 launch/节点**同一份**判定：symlink-install 下取源码树目录）
MAP_DIR="$(py_guard dirs 2>/dev/null | sed -n 's/^map_dir=//p')"
PCD_DIR="$(py_guard dirs 2>/dev/null | sed -n 's/^pcd_dir=//p')"
[ -n "$MAP_DIR" ] || { echo "[map_archive] ❌ 拿不到 map 目录（先 source install/setup.bash？）" >&2; exit 2; }

# ---- 会话状态（launch 写的；用于宽松取名/出生点回退；save 走 guard 的严格解析）
#      $MAP_ARCHIVE_SESSION_FILE 只给测试/排查用（把"会被覆盖的那份文件"指到别处，验证拒绝路径）
SESSION="${MAP_ARCHIVE_SESSION_FILE:-$MAP_DIR/.session.yaml}"
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
  # ★ 2026-10-06 事故后：名字**不再**从 .session.yaml 直接取。
  #   唯一出处 = guard 的 session-resolve（①--name ＞ ②图上活会话播报器 ＞ ③活 launch 进程
  #   ＞ ④.session.yaml+活性证明 ＞ ⑤拒绝）。它会打印完整证据链，并在拒绝时给出出路。
  #   这样做的原因：.session.yaml 会被任何后启动的 launch 覆盖（当天 31.89 MB 位姿图就是
  #   这么写到并发测试栈的名字上的），而"活栈自己在 ROS 图上广播的名字"谁也覆盖不了。
  local args=(session-resolve --session-file "$SESSION") out rc=0
  if [ -z "$NAME" ] && [ -n "${MAP_NAME:-}" ]; then
    NAME="$MAP_NAME"; echo "[map_archive] 名字来自 \$MAP_NAME=$NAME（等同 --name，仍会与活栈交叉核对）"
  fi
  [ -n "$NAME" ] && args+=(--name "$NAME")
  [ -n "$WORLD" ] && args+=(--world "$WORLD")
  [ "$ALLOW_XSESSION" = 1 ] && args+=(--allow-cross-session)
  out="$(py_guard "${args[@]}")"; rc=$?
  printf '%s\n' "$out"
  while IFS= read -r line; do
    case "$line" in
      session_source=*)       SESSION_SOURCE="${line#session_source=}";;
      session_name=*)         SESSION_NAME_RESOLVED="${line#session_name=}";;
      session_world=*)        SESSION_WORLD="${line#session_world=}";;
      session_spawn=*)        SESSION_SPAWN="${line#session_spawn=}";;
      session_id=*)           SESSION_ID="${line#session_id=}";;
      session_started_at=*)   SESSION_STARTED="${line#session_started_at=}";;
      session_archive_base=*) SESSION_ARCHIVE_BASE="${line#session_archive_base=}";;
    esac
  done <<< "$out"
  if [ "$rc" != 0 ]; then
    echo "[map_archive] ❌ 拒绝存档：**没法确认「这次该写哪个名字」**（退出码 $rc）—— 没有写任何文件。" >&2
    echo "              依据在上面（每条查了什么、缺哪条证明、以及 --name 等出路）；详见 docs/continue_mapping.md §11" >&2
    return "$rc"
  fi
  [ -n "$SESSION_NAME_RESOLVED" ] || { echo "[map_archive] ❌ 内部错误：解析成功但没拿到名字" >&2; return 2; }
  NAME="$SESSION_NAME_RESOLVED"
  echo "[map_archive] ✅ 会话名已确认：name=$NAME world=${SESSION_WORLD:-?} 来源=$SESSION_SOURCE" >&2
  return 0
}

resolve_world() {
  [ -n "$WORLD" ] && return 0
  [ -n "${RM_WORLD:-}" ] && { WORLD="$RM_WORLD"; echo "[map_archive] 场地来自 \$RM_WORLD=$WORLD"; return 0; }
  if [ -n "$SESSION_WORLD" ]; then
    WORLD="$SESSION_WORLD"
    echo "[map_archive] 场地来自活会话证据（来源=$SESSION_SOURCE）：world=$WORLD"
    return 0
  fi
  local v; v="$(session_get world)"
  if [ -n "$v" ]; then WORLD="$v"; echo "[map_archive] 场地来自会话状态文件：world=$WORLD"; return 0; fi
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

# ---- 宽松取名（info / backups / backup / restore 用；这些**不写存档**，且常在栈关掉之后跑）
resolve_name_loose() {
  [ -n "$NAME" ] && return 0
  [ -n "${MAP_NAME:-}" ] && { NAME="$MAP_NAME"; echo "[map_archive] 名字来自 \$MAP_NAME=$NAME"; return 0; }
  local v; v="$(session_get map_name)"
  if [ -n "$v" ]; then
    NAME="$v"
    echo "[map_archive] ⚠️ 名字来自 $SESSION —— 这个文件**会被任何后启动的 launch 覆盖**（2026-10-06 事故根因）。" >&2
    echo "              本子命令不写存档 ⇒ 只提示；save 则要求它被「活栈证据」钉住，否则拒绝。" >&2
    echo "              要锁定名字请显式：map_archive.sh ${CMD:-info} --name $NAME" >&2
    return 0
  fi
  v="$(ros_param /cloud_accumulator map_name)"
  [ -n "$v" ] && { NAME="$v"; echo "[map_archive] 名字来自运行中的 /cloud_accumulator：map_name=$NAME"; return 0; }
  v="$(ros_param /slam_toolbox map_file_name)"
  [ -n "$v" ] && { NAME="$(basename "$v")"; echo "[map_archive] 名字来自运行中的 /slam_toolbox：map_file_name=$v"; return 0; }
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

# ---- 3D 先验体检：**独立于节点**再量一次 PCD 的 bbox/z 跨度（写完之后）
#      为什么还要在 shell 里量一次：`~/save` 的体检在累加器节点的日志里，而 map_archive.sh 是在
#      **另一个终端**跑的；这里再打一遍数字，保证"存档这一步"的输出里就有证据。
#      判定/文案在 tools/scripts/mapping/pcd_bbox_health.py（可单独跑、可测；退出码 3 = 不合理）。
pcd_health() {   # pcd_health <pcd> <z_span_warn>
  python3 "$WS/tools/scripts/mapping/pcd_bbox_health.py" "$1" --z-span-warn "${2:-3.0}" || true
}

# ============================================================ save
cmd_save() {
  # ---- 名字：严格解析（--name ＞ 活图播报器 ＞ 活 launch 进程 ＞ .session.yaml+活性证明 ＞ 拒绝）
  #      拒绝 ⇒ 直接非 0 退出（**没有写任何文件**）；这一段是 2026-10-06 事故的直接修复。
  local rrc=0
  resolve_name || rrc=$?
  [ "$rrc" = 0 ] || exit "$rrc"
  resolve_world || { echo "[map_archive] ❌ 无法确定 world：请显式 --world W（例如 --world RMUC2026）" >&2; exit 2; }
  local base="$MAP_DIR/$NAME"
  echo "[map_archive] 存档: name=$NAME world=$WORLD（会话来源=$SESSION_SOURCE${SESSION_ID:+ session_id=$SESSION_ID}）"
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

  # ---- ★ --check / --dry-run：把"会用哪个名字 + 证据链 + 会做什么"打全，然后**什么都不写**
  if [ "$DRY_RUN" = 1 ]; then
    echo
    echo "[map_archive] 🧪 --check/--dry-run：**没有写任何文件**。若现在真的 save，会依次做："
    echo "    ① 覆盖前备份既有 map/$NAME.{posegraph,data,meta.yaml}（keep=$KEEP 代：*.prev-<时间戳>）"
    echo "    ② ros2 service call /slam_toolbox/serialize_map \"{filename: '$base'}\""
    echo "    ③ 等 $base.{posegraph,data} 落盘（最多 ${TIMEOUT}s）后刷新 sidecar（world=$WORLD${SESSION_SPAWN:+ spawn=$SESSION_SPAWN}）"
    echo "    ④ 若 /cloud_accumulator/save 在 ⇒ 顺带覆盖 PCD/$NAME.pcd（它自己备份）"
    echo "    名字来源=$SESSION_SOURCE；完整证据链见上面 ①~⑤ 五行。"
    exit 0
  fi

  # ---- 记录写之前的指纹（用于确认"真的被重写了"，而不是"本来就在那儿"）
  local pre_pg pre_data
  pre_pg="$(stat -c '%s %Y' "$base.posegraph" 2>/dev/null || echo "none")"
  pre_data="$(stat -c '%s %Y' "$base.data" 2>/dev/null || echo "none")"

  # ---- ★ 覆盖前备份（2026-10-06 事故后新增）：写坏了一代就能回退
  #      谁写谁备份：本脚本写 posegraph/data/sidecar ⇒ 备份这三个；
  #      PCD 由 cloud_accumulator 的 ~/save 自己备份（它写那个文件）—— 避免双重备份/两套命名。
  echo "[map_archive] 覆盖前备份（keep=$KEEP 代；命名 <文件>.prev-<时间戳>）"
  py_guard backup --name "$NAME" --kind posegraph --keep "$KEEP" || {
    echo "[map_archive] ⚠️ 备份失败（不挡落盘，但请自己确认既有存档可回退）" >&2; }

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
  #      出生点优先取**活会话证据**（播报器/活 launch），取不到才回退会话状态文件。
  local spawn_args=()
  local sp="$SESSION_SPAWN"
  [ -n "$sp" ] || sp="$(session_get map_start_pose)"
  [ -n "$sp" ] && spawn_args+=(--map-start-pose "$sp")
  py_guard write-manifest --kind posegraph --name "$NAME" --world "$WORLD" \
      --resolution "$RESOLUTION" "${spawn_args[@]}" || {
        echo "[map_archive] ❌ sidecar 写入失败" >&2; exit 4; }

  # ---- 3D 先验（累加器在跑就顺带存；它自己也会过守卫 + 写前体检 + 写前备份）
  local cloud_note="（没有 /cloud_accumulator —— 本次未存 3D 先验；要它请加 cloud_accumulator:=True）"
  local cloud_pcd=""
  if [ "$WITH_CLOUD" = 1 ] && timeout 10 ros2 service list 2>/dev/null | grep -qx '/cloud_accumulator/save'; then
    echo "[map_archive] 顺带存 3D 先验：/cloud_accumulator/save（覆盖前备份由累加器自己打印）"
    if timeout 300 ros2 service call /cloud_accumulator/save std_srvs/srv/Trigger 2>&1 | tee /tmp/map_archive_cloud.$$.log | tail -3; then :; fi
    if grep -q 'success=True' /tmp/map_archive_cloud.$$.log 2>/dev/null; then
      cloud_note="✅ PCD/$NAME.pcd 已更新"
      cloud_pcd="$PCD_DIR/$NAME.pcd"
    else
      cloud_note="⚠️ /cloud_accumulator/save 未成功（见上面的 message；守卫拒绝时会明确说明）"
    fi
    grep -q '健康告警' /tmp/map_archive_cloud.$$.log 2>/dev/null && \
      cloud_note="$cloud_note ⚠️（累加器报了健康告警：看上面 message 与节点日志）"
  fi

  echo
  echo "[map_archive] ✅ 存档完成（同名覆盖；写前已备份，回滚：map_archive.sh restore --name $NAME）"
  echo "  会话来源: $SESSION_SOURCE${SESSION_ID:+（session_id=$SESSION_ID${SESSION_STARTED:+，起于 $SESSION_STARTED}）}"
  echo "  位姿图 : $base.posegraph  $(human "$base.posegraph")"
  echo "  数据集 : $base.data       $(human "$base.data")"
  echo "  sidecar: $base.meta.yaml  $(human "$base.meta.yaml")"
  echo "  3D 先验: $cloud_note"
  echo "  写前指纹: posegraph=$pre_pg data=$pre_data"
  echo "  写后指纹: posegraph=$(stat -c '%s %Y' "$base.posegraph") data=$(stat -c '%s %Y' "$base.data")"
  echo "  备份代（位姿图侧，最新在前）："
  py_guard backups --name "$NAME" --kind posegraph 2>/dev/null | sed 's/^/    /' || true
  # 3D 先验独立体检（z 跨度不合理 ⇒ 响亮 WARNING；仍然已经写出去了）
  [ -n "$cloud_pcd" ] && pcd_health "$cloud_pcd" "${CLOUD_WARN_Z_SPAN:-3.0}"
  echo "  下次续建（同一个命令就行，launch 会自动反序列化并接着建）："
  echo "      ros2 launch rm_nav_bringup bringup_sim.launch.py world:=$WORLD mode:=mapping \\"
  echo "          lio:=fastlio mapper:=slam_toolbox map_name:=$NAME"
  rm -f /tmp/map_archive_serialize.$$.log /tmp/map_archive_cloud.$$.log
}

# ============================================================ info / list / adopt / dirs
cmd_info() {
  resolve_name_loose || { echo "[map_archive] ❌ 无法确定存档名：请显式 --name X" >&2; exit 2; }
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
  py_guard backups --name "$NAME" 2>/dev/null || true
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

# ============================================================ backups / restore
# （2026-10-06 事故后新增：save 是同名覆盖，一次坏 save 就能把好存档换掉）
cmd_backups() {
  resolve_name_loose || { echo "[map_archive] ❌ 无法确定存档名：请显式 --name X" >&2; exit 2; }
  py_guard backups --name "$NAME" --kind "$KIND" ${LIST_JSON:+--json}
}

cmd_backup() {
  resolve_name_loose || { echo "[map_archive] ❌ 无法确定存档名：请显式 --name X" >&2; exit 2; }
  echo "[map_archive] 手工备份一代：name=$NAME kind=$KIND keep=$KEEP"
  py_guard backup --name "$NAME" --kind "$KIND" --keep "$KEEP"
}

cmd_restore() {
  resolve_name_loose || { echo "[map_archive] ❌ 无法确定存档名：请显式 --name X" >&2; exit 2; }
  local args=(restore --name "$NAME" --kind "$KIND" --keep "$KEEP")
  [ -n "$FROM" ] && args+=(--from "$FROM")
  [ "$DRY_RUN" = 1 ] && args+=(--dry-run)
  echo "[map_archive] 回滚存档 $NAME（map/ 与 PCD/ 两侧；keep=$KEEP）"
  py_guard "${args[@]}"
  local rc=$?
  if [ "$rc" = 0 ] && [ "$DRY_RUN" = 0 ]; then
    echo
    echo "[map_archive] 恢复后核对（文件大小/时间戳）："
    py_guard show --kind posegraph --name "$NAME" 2>/dev/null | sed -n '/文件:/,$p' | sed 's/^/  /'
    py_guard show --kind pcd --name "$NAME" 2>/dev/null | sed -n '/文件:/,$p' | sed 's/^/  /'
    echo "  ⚠️ 恢复的是**上一代**存档：它当时的 world/spawn 由同代 sidecar 一起恢复 ⇒ 守卫看到的是一套自洽的。"
    echo "     真要继续建图：ros2 launch rm_nav_bringup bringup_sim.launch.py world:=<sidecar 里的 world> \\"
    echo "         mode:=mapping mapper:=slam_toolbox map_name:=$NAME"
  fi
  exit $rc
}

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
    --kind) KIND="$2"; shift 2;;
    --from) FROM="$2"; shift 2;;
    --keep) KEEP="$2"; shift 2;;
    --dry-run|--check) DRY_RUN=1; shift;;          # save 的 --check/--dry-run = 只打印不写盘
    --allow-cross-session) ALLOW_XSESSION=1; shift;;  # 明确要"另存一份"（名字≠活栈会话名）时用
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
  backup) cmd_backup;;
  backups) cmd_backups;;
  restore) cmd_restore;;
  ''|-h|--help|help) usage; exit 0;;
  *) echo "[map_archive] 未知子命令：$CMD" >&2; usage; exit 2;;
esac
