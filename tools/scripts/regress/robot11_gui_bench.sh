#!/usr/bin/env bash
# ============================================================================
# robot11_gui_bench.sh —— `robot:=robot11` 带 GUI 的**代价实测**（2026-10-07 Phase 4）
#
# 为什么需要（用户 2026-10-07 的 GUI 跑）：`gzclient` 被 SIGKILL（exit code -9）、rviz2 黑屏。
# 怀疑对象：这台车的视觉是上游原始 STL，`base_link.STL` 一项就是 **99 MiB / 2,078,226 三角形**
# （+ 云台 l11 8.8 MB / 17.6 万面）。本脚本把"视觉档位 × GUI"量成一张表：
#   · gzclient 有没有活到窗口结束（还是被 SIGKILL / 段错误）
#   · gzclient / gzserver / rviz2 的**峰值 RSS**
#   · RTF、/odom 帧数（GUI 把 gzserver 拖慢了多少）
#   · 截图里"非黑像素占比"（rviz2/gzclient 到底画没画东西 —— 防"进程活着但黑屏"）
#
# ⚠️ 需要一个 X display。**无头机器**（如本仓 CI/沙箱）没有 X server 时本脚本会拒绝跑并打印
#    "无法实测 GUI"，这是**故意**的：GUI 结论不许靠推断冒充实测（docs/robot_models.md §12）。
#    有 Xvfb 的机器可以：`XVFB_BIN=/path/to/Xvfb tools/scripts/regress/robot11_gui_bench.sh ...`
#
# 用法：
#   tools/scripts/regress/robot11_gui_bench.sh <tag> [--visual decimated|full] [--seconds 40] \
#       [--world RMUC2026] [--mode mapping] [--rviz true|false]
# 输出：.tmp_robotslot/<tag>/{launch.log,rss.csv,shot-*.xwd,shot-*.txt,summary.txt}
#
# 隔离约定（同 run_robot_model_probe.sh）：HOME=$REPO/.tmp_robotslot/<tag>/gzhome（**不用 /tmp**：
#   本仓的沙箱里 /tmp 每次 bash 调用都是新的 tmpfs）、非默认 ROS_DOMAIN_ID、专用 GAZEBO_MASTER_URI。
# ============================================================================
set -o pipefail
set +u   # /opt/ros/humble/setup.bash 自己不干净（引用未绑定的 AMENT_TRACE_SETUP_FILES）

TAG="${1:?用法: robot11_gui_bench.sh <tag> [--visual decimated|full] [--seconds 40]}"
shift
VISUAL=decimated
SECONDS_WIN=40
WORLD=RMUC2026
MODE=mapping
RVIZ=true
while [ $# -gt 0 ]; do
  case "$1" in
    --visual) VISUAL="$2"; shift 2 ;;
    --seconds) SECONDS_WIN="$2"; shift 2 ;;
    --world) WORLD="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --rviz) RVIZ="$2"; shift 2 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$REPO/.tmp_robotslot/$TAG"
mkdir -p "$OUT"

HASH=$(printf '%s' "$TAG" | cksum | cut -d' ' -f1)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( 30 + HASH % 61 ))}"
# 端口先探测再占用（同 run_robot_model_probe.sh：残留 gzserver 占着端口 ⇒ 新 gzserver exit 255）
if [ -z "${GAZEBO_MASTER_URI:-}" ]; then
  PORT=$(python3 -c "
import socket
p = 11350 + $HASH % 500
for _ in range(200):
    s = socket.socket()
    try:
        s.bind(('127.0.0.1', p)); s.close(); print(p); break
    except OSError:
        s.close(); p += 1
else:
    print(p)
")
  export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
fi
export HOME="$OUT/gzhome"          # ⚠️ 不用 /tmp：沙箱里 /tmp 每次调用都是新 tmpfs
mkdir -p "$HOME"
export ROS_LOG_DIR="$OUT/roslog"
mkdir -p "$ROS_LOG_DIR"

# ---- display：没有就**拒绝跑**（GUI 结论不许靠推断冒充实测） ----
XVFB_PID=""
if [ -z "${DISPLAY:-}" ]; then
  if [ -n "${XVFB_BIN:-}" ] && [ -x "${XVFB_BIN}" ]; then
    DISP=":$(( 90 + HASH % 9 ))"
    "$XVFB_BIN" "$DISP" -screen 0 3200x1200x24 -nolisten tcp > "$OUT/xvfb.log" 2>&1 &
    XVFB_PID=$!
    sleep 2
    export DISPLAY="$DISP"
    echo "[gui] 无 DISPLAY ⇒ 用 XVFB_BIN=$XVFB_BIN 起了虚拟屏 $DISP（**软件 GL**：渲染吞吐不代表真 GPU）"
  else
    echo "[gui] ❌ 没有 DISPLAY、也没有 XVFB_BIN ⇒ **无法实测 GUI**。"
    echo "[gui]    带 GUI 的结论只能标成'未实测'（这是本脚本故意不做的事：不拿推断冒充实测）。"
    exit 3
  fi
fi
echo "[gui] DISPLAY=$DISPLAY"
glxinfo -B 2>/dev/null | grep -E "OpenGL renderer|OpenGL version" | sed 's/^/[gui] /' || true

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
GZ_PORT="${GAZEBO_MASTER_URI##*:}"

cleanup() {
  timeout 10 ros2 topic pub -r 5 -t 5 /cmd_vel_chassis geometry_msgs/msg/Twist \
      '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 3
  # /odom 与 /cmd_vel_chassis 的契约不受影响；这里只清本 MASTER_URI 上的 gazebo 进程。
  pkill -f "gzserver.*$GZ_PORT" 2>/dev/null || true
  pkill -f "gzclient.*$GZ_PORT" 2>/dev/null || true
  [ -n "${LAUNCH_PID:-}" ] && kill -9 "$LAUNCH_PID" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  sleep 1
  [ -n "$XVFB_PID" ] && kill -9 "$XVFB_PID" 2>/dev/null || true
}
trap cleanup EXIT
pkill -f "gzserver.*$GZ_PORT" 2>/dev/null || true
sleep 1

# ⚠️ `nav_rviz` 的取值必须与 launch 里的比较逐字一致（`'mapping' and 'True'`）——
#    传 `true` 小写**不会**起 rviz2（实测：launch.log 里连 rviz2 的 "process started" 都没有）。
case "$RVIZ" in true|True|1) RVIZ=True ;; *) RVIZ=False ;; esac
echo "[gui] launch: world=$WORLD mode=$MODE robot11_visual=$VISUAL nav_rviz=$RVIZ"
ros2 launch rm_nav_bringup bringup_sim.launch.py world:="$WORLD" mode:="$MODE" \
    lio:=small_point_lio robot:=robot11 robot11_visual:="$VISUAL" \
    nav_rviz:="$RVIZ" map_autocontinue:=False > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

# ---- 等 gzclient/rviz2 真的起来（软件 GL 下 gzserver 加载世界很慢）再开始采样窗口 ----
WAIT_MAX=180
for i in $(seq 1 "$WAIT_MAX"); do
  pgrep -x gzclient >/dev/null && pgrep -x rviz2 >/dev/null && break
  sleep 1
done
echo "[gui] 窗口起点：等了 ${i}s（gzclient=$(pgrep -x gzclient >/dev/null && echo up || echo down) rviz2=$(pgrep -x rviz2 >/dev/null && echo up || echo down)）"

# ---- 采样：RSS + 存活 + 截图 ------------------------------------------------
echo "t_s,rss_gzclient_kb,rss_gzserver_kb,rss_rviz2_kb,alive_gzclient,alive_rviz2" > "$OUT/rss.csv"
peak_gz() { local pat="$1"; ps -o rss= -C "$pat" 2>/dev/null | sort -rn | head -1 | tr -d ' '; }
for t in $(seq 1 "$SECONDS_WIN"); do
  sleep 1
  gc=$(ps -o rss= -C gzclient 2>/dev/null | sort -rn | head -1 | tr -d ' ')
  gs=$(ps -o rss= -C gzserver 2>/dev/null | sort -rn | head -1 | tr -d ' ')
  rv=$(ps -o rss= -C rviz2 2>/dev/null | sort -rn | head -1 | tr -d ' ')
  agc=0; pgrep -x gzclient >/dev/null && agc=1
  arv=0; pgrep -x rviz2 >/dev/null && arv=1
  echo "$t,${gc:-0},${gs:-0},${rv:-0},$agc,$arv" >> "$OUT/rss.csv"
  if [ "$t" = "$SECONDS_WIN" ]; then
    xwd -root -silent > "$OUT/shot-final.xwd" 2>/dev/null || true
    xwininfo -root -tree > "$OUT/windows.txt" 2>/dev/null || true
  fi
  if [ "$t" = "$(( SECONDS_WIN / 2 ))" ]; then
    xwd -root -silent > "$OUT/shot-mid.xwd" 2>/dev/null || true
  fi
done

# ---- 截图统计（黑屏判据）---------------------------------------------------
python3 - "$OUT" <<'PY' | tee "$OUT/shot-summary.txt"
import os, re, struct, sys
import numpy as np
out = sys.argv[1]
# 无窗口管理器时 Qt/Gazebo 会把自己的窗口摆到 +1585 之类的坐标 ⇒ 视窗太小的 Xvfb 上
# **root 截图只覆盖 gzclient**（实测过一次）。所以这里同时按窗口矩形裁一块出来，
# 让"rviz2 到底画没画"也有像素证据（窗口几何来自同一次 xwininfo -root -tree）。
rects = {}
wp = os.path.join(out, 'windows.txt')
if os.path.isfile(wp):
    for line in open(wp):
        m = re.search(r'0x[0-9a-f]+\s+"([^"]*)".*?(\d+)x(\d+)\+(-?\d+)\+(-?\d+)', line)
        if not m:
            continue
        name, w, h, x, y = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4)), int(m.group(5))
        if 'RViz' in name or 'rviz' in name.lower():
            if w * h > rects.get('rviz', (0, 0, 0, 0, ''))[2] * rects.get('rviz', (0, 0, 0, 0, ''))[3]:
                rects['rviz'] = (x, y, w, h, name)
for name in ('shot-mid', 'shot-final'):
    p = os.path.join(out, name + '.xwd')
    if not os.path.isfile(p) or os.path.getsize(p) < 200:
        print('%-12s 没有截图（xwd 失败 / X server 不可用）' % name)
        continue
    b = open(p, 'rb').read()
    hdr = struct.unpack('>25I', b[:100])
    hsz, ver, fmt, depth, w, h = hdr[0], hdr[1], hdr[2], hdr[3], hdr[4], hdr[5]
    bo, bpp, bpl, ncol = hdr[7], hdr[11], hdr[12], hdr[19]
    data = np.frombuffer(b[hsz:hsz + bpl * h], dtype=np.uint8).reshape(h, bpl)
    px = data[:, :w * (bpp // 8)].reshape(h, w, bpp // 8)
    def frac(sub):
        r = sub[:, :, :3].astype(np.int16)
        return float((r.max(axis=2) > 12).mean()), len(np.unique(sub[:, :, :3].reshape(-1, 3), axis=0))
    nb, nc = frac(px)
    print('%-12s %dx%d depth=%d bpp=%d 非黑像素=%.4f 颜色数(粗)=%d'
          % (name, w, h, depth, bpp, nb, nc))
    if 'rviz' in rects:
        x, y, rw, rh, rname = rects['rviz']
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(w, x + rw), min(h, y + rh)
        if x1 > x0 and y1 > y0:
            nb2, nc2 = frac(px[y0:y1, x0:x1])
            print('  └ rviz2 窗口矩形 %dx%d+%d+%d 内：非黑=%.4f 颜色数=%d（%s）'
                  % (rw, rh, x, y, nb2, nc2, rname[:48]))
        else:
            print('  └ rviz2 窗口 %dx%d+%d+%d **在 root 之外** ⇒ root 截图看不到它' % (rw, rh, x, y))
PY

# ---- 结果 ------------------------------------------------------------------
GC_DEAD_LINE=$(grep -nE "gzclient.*(process has died|died|exit code)" "$OUT/launch.log" | tail -1)
{
  echo "tag=$TAG visual=$VISUAL world=$WORLD mode=$MODE nav_rviz=$RVIZ display=$DISPLAY"
  echo "gzclient 死亡行: ${GC_DEAD_LINE:-<无>}"
  echo "rviz2   死亡行: $(grep -nE 'rviz2.*(process has died|died|exit code)' "$OUT/launch.log" | tail -1)"
  python3 - "$OUT" <<'PY'
import csv, os, sys
out = sys.argv[1]
rows = list(csv.DictReader(open(os.path.join(out, 'rss.csv'))))
def pk(k):
    return max(int(r[k]) for r in rows) // 1024 if rows else -1
def last(k):
    return int(rows[-1][k]) // 1024 if rows else -1
print('峰值 RSS (MiB): gzclient=%d gzserver=%d rviz2=%d' % (pk('rss_gzclient_kb'), pk('rss_gzserver_kb'), pk('rss_rviz2_kb')))
print('末次 RSS (MiB): gzclient=%d gzserver=%d rviz2=%d' % (last('rss_gzclient_kb'), last('rss_gzserver_kb'), last('rss_rviz2_kb')))
print('存活: gzclient=%s (最后 5 秒: %s)  rviz2=%s' % (
    rows[-1]['alive_gzclient'] if rows else '?',
    ''.join(r['alive_gzclient'] for r in rows[-5:]), rows[-1]['alive_rviz2'] if rows else '?'))
PY
  echo "RTF/odom（从 launch.log 抓）: $(grep -oE 'RTF[^,]*' "$OUT/launch.log" | tail -1)"
  echo "odom 条数: $(grep -cE '\[slope_speed\]|odom' "$OUT/launch.log")（仅供参考，正式口径看 probe.json）"
} | tee "$OUT/summary.txt"
echo "[gui] 输出目录: $OUT"
