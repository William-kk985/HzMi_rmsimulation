#!/usr/bin/env bash
# =============================================================================
# fetch_and_patch_mid360.sh —— 一键获取 + 打补丁 + 编译 Gazebo Fortress 的
#                              Livox MID-360 CPU 射线传感器（opt-in 后端 gz）
# =============================================================================
#
# 这个脚本**不 vendor 任何第三方源码进本仓库**：它把三个上游钉在固定 revision，
# 克隆到仓库外（或 .tmp_gzlivox2/ 这类被 .gitignore 忽略的目录），套上**我们自己的**
# 补丁（tools/scripts/gzsim/patches/*.patch，见同目录 ATTRIBUTION.md），再 colcon
# 编到独立的 build/install 里。
#
#   上游 ①  uos/rmagine                        tag 2.4.3      BSD-3-Clause
#           —— raycasting 引擎本体（Embree 后端）
#   上游 ②  uos/rmagine_gazebo_plugins          c6dc04e        BSD-3-Clause
#           —— gz-sim 系统插件：MapRegistry + Embree BVH 建树
#   上游 ③  ashduwihch/mid360_gazebo_harmonic   497149a        MIT
#           —— MID-360 传感器系统（CSV 图案 + 求交 + ROS 发布）
#
# 补丁（全部是我们的，逐条理由写在 patch 头部）：
#   patches/0001-rmagine-gazebo-plugins-self-exclusion.patch  (上游②)
#   patches/0002-mid360-fortress-compat.patch                (上游③)
#   patches/0003-mid360-ros-contract.patch                   (上游③)
#
# 用法：
#   tools/scripts/gzsim/fetch_and_patch_mid360.sh                # 获取+打补丁+编译
#   tools/scripts/gzsim/fetch_and_patch_mid360.sh --no-build     # 只获取+打补丁
#   tools/scripts/gzsim/fetch_and_patch_mid360.sh --rebuild      # 强制重新编译
#   GZSIM_WS=/opt/gzsim_ws tools/scripts/gzsim/fetch_and_patch_mid360.sh
#
# 编完之后（launch 会自己找，但如果要在裸 shell 里 `ign gazebo` 就必须 source）：
#   source <GZSIM_WS>/install/setup.bash
#
# ⚠️ Embree 4（Apache-2.0）是**构建期下载**：rmagine 的 CMake 用 FetchContent 拉
#    prebuilt 包（apt 只有 3.12，不可用）。离线/CI 要么预置 tarball、要么
#    `-DRMAGINE_DEPENDENCY_MODE=Local` 指到系统里的 Embree ≥4。
#    实测：干净机器第一次 configure 会联网下载 ~13 s，之后走 build 目录缓存。
# =============================================================================
set -euo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
PATCH_DIR="$REPO/tools/scripts/gzsim/patches"

# 三个 pin（改这里 = 换上游版本；改完必须重跑并重新验收）
PIN_RMAGINE_TAG="${GZSIM_PIN_RMAGINE_TAG:-2.4.3}"
PIN_RGP="${GZSIM_PIN_RGP:-c6dc04e2570fe64742d2e6a54e0641b0e5bf7fde}"
PIN_MID360="${GZSIM_PIN_MID360:-497149a82b5fcf7ca0645995ba0b3b937fe033a6}"

# 工作区：默认落在仓库内被忽略的 scratch 目录；可用 GZSIM_WS 指到仓库外
GZSIM_WS="${GZSIM_WS:-$REPO/.tmp_gzlivox2/gzsim_ws}"
UPSTREAM="${GZSIM_UPSTREAM:-$GZSIM_WS/upstream}"
SRC="$GZSIM_WS/src"
BUILD="$GZSIM_WS/build"
INSTALL="$GZSIM_WS/install"

DO_BUILD=1
FORCE=0
for arg in "$@"; do
  case "$arg" in
    --no-build) DO_BUILD=0 ;;
    --rebuild)  FORCE=1 ;;
    -h|--help)  sed -n '2,45p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------- 0. 依赖自检
say "0/5 检查本机依赖（Fortress + ROS 2 Humble + livox_ros_driver2）"
missing=0
check() { if [ -e "$1" ]; then echo "  ok   $2"; else echo "  MISS $2  ($1)"; missing=1; fi; }
check /usr/lib/x86_64-linux-gnu/libignition-gazebo6-physics-system.so "ignition-gazebo6"
check /opt/ros/humble/setup.bash "ROS 2 Humble"
[ "$missing" = 0 ] || { echo "缺少依赖，先装：sudo apt install ros-humble-ros-gz ignition-fortress"; exit 1; }
if [ -f "$REPO/install/setup.bash" ]; then
  echo "  ok   repo install/setup.bash（CustomMsg 支持来自这里的 livox_ros_driver2）"
else
  echo "  警告: 没找到 $REPO/install/setup.bash —— 先 colcon build 本仓，"
  echo "        否则插件编出来只能发 PointCloud2（发不了 /livox/lidar 的 CustomMsg）。"
fi

mkdir -p "$UPSTREAM" "$SRC" "$BUILD" "$INSTALL"

fetch() {  # fetch <name> <url> <ref-kind> <ref>
  local name="$1" url="$2" kind="$3" ref="$4"
  if [ -d "$UPSTREAM/$name/.git" ]; then
    echo "  reuse $UPSTREAM/$name"
  else
    local i
    for i in 1 2 3 4 5; do
      rm -rf "$UPSTREAM/$name"
      if [ "$kind" = tag ]; then
        git clone -q --branch "$ref" --depth 1 "$url" "$UPSTREAM/$name" && break
      else
        git clone -q "$url" "$UPSTREAM/$name" && break
      fi
      echo "  clone 失败（第 $i 次），10 s 后重试：$url" >&2
      sleep 10
    done
    [ -d "$UPSTREAM/$name" ] || { echo "clone 彻底失败：$url" >&2; exit 1; }
  fi
  if [ "$kind" != tag ]; then
    git -C "$UPSTREAM/$name" fetch -q --all --tags || true
  fi
  # 每次都以「干净的上游 revision」为起点：上一次运行已经把我们的补丁提交/留在了
  # 工作区里，直接 checkout 会被拒（"本地修改将被覆盖"）。这里是**我们自己的**
  # 上游克隆，reset --hard + clean 只丢补丁，而补丁下一步会重新打一遍，所以幂等。
  git -C "$UPSTREAM/$name" reset -q --hard "$ref"
  git -C "$UPSTREAM/$name" clean -qfd
  local got; got=$(git -C "$UPSTREAM/$name" rev-parse HEAD)
  echo "  pin  $name @ $got"
}

# ------------------------------------------------------------ 1. 取三个上游
say "1/5 获取上游（钉住 revision）"
fetch rmagine      https://github.com/uos/rmagine.git                      tag  "$PIN_RMAGINE_TAG"
fetch rgp          https://github.com/uos/rmagine_gazebo_plugins.git       rev  "$PIN_RGP"
fetch mid360       https://github.com/ashduwihch/mid360_gazebo_harmonic.git rev "$PIN_MID360"

# ------------------------------------------------------------ 2. 打我们的补丁
say "2/5 应用我们自己的补丁（幂等：已打过就跳过）"
apply_patch() {  # apply_patch <repo-dir> <patch>
  local dir="$1" patch="$2"
  if git -C "$dir" apply --reverse --check "$patch" >/dev/null 2>&1; then
    echo "  已应用 $(basename "$patch")"
  elif git -C "$dir" apply --check "$patch" >/dev/null 2>&1; then
    git -C "$dir" apply "$patch"
    echo "  已打   $(basename "$patch")"
  else
    echo "  ✗ 补丁打不上：$(basename "$patch")" >&2
    echo "    上游 revision 变了吗？pin 在脚本头部。" >&2
    exit 1
  fi
}
apply_patch "$UPSTREAM/rgp"    "$PATCH_DIR/0001-rmagine-gazebo-plugins-self-exclusion.patch"
apply_patch "$UPSTREAM/mid360" "$PATCH_DIR/0002-mid360-fortress-compat.patch"
apply_patch "$UPSTREAM/mid360" "$PATCH_DIR/0003-mid360-ros-contract.patch"
apply_patch "$UPSTREAM/mid360" "$PATCH_DIR/0004-mid360-custommsg-tag.patch"

# 只把「上游③ 的 MID-360 包」和另外两个链接进工作区；上游③ 自带的 rmagine /
# rmagine_gazebo_plugins 副本**不用**（我们用钉住的独立上游 ①②，避免版本漂移）。
ln -sfn "$UPSTREAM/rmagine"                           "$SRC/rmagine"
ln -sfn "$UPSTREAM/rgp"                               "$SRC/rmagine_gazebo_plugins"
ln -sfn "$UPSTREAM/mid360/packages/mid360_simulation_plugin_ros2" "$SRC/mid360_simulation_plugin_ros2"
echo "  ws/src: $(ls -1 "$SRC" | tr '\n' ' ')"

if [ "$DO_BUILD" = 0 ]; then
  say "--no-build 指定：跳过编译。源码在 $SRC"
  exit 0
fi

# ----------------------------------------------------------------- 3. 编译
say "3/5 colcon build（build=$BUILD install=$INSTALL）"
set +u
# shellcheck disable=SC1091
. /opt/ros/humble/setup.bash
if [ -f "$REPO/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  . "$REPO/install/setup.bash"
fi
set -u

if [ "$FORCE" = 1 ]; then rm -rf "$BUILD" "$INSTALL"; fi
cd "$GZSIM_WS"
colcon build \
  --base-paths "$SRC" \
  --build-base "$BUILD" \
  --install-base "$INSTALL" \
  --packages-select rmagine rmagine_gazebo_plugins mid360_simulation_plugin_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_cohesion+

# ----------------------------------------------------------------- 4. 验收自检
say "4/5 产物自检"
for so in "$INSTALL/rmagine_gazebo_plugins/lib/rmagine_gazebo_plugins/librmagine_embree_map_system.so" \
          "$INSTALL/mid360_simulation_plugin_ros2/lib/libmid360_system.so"; do
  if [ -f "$so" ]; then
    echo "  ok   $(basename "$so")  ($(du -h "$so" | cut -f1))"
  else
    echo "  ✗ 缺 $so" >&2; exit 1
  fi
done
# ⚠️ 这里刻意**不写** `cmd | grep -q`：`grep -q` 一命中就关管道，写端吃 SIGPIPE，
#    在 `set -o pipefail` 下整条管道被判成失败 —— 明明有符号也会报「没有」（实测踩过，
#    退出码 141）。用 bash 的 case 做子串匹配，彻底没有管道。
LDD_OUT=$(ldd "$INSTALL/mid360_simulation_plugin_ros2/lib/libmid360_system.so")
case "$LDD_OUT" in
  *libignition-gazebo6*) echo "  ok   链到 ignition-gazebo6（不是 gz-sim8）" ;;
  *) echo "  ✗ 没有链到 ignition-gazebo6 —— 编出的是 Harmonic 分支？" >&2; exit 1 ;;
esac
NM_OUT=$(nm -D --defined-only \
  "$INSTALL/mid360_simulation_plugin_ros2/lib/libmid360_system.so")
case "$NM_OUT" in
  *IgnitionPluginHook*) echo "  ok   导出 IgnitionPluginHook（Fortress 注册宏生效）" ;;
  *) echo "  ✗ 没导出 IgnitionPluginHook —— GZ_ADD_PLUGIN 没换成 IGNITION_ADD_PLUGIN" >&2
     exit 1 ;;
esac

say "5/5 完成"
cat <<EOF
install 前缀 = $INSTALL

在裸 shell 里跑 ign gazebo 前先 source：
  source $INSTALL/setup.bash

在本仓的 opt-in 后端里跑（launch 会自动找到这个前缀，也接受 GZSIM_PREFIX 覆盖）：
  ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz \\
      world:=RMUC2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 robot11_mount:=sensor

下一层文档：docs/fortress_sim_backend.md
EOF
