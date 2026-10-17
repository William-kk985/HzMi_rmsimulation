#!/usr/bin/env bash
# =============================================================================
# tools/scripts/lib/scoped_cleanup.sh —— 「只杀本 run 自己的进程」的收尾工具
# -----------------------------------------------------------------------------
# 背景（为什么需要它）：本仓旧的 bench/ab 脚本收尾写的是
#     pkill -9 -x {gzserver,gzclient}      # 全机杀，不看 GAZEBO_MASTER_URI
#     pkill -9 -f 'ros2 launch'            # 会把别人的 launch 一起杀
# 这会误伤**同机别人正在跑/正在看的**栈（历史事故见 docs/gazebo_gui_troubleshooting.md §4.2、
# 以及 docs/machine_hygiene_runbook.md）。另一个"看着像保护、其实永远匹配不到"的写法是
#     pkill -f "gzserver.*$GZ_PORT" / pkill -f "gzclient.*$GZ_PORT"
# —— `gzserver <world>` / `gzclient --gui-client-plugin=...` 的 **cmdline 里没有端口**
# （URI 在环境变量里）⇒ 这两条是空操作。
#
# 本文件提供的判定：一个 PID 算「本 run 的」当且仅当下面**任一**成立
#   ① 它在本次运行的进程组里：scripts 用 `setsid ... &` 起栈 ⇒ PGID=$LPID（最优先、无需名字匹配）
#   ② 它的 /proc/<pid>/environ 里 `GAZEBO_MASTER_URI` == 本 run 的 $GAZEBO_MASTER_URI
#   ③ 它是本脚本（$$）、或本 run 的 launch（$LPID/$LAUNCH_PID）的后代
# 并且**永不**杀 $$ 及其祖先进程（防止 `bash -c "... pkill ..."` 那种自伤）。
# 名字/命令行只用来"挑候选"，**绝不单独作为杀的理由**。
#
# 用法（脚本里，`WS` 定义之后）：
#   . "$WS/tools/scripts/lib/scoped_cleanup.sh"
#   ...
#   cleanup() {
#     kill -INT -- "-$PGID" 2>/dev/null                    # 先整组 SIGINT（正规收尾）
#     sc_kill_own_ros2_launch 9                            # 只杀本 run 的 ros2 launch
#     sc_kill_own_gz 9                                     # 只杀本 run 的 gzserver/gzclient
#     sc_kill_own_pattern 9 'slam_toolbox|small_point_lio' # 只杀本 run 的节点
#   }
#   trap cleanup EXIT
#
# 可选环境变量：
#   SC_URI             覆盖 URI 比对值（默认取 $GAZEBO_MASTER_URI）
#   SC_PGID            覆盖进程组（默认取 $PGID）
#   SC_SCOPE_VERBOSE=1 把"跳过/杀掉"的判定打到 stderr（排障用）
# =============================================================================

# 本文件被 source 使用；不改变调用方的 set -e/-u/pipefail 行为。

_sc_effective_uri()  { printf '%s' "${SC_URI:-${GAZEBO_MASTER_URI:-}}"; }
_sc_effective_pgid() { printf '%s' "${SC_PGID:-${PGID:-}}"; }

_sc_dbg() { [ "${SC_SCOPE_VERBOSE:-0}" = "1" ] && printf '[scope] %s\n' "$*" >&2; return 0; }

# /proc/<pid>/status 的 PPid（对 comm 里带空格/括号的进程也稳，不用解析 stat）
_sc_ppid_of() { awk '/^PPid:/{print $2; exit}' "/proc/$1/status" 2>/dev/null; }

# $1 的祖先链里是否出现 $2（含 $1 自身 == $2）
_sc_chain_contains() {
  local p="$1" want="$2" n=0
  case "$want" in ''|*[!0-9]*) return 1;; esac
  while :; do
    case "$p" in ''|*[!0-9]*) return 1;; esac
    [ "$p" -gt 1 ] || return 1
    [ "$p" = "$want" ] && return 0
    p=$(_sc_ppid_of "$p")
    n=$((n + 1))
    [ "$n" -gt 96 ] && return 1
  done
}

# 自伤保护：$$、$BASHPID、以及 $$ 的所有祖先都不许杀
_sc_pid_is_self_or_ancestor() {
  local pid="$1"
  [ "$pid" = "$$" ] && return 0
  [ -n "${BASHPID:-}" ] && [ "$pid" = "${BASHPID}" ] && return 0
  _sc_chain_contains "$$" "$pid" && return 0
  return 1
}

_sc_pgid_of() { ps -o pgid= -p "$1" 2>/dev/null | tr -d ' '; }

# /proc/<pid>/environ 里的 GAZEBO_MASTER_URI（读不到/没有则非 0）
_sc_uri_of_pid() {
  local pid="$1" kv
  [ -r "/proc/$pid/environ" ] || return 1
  while IFS= read -r -d '' kv; do
    case "$kv" in
      GAZEBO_MASTER_URI=*) printf '%s' "${kv#GAZEBO_MASTER_URI=}"; return 0;;
    esac
  done < "/proc/$pid/environ" 2>/dev/null
  return 1
}

# ★ 核心判据：这个 PID 是不是「本 run 自己的」
sc_pid_owned() {
  local pid="$1" want pgid u root
  case "$pid" in ''|*[!0-9]*) return 1;; esac
  [ "$pid" -gt 1 ] || return 1
  _sc_pid_is_self_or_ancestor "$pid" && { _sc_dbg "skip $pid（本 shell/其祖先）"; return 1; }

  # ① 本 run 的进程组（setsid 起的 launch ⇒ PGID = launch PID）
  pgid=$(_sc_effective_pgid)
  if [ -n "$pgid" ] && [ "$(_sc_pgid_of "$pid")" = "$pgid" ]; then
    _sc_dbg "own $pid（进程组 $pgid）"; return 0
  fi

  # ② environ 里的 GAZEBO_MASTER_URI 与本 run 完全一致
  want=$(_sc_effective_uri)
  u=$(_sc_uri_of_pid "$pid")
  if [ -n "$u" ] && [ -n "$want" ] && [ "$u" = "$want" ]; then
    _sc_dbg "own $pid（master URI $u）"; return 0
  fi

  # ③ 本脚本（$$）/ 本 run launch（$LPID/$LAUNCH_PID）的后代
  for root in "$$" "${SC_ROOT_PID:-}" "${LPID:-}" "${LAUNCH_PID:-}"; do
    [ -n "$root" ] || continue
    if [ "$pid" = "$root" ] || _sc_chain_contains "$pid" "$root"; then
      _sc_dbg "own $pid（$root 的后代/自身）"; return 0
    fi
  done

  _sc_dbg "skip $pid（uri=${u:-<none>} want=${want:-<none>}）"
  return 1
}

# 命令行匹配 ERE 的全部 PID（**不过滤**归属；只是"挑候选"）
sc_pids_by_pattern() {
  local re="$1" p cmd
  for p in /proc/[0-9]*; do
    p=${p#/proc/}
    [ -r "/proc/$p/cmdline" ] || continue
    cmd=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null) || continue
    [ -n "$cmd" ] || continue
    [[ "$cmd" =~ $re ]] && printf '%s\n' "$p"
  done
  return 0
}

# 命令行匹配 ERE **且** 属于本 run 的 PID
sc_own_pids_by_pattern() {
  local p
  for p in $(sc_pids_by_pattern "$1"); do
    sc_pid_owned "$p" && printf '%s\n' "$p"
  done
  return 0
}

# 本 run 的 gzserver/gzclient（**名字只是候选**；comm 精确是 gzserver/gzclient，
# 然后必须过 sc_pid_owned —— 别人的 gazebo 在同名但 URI/进程组/血缘上都对不上）
sc_own_gz_pids() {
  local p comm
  for p in /proc/[0-9]*; do
    p=${p#/proc/}
    comm=$(cat "/proc/$p/comm" 2>/dev/null) || continue
    case "$comm" in
      gzserver|gzclient) sc_pid_owned "$p" && printf '%s\n' "$p";;
    esac
  done
  return 0
}

# ---- 动作 -------------------------------------------------------------------

# sc_kill_pids <SIG> <pid...>：只杀通过归属判定的 PID
sc_kill_pids() {
  local sig="$1" p n=0; shift
  for p in "$@"; do
    sc_pid_owned "$p" || continue
    kill -"$sig" "$p" 2>/dev/null && { n=$((n + 1)); _sc_dbg "kill -$sig $p"; }
  done
  _sc_dbg "killed=$n sig=$sig"
  return 0
}

# sc_kill_own_gz <SIG>：只清本 run 的 gzserver/gzclient（服务过 /proc/environ 或进程组）
sc_kill_own_gz() {
  local sig="${1:-9}" p n=0
  for p in $(sc_own_gz_pids); do
    kill -"$sig" "$p" 2>/dev/null && n=$((n + 1))
  done
  _sc_dbg "own gz killed=$n sig=$sig"
  return 0
}

# sc_kill_own_pattern <SIG> <ERE>：只清本 run 里命令行匹配的进程
sc_kill_own_pattern() {
  local sig="${1:-9}" re="$2" p n=0
  for p in $(sc_own_pids_by_pattern "$re"); do
    kill -"$sig" "$p" 2>/dev/null && n=$((n + 1))
  done
  _sc_dbg "own pattern killed=$n sig=$sig re=$re"
  return 0
}

# sc_kill_own_ros2_launch <SIG> [ERE]：只清本 run 的 ros2 launch（默认 'ros2 launch'）
sc_kill_own_ros2_launch() {
  sc_kill_own_pattern "${1:-9}" "${2:-ros2 launch}"
}

# 一行报告：本 run 还剩下的进程（pid:comm），干净时输出空串
sc_own_leftovers_desc() {
  local p out=""
  for p in $(sc_own_pids_by_pattern "${1:-gzserver|gzclient|ros2 launch}"); do
    out="$out $p:$(cat "/proc/$p/comm" 2>/dev/null)"
  done
  printf '%s' "${out# }"
}

# 计数：本 run 的 gzserver/gzclient 还有几个
sc_own_gz_count() {
  local n
  n=$(sc_own_gz_pids | wc -l)
  printf '%s' "$n"
}
