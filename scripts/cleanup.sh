#!/usr/bin/env bash
# 終わり損ねたノードを止める。launch が途中で落ちたときなど、ノードだけが残って
# LiDAR のポートや同じ名前のノードがぶつかるときに使う。
#
#   scripts/cleanup.sh            残っているノードを一覧にして止める (SIGINT、残れば SIGKILL)
#   scripts/cleanup.sh --dry-run  一覧を出すだけで止めない
#
# 止めるのは次のプロセス:
# - このワークスペースの install/<パッケージ>/lib/ から起動したもの (各ノード)
# - bringup の laser_static_tf (static_transform_publisher) と robomas_container (component_container)
# - ros2 launch oumuamua_bringup ...
# ロボットを止める (速度 0) のは scripts/stop.sh。こちらはノードを終わらせるだけ。
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set -u

dry_run=false
case "${1:-}" in
  "") ;;
  --dry-run) dry_run=true ;;
  *) echo "usage: $0 [--dry-run]" >&2; exit 2 ;;
esac

# コマンド行の先頭 (Python のノードなら python3 の次) が実行ファイルのものだけを拾う。
# 引数に同じ文字列を含むだけのプロセス (エディタ、シェルなど) は拾わない
py='^([^ ]*/)?python3[^ ]* '
patterns=(
  "^($py)?$ws/install/[^ ]+/lib/"
  "^[^ ]*/static_transform_publisher .*__node:=laser_static_tf( |$)"
  "^[^ ]*/component_container .*__node:=robomas_container( |$)"
  "${py}[^ ]*/ros2 launch oumuamua_bringup "
)

# 自分と、自分を起動したプロセス (シェルなど) は止めない
self=$$
ancestors=" $self "
p=$self
while [ "$p" -gt 1 ] 2>/dev/null; do
  p=$(ps -o ppid= -p "$p" | tr -d ' ')
  [ -n "$p" ] || break
  ancestors+="$p "
done

find_pids() {
  for pat in "${patterns[@]}"; do
    pgrep -f -- "$pat"
  done | sort -un | while read -r pid; do
    [[ "$ancestors" == *" $pid "* ]] || echo "$pid"
  done
}

pids=$(find_pids)
if [ -z "$pids" ]; then
  echo "残っているノードはありません"
  exit 0
fi

echo "残っているプロセス:"
# shellcheck disable=SC2086
ps -o pid=,args= -p $(echo $pids | tr ' ' ',') | sed "s#$ws/install/##g" | cut -c1-150
if $dry_run; then
  exit 0
fi

# shellcheck disable=SC2086
kill -INT $pids 2>/dev/null
# 5 s まで待つ
for _ in $(seq 50); do
  left=$(find_pids)
  [ -z "$left" ] && break
  sleep 0.1
done

left=$(find_pids)
if [ -n "$left" ]; then
  echo "SIGINT で終わらなかったので SIGKILL: $(echo $left)"
  # shellcheck disable=SC2086
  kill -KILL $left 2>/dev/null
  sleep 0.5
fi

left=$(find_pids)
if [ -n "$left" ]; then
  echo "まだ残っています: $(echo $left)" >&2
  exit 1
fi
echo "すべて止めました"
