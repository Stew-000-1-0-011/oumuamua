#!/usr/bin/env bash
# ロボットを止める (速度 0)。bringup.launch.py を起動したまま、別の端末で実行する。
#
#   scripts/stop.sh          経路の目標を止め、追従と harite を切り、速度 0 の指令を流す
#   scripts/stop.sh --free   さらに各車輪の出力を切る (手で押せるようになる。走らせるには再起動か各車輪の ~/enable true)
#
# 追従 (tracker_node) は切ったままになる。再び走らせるときは
#   ros2 service call /tracker_node/enable std_srvs/srv/SetBool "{data: true}"
# か、bringup を起動し直す。
# harite も無効のままになる。再開: ros2 service call /harite/enable std_srvs/srv/SetBool "{data: true}"
# PC やノードが落ちているときは止められない。最後の手段は緊急停止スイッチ。
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash
fi
# shellcheck disable=SC1091
source "$ws/install/setup.bash"
set -u

exec python3 "$ws/scripts/stop.py" "$@"
