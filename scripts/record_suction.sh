#!/usr/bin/env bash
# 吸引の状態 (電流・電源電圧・飽和・目標電流・オン/オフ) を CSV に記録する。受信だけで何も送らない。
# bringup.launch.py を起動したまま、別の端末で実行する。
#
#   scripts/record_suction.sh                 Ctrl+C まで記録する
#   scripts/record_suction.sh --duration 600  600 s 記録する
#   scripts/record_suction.sh -o out.csv      出力先を決める (既定は ~/.ros/log/suction_<日時>.csv)
#
# 終わると、目標電流が 0 でなかった間の電流 (平均・ばらつき・飽和) を左右ごとに出す。
# Status は 1 回ごとの測定値なので、1 つずつではなく平均で見ること。
#
# ネットワークを切り替えると、切り替える前に起動した bringup とは DDS がつながらなくなる。
# 切り替えるなら、bringup とこのスクリプトを両方 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST で起動する。
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash
fi
# shellcheck disable=SC1091
source "$ws/install/setup.bash"
set -u

output="$HOME/.ros/log/suction_$(date +%Y%m%d-%H%M%S).csv"
args=()
while [ $# -gt 0 ]; do
  case "$1" in
    -o) output="$2"; shift 2 ;;
    *) args+=("$1"); shift ;;
  esac
done
mkdir -p "$(dirname "$output")"

exec python3 "$ws/scripts/record_suction.py" "$output" "${args[@]}"
