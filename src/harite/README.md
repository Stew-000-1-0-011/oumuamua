# harite

ロボマス ID 3 / C610 による、吸引なしの昇降機構。
`harite_node` は **robomas_plugins** の `RobomasFrame` と `RobomasTarget` を使う。
`RobomasFrame.mode = 1` でボードの速度 PI 制御を有効にし、`robomas_bridge` 経由で指令を送る。
速度制御ループや CAN プロトコルはここでは実装しない。

```text
/harite/target_velocity (Float64、ロータ rad/s)
  -> harite_node -> /robomas_target3 (RobomasTarget)
                -> /robomas_frame (RobomasFrame、motor=2、c620=false、mode=1)
  -> robomas_bridge (robomas_plugins) -> USB-CAN ボード -> ロボマス ID 3
```

位置制御・高さプリセット・原点合わせ・吸引は持たない。行程や伝達機構の寸法は不要。
入力は減速前のロータ角速度 [rad/s] で、m/s や rpm ではない。
ボードの [MotorCtrl.cpp](https://github.com/YUKICHI6105/robomas_center/blob/main/Core/Src/MotorCtrl.cpp)
は ESC の rpm を rad/s に変換し、速度目標と比較する。正負と実際の昇降方向の対応は取付で決まる。

## 起動と操作

設定は `oumuamua_bringup/config/harite.yaml` にまとめてある。ノードのパラメータに既定値はない。
`bringup.launch.py` は実機で既定で harite を起動する。`harite:=false` なら起動しない。
`sim:=true` では harite を起動せず、昇降のシミュレーションもしない。
ID 1 / 2 の gripper、task_commander、スマホコントローラからは独立している。

```bash
colcon build --packages-up-to oumuamua_bringup
source install/setup.bash
ros2 launch oumuamua_bringup bringup.launch.py

# 継続して速度を送る。Ctrl-C で送信を止めると、既定 0.2 s で速度 0 になる
ros2 topic pub --rate 20 /harite/target_velocity std_msgs/msg/Float64 '{data: 10.0}'
# 逆向き
ros2 topic pub --rate 20 /harite/target_velocity std_msgs/msg/Float64 '{data: -10.0}'

# 送信が続いていても止める (ボードの速度制御を無効にする)
ros2 service call /harite/enable std_srvs/srv/SetBool '{data: false}'
# 再開。有効化後に届く新しい速度指令から動く
ros2 service call /harite/enable std_srvs/srv/SetBool '{data: true}'

# ボードへ送った目標を確認する (実測速度ではない)
ros2 topic echo /harite/command
```

harite 単体を既存の robomas_bridge に接続する場合:

```bash
ros2 run harite harite_node --ros-args \
  --params-file src/oumuamua_bringup/config/harite.yaml
```

| インターフェース | 型 | 内容 |
| --- | --- | --- |
| `~/target_velocity` | `std_msgs/msg/Float64` | ロータ角速度 [rad/s]。継続して送る |
| `~/enable` | `std_srvs/srv/SetBool` | 速度制御の有効・無効 |
| `~/command` | `std_msgs/msg/Float64` | 実際に送信した目標 [rad/s] |

- 起動直後は速度 0。`max_velocity` の絶対値上限 (既定 100 rad/s) で指令を切り詰める。
- 指令が `command_timeout` (既定 0.2 s) 途切れたら、次の制御周期で速度 0 を送る。
  判定は単調時計なので ROS 時刻が止まっても動く。NaN / Inf も指令を消して速度 0 にする。
- 無効中の指令は捨て、有効化後は新しい指令を待つ。速度 0 と制御無効は異なり、速度 0 はボードの速度制御を維持する。
- ボードが接続する前に設定を送ると捨てられるため、起動・有効状態変更時に `config_repeat` 回まで再送する。
  再送終了後にボードを再接続した場合は、harite を再起動するか `~/enable false` → `true` で設定を送り直す。
- `scripts/stop.sh` は harite も無効にする。終了時には速度 0 と制御無効を送るが、通信断・強制終了時の停止は保証しない。
- 上下限の検出や位置保持は行わない。行程の終端では操作側で止める。

## テスト

ROS 非依存のテストで、タイムアウト、正負の速度制限、無効化・再有効化、非有限値を確認する。

```bash
c++ -std=c++17 -Wall -Wextra -Wpedantic -Isrc/harite/include \
  src/harite/test/velocity_test.cpp -o /tmp/harite_velocity_test
/tmp/harite_velocity_test
```

ROS 環境では `colcon test --packages-select harite` でも同じテストを実行できる。
