# gripper

把持機構 (昇降 + 吸引) 1 つぶんを動かすノード `gripper_node`。機構の数だけ別の名前で立てる
(oumuamua_bringup は `gripper0`, `gripper1`)。

- 昇降: ロボマス (C610 / C620) を、[robomas_plugins](https://github.com/Stew-000-1-0-011/robomas_plugins)
  の USB-CAN ボードの位置制御 (`RobomasFrame.mode = 2`) で動かす。高さは起動時の位置を 0 とする
- 吸引: mini-shirasu を電流制御で動かす。電流の制御そのものは mini_shirasu_node (`mode: current`) が行い、
  このノードはオンなら `suction.current` [A]、オフなら 0 A を送り続ける

**実機ではまだ動かしていない。** ボードの位置目標の単位、モータの ID、吸引の電流などは仮の値。

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/gripper/lift.hpp`, `src/lift.cpp` | ロボマスのフィードバックの読み取り、ロータ角の数え上げ、位置指令の速度制限。ROS非依存 |
| `src/gripper_node.cpp` | ノード本体 |
| `srv/SetLiftPosition.srv` | 決めておいた高さへ番号で動かすサービス |
| `action/MoveLift.action` | 高さを直接指定して動かすアクション (調整用) |
| `msg/GripperState.msg` | 状態 |
| `config/gripper_node.yaml` | パラメータ (1 つぶん。意味はここのコメント) |
| `test/lift_test.cpp` | `lift.cpp` のテスト (ROS不要) |

## インターフェース

| 方向 | 名前 | 型 |
| --- | --- | --- |
| srv | `~/set_lift_position` | `gripper/srv/SetLiftPosition` (`lift.positions` の何番目の高さへ動かすか。動き出したらすぐ返る) |
| action | `~/move_lift` | `gripper/action/MoveLift` (ゴール: 高さ [m] を直接。調整用) |
| srv | `~/set_suction` | `std_srvs/srv/SetBool` (true でオン、false でオフ) |
| srv | `~/enable_lift` | `std_srvs/srv/SetBool` (true で位置制御、false で無効) |
| pub | `~/state` | `gripper/msg/GripperState` |
| pub | `robomas_frame_topic` (既定 `/robomas_frame`) | `robomas_plugins/msg/RobomasFrame` (起動時・有効/無効の切り替え時) |
| pub | `robomas_target_topic` (既定 `/robomas_target<lift.motor>`) | `robomas_plugins/msg/RobomasTarget` (`control_rate` で) |
| pub | `suction.target_topic` | `std_msgs/msg/Float64` [A] (`control_rate` で) |
| sub | `can_rx_topic` (既定 `/robomas_can_rx`) | `robomas_plugins/msg/Frame` (ロボマスのフィードバック 0x200 + k) |

使う高さ (下・中段・上段など) は `lift.positions` に並べておき (oumuamua_bringup では `config/gripper.yaml`)、
`~/set_lift_position` で番号を送る。着いたかは `~/state` の `lift_arrived` (と `lift_position`) で見る。

```bash
ros2 service call /gripper0/set_lift_position gripper/srv/SetLiftPosition "{index: 2}"   # 上段へ
ros2 topic echo /gripper0/state --field lift_arrived
ros2 action send_goal /gripper0/move_lift gripper/action/MoveLift "{height: 0.10}" --feedback
ros2 service call /gripper0/set_suction std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /gripper0/state
```

## 昇降の動き

- 位置の目標は、ゴールへ向けて `lift.max_speed` [m/s] で動かしながら送る (急に飛ばさない)。
  ボードに送るのは `lift.target_scale * 高さ + lift.target_offset`
- アクションのゴールは `lift.min_height..lift.max_height` に丸める (`lift.positions` は範囲外なら起動時にエラー)。
  動いている途中で新しいゴール (サービスでもアクションでも) が来たら、そちらへ向かい直す。アクションのゴールは失敗 (abort) で終わる。
  キャンセルされたら、その時点の指令で止まる
- 成功の条件: 指令がゴールに着き、`lift.settle_time` だけ経ったとき。フィードバックがあれば、実際の高さもゴールから `lift.tolerance` 以内であること。
  `距離 / max_speed + lift.timeout_margin` 以内に着かなければ失敗
- フィードバック: ロボマスの ESC が出す 0x200 + k が `can_rx_topic` に届き、`lift.meters_per_output_revolution` が 0 でなければ、
  ロータの角度を数え上げて今の高さとする (`ロータの回転 / gear_ratio * meters_per_output_revolution`)。
  届かなければ、送っている指令を今の高さとみなす (`GripperState.lift_feedback` が false)
- ボードがつながる前に設定を送ると捨てられるので、起動直後は 1 s ごとに `lift.config_repeat` 回まで送り直す
- ノードが終わるときは、昇降を無効にし吸引を 0 A にする (届くとは限らない)。
  落ちたときは、mini_shirasu_node が目標の途切れ (`target_timeout`) で吸引を 0 A にする

## 未確認のこと

- ボードの位置制御の目標の単位と、その基準 (ボードの起動時か、モータの電源投入時か)。`lift.target_scale` と `lift.target_offset` で合わせる
- ボードがロボマスのフィードバック (0x201..) を `robomas_can_rx` に流すか
