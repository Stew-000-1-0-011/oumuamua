# gripper

把持機構 (昇降 + 吸引) 1 つぶんを動かすノード `gripper_node`。機構の数だけ別の名前で立てる
(oumuamua_bringup は `gripper0`, `gripper1`)。

- 昇降: ロボマス (C610 / C620) を、[robomas_plugins](https://github.com/Stew-000-1-0-011/robomas_plugins)
  の USB-CAN ボードの位置制御 (`RobomasFrame.mode = 2`) で動かす。高さは起動時の位置を 0 とする
- 吸引: mini-shirasu を動かす。制御そのものは mini_shirasu_node が行い (`mode: current` なら電流制御、`mode: duty` ならデューティ)、
  このノードはオンなら `suction.target`、オフなら 0 を `suction.target_topic` (その mode の `~/target_current` / `~/target_duty`) に送り続ける

**実機ではまだ動かしていない。** ボードの位置目標の単位、モータの ID、吸引の電流などは仮の値。

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/gripper/lift.hpp`, `src/lift.cpp` | ロボマスのフィードバックの読み取り、ロータ角の数え上げ、位置指令の速度制限。ROS非依存 |
| `src/gripper_node.cpp` | ノード本体 |
| `srv/SetLiftPosition.srv` | 決めておいた高さへ番号で動かすサービス |
| `action/MoveLift.action` | 高さを直接指定して動かすアクション (調整用) |
| `msg/GripperState.msg` | 状態 |
| `test/lift_test.cpp` | `lift.cpp` のテスト (ROS不要) |


**パラメータに既定値は無い。** 全部パラメータファイルで与える (足りなければ起動時にエラーで止まる)。この機体の値と各項目の説明は oumuamua の `src/oumuamua_bringup/config/gripper.yaml` にある。

## インターフェース

| 方向 | 名前 | 型 |
| --- | --- | --- |
| srv | `~/set_lift_position` | `gripper/srv/SetLiftPosition` (`lift.positions` の何番目の高さへ動かすか。動き出したらすぐ返る) |
| action | `~/move_lift` | `gripper/action/MoveLift` (ゴール: 高さ [m] を直接。調整用) |
| srv | `~/set_suction` | `std_srvs/srv/SetBool` (true でオン、false でオフ) |
| srv | `~/enable_lift` | `std_srvs/srv/SetBool` (true で位置制御、false で無効) |
| srv | `~/zero_lift` | `std_srvs/srv/Trigger` (今の位置を高さ 0 にし直す。動いている途中なら止め、アクションのゴールは失敗で終わる) |
| pub | `~/state` | `gripper/msg/GripperState` |
| pub | `robomas_frame_topic` (例 `/robomas_frame`) | `robomas_plugins/msg/RobomasFrame` (起動時・有効/無効の切り替え時・`~/zero_lift`) |
| pub | `robomas_target_topic` (例 `/robomas_target<lift.motor>`) | `robomas_plugins/msg/RobomasTarget` (`control_rate` で) |
| pub | `suction.target_topic` | `std_msgs/msg/Float64` [A] (`control_rate` で) |
| sub | `can_rx_topic` (例 `/robomas_can_rx`) | `robomas_plugins/msg/Frame` (ロボマスのフィードバック 0x200 + k) |

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
  届かなければ状態表示には指令値を使う (`GripperState.lift_feedback` が false)。
  電流保護には実測が必要なので、フィードバックが無い間の新しい移動要求は拒否する
- ボードがつながる前に設定を送ると捨てられるので、起動直後は 1 s ごとに `lift.config_repeat` 回まで送り直す
- ノードが終わるときは、昇降を無効にし吸引を 0 A にする (届くとは限らない)。
  落ちたときは、mini_shirasu_node が目標の途切れ (`target_timeout`) で吸引を 0 A にする

## 電流による昇降端の保護

左右の昇降 (`gripper0` / `gripper1`) では、CAN の実測電流の絶対値が
`lift.overcurrent_threshold` [A] 以上のまま `lift.overcurrent_duration` [s] 続くと停止する。
設定は `oumuamua_bringup/config/gripper.yaml` の共通 `lift` 以下で指定し、各 `gripper<i>` で上書きできる。
暫定値は **8.0 A / 0.2 s**。この値は実機で校正済みではない。通常負荷の電流・起動時ピークと
ESC の保護停止までの時間を確認して設定する。左右で異なる値にもできる。

```yaml
gripper0:
  ros__parameters:
    lift:
      overcurrent_threshold: 8.0
      overcurrent_duration: 0.2
```

- 向きは、送信中の位置指令と実測高さの差から決める。左右のモータの回転方向の違いも反映する。
  上段の目標が機構の外にあってランプ指令が既に到達済みでも、実測位置との差があれば監視を続ける。
  位置が一致していても高電流が続く場合は、電流の向きから押している側を判定する。
- 検出時は実測高さを上限または下限として記録し、進行中のアクションを失敗 (`reached=false`) で終了する。
  `lift_arrived` も false となる。サービスで開始した移動の停止は `~/state` で確認する。
- 到達できない位置指令を破棄し、実測位置で保持し直す。ボードに位置モードを再送して PI の積分値を消す。
  同時に変わるボードの原点は内部で補正するため、下段・中段などの高さ指定は引き続き同じ座標で使える。
- 検出した限界以上・以下の要求は、位置番号サービスと高さ指定アクションの両方で拒否する。
  制限の内側への退避は許可する。電流低下・逆方向への移動・enable の切り替えでは限界を解除しない。
- `~/zero_lift` では、検出済みの限界も新しい原点へ座標変換する。ノードを再起動すると記録は消える。
- 電流がしきい値未満に戻る、指令方向が変わる、またはフィードバックが
  `lift.feedback_timeout` より長く途切れると、継続時間の計測をやり直す。時間は単調時計で測る。
- `~/state` の `lift_current` [A]、`lift_upper_limited` / `lift_lower_limited`、
  `lift_upper_limit` / `lift_lower_limit` [m] で状態を確認できる。
  `lift_feedback=false` のとき電流表示は古い値であり、電流保護は判定できない。
- 初回の通信確立後、フィードバックが古い間は位置モードの再設定・再有効化・零点設定を拒否または延期する。
  原点変更前の位置指令は、時刻を使ってブリッジ側で破棄する。

同梱の `robomas_plugins` は位置モード設定前にボードのフィードバック転送を有効にする。
サブモジュールも `git submodule update --init --recursive` でこのブランチの指定版へ更新すること。

これは ROS 経由のフィードバックによる保護で、実機 ESC の停止を必ず防げる保証ではない。
停止・保持・逆方向への退避は実機で確認すること。CI では短いピーク、継続時間、通信断、再指令拒否、
両側の制限、左右の逆回転、アクションの失敗通知、停止時のボード指令と退避を疑似フィードバックで検証する。

## 未確認のこと

- ボードの位置制御の目標の単位と、その基準 (ボードの起動時か、モータの電源投入時か)。`lift.target_scale` と `lift.target_offset` で合わせる
- ボードがロボマスのフィードバック (0x201..) を `robomas_can_rx` に流すか
