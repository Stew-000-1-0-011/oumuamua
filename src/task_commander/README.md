# task_commander

機体に作業を 1 つずつ指示するノード `task_commander_node`。アクション `~/run` (`task_commander/action/RunTask`) で作業を受け、
走行は path_follower の `~/follow`、昇降と吸引は gripper_node (`gripper<i>`) の `~/set_lift_position` / `~/set_suction` に任せる。
どの作業をどの順でさせるか (試合の段取り) は、このノードの外 (人の操作や、上に置く別のノード) が決める。

| 番号 | 作業 | 引数 | すること |
| --- | --- | --- | --- |
| 0 | `PICK` | `gripper`, `block` | 昇降を clear に上げて吸引をオンにし、経由点 (`pick.via`) で回収の向きに回ってから、回収の姿勢 (`pick.poses`) の手前 (`pick.standoff`) から寄る。昇降を pick に下ろしきって `pick.grip_time` 待ち、carry に上げて手前へ戻る。吸う前に失敗したら吸引を切る |
| 1 | `TO_SLALOM_ENTRANCE` | | 昇降を全部 carry にして、スラロームの入口へ (今の位置から直線) |
| 2 | `TO_START` | | 同じく、スタート地点へ |
| 3 | `SLALOM_FORWARD` | | 昇降を全部 carry にして、経路 `slalom_forward` (入口 -> 出口) を走る |
| 4 | `SLALOM_BACKWARD` | | 同じく、経路 `slalom_backward` (出口 -> 入口) を走る |
| 5 | `TO_SHELF_CELL` | `gripper`, `column` | 今の y のまま横へ動き、把持機構 `gripper` の吸ったノーツの x を列 `column` の中心に合わせて、棚の方 (`place.yaw`) を向く。前後 (棚へ差し込む)・昇降・吸引は変えない (手で)。`row` は使わない |
| 6 | `TO_SLALOM_EXIT` | | 昇降を全部 carry にして、スラロームの出口へ (今の位置から直線) |
| 7 | `TO_SHELF_FRONT` | | 昇降を全部 carry にして、棚の前へ。中央の列の正面で棚の方を向き、吸ったノーツの前の面 (機体から遠い面) が棚の中心面から `shelf.front_distance` (0.33 m) の所 |
| 8 | `PICK_PAIR` | `block` | gripper0 でブロック `block`、gripper1 でブロック `block - 1` を同時に回収する。姿勢は `pick.pairs` (今は 4 と 3、2 と 1)。手順は `PICK` と同じで、昇降と吸引は 2 つ同時に動かす |

```bash
ros2 action send_goal /task_commander/run task_commander/action/RunTask "{type: 0, gripper: 0, block: 2}" --feedback   # 把持 0 でブロック 2 を回収
ros2 action send_goal /task_commander/run task_commander/action/RunTask "{type: 1}"                                   # スラロームの入口へ
ros2 action send_goal /task_commander/run task_commander/action/RunTask "{type: 3}"                                   # スラロームの往路
ros2 action send_goal /task_commander/run task_commander/action/RunTask "{type: 7}"                                   # 棚の前へ
ros2 action send_goal /task_commander/run task_commander/action/RunTask "{type: 5, gripper: 0, column: 1}"            # 把持 0 のノーツを中央の列へ
ros2 topic echo /task_commander/state    # 持っているか・回収したブロック
```

- 作業中に次のゴールが来たら断る。止めるときはゴールを取り消す (走行のゴールも取り消し、昇降はその場で止まる)
- **動く前の確認**: 直線で動く前に、機体 (外形 `footprint` の多角形。向きも考える) が壁・棚・まだ回収していない
  ブロックから `min_clearance` 以上離れているかを確かめ、近づくなら動かずに失敗を返す (どこで近づくかをメッセージに出す)。
  持ち上げて上を通る把持機構は外形に入れていない。スラロームの往復は、経路を作るときに外形で確かめてある
- **回収の位置**: 回収するときの機体の中心の姿勢を、ブロックごと・把持機構ごとに `pick.poses.block<i>` で決める
  (`[x, y, yaw]` を gripper0, gripper1 の順に)。吸引は寄る前に回しておき、そこで昇降を下ろして `pick.grip_time` 待って持ち上げる
- **覚えていること**: どの把持機構がブロックを持っているか、どのブロックを回収したか。
  持っている把持機構での PICK、回収済みのブロックの PICK は断る。持っている把持機構の吸引が切れたら (手で置いたら)、
  持っていないことにする。リトライなどで実際と合わなくなったら `~/reset` (`std_srvs/srv/Trigger`) で忘れさせる

## パラメータ

`oumuamua_bringup/config/task_commander.yaml` にある。場所はフィールド座標系 `[x, y, yaw]`。

- 吸着面の中心の位置 (`grippers.suction_offsets`、機体座標系) から、吸ったノーツの中心 (ノーツの大きさの半分だけ前) を求め、
  棚では、それの x が列の中心に来る機体の位置 (向きは `place.yaw`) を計算する
- 昇降の高さは gripper.yaml の `lift.positions` の番号で指す (`lift.pick`, `lift.clear`, `lift.carry`)
- スラロームの入口・出口は `config/paths/slalom.route.yaml` の始点・終点と合わせる

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/task_commander/geometry.hpp`, `src/geometry.cpp` | 道具を目標に合わせる機体の姿勢、機体と障害物の間隔。ROS非依存 |
| `src/task_commander_node.cpp` | ノード本体。作業ごとにスレッドを立てて順に進める |
| `action/RunTask.action` | 作業 |
| `test/geometry_test.cpp` | `geometry.cpp` のテスト (ROS不要) |
