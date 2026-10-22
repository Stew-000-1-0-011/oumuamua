# task_commander

機体に作業を 1 つずつ指示するノード `task_commander_node`。アクション `~/run` (`task_commander/action/RunTask`) で作業を受け、
走行は path_follower の `~/follow`、昇降と吸引は gripper_node (`gripper<i>`) の `~/set_lift_position` / `~/set_suction` に任せる。
どの作業をどの順でさせるか (試合の段取り) は、このノードの外 (人の操作や、上に置く別のノード) が決める。

| 番号 | 作業 | 引数 | すること |
| --- | --- | --- | --- |
| 0 | `PICK` | `gripper`, `block` | 昇降を clear に上げ、ブロックの真上の手前 (`pick.standoff`) へ動き、真上へ寄る。昇降を pick に下ろして吸引をオン、carry に上げて手前へ戻る |
| 1 | `TO_SLALOM_ENTRANCE` | | 昇降を全部 carry にして、スラロームの入口へ (今の位置から直線) |
| 2 | `TO_START` | | 同じく、スタート地点へ |
| 3 | `SLALOM_FORWARD` | | 昇降を全部 carry にして、経路 `slalom_forward` (入口 -> 出口) を走る |
| 4 | `SLALOM_BACKWARD` | | 同じく、経路 `slalom_backward` (出口 -> 入口) を走る |
| 5 | `PLACE` | `gripper`, `column`, `row` | 昇降をその段の approach に上げ、マスの手前 (`place.standoff`) からマスへ差し込む。release に下ろして吸引をオフ、approach に戻して手前へ下がり、carry に下ろす |
| 6 | `TO_SLALOM_EXIT` | | 昇降を全部 carry にして、スラロームの出口へ (今の位置から直線) |

```bash
ros2 action send_goal /task_commander/run task_commander/action/RunTask "{type: 0, gripper: 0, block: 2}" --feedback   # 把持 0 でブロック 2 を回収
ros2 action send_goal /task_commander/run task_commander/action/RunTask "{type: 1}"                                   # スラロームの入口へ
ros2 action send_goal /task_commander/run task_commander/action/RunTask "{type: 3}"                                   # スラロームの往路
ros2 action send_goal /task_commander/run task_commander/action/RunTask "{type: 5, gripper: 0, column: 1, row: 0}"    # 中央の列の 1 段目へ
ros2 topic echo /task_commander/state    # 持っているか・回収したブロック・置いたマス
```

- 作業中に次のゴールが来たら断る。止めるときはゴールを取り消す (走行のゴールも取り消し、昇降はその場で止まる)
- **動く前の確認**: 直線で動く前に、機体 (半径 `robot_radius` の円) が壁・棚・まだ回収していないブロックから
  `min_clearance` 以上離れているかを確かめ、近づくなら動かずに失敗を返す (どこで近づくかをメッセージに出す)。
  把持機構の張り出しは円に入れていない。スラロームの往復は、経路を作るときに外形で確かめてある
- **覚えていること**: どの把持機構がブロックを持っているか、どのブロックを回収したか、どのマスに置いたか。
  持っていない把持機構での PLACE、回収済みのブロックの PICK、置いたマスへの PLACE は断る。
  リトライなどで実際と合わなくなったら `~/reset` (`std_srvs/srv/Trigger`) で忘れさせる
- **2 段目より上** (ルール 2.5.2): 同じ列の下の段に自分で置いていなければ警告を出す (相手のノーツでもよいので止めはしない)

## パラメータ

`oumuamua_bringup/config/task_commander.yaml` にある。場所はフィールド座標系 `[x, y, yaw]`。

- 把持機構の取付位置 (`grippers.offsets`、機体座標系) と、回収・配置のときの機体の向き (`pick.yaw`, `place.yaw`) から、
  吸着面の中心がブロックやマスの中心に来る機体の姿勢を計算する
- 昇降の高さは gripper.yaml の `lift.positions` の番号で指す (`lift.pick`, `lift.clear`, `lift.carry`, `lift.row_approach`, `lift.row_release`)
- スラロームの入口・出口は `config/paths/slalom.route.yaml` の始点・終点と合わせる

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/task_commander/geometry.hpp`, `src/geometry.cpp` | 道具を目標に合わせる機体の姿勢、機体と障害物の間隔。ROS非依存 |
| `src/task_commander_node.cpp` | ノード本体。作業ごとにスレッドを立てて順に進める |
| `action/RunTask.action` | 作業 |
| `test/geometry_test.cpp` | `geometry.cpp` のテスト (ROS不要) |
