# task_commander

機体に作業を 1 つずつ指示するノード `task_commander_node`。アクション `~/run` (`task_commander/action/RunTask`) で作業を受け、
走行は path_follower の `~/follow`、昇降と吸引は gripper_node (`gripper<i>`) の `~/set_lift_position` / `~/set_suction` に任せる。
どの作業をどの順でさせるか (試合の段取り) は、このノードの外 (人の操作や、上に置く別のノード) が決める。

| 番号 | 作業 | 引数 | すること |
| --- | --- | --- | --- |
| 0 | `PICK` | `gripper`, `block` | 実形状で計画後、全昇降carryで手前へ移動。選択側clearで接近し、pick・吸引・carry到達確認の後に退避 |
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

## 5箱の回収設定

`oumuamua_bringup/config/field.yaml` の既存キーに、箱の初期中心x座標を5個 [m] 指定する。

```yaml
notes_x: [-2.699, -2.499, -2.299, -2.099, -1.899] # 暫定例。実測値へ置換する
notes_y: 0.125
note_size: 0.15
```

配列順が箱番号0〜4で、内部では並べ替えない。個数自体は設定依存で、コードに5を固定していない。
空配列、非有限値、重複・接触/重なり、壁やフィールド外にある箱は拒否する。
robot側とcontroller bridgeに同じfieldファイルを読ませ、変更後は再ビルド・関連ノードを再起動する。
controllerは `notes_x` の個数から5個の回収ボタンを作り、保持/回収状態をこのノードから受け取るため、通信方式や入力画面の追加は不要。

PICKでは、共有の `footprints/raised.yaml` / `lowered.yaml` を+pi/2でbase_linkへ変換し、
壁・残箱・姿勢変化を含む点間の外形を検査する。吸着接触の除外は `pick.contact_regions` の選択側だけで、
足回りや反対側の機構を対象箱の検査から外さない。clear中も全下形状を使うため、反対側は上がっている実機より保守的な判定となる。
優先yaw・通路・手前距離は `pick.*` で指定する。候補yawが通れなければ指定範囲内を `yaw_resolution` で細かく調べる。
真っすぐの接近を優先し、隣の箱を避けるために外側(+y)の斜め接近も検査する。

暫定の密な配置では最初に箱4を左(gripper0)で回収する候補が有効。
各箱を運搬・配置してから戻る運用では、右側の箱から4→3→2→1→0の順で全5箱の経路を確認している。
どの箱/左右の組合せでも常に回収できるという意味ではなく、現在の残箱と実際の自己位置で毎回検査する。
図と可否一覧は `docs/superpowers/artifacts/pickup/` を参照。1アクションで5個を自動連続回収する機能ではない。

PICKの昇降には取り消し可能な既存 `gripper/action/MoveLift` を使う。
高さ・許容差はlaunchが実際の `gripper_params` の共通/個別 `lift.positions` / `tolerance` から解決する。
`carry - pick` からpick/carry双方の高さ許容差を差し引いても、箱・実物の壁の高さ+余裕を確保する必要があり、満たさなければ起動設定を拒否する。
棚と外周に対しては、昇降を上げた移動でも腕を含む全下形状の掃引を確認する。
実機は新しいCANフィードバックと目標高さへの到達を確認する。明示した `sim:=true` のみ指令値による模擬到達を許可する。
自動回収後の昇降目標は高さ指定なので、状態の `lift_position` は -1 となり得る。

吸引要求前の失敗では箱を未回収のまま残す。吸引要求を出した後で応答が不明、carry失敗、取り消しになった場合は、
箱と把持機構を「状態不明・使用不可」として記録し、自動タスクを拒否する。床の障害物としても残す。
`~/state` の従来のholding/picked表示は使用不可を伝え、追加の `uncertain grippers:` 行で理由を示す。
箱の実位置と機構を確認・復旧してから `~/reset` を明示的に呼ぶ。退避だけの失敗では保持中の記録を維持する。
吸引圧センサーはないため、保持成立の物理的な実測保証ではない。工具中心・接触領域・昇降高さは実機で校正する。

この環境でのオフラインテスト:

```bash
g++ -std=c++23 -O2 -Isrc/task_commander/include src/task_commander/src/geometry.cpp src/task_commander/src/pickup.cpp src/task_commander/test/pickup_test.cpp -o /tmp/pickup_test
/tmp/pickup_test
g++ -std=c++23 -O2 -Isrc/task_commander/include src/task_commander/src/pickup_execution.cpp src/task_commander/test/pickup_execution_test.cpp -o /tmp/pickup_execution_test
/tmp/pickup_execution_test
python src/oumuamua_bringup/test/pickup_parameters_test.py
```

ROS環境では `colcon test --packages-select task_commander oumuamua_bringup` で実ノードと模擬action/serviceを使う接続テストも行う。
PICKの形状/高さパラメータはbringupが生成するため、通常は `bringup.launch.py` 経由で起動する。

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/task_commander/geometry.hpp`, `src/geometry.cpp` | 道具を目標に合わせる機体の姿勢、機体と障害物の間隔。ROS非依存 |
| `src/task_commander_node.cpp` | ノード本体。作業ごとにスレッドを立てて順に進める |
| `action/RunTask.action` | 作業 |
| `test/geometry_test.cpp` | `geometry.cpp` のテスト (ROS不要) |
