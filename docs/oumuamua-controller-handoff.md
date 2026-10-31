# oumuamua-controller の改修のお願い

本体 (oumuamua) で、フィールドの寸法、ノーツの数と位置、task_commander の作業の種類が変わりました。
oumuamua-controller 側で直してほしい所をまとめます。**必須** は直さないと動かない・テストが落ちる所、
**推奨** は直したほうがよい所、**参考** は判断の材料です。

## 作業の前に (参考)

- 本体の変更はブランチ `field-blue-measurements` にあります。
- controller はブランチ `fix/upside-down-scan` (`b6f4816`、main の `3871829` の上) から始めてください。
  上下逆さに付いた LiDAR の点群が鏡写しに描かれていたのを直してあります。
- `test_completion.py::test_complete_file` は今回の変更の前から失敗していて、ここで頼む内容とは関係ありません。

## 1. 自動制御の作業の種類 (必須)

棚に置く作業 `PLACE` (5 番) をなくし、位置と向きだけ合わせる作業に変えました。昇降を段に合わせる、吸引を切る、
置いたマスを覚える、は task_commander ではしなくなります。棚に置く動きは手で操作します。
2 個同時の回収 (8 番) も足しました。作業の定義は `src/task_commander/action/RunTask.action`。

| 番号 | 作業 | 引数 | すること |
| --- | --- | --- | --- |
| 0 | `PICK` | `gripper`, `block` | 変わらず (回収の手順は下の 5 を参照) |
| 5 | `TO_SHELF_CELL` (旧 `PLACE`) | `gripper`, `column` | 今の y (前後) のまま横へ動き、把持機構 `gripper` の吸ったノーツの x を列 `column` の中心に合わせて、棚の方を向く。前後・昇降・吸引は変えない。`row` は送られても使わない |
| 7 | `TO_SHELF_FRONT` (新規) | なし | 昇降を運搬の高さにして棚の前へ。中央の列の正面で棚の方を向き、吸ったノーツの前の面が棚の中心面 (y = 0) から 0.33 m の所 (機体の中心は y = 0.749) |
| 8 | `PICK_PAIR` (新規) | `block` | gripper0 でブロック `block`、gripper1 でブロック `block - 1` を同時に回収する。今は `block` = 4 (4 と 3) と 2 (2 と 1) だけ |

直してほしい所:

- `crates/app/src/auto.rs` の `TaskKind::Place = 5` を `TO_SHELF_CELL` にあたる名前に変え、`ToShelfFront = 7` と
  `PickPair = 8` を足す。表示名 (例の「配置 右 列 1 段 0」) から段を外す。
- 自動操作の画面に「棚の前へ」(7 番) と「2 個同時に回収」(8 番、4+3 と 2+1) のボタンを足す。
  5 番は列と把持機構だけを選べるようにする。
- `manual.rs` の `task_kind_name`、`glue.rs` の `auto_run(TaskKind::Place …)` も合わせる。
- 模擬ロボット (`ros/…/fake_robot.py`) の `TASK_STAGES` の 5 番を「マスへ移動」だけにし、7 番と 8 番を足す。
  置いたマスを覚える処理は消す。8 番は 2 つの把持機構を両方「持っている」にする。

## 2. 棚に置く手順 (必須)

手で次のように操作する前提です。昇降と吸引の操作は今の画面にあります。

1. `TO_SHELF_FRONT` で棚の前へ。
2. `TO_SHELF_CELL` で、置く把持機構のノーツを列に合わせる (横に動くだけで、棚には近づかない)。
3. 昇降を置きたい段の高さへ (`LIFT_TO`)。
4. 手で前に出して、ノーツをマスへ差し込む。
5. 吸引を切る (`SET_SUCTION`)。task_commander は、持っている把持機構の吸引が切れると「持っていない」に戻す。
6. 手で後ろに下がる。

**お願い (推奨)**: 4 と 6 の前後の動きを、自動操作の画面 (列を選ぶボタンと同じ画面) から手でできるようにしてください。
機体の前後 (機体座標系の x) だけを動かす操作で、押している間だけゆっくり (0.1〜0.2 m/s 程度) 進む・下がる、のようなもの。
今の向き (棚の方、-y) では、機体の前が棚へ近づく向きです。

自動操作が「持っている把持機構」を選ぶ処理は、吸引を切った時点で表示が切り替わるので、そのまま使えます。

## 3. 覚えていることの表示 (必須)

task_commander の `~/state` から「placed cells」の行がなくなりました。`task_memory.parse` は行がなくても
空のリストを返すので壊れませんが、置いたマスは常に空になります。置いたマスの表示は消してください。
`~/reset` の返答も「forgot picked blocks and held blocks」に変わっています。
(`task_memory.py`、`server.py` の `TASK_MEMORY`、`test_task_memory.py`)

## 4. フィールドの既定値 (必須)

bridge の既定値が新しい `src/oumuamua_bringup/config/field.yaml` と合わず、
`test_default_config_matches_bridge_yaml` が失敗しています。

| 項目 | 前 | 今 |
| --- | --- | --- |
| `blocks` (ノーツの数) | 6 | 5 |
| `field_bounds` | ±2.786 | ±2.785 |
| `field_obstacles` | 壁 12 枚 + 棚 | 壁 11 枚 + 棚 (スタート側の短い壁 1 枚がなくなり、壁の厚みは 0.040 m) |

- `config.py` の `DEFAULTS` を `field.yaml` の値に合わせる。
- `auto.rs` の `DEFAULT_BLOCKS` を 6 から 5 に。
- `test_fake_robot.py` の `test_fake_scan_hits_walls` にある外周の x、2.786 を 2.785 に。
- 必要なら `test_server.py` の設定の例 (`blocks: 6`、古い壁) も。テスト用のデータなので必須ではありません。

## 5. 回収のしかたと、組み合わせ (参考)

設定は `src/oumuamua_bringup/config/task_commander.yaml` の `pick`。

- 回収するときの機体の中心の姿勢は、ブロックごと・把持機構ごとに `pick.poses.block<i>` に書いてある
  (`[x, y, yaw]` を gripper0, gripper1 の順)。2 個同時のときは `pick.pairs`。
- 手順: 昇降を寄る高さに上げて吸引をオン → 経由点 `pick.via` (-2.2, 0.75, -90°) で回収の向きに回る →
  手前 (`pick.standoff` 0.15 m) → 回収の姿勢 → 昇降を下ろしきって 1.5 s 待つ → 持ち上げて手前へ戻る。
  吸う前に失敗したら吸引を切る。

今の値では全部の組み合わせで届きます。かっこ内は壁やほかのブロックとの一番狭い間隔 (ノーツが全部残っているとき)。

| ブロック | x [m] | gripper0 (左) | gripper1 (右) |
| --- | --- | --- | --- |
| 0 | -2.635 | -127° (2.3 cm) | -90° (4.8 cm) |
| 1 | -2.405 | -96° (2.4 cm) | -90° (11.9 cm) |
| 2 | -2.175 | -90° (11.9 cm) | -90° (11.9 cm) |
| 3 | -1.945 | -90° (11.9 cm) | -90° (9.9 cm) |
| 4 | -1.715 | -90° (11.9 cm) | -90° (11.9 cm) |

gripper0 でブロック 0、1 を取ると斜めになり、間隔も狭いので、そこは gripper1 を使うほうが安全です。
全部運ぶなら `PICK_PAIR` 4 (4 と 3)、`PICK_PAIR` 2 (2 と 1)、`PICK` gripper1 でブロック 0、の 3 回が速いです。

## 6. 機体の描き方 (推奨)

機体は今、半径 0.32 m の円で描いています (`controller_bridge.yaml` の `robot_radius`)。task_commander と経路は、
CAD から取った外形の多角形で間隔を確かめるようになりました。画面も同じ外形で描くと、壁との間隔が見た目と合います。

- 外形: `task_commander.yaml` の `footprint` (機体座標系、x が前で y が左。前 0.159 m、左右 0.236 / 0.226 m、
  後ろ 0.331 m は LiDAR)。
- 吸着面の中心: `grippers.suction_offsets` = 前 0.269 m、左右 ±0.115 m (gripper0 が左)。
  吸ったノーツの中心は、そこからノーツの大きさの半分だけ前 (0.344 m)。

## 7. 経路 (参考)

`start_to_shelf` / `shelf_to_start` / `fast_start_to_shelf` / `fast_shelf_to_start` は消しました。
path_follower の経路は `slalom_forward` / `slalom_backward` だけです (`path_follower.yaml` の `paths`)。
controller からこれらの名前を使っている所はありませんでした。
