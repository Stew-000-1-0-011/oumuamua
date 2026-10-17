# oumuamua_bench

oumuamua の構成 (自己位置推定や経路追従のやり方) どうしを、シミュレーションの閉ループで比べる。

構成ごと・状況ごと・乱数の種ごとに `bringup.launch.py sim:=true` を立て、path_follower で本番の経路を往復させて
(`start_to_shelf` -> `shelf_to_start`)、追従・自己位置・計算負荷を測る。

```bash
# 全部の構成 x 全部の状況 x 種 3 つ (1 走行 40 秒ほど)
container/run.sh ros2 run oumuamua_bench bench_run.py --out /ws/bench/run1 --seeds 3
# 絞る
container/run.sh ros2 run oumuamua_bench bench_run.py --out /ws/bench/run1 \
  --variants sotoba_est --scenarios nominal tilt --seeds 5
# 表にする (状況ごとに 1 表。--by variant なら構成ごと)
container/run.sh ros2 run oumuamua_bench bench_summary.py /ws/bench/run1/runs.jsonl
```

結果は `<out>/runs.jsonl` (1 走行 1 行、追記) と、走行ごとの launch のログ `<out>/logs/`。
`bench/` は `.gitignore` に入れてある。

## 構成と状況

- `config/variants.yaml`: 構成。名前と、bringup.launch.py に渡す引数。自己位置推定や経路追従を置き換えるブランチは、ここに足す
- `config/scenarios.yaml`: 状況。robot_sim に追加で読ませるパラメータ (bringup の `sim_params` 引数で渡す)

| 状況 | 内容 |
| --- | --- |
| `nominal` | 外乱なし |
| `offset` | 置いた位置が初期シードから 0.1 m・0.08 rad ずれている |
| `tilt` | 振動で LiDAR が傾く (速さに比例する揺れ) |
| `obstacle` | フィールドの中に、モデルに無いロボット大の箱 (経路には掛からない) |
| `distort` | 1 スキャンを 25 ms かけて撮る (回転式 LiDAR の歪み) |
| `slip` | 車輪が 5 % 滑る (オドメトリが実際より大きい) |

乱数の種は robot_sim の雑音 (距離・機体速度・傾き) を変える。

## 測るもの

| 項目 | 意味 |
| --- | --- |
| 経路からのずれ | 真値の位置と経路 (折れ線) の距離。rms / 95% / 最大 |
| 目標からの遅れ (`ref_lag`) | 真値と、path_follower が出している目標の距離 |
| 壁との間隔 | 機体の外形 (`start_to_shelf.route.yaml` の `footprint`) から地図の点までの最小の距離。負なら壁に食い込んでいる (シミュレーションではすり抜ける) |
| 往路・復路 | 走り出してから、終点の 5 cm 以内に入って出なくなるまでの時間 |
| 終点の誤差 | 目標が終点に着いて 1 秒後の、真値と終点の差 |
| 推定誤差 | TF `field -> base_link` の推定と、推定が指す時刻の真値の差 |
| 「今」の誤差 | 真値の各時刻に、そこまでに届いた最新の推定と比べた差 (推定の遅れも含む) |
| CPU | 走っている間の CPU 時間 / 経過時間。ノードの実行ファイルごとに測り、自己位置推定 (`localization`)・経路追従 (`navigation`)・その他にまとめる。robot_sim は数えない |
| 1 スキャンあたり | 自己位置推定の CPU 時間 / スキャン数 |
| メモリ | 自己位置推定のノードの最大の常駐メモリの合計 |

推定は TF の鎖をたどって取るので、`field -> base_link` が直接でも `field -> odom -> base_link` でもよい。
区分 (`GROUPS`) は `bench_run.py` の中にある。新しいノードを足したら、そこにも足す。

## 地図

AMCL や点群の位置合わせのように地図を使う構成のために、sotoba_node のモデルと同じ形の地図を
`oumuamua_bringup/config/maps/field.{pgm,yaml,pcd}` に置いてある。フィールドの形 (sotoba_ros の `objects.cpp`) や
`include_*` を変えたら作り直す:

```bash
container/run.sh ros2 run oumuamua_sim make_field_map --ros-args \
  --params-file /ws/src/sotoba_ros/config/sotoba_node.yaml \
  --params-file /ws/src/oumuamua_bringup/config/sotoba_node.yaml \
  -p map.output_dir:=/ws/src/oumuamua_bringup/config/maps
```

## 実機で使うときの注意

測り方は実機の PC でもそのまま使えるが、真値が無いので追従と自己位置の欄は意味を持たない。
実機では止まった位置を床の印から測る、などで確かめる。計算負荷はこのクラウド環境と実機の PC で大きく違うので、
絶対値は実機の PC で測ったものを見ること。
