# path_follower

起動前に作った経路ファイルを読み、速度計画を付けて、軌道追従ノード
([holonomic_tracker](https://github.com/Stew-000-1-0-011/holonomic_tracker) の tracker_node) に時刻ごとの目標を出すノード。
経路ファイルを作るスクリプトも入っている。

- **経路を作る** (起動前): `scripts/make_path.py` が、経路の定義 (折れ線の頂点・角を丸める半径・機体の向き) から経路ファイルを作る
- **走る** (実行時): `path_follower_node` が経路ファイルを読み、`~/start/<経路の名前>` で始点から走り出して、目標を `reference_topic` に出す

## 構成

| ファイル | 役割 |
| --- | --- |
| `scripts/make_path.py` | 経路の定義 (YAML) -> 経路ファイル (CSV)。障害物との間隔の確認と図も出せる |
| `include/path_follower/trajectory.hpp`, `src/trajectory.cpp` | 経路ファイルの読み込みと速度計画。ROS非依存 |
| `src/path_follower_node.cpp` | ノード本体 |
| `test/trajectory_test.cpp` | 速度計画のテスト (ROS不要) |


**パラメータに既定値は無い。** 全部パラメータファイルで与える (足りなければ起動時にエラーで止まる)。この機体の値と各項目の説明は oumuamua の `src/oumuamua_bringup/config/path_follower.yaml` にある。

## 経路を作る

```bash
ros2 run path_follower make_path.py route.yaml path.csv --plot preview.png
ros2 run path_follower make_path.py route.yaml path_back.csv --reverse   # 同じ経路を逆にたどる
```

経路の定義の書き方は `make_path.py --help` と、oumuamua_bringup の `config/paths/start_to_shelf.route.yaml` を参照。

- `corners`: 折れ線の頂点 `[x, y, 半径]`。角は半径の円弧で丸める
- `yaw`: 始点からの道のり [m] と、そこでの機体の向き [rad]。全方向移動なので、向きは進む向きと無関係に決められる。
  間は滑らかにつなぐ。その場での旋回 (位置を変えずに向きだけ変える) は書けない
- `follow_heading`: 道のり `from` から先は、機体の向きを進む向きに合わせる (それまでは `yaw` のとおり)。
  `to` を書くと、そこで `yaw` のとおりに戻す (狭い区間だけ向きを合わせたいとき)
- `obstacles` / `bounds` と、機体の外形 (`footprint` の多角形か `robot_radius` の円): 書いておくと、
  外形から障害物までの最小の間隔を出す (機体の向きも考える)。`min_clearance` より近ければ失敗して、経路ファイルを作らない
- `lidar`: LiDAR の取付位置。図にその軌跡を描く
- `footprint_file`: 経路YAMLからの相対パスで、添付形式の `footprint: "[[x,y],...]"` またはリストを読む。
  埋込 `footprint` と併用しない。`footprint_yaw` [rad] で原形状からbase_linkへ回す。
- `endpoint_footprint_file`: 同じ変換で入口・出口だけ検証する別形状。昇降下での通過を認める指定ではない。
- `field_wall_padding` / `field_bounds_padding`: 共有フィールドを変えず、検証上の内壁を各面から拡大／外周内面を内側へ移す距離 [m]。
- `--reverse`: 同じ経路を終点から始点へたどる経路ファイルを作る。各点での機体の向きは同じなので、
  行きで前を向いて通った所を、帰りは後ろ向きに通る

経路ファイルは 1 行に `x, y, yaw` (フィールド座標系、yaw は ±pi で折り返さない) を並べた CSV。
`#` で始まる行と見出しの行は読み飛ばす。経路ファイルは作ったものをリポジトリに入れておき、定義を変えたら作り直す。

検証には、5桁へ丸めたCSV座標を使う。多角形の辺交差・包含・辺間距離を計算し、位置とyawの線形補間も含める。
各区間を機体上の点の最大移動量が4 mm以下になるよう分割し、隣接サンプル間の移動上界の半分（2 mm以下）を
測定値から引いた `conservative swept clearance` を出す。表示値が `min_clearance` 未満なら既存CSVを上書きしない。
円指定は描画用の多角形近似でなく、真の円半径で検証する。これは理想軌道の幾何検証であり、実機の追従誤差は別途確認する。

## 走らせる

```bash
ros2 run path_follower path_follower_node --ros-args --params-file /path/to/oumuamua/src/oumuamua_bringup/config/path_follower.yaml -p path_file:=/path/to/path.csv
ros2 service call /path_follower/start std_srvs/srv/Trigger    # 走り出す
ros2 service call /path_follower/stop std_srvs/srv/Trigger     # 目標を出すのをやめる (追従ノードが止める)
```

経路は複数持てる。`path_dir` に置いた `<名前>.csv` を `paths` に並べると、経路ごとに `~/start/<名前>` ができる
(`~/start` は最初の経路)。走っている途中で別の経路を始めると、その経路の始点から走り直す。

| 方向 | 名前 | 型 |
| --- | --- | --- |
| pub | `reference_topic` (例 `/tracker_node/reference`) | `holonomic_tracker/msg/TrackingReference` (位置・向き・速度FF) |
| pub | `~/path` | `nav_msgs/msg/Path` (最後に走り出した経路。RViz 用、transient local) |
| sub | `odom_topic` (例 `/state_estimator/odom`) | `nav_msgs/msg/Odometry` (自己位置) |
| srv | `~/start/<名前>`, `~/start` | `std_srvs/srv/Trigger` (その経路の始点から走り出す。`~/start` は最初の経路) |
| srv | `~/stop` | `std_srvs/srv/Trigger` |
| action | `~/follow` | `path_follower/action/FollowPath` (下の「アクション」) |

### アクション `~/follow`

経路を走り、終点で止まるまでを 1 つのゴールにする。task_commander が使う。

- `route` に `paths` の名前を書くと、その経路を走る (始点の確認は `~/start` と同じ)
- `route` が空なら、今の自己位置から `(x[i], y[i], yaw[i])` を順に直線でつないだ経路を作って走る。
  向きは各辺の上で一定の割合で回す。頂点では曲率の上限で遅くなる。その場での旋回はできない (位置が同じで向きだけ違う頂点は失敗)
- `max_speed` (0 でなければ) で `limits.max_speed` をさらに下げる
- 成功: 目標が終点に着いた後、自己位置が `goal_tolerance` [m]・`goal_yaw_tolerance` [rad] 以内に `goal_settle_time` [s] 留まった。
  `goal_timeout` [s] 経っても留まらなければ失敗
- 走っている途中に新しいゴール・`~/start`・`~/stop` が来たら、前のゴールは打ち切る。取り消すと目標を出すのをやめる (追従ノードが止める)

### 速度計画

経路の各点で許される速さを、最高速 (`limits.max_speed`)、横加速度 (`limits.max_lateral_accel`、曲率から)、
角速度 (`limits.max_yaw_rate`、道のりあたりの向きの変化から) の一番厳しいもので決め、
両端で止まるように前後から加減速 (`limits.max_accel`) の上限をかける。曲がり角と、向きを変える区間で自動的に遅くなる。

曲率と向きの変化は、各点の前後 3 cm 以上離れた点から測る。隣どうしの点で測ると、点を細かく打つほど
座標の丸め誤差で曲率が大きく見えて、計画が遅くなる (1 mm 刻みで 2 倍以上)。点の間隔は 1 cm 程度で足りる。

### 安全のための動き

- **始点の確認**: `~/start` のとき、機体が経路の始点から `start_tolerance` より離れていたら走り出さない
  (離れたまま始めると、追従ノードが始点へ一気に寄せようとする)。自己位置が来ていないときも走り出さない
- **機体を待つ**: 機体が目標から `max_tracking_error` より離れたら、追いつくまで目標を進めない。
  何かに引っかかったときに、目標だけが先へ行って追従ノードが全力で追いかけるのを防ぐ
- **終点**: 着いた後も終点を出し続けて、そこに留まる (`hold_at_goal`)。`~/stop` でやめる

障害物は見ていない。経路が壁に当たらないことは、経路を作るときに確かめておくこと。

## テスト

```bash
colcon test --packages-select path_follower && colcon test-result --verbose
# ROSのない環境で生成器だけテストする場合 (PyYAMLが必要):
python -m unittest discover -s src/path_follower/test -p '*_test.py' -v
```

台形・三角形の速度計画、曲がり角での減速、角速度の上限、両端で止まること、経路ファイルの読み込みを確認する。
