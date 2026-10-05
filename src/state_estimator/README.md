# state_estimator

機体の平面姿勢 (x, y, yaw) と機体速度を推定するノード。[sotoba_ros](https://github.com/Stew-000-1-0-011/sotoba_ros) の
sotoba_node と対になって動く。

- **予測**: `~/body_velocity` (下位層が推定した実際の機体速度、機体座標系) で姿勢を進める
- **観測**: sotoba_node の事後分布のうちフィールド物体から、機体の姿勢を求めて入れる
- **出力**: 現在の推定を `~/odom` と TF `field -> base_link` に、sotoba_node への事前分布を `prior_beliefs` に出す

事前 -> sotoba -> 事後 -> ここ、と計算が循環する。その中で時刻をまたいで状態を持つのはこのノードだけにしてある
(sotoba_node は `prior_source: external` なら毎スキャン事前を受けて事後を返すだけ)。

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/state_estimator/filter.hpp`, `src/filter.cpp` | 拡張カルマンフィルタ (遅れて届く観測をその時刻で入れる)。ROS非依存 |
| `include/state_estimator/belief_geometry.hpp`, `src/belief_geometry.cpp` | 機体の姿勢と sotoba の信念の相互変換、事前の差し引き。ROS非依存 |
| `src/state_estimator_node.cpp` | ノード本体 |
| `config/state_estimator.yaml` | パラメータ |
| `test/estimator_test.cpp` | フィルタと変換のテスト (ROS不要) |

## トピック

| 方向 | トピック | 型 |
| --- | --- | --- |
| sub | `~/body_velocity` | `geometry_msgs/msg/TwistStamped` (機体座標系。remap してつなぐ) |
| sub | `posterior_topic` (既定 `/sotoba_node/posterior_beliefs`) | `sotoba_ros/msg/BeliefArray` |
| sub | `initial_topic` (既定 `/sotoba_node/initial_beliefs`) | `sotoba_ros/msg/BeliefArray` (transient local。初期化と物体の位置関係に使う) |
| sub | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` (RViz の 2D Pose Estimate。その姿勢でやり直す) |
| sub | TF `base_frame -> lidar_frame` | LiDAR の取付 |
| pub | `~/odom` | `nav_msgs/msg/Odometry` (姿勢はフィールド座標系、速度は機体座標系) |
| pub | TF `field_frame -> base_frame` | |
| pub | `prior_topic` (既定 `/sotoba_node/prior_beliefs`) | `sotoba_ros/msg/BeliefArray` |
| srv | `~/reset` | `std_srvs/srv/Trigger` (sotoba の定義上の初期姿勢でやり直す) |

`~/odom`・TF・事前は `publish_rate` (既定 100 Hz) で出す。sotoba_node は事前を溜めておき、
スキャン時刻以前で最新のものを使う。

## 状態と計算

状態は `[x, y, yaw, vx, vy, omega]` (姿勢はフィールド座標系、速度は機体座標系)。

- 予測: 速度は入力 u に `tau` の 1 次遅れで追従する (`tau = 0` なら u そのもの)。姿勢は速度を向きで回して積分し、
  姿勢の雑音 (`pose_noise_*`、滑りや入力の誤差) を足す
  - u が車輪オドメトリのような「実際の速度の推定値」なら `tau = 0`
  - u が指令値 (cmd_vel) なら `tau` を下位の速度制御の時定数にする
- 観測: スキャン時刻の姿勢の観測として入れる。最後に観測を入れた時刻の状態 (アンカー) からその時刻まで入力の履歴で予測し、
  更新したものを新しいアンカーにする。現在の推定は毎回アンカーから予測し直す。アンカーより古い観測は捨てる

### sotoba への事前

フィールド座標系での各物体の姿勢 (sotoba の定義上の初期姿勢から作り、推定が取れるたびに覚え直す) を、
推定した機体の姿勢と LiDAR の取付で LiDAR 座標系に写して平均にする。情報行列は、機体の平面の共分散に
面外 (z, roll, pitch) の小さな不確かさ (`prior.sigma_*`) を足したものを、数値微分で sotoba の左摂動に写して逆にしたもの。
フィールド以外の物体には、フィールドに対する位置関係の不確かさ (`prior.relation_sigma_*`) も足す。

### 二重計上を避ける

sotoba の事後には、ここが送った事前がすでに含まれている。そのまま観測として入れると同じ情報を 2 回数えて、
推定を実際より確かだと思い込む。そこで、sotoba と同じ規則 (スキャン時刻以前で最新) で使われた事前を選び、
事後から差し引いて「スキャンだけ」の信念にしてから入れる (`subtract_prior`)。
情報行列は引き算、平均は事後の平均で勾配が 0 になる条件から求める。見えない方向は情報 0 として扱う。

sotoba が `use_prior: false` (事前をシードにしか使わない) なら事後に事前は入っていないので、`subtract_prior: false` にすること。

## 初期化とリセット

- 起動時: sotoba の `initial_beliefs` (定義上の初期姿勢) と `initial_sigma_*` で初期化し、すぐ事前を出し始める。
  sotoba は事前が来るまでスキャンを捨てるので、これで循環が立ち上がる
- `~/reset`: 同じく定義上の初期姿勢でやり直す。`/initialpose`: 与えた姿勢でやり直す
- リセットしたら、リセット時刻より前のスキャンの事後は捨てる (リセット前の事前から作られているので)
- sotoba の推定が使えない (失敗・ゲートでの棄却) ことが `inflate_after_failures` 回続いたら、
  姿勢は捨てずに不確かさだけ足す (sotoba が広めに探し直せるように)。元の位置へ戻すリセットは自動ではしない
- 観測が `measurement_timeout` より途切れたら `~/odom` と TF を止める (下流の tracker はタイムアウトで止まる)。
  事前は出し続ける

## 分かっている課題

- **スキャンの歪みを補正していない**。回転式 LiDAR は 1 周の間に機体が動くので、速く回るほど点群が歪む。
  sotoba はスキャンを一瞬で撮ったものとして扱うので、推定が偏る (シミュレータで 6 rad/s・1 周 50 ms のとき yaw で 0.15〜0.25 rad)。
  このノードは機体の動きを知っているので、点群の補正 (deskew) に使える見込み
- sotoba の推定の時刻はスキャンの撮り始め。歪みがあると実際には 1 周の途中の姿勢に近い
- 観測の不確かさは sotoba の情報行列そのもの (+ `measurement_floor_*`)。歪みなどのモデル化していない誤差を含まないので、
  速く回るときは過信になる

## テスト

```bash
colcon test --packages-select state_estimator && colcon test-result --verbose
```

速度の積分、1 次遅れ、遅れて届く観測による現在の推定の補正、順序の入れ替わり、ゲート、
機体の姿勢 -> 事前 -> 機体の姿勢の往復、事後からの事前の差し引き (見えない方向があるとき) を確認する。
