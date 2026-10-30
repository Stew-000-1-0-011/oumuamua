# oumuamua

ROS 2 Lyrical Luth (Ubuntu 26.04) ワークスペース。外部ドライバなどは `src/` 以下の git submodule、機体の起動設定や `gripper`・`harite` などはこのリポジトリで管理する。

| パッケージ | 内容 |
| --- | --- |
| `src/sotoba_ros` | sotoba の ICP による LiDAR 物体姿勢推定 |
| `src/urg_node2` | 北陽 URG LiDAR ドライバ |
| `src/holonomic_tracker` | 全方向移動ロボットの追従制御 |
| `src/omni_chassis` | 足回りドライバ。`cmd_vel` を N 輪オムニの逆運動学で車輪ごとのモータ目標角速度にする |
| `src/mini_shirasu_ros` | mini-shirasu (ブラシ付き DC モータドライバ) と CAN で話す |
| `src/robomas_plugins` | USB-CAN ブリッジ (crs-kouhou/robomas_plugins のフォーク)。`robomas_can_tx` / `robomas_can_rx` だけを使う |
| `src/oumuamua_bringup` | 上をまとめて起動する launch と、この機体のパラメータ |
| `src/oumuamua_sim` | `/cmd_vel` から先のシミュレータ (平面運動と 2D LiDAR) |
| `src/state_estimator` | 自己位置と機体速度の推定。sotoba_node と対になって事前分布を渡す |
| `src/gripper` | 把持機構 (ロボマスによる昇降 + mini-shirasu の電流制御による吸引) 1 つぶんを動かすノード。機構の数だけ立てる |
| `src/harite` | 吸引なしの昇降機構。ロボマス ID 3 / C610 を robomas_plugins 経由で速度制御する |
| `src/task_commander` | 機体に作業 (ブロックの回収、スラロームの往復、棚への配置、各地点への移動) を 1 つずつ指示する |
| `src/path_follower` | 起動前に作った経路ファイルを読み、速度計画を付けて tracker_node に目標を出す。経路ファイルを作るスクリプト付き |

## つながり

- `/scan` (urg_node2) -> sotoba_node。sotoba_node は state_estimator から事前分布 (`/sotoba_node/prior_beliefs`) を受け、
  事後分布 (`/sotoba_node/posterior_beliefs`) を返す
- state_estimator は機体速度 (`~/body_velocity`) で予測し、sotoba の事後で更新する。
  推定を `/state_estimator/odom` と TF `field -> base_link` に出す
- tracker_node は `/state_estimator/odom` を使って `/cmd_vel` (`TwistStamped`) を出す
- chassis_node -> `/wheel<i>/target_velocity` -> mini_shirasu_node (`wheel0..2`)
  -> `/robomas_can_tx` -> robomas_bridge (USB-CAN) -> mini-shirasu 3 枚 (3 輪オムニ)
- 把持機構 i (`i = 0, 1`): gripper_node (`gripper<i>`) が `~/set_lift_position` (`config/gripper.yaml` の `lift.positions` の番号) で昇降し、`~/set_suction` で吸引をオン/オフする。
  昇降は `/robomas_frame`・`/robomas_target<k>` -> robomas_bridge -> ロボマス (ボードの位置制御)、
  吸引は `/suction<i>/target_current` -> mini_shirasu_node (`suction<i>`、電流制御) -> mini-shirasu。詳しくは `src/gripper/README.md`
- harite は `/harite/target_velocity` (ロータ角速度 [rad/s]) を受け、`/robomas_frame` (速度モード)・`/robomas_target3` -> robomas_bridge -> ロボマス ID 3 を動かす。吸引・位置制御は持たない。詳しくは [src/harite/README.md](src/harite/README.md)
- task_commander が `~/run` (アクション) で作業を受け、path_follower の `~/follow` と gripper<i> を順に動かす。詳しくは `src/task_commander/README.md`
- 車輪の実測: mini_shirasu_node の `~/joint_state` -> chassis_node が機体速度にして `~/body_velocity`
  -> state_estimator の `~/body_velocity` (車輪オドメトリ)

計算が循環する (事前 -> sotoba -> 事後 -> state_estimator) ので、時刻をまたいで状態を持つのは
state_estimator だけにしてある。リセットは state_estimator の `~/reset` か `/initialpose` (RViz の 2D Pose Estimate) で行う。
詳しくは `src/state_estimator/README.md`。

`estimator:=false` なら state_estimator を立てず、sotoba_ros の persistence_predictor (持続予測。機体の動きは知らない) が
事前分布を出す。sotoba_node が TF `field -> base_link` を出し、tracker_node はそれを使う。

tracker_node への目標 (`/tracker_node/reference`, `holonomic_tracker/msg/TrackingReference`) は path_follower が出す。
起動しただけでは走らない。経路の始点に置いて、次で走り出す:

```bash
ros2 service call /path_follower/start/start_to_shelf std_srvs/srv/Trigger   # スタート位置 -> 棚の正面
ros2 service call /path_follower/start/shelf_to_start std_srvs/srv/Trigger   # 棚の正面 -> スタート位置
ros2 service call /path_follower/stop std_srvs/srv/Trigger                   # 目標を止める (tracker_node が機体を止める)
```

経路は `src/oumuamua_bringup/config/paths/` にある。スラロームの往復 (`slalom_forward.csv`, `slalom_backward.csv`) は
`slalom.route.yaml` から作り、task_commander が使う。`start_to_shelf.route.yaml` が定義で、そこから
行き (`start_to_shelf.csv`) と帰り (`shelf_to_start.csv`。同じ経路を逆にたどる) を作ってある。定義を変えたら両方作り直す:

スラロームは添付CADの上下形状を `config/footprints/{raised,lowered}.yaml` に保存し、原座標を
`(x_base,y_base)=(-y_attachment,x_attachment)` で変換して検証する。往復とも全昇降がcarryに到達してから通過し、
入口・出口では下形状も40 mm以上の余裕を確認する。本文の壁厚38 mmに合わせて検証上だけ壁を拡大する。
再生成コマンドは `config/paths/slalom.route.yaml` 冒頭、図と検証結果は `docs/superpowers/artifacts/` を参照。
上形状を使う前提として、持ち上げた機構・保持ノーツの下面が90 mmの壁より高いことを実機で確認する。

```bash
cd src/oumuamua_bringup/config/paths
ros2 run path_follower make_path.py start_to_shelf.route.yaml start_to_shelf.csv --plot /tmp/preview.png
ros2 run path_follower make_path.py start_to_shelf.route.yaml shelf_to_start.csv --reverse
```

定義にはフィールドの壁と機体の外形 (半径 0.30 m の円 + 後ろに飛び出した LiDAR) も書いてあり、
外形が壁に `min_clearance` より近づく経路は作るのに失敗する。機体の向きは、最初の隙間から先は進む向きに合わせてある
(機体の後ろ (LiDAR の側) を進む向きに向ける。帰りは同じ向きのまま逆にたどるので、前向きに走る)。どの経路を読むかは `config/path_follower.yaml` の `paths`。
速さの上限は `config/path_follower.yaml`。詳しくは `src/path_follower/README.md`。

## 起動

```bash
ros2 launch oumuamua_bringup bringup.launch.py
```

| 引数 | 既定 | 意味 |
| --- | --- | --- |
| `lidar` | `true` | urg_node2 を起動する (configure -> activate まで自動) |
| `lidar_connection` | `serial` | `ether` / `serial`。`config/urg_<...>.yaml` を使う。serial は `/dev/serial/by-id/usb-Hokuyo_...` を開き、`container/run.sh` はつながっていればこれをコンテナに渡す |
| `lidar_x` | `-0.28` | base_link から見た LiDAR の取付位置 [m] (static TF `base_link -> laser`)。LiDAR は機体の後ろ側で、車輪 0 の 8 cm 先 |
| `lidar_y` | `0.0` | 同上 (横方向の位置) |
| `lidar_yaw` | `pi` | LiDAR の向き [rad]。機体の後ろ (-x) を向いている |
| `lidar_z` | `0.14` | 走査面の高さ [m]。sotoba_node の `lidar_height` にも渡す |
| `lidar_upside_down` | `true` | 逆さ付けか。TF の roll と sotoba_node の同名パラメータに渡す |
| `chassis_params` | `config/chassis_node.yaml` | chassis_node のパラメータ |
| `wheel_count` | `3` | mini_shirasu_node (`wheel0..`) の数。`chassis_node.yaml` の車輪数と合わせる |
| `mini_shirasu_params` | `config/mini_shirasu.yaml` | mini_shirasu_node (`wheel0..`) のパラメータ |
| `gripper_count` | `2` | 把持機構 (`gripper<i>` と `suction<i>`) の数。`sim:=true` なら `gripper<i>` だけ立てる |
| `gripper_params` | `config/gripper.yaml` | gripper_node (`gripper0..`) のパラメータ |
| `suction_params` | `config/suction.yaml` | 吸引の mini_shirasu_node (`suction0..`) のパラメータ |
| `harite` | `true` | 吸引なしの昇降 harite を起動する。`sim:=true` では起動しない |
| `harite_params` | `config/harite.yaml` | harite のパラメータ (ID 3、C610、速度指令の上限とタイムアウト) |
| `bridge` | `true` | robomas_bridge (USB-CAN) を起動する |
| `rviz` | `false` | RViz2 を起動する |
| `sim` | `false` | `/cmd_vel` から先をシミュレータにする (下の「シミュレーション」) |
| `path_dir` | `config/paths` | path_follower の経路ファイルを置いた場所 (どれを読むかは `path_follower.yaml` の `paths`) |
| `estimator` | `true` | state_estimator を使う。`false` なら sotoba_ros の持続予測で事前を出す |
| `velocity_source` | `wheels` | state_estimator の速度入力。`wheels`: 車輪オドメトリ (`sim:=true` なら robot_sim の模擬)、`cmd_vel`: 指令値 |
| `cmd_tau` | `0.18` | `velocity_source:=cmd_vel` のときの、指令への追従の時定数 [s] (実機の記録で 0.18 s) |

**パラメータは全部 `src/oumuamua_bringup/config/` にある。** 各ノードは既定値を持たないので、ここに無い項目があると
そのノードは起動時にエラーで止まる。パッケージ側には設定ファイルを置いていない。

| ファイル | ノード |
| --- | --- |
| `field.yaml` | **フィールドの寸法** (外周・壁・棚・ノーツ・スタート位置)。sotoba_node・robot_sim・task_commander・コントローラの bridge と、経路の定義 (`config/paths/*.route.yaml` の `field:`) がここから読む。フィールドの値はここにしか書かない |
| `chassis_node.yaml` | chassis_node (omni_chassis) |
| `mini_shirasu.yaml` | wheel0..2 (mini_shirasu_node) |
| `gripper.yaml` / `suction.yaml` | gripper0..1 (gripper_node) / suction0..1 (mini_shirasu_node) |
| `harite.yaml` | harite_node (ロボマスの速度制御専用の昇降) |
| `task_commander.yaml` | task_commander |
| `sotoba_node.yaml` | sotoba_node と persistence_predictor (sotoba_ros)。robot_sim もノーツの有無をここから読む |
| `state_estimator.yaml` | state_estimator |
| `tracker_node.yaml` | tracker_node (holonomic_tracker) |
| `path_follower.yaml` | path_follower |
| `robot_sim.yaml` | robot_sim (oumuamua_sim、`sim:=true`) |
| `controller_bridge.yaml` | oumuamua-controller の bridge (`ros2 launch oumuamua_controller_bridge bridge.launch.py` が field.yaml と一緒に読む) |
| `urg_serial.yaml` / `urg_ether.yaml` | urg_node2 |

LiDAR の取付 (`lidar_*`) や `estimator` で決まる値 (`pose_source`、`odom_topic`、`publish_tf` など) は、
`bringup.launch.py` が launch 引数から上書きする (ファイルにも同じ既定の値を書いてある)。

次は**例・仮の値のまま**なので、実機に合わせること。

- `src/oumuamua_bringup/config/chassis_node.yaml`: 3 輪オムニ (機体正面 = 車輪 0 から中心を見た向き = 把持機構の側。
  車輪 0 は後ろ、車輪 0..2 は左回りに 120 度ずつ、中心から 0.2 m、半径 0.06 m)。各車輪の正転の向き (`gear_ratio` の符号) は未確認
- `src/oumuamua_bringup/config/mini_shirasu.yaml`: 基板ごとの CAN ID。ファームの ID は
  `minishirasu-firm/src/config.rs` のコンパイル時定数 なので、
  複数枚つなぐなら基板ごとに書き換えて焼き、ここと合わせる。今は基板 i に 目標 0x110+i / 状態 0x120+i / コマンド 0x130+i / 応答 0x140+i を割り当てている

- `src/oumuamua_bringup/config/task_commander.yaml`: 把持機構の取付位置 (`grippers.offsets`、今は機体の前 0.45 m・左右 0.12 m と仮定) と、
  回収・配置のときの機体の向き
- `src/oumuamua_bringup/config/gripper.yaml`, `suction.yaml`: 把持機構。ロボマスの ID と種類、昇降の高さ (`lift.positions`)、ボードの位置目標の単位
  (`lift.target_scale`)、行程、吸引の強さ (`suction.target`。左は電流制御 [A]、右はデューティ制御)、
  吸引の基板の CAN ID (左が基板 4 = 0x114..、右が基板 5 = 0x115..)

モータドライバの設定 (`settings.*`) は mini-shirasu2 の `bench.rs` で実機が動いた値で、
速度・位置は駆動軸 (ホイール) 基準。そのため chassis_node の `gear_ratio` は ±1、
`max_wheel_speed` はドライバの `wmax` (20 rad/s) と揃えてある。

mini-shirasu のファームには通信タイムアウトが無いので、PC やノードが落ちたときに止めるのは
緊急停止スイッチに頼ることになる。
機体の前 (+x) は把持機構の側で、LiDAR と車輪 0 は後ろ。スタート位置 (`field.yaml` の `start_pose`) は
機体中心の姿勢で、sotoba の初期シード (LiDAR の姿勢) はそこから LiDAR の取付で求める。

## シミュレーション (`sim:=true`)

`/cmd_vel` から先 (足回り・モータドライバ・USB-CAN・LiDAR) を `oumuamua_sim` の `robot_sim` に置き換える。
sotoba_node と tracker_node は実機と同じものが動く。

```bash
container/run.sh ros2 launch oumuamua_bringup bringup.launch.py sim:=true
# 目標を出す (例: 初期位置のまわりを半径 0.1 m で回る)
container/run.sh ros2 run holonomic_tracker circle_reference_publisher --ros-args \
  -p center_x:=0.021 -p center_y:=0.444 -p radius:=0.1 -p period:=8.0 -p spin:=0.0 \
  -r reference:=/tracker_node/reference
```

robot_sim がやること:

- `/cmd_vel` (機体座標系) に、軸ごとの 1 次遅れ (`plant.tau_*`)・加速度制限・速度の倍率 (滑りの模擬) で追従し、姿勢を積分する
- sotoba_ros の `objects.cpp` と**同じフィールド形状**にレイキャストして `/scan` を出す
  (`scan_hz`、`ray_num`、`range_noise_stddev`、`scan_latency`)。LiDAR の取付 (`lidar_*` 引数) と上下逆さも反映する
- 車輪オドメトリの代わりに機体速度の実測 (`/robot_sim/body_velocity`) を出す。滑り (`plant.velocity_scale_*`) は知らない値で、雑音が乗る
- `scan_duration` > 0 なら、1 スキャンの光線を時間をずらして撮る (回転式 LiDAR の、動きながら撮ることによる歪み)
- 真の姿勢を `/robot_sim/truth_pose` (`PoseStamped`)・`/robot_sim/truth_odom` と TF `field -> base_link_truth` に出す。
  推定 (TF `field -> base_link`) と並べて比べられる
- 真の初期姿勢は、sotoba の初期シード (`start_*`) から `start_offset_*` だけずらしてある (初期の推定誤差の模擬)
- `/robot_sim/reset` (`std_srvs/Trigger`) で初期姿勢に戻す
- `tilt.*` で機体の傾き (取付の傾き + 揺れ + 雑音。揺れの振幅は速さに比例させることもできる) を、
  `environment.*` でフィールドの外 (床・部屋の壁・人や机の代わりの箱) を模擬する。傾いた光線は壁を越えて外の物や床に当たる

フィールドの寸法とスタート位置は sotoba_node と同じ `field.yaml` を読むので食い違わない。
パラメータは `src/oumuamua_bringup/config/robot_sim.yaml`。

模擬していないもの: 壁やノーツとの衝突 (すり抜ける)、車輪ごとの速度上限、モータドライバと CAN。

### LiDAR の傾きへの対策

振動で LiDAR が傾くと、光線が壁を越えて外の物に当たったり、床に当たったりする。
sotoba は点を最寄りの壁に対応付けるので、これらの点を壁と取り違えて推定がずれる。
`src/oumuamua_bringup/config/` で次の対策を入れてある:

- sotoba: `huber_k: 2.0` (残差の大きい点の重みを下げる)、`accept_distance_begin: 0.3` (対応付けの範囲を初回から狭める)、
  `ray_gate: 0.1` (state_estimator の事前の姿勢で光線をモデルに当て、予測距離と 0.1 m 以上違う点を捨てる)
- state_estimator: `filter.gate_sigma: 5.0` (予測から大きく外れた sotoba の推定を捨てる)

シミュレーション (初期位置付近を半径 0.2 m・周期 2 s で回りながら 1 回転、揺れ 8 Hz) で、
スキャン時刻での位置誤差 (rms / 最大) を比べた結果:

| 揺れの振幅 | 対策なし | 対策あり |
| --- | --- | --- |
| 0 | 4 mm / 8 mm | 4 mm / 9 mm |
| 0.12 rad (7°) | 36 mm / 193 mm、向きが 0.6 rad 飛ぶ | - |
| 0.16 rad (9°) | 見失う | 24 mm / 86 mm |
| 0.20 rad (11°) | - | 31 mm / 78 mm |
| 0.25 rad (14°) | - | 45 mm / 148 mm (見失いはしない) |

この位置ではまわりの壁が 1 m 前後と近く、振幅 0.08 rad 以下では壁を越える点がほとんど出ない (誤差は 15 mm 程度)。
残る誤差は、傾きで距離が 1/cos 倍に伸びることや、近くの壁に誤対応する床の点によるもの。
傾きそのものを知るには IMU が要る。

試すとき: `ros2 param set /robot_sim tilt.amplitude 0.16` (実行中に変えられる)。

### launch のパラメータの上書きについて

launch_ros の `parameters=[{...}]` (辞書) は `"/**"` として書かれ、パッケージのパラメータファイルに
ノード名で書かれた値に**負ける**。`ros_arguments` の `-p` もパラメータファイルより前に並ぶので負ける。
`bringup.launch.py` で launch 引数から切り替える値は、`NodeParams` (ノード名をキーにした一時ファイル) を
`parameters` の最後に置いて渡している。

## CAN 通信のテスト (モータは回さない)

mini-shirasu 1 枚と USB-CAN ブリッジ (robomas_plugins が話す Debug_CAN ボード) だけで、話せるかを確かめる。
mini-shirasu のファームの CAN 送受信はまだ実機で動かしたことがないので、ここから始める。

準備:

- USB-CAN ボードの udev ルールを入れて `/dev/robomas` ができること
  (`sudo cp src/robomas_plugins/udev/60-robomas.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`)。
  `container/run.sh` は `/dev/robomas` があればコンテナに渡す
- CAN は 1Mbps。USB-CAN ボード側のビットレートを合わせる。終端抵抗も確認する
- 基板のファームの CAN ID (`minishirasu-firm/src/config.rs`) と `config/mini_shirasu.yaml` の `wheel<board>` を合わせる。
  基板 n のファームは 目標 0x11n / 状態 0x12n / コマンド 0x13n / 応答 0x14n で、`wheel<n>` と同じ
- 基板のログ (defmt / RTT) を見られるようにしておく

### WSL2 から使う場合

- USB-CAN ボードは [usbipd-win](https://github.com/dorssel/usbipd-win) で WSL2 に渡す
  (Windows 側で `usbipd list` -> `usbipd bind --busid <ID>` -> `usbipd attach --wsl --busid <ID> --auto-attach`)。
  ボードがつなぎ直されると Windows 側に戻るので `--auto-attach` を付ける
- `/dev/robomas` を作る udev ルールは、`/etc/wsl.conf` で `systemd=true` にしていないと動かない。
  その場合は `/dev/ttyACM0` などのまま、`DEVICES=/dev/ttyACM0:/dev/robomas container/run.sh ...` でコンテナ内の名前を合わせる
- コンテナエンジンは WSL2 のディストリビューションの中に入れた Podman (か Docker Engine) を使う。
  Docker Desktop のコンテナは別の VM で動くので、ディストリビューションに渡したデバイスが見えない
- ワークスペースは WSL2 側のファイルシステム (`~/` 以下) に置く。`/mnt/c` 以下だとビルドがかなり遅い

### 1. フレームを手で 1 つ送る

ブリッジだけを立てて、`SetMode(無効)` を 1 フレーム送り、`Ack` が返るかを見る。

```bash
container/run.sh ros2 launch oumuamua_bringup can_test.launch.py node:=false
# ブリッジのログに "negotiation success" が出るまで待つ。別の端末で:
container/run.sh ros2 topic echo /robomas_can_rx
# さらに別の端末で:
container/run.sh ros2 topic pub --once /robomas_can_tx robomas_plugins/msg/Frame \
  "{id: 0x130, dlc: 5, data: [0x02, 0x10, 0x02, 0x57, 0x00, 0, 0, 0]}"
```

`02 10 02 57 00` は `COBS(10 00 | CRC 57) + 00`、つまり `SetMode(0)`。うまくいけば
`/robomas_can_rx` に `id: 320` (0x140)、`dlc: 5`、`data: [4, 48, 16, 137, 0, ...]` (`Ack(SetMode)`) が返る。

- 何も返らず、基板のログにも何も出ない: ブリッジから基板まで届いていない (ビットレート、配線、終端、ID、ファームの受信)
- 基板のログに `nack:` が出る: 届いているが中身が違う。その行を mini-shirasu2 側に渡す
- 基板は受けているのに返らない: ファームの送信かブリッジの受信

### 2. ノードで設定を送る

```bash
container/run.sh ros2 launch oumuamua_bringup can_test.launch.py board:=0
```

mini_shirasu_node が `SetMode(無効)` と全設定の `SetParam` を Ack を待ちながら送る。**有効化はしない**。
ログに `configured (disabled)` が出て、`/wheel0/status` に Status が約 50Hz で届けばよい
(`ros2 topic hz /wheel0/status`)。母線電圧 `vdc` が実測と合うことも見る。
手でホイールを回すと `position` と `velocity` が動く (駆動軸基準。回転方向の符号もここで見ておく)。

実機なしで流れを確かめるには、LiDAR とブリッジを切って、sotoba_ros の合成スキャンと
mini_shirasu_ros の模擬基板を流す:

```bash
ros2 launch oumuamua_bringup bringup.launch.py lidar:=false bridge:=false
ros2 run sotoba_ros fake_scan_publisher
for i in 0 1 2; do
  ros2 run mini_shirasu_ros fake_mini_shirasu --ros-args -r __node:=fake$i \
    -p can_id.target:=$((0x110+i)) -p can_id.status:=$((0x120+i)) \
    -p can_id.command:=$((0x130+i)) -p can_id.response:=$((0x140+i)) &
done
```

## 取得

```bash
git clone --recursive https://github.com/Stew-000-1-0-011/oumuamua.git
# 既に clone 済みなら
git submodule update --init --recursive
```

## ビルド (Podman / Docker)

ホストに ROS 2 Lyrical が無くても、コンテナ内でビルドできる。
`container/Containerfile` は `docker.io/library/ros:lyrical` (GCC 15) に
rosdep の依存と sotoba (+ Sophus 1.24.6) を入れたもので、Podman/Buildah と Docker のどちらでもビルドできる。

```bash
container/run.sh colcon build       # 初回はイメージもビルドする
container/run.sh colcon test
container/run.sh                    # 対話シェル (/ws がこのリポジトリ)
CONTAINER_ENGINE=docker container/run.sh colcon build   # 既定は podman があれば podman
```

依存 (`package.xml`) を変えたら `REBUILD=1 container/run.sh` でイメージを作り直す。

### TLS を傍受するプロキシの内側でビルドする場合

通常は不要。社内・学内プロキシが HTTPS を傍受していて `rosdep update` や
`git clone` が証明書エラーになるときだけ、プロキシの CA 証明書を渡す:

```bash
EXTRA_CA_CERT=/path/to/proxy-ca.crt container/run.sh colcon build
# 直接ビルドするなら
podman build --secret id=extra_ca,src=/path/to/proxy-ca.crt -t localhost/oumuamua:lyrical -f container/Containerfile .
```

CA はビルド中の RUN でだけ使われ、イメージには残らない。
プロキシが `127.0.0.1` など host 側にある場合は `podman build --network=host` も要る。

## C++ 規格

`colcon_defaults.yaml` で `CMAKE_CXX_STANDARD=26` を既定として渡す。
各パッケージが自分で規格を決めている場合はそちらが優先される
(rosidl が生成するメッセージの型サポートは rosidl 自身の設定で C++20)。
コンテナ外で colcon を使う場合は `COLCON_DEFAULTS_FILE=$PWD/colcon_defaults.yaml` にする。
