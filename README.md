# oumuamua

ROS 2 Lyrical Luth (Ubuntu 26.04) ワークスペース。`oumuamua_bringup` 以外のパッケージは `src/` 以下の git submodule。

| パッケージ | 内容 |
| --- | --- |
| `src/sotoba_ros` | sotoba の ICP による LiDAR 物体姿勢推定 |
| `src/urg_node2` | 北陽 URG LiDAR ドライバ |
| `src/holonomic_tracker` | 全方向移動ロボットの追従制御 |
| `src/omni_chassis` | 足回りドライバ。`cmd_vel` を N 輪オムニの逆運動学で車輪ごとのモータ目標角速度にする |
| `src/mini_shirasu_ros` | mini-shirasu (ブラシ付き DC モータドライバ) と CAN で話す |
| `src/robomas_plugins` | USB-CAN ブリッジ (crs-kouhou/robomas_plugins のフォーク)。`robomas_can_tx` / `robomas_can_rx` だけを使う |
| `src/oumuamua_bringup` | 上をまとめて起動する launch と、この機体のパラメータ |

## つながり

`/scan` (urg_node2) -> sotoba_node -> TF `field -> base_link` -> tracker_node -> `/cmd_vel`
-> chassis_node -> `/wheel<i>/target_velocity` -> mini_shirasu_node (`wheel0..2`)
-> `/robomas_can_tx` -> robomas_bridge (USB-CAN) -> mini-shirasu 3 枚 (3 輪オムニ)

tracker_node に目標 (`/tracker_node/reference`, `holonomic_tracker/msg/TrackingReference`)
を出すノードはまだ無い。

## 起動

```bash
ros2 launch oumuamua_bringup bringup.launch.py
```

| 引数 | 既定 | 意味 |
| --- | --- | --- |
| `lidar` | `true` | urg_node2 を起動する (configure -> activate まで自動) |
| `lidar_connection` | `ether` | `ether` / `serial`。`config/urg_<...>.yaml` を使う |
| `lidar_x`, `lidar_y`, `lidar_yaw` | `0.0` | base_link から見た LiDAR の取付 (static TF `base_link -> laser`) |
| `lidar_z` | `0.14` | 走査面の高さ [m]。sotoba_node の `lidar_height` にも渡す |
| `lidar_upside_down` | `true` | 逆さ付けか。TF の roll と sotoba_node の同名パラメータに渡す |
| `chassis_params` | `config/chassis_node.yaml` | chassis_node のパラメータ |
| `wheel_count` | `3` | mini_shirasu_node (`wheel0..`) の数。`chassis_node.yaml` の車輪数と合わせる |
| `mini_shirasu_params` | `config/mini_shirasu.yaml` | mini_shirasu_node (`wheel0..`) のパラメータ |
| `bridge` | `true` | robomas_bridge (USB-CAN) を起動する |
| `rviz` | `false` | RViz2 を起動する |

次は**例・仮の値のまま**なので、実機に合わせること。

- `src/oumuamua_bringup/config/chassis_node.yaml`: 3 輪オムニ (機体正面 = 中心から車輪 0 の向き、
  車輪 0..2 は左回りに 120 度ずつ、中心から 0.2 m、半径 0.06 m)。各車輪の正転の向き (`gear_ratio` の符号) は未確認
- `src/oumuamua_bringup/config/mini_shirasu.yaml`: 基板ごとの CAN ID。ファームの ID は
  `minishirasu-firm/src/config.rs` のコンパイル時定数 なので、
  複数枚つなぐなら基板ごとに書き換えて焼き、ここと合わせる。今は基板 i に 目標 0x110+i / 状態 0x120+i / コマンド 0x130+i / 応答 0x140+i を割り当てている

モータドライバの設定 (`settings.*`) は mini-shirasu2 の `bench.rs` で実機が動いた値で、
速度・位置は駆動軸 (ホイール) 基準。そのため chassis_node の `gear_ratio` は ±1、
`max_wheel_speed` はドライバの `wmax` (20 rad/s) と揃えてある。

mini-shirasu のファームには通信タイムアウトが無いので、PC やノードが落ちたときに止めるのは
緊急停止スイッチに頼ることになる。
sotoba_node の初期姿勢 (`start_x` など) は
LiDAR の姿勢である点に注意 (base_link ではない)。

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
