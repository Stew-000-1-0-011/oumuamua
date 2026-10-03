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
-> chassis_node -> `/wheel<i>/target_velocity` -> mini_shirasu_node (`wheel0..3`)
-> `/robomas_can_tx` -> robomas_bridge (USB-CAN) -> mini-shirasu 4 枚

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
| `mini_shirasu_params` | `config/mini_shirasu.yaml` | mini_shirasu_node (`wheel0..3`) のパラメータ |
| `bridge` | `true` | robomas_bridge (USB-CAN) を起動する |
| `rviz` | `false` | RViz2 を起動する |

次は**例・仮の値のまま**なので、実機に合わせること。

- `src/oumuamua_bringup/config/chassis_node.yaml`: 車輪配置・減速比
- `src/oumuamua_bringup/config/mini_shirasu.yaml`: モータドライバの設定 (`settings.*`) と、
  基板ごとの CAN ID (ファームの `config.rs` と合わせる。今は基板 i に 0x100+2i などを割り当てている)
sotoba_node の初期姿勢 (`start_x` など) は
LiDAR の姿勢である点に注意 (base_link ではない)。

実機なしで流れを確かめるには、LiDAR とブリッジを切って、sotoba_ros の合成スキャンと
mini_shirasu_ros の模擬基板を流す:

```bash
ros2 launch oumuamua_bringup bringup.launch.py lidar:=false bridge:=false
ros2 run sotoba_ros fake_scan_publisher
for i in 0 1 2 3; do
  ros2 run mini_shirasu_ros fake_mini_shirasu --ros-args -r __node:=fake$i \
    -p can_id.target:=$((0x100+2*i)) -p can_id.status:=$((0x101+2*i)) \
    -p can_id.command:=$((0x200+2*i)) -p can_id.response:=$((0x201+2*i)) &
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
