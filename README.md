# oumuamua

ROS 2 Lyrical Luth (Ubuntu 26.04) ワークスペース。各パッケージは `src/` 以下の git submodule。

| パッケージ | 内容 |
| --- | --- |
| `src/sotoba_ros` | sotoba の ICP による LiDAR 物体姿勢推定 |
| `src/urg_node2` | 北陽 URG LiDAR ドライバ |
| `src/holonomic_tracker` | 全方向移動ロボットの追従制御 |
| `src/robomas_plugins` | RoboMaster モータ用ブリッジ (crs-kouhou/robomas_plugins のフォーク) |

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
