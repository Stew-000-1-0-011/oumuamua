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

## ビルド (Docker / Podman)

ホストに ROS 2 Lyrical が無くても、コンテナ内でビルドできる。
イメージは `ros:lyrical` (GCC 15) に rosdep の依存と sotoba (+ Sophus 1.24.6) を入れたもの。

```bash
docker/run.sh colcon build          # 初回はイメージもビルドする
docker/run.sh colcon test
docker/run.sh                       # 対話シェル (/ws がこのリポジトリ)
CONTAINER_ENGINE=podman docker/run.sh colcon build
```

依存 (`package.xml`) を変えたらイメージを作り直す:
`docker build -t oumuamua:lyrical -f docker/Dockerfile .`

## C++ 規格

ワークスペース全体を **C++26** でビルドする (`colcon_defaults.yaml`)。
`CMAKE_CXX_STANDARD` を固定で書いているパッケージがあっても、
`cmake/force_cxx_standard.cmake` を `CMAKE_PROJECT_INCLUDE` で差し込み、
全ターゲットの `CXX_STANDARD` を `OUMUAMUA_CXX_STANDARD` (=26) に揃える。
rosidl が生成する型サポートの C コードは対象外。

コンテナ外で colcon を使う場合は `COLCON_DEFAULTS_FILE=$PWD/colcon_defaults.yaml` にし、
`CMAKE_PROJECT_INCLUDE` のパスを書き換えること。
