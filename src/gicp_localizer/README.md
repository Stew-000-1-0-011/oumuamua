# gicp_localizer

2D LiDAR のスキャンを、フィールドの点群地図に [small_gicp](https://github.com/koide3/small_gicp) (GICP) で
位置合わせして自己位置を出す。oumuamua_bringup の `localization:=gicp` で立つ。

- 地図: `map_file` の PCD (oumuamua_sim の `make_field_map` が作る、走査面上の壁の点)
- 初期値: スキャンの時刻の TF `field -> base_link`。無ければ、自分が最後に求めた `field -> odom` と
  その時刻の `odom -> base_link` (body_odometry) から作る
- 出力: `~/pose` (`PoseWithCovarianceStamped`、スキャンの時刻)。`publish_tf` なら TF `field -> odom` も出す (AMCL と同じ形)

## やっていること

- 2D の点は直線上に並ぶので、そのままでは GICP の共分散で「壁の法線」と「z」の区別がつかない。
  地図とスキャンの点を z = ±`layer_offset` にも複製して、近傍が鉛直な面になるようにする
- z・roll・pitch は small_gicp の `RestrictDoFFactor` で固定し、x・y・yaw だけを解く
- 粗 (`max_correspondence_distance_coarse`) -> 細 (`max_correspondence_distance`) の 2 段で合わせる。
  粗い段の距離が広いと、モデルに無い物 (棚など) の点を近くの壁に引き寄せてずれる (0.5 m で数 cm ずれた)
- 対応の取れた点の割合 (`min_inlier_ratio`) や、初期値からの動き (`max_jump_*`) が大きすぎる結果は使わない
- 共分散は small_gicp の H (情報行列) の逆を、残差の大きさで目盛り直したもの

パラメータは `config/gicp_localizer.yaml`。
