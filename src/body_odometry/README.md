# body_odometry

機体速度 (`geometry_msgs/TwistStamped`、機体座標系) を積分して、オドメトリと TF `odom -> base_link` を出す。
AMCL や Nav2 のように odom フレームを前提にするものと組み合わせる用。
oumuamua の既定の構成 (sotoba_node + state_estimator) では使わない。

| 方向 | 名前 | 型 |
| --- | --- | --- |
| sub | `~/body_velocity` | `geometry_msgs/TwistStamped` (chassis_node や robot_sim の `~/body_velocity` を remap する) |
| pub | `~/odom` | `nav_msgs/Odometry` (`odom_frame` から見た `base_frame`、速度は機体座標系) |
| TF | `odom_frame -> base_frame` | `publish_tf: true` のとき |

| パラメータ | 既定 | 意味 |
| --- | --- | --- |
| `odom_frame` / `base_frame` | `odom` / `base_link` | フレーム名 |
| `publish_tf` | `true` | TF を出すか |
| `pose_variance` / `twist_variance` | `1e-4` / `1e-3` | メッセージに付ける共分散 (対角) |
| `max_dt` | `0.5` | メッセージの間隔がこれより空いたら、その区間は積分しない [s] |

姿勢は、前のメッセージの速度で区間を、中点の向きを使って積分する。時刻はメッセージの stamp。
滑りは知らないので、時間とともにずれていく (それを地図との照合で直すのが AMCL などの仕事)。
