# スラローム往復の検証結果

対象は入口 `[-2.05, 0.668, pi]` → 出口 `[1.25, 1.273, pi]` とその逆。
両方向とも480点、経路長4.7903 m。CSVのx・y・yawは既存と一致する。
実形状でも十分な余裕を確保できるため、既存の中央通過線と半径0.30 mの角丸を維持して再生成した。
変更の中心は、近似円から添付の正確な形状への置換と、幾何検証の強化である。

## 条件

- `config/footprints/raised.yaml` と `lowered.yaml` は添付ファイルとバイト単位で一致する。
- 確認済みの座標変換は `(x_base,y_base)=(-y_attachment,x_attachment)`。経路YAMLで+pi/2を適用する。
- 走行中は全昇降上。既存 `task_commander::slalom()` が全昇降のcarry到達を待ってから経路を実行する。
- 入口・出口は昇降下の形状も確認する。下状態でスラロームを通過するための経路ではない。
- 競技本文の壁厚38 mmを用い、既存33 mm内壁の各面を2.5 mm拡大、外周内面を5 mm内側へ移した。
- 丸め後CSVの位置/yawをC++実装と同様に線形補間する。多角形と矩形の正確な距離に、点間移動の上界を差し引く。

## 壁・棚・外周との間隔

| 対象 | 間隔 |
|---|---:|
| 往路・復路、上形状の全掃引（安全側の下限） | **123.039 mm** |
| 入口、下形状（静止） | 311.755 mm |
| 出口、下形状（静止） | 151.534 mm |
| 設定した必要最小間隔 | 40 mm |

掃引の下限は同じ位置と姿勢の逆順でも成立する。復路は180°向きを変えず同じ姿勢を維持する。
図の灰色矢印は機体の向き、緑丸→赤四角が走行順、紫線は端点での下形状を表す。

- [入口→出口](slalom_forward.png)
- [出口→入口](slalom_backward.png)

## 確認内容

- Python unittest 16件成功。原形状の変換、包含・交差、凹形状、点間移動/旋回、真円、CSV丸め、端点違反時の非上書き、両方向の余裕、task端点一致、再生成のバイト一致を確認。
- 既存のROS非依存C++ `trajectory_test.cpp` は成功。
- 実際の両CSVを `load_path_csv` / `Trajectory` で読み、各方向10,001時刻の位置・yaw・速度・角速度が有限で、終点で停止することを確認。
- 既存速度上限 (2.0 m/s、2.5 m/s²、横加速度1.5 m/s²、角速度2.5 rad/s) による理想所要時間は両方向6.5911 s。実測時間ではない。
- 全ROSビルド・実機走行は未実施（この環境にROSとサブモジュールがないため）。
- 独立コードレビュー実施済み。本体コードの重大な問題なし。丸め検査のテスト条件を補強し、丸めを外す変異で失敗すること、復元後16件すべて成功することを確認した。

## 実装時の判断と残る検証範囲

作業用ソースはGitHub連携から取得したスナップショットだったため、その専用ディレクトリにローカル作業ブランチを作った。
既存チェックアウトや共有ブランチを変更せず、上流のコミット履歴とは別の記録になる。適用用パッチは元の上流ファイルに対して作成・適用確認した。
クラウドスキルの補助スクリプトにローカル実行パスがないため、タスク境界・テスト・判断は同じ形式で手動記録した。

レビューで実測の代わりに判断しないものは、carry高さ、追従誤差、CAD計測精度、全ROS統合、他の棚経路の実機適合性。
今回の添付形状は自己交差がないことと原本とのバイト一致を別途確認した。任意の自己交差多角形の修復は生成器の対象外。
ユーザーが既存リポジトリへ適用できるようパッチと変更ファイル一式も用意した。

## 再生成・テスト（リポジトリルート）

```bash
python src/path_follower/scripts/make_path.py src/oumuamua_bringup/config/paths/slalom.route.yaml src/oumuamua_bringup/config/paths/slalom_forward.csv --plot docs/superpowers/artifacts/slalom_forward.png
python src/path_follower/scripts/make_path.py src/oumuamua_bringup/config/paths/slalom.route.yaml src/oumuamua_bringup/config/paths/slalom_backward.csv --reverse --plot docs/superpowers/artifacts/slalom_backward.png
python -m unittest discover -s src/path_follower/test -p '*_test.py' -v
g++ -std=c++23 -O2 -Isrc/path_follower/include src/path_follower/src/trajectory.cpp src/path_follower/test/trajectory_test.cpp -o /tmp/trajectory_test
/tmp/trajectory_test
```

上形状を使う実機条件として、carry時に持ち上げた機構・保持ノーツの下面が90 mmの壁より高いことを確認する。
上記間隔は理想軌道の幾何検証値であり、実機の追従誤差を含まない。
