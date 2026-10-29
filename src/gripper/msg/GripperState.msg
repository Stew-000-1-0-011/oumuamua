# 把持機構 (昇降 + 吸引) 1 つぶんの状態
std_msgs/Header header

float64 lift_height        # [m] 今の高さ (lift_feedback が false なら指令値)
float64 lift_command       # [m] 今送っている位置指令
float64 lift_goal          # [m] 最後に受け付けたゴール
int32 lift_position        # lift_goal が lift.positions の何番目か (~/set_lift_position で決めたとき)。それ以外は -1
bool lift_moving           # ゴールへ向かっている途中か
bool lift_arrived          # 最後のゴールに着いたか (動いている途中・時間切れ・止めたときは false)
bool lift_feedback         # ロボマスのフィードバック (CAN 0x200 + モータ番号) が届いているか

bool suction_on
float64 suction_target     # mini-shirasu に送っている目標 (電流 [A] かデューティ。suction.target_topic の種類による)
