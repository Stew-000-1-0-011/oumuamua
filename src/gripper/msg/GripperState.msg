# 把持機構 (昇降 + 吸引) 1 つぶんの状態
std_msgs/Header header

float64 lift_height        # [m] 今の高さ (lift_feedback が false なら指令値)
float64 lift_command       # [m] 今送っている位置指令
float64 lift_goal          # [m] 最後に受け付けたゴール
bool lift_moving           # ゴールへ向かっている途中か
bool lift_feedback         # ロボマスのフィードバック (CAN 0x200 + モータ番号) が届いているか

bool suction_on
float64 suction_current    # [A] mini-shirasu に送っている電流の目標
