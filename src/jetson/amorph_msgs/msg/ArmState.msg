
std_msgs/Header header

float32 imu_yaw
float32 imu_pitch
float32 imu_roll

float32 imu_yaw_target

geometry_msgs/Pose target_end_effector_pose

float32 base_yaw_encoder
float32 base_yaw_encoder_speed
float32 yaw_encoder
float32 yaw_encoder_speed
float32 pitch_1_encoder
float32 pitch_1_encoder_speed
float32 pitch_2_encoder
float32 pitch_2_encoder_speed
float32 end_yaw_1_encoder
float32 end_yaw_1_encoder_speed
float32 end_pitch_encoder
float32 end_pitch_encoder_speed
float32 end_yaw_2_encoder
float32 end_yaw_2_encoder_speed

uint8 FSM_STATE_UNKNOWN = 0
uint8 FSM_STATE_IDLE = 1
uint8 FSM_STATE_MANUAL = 2
uint8 FSM_STATE_EXEC = 3
uint8 FSM_STATE_NOFORCE = 4
uint8 FSM_STATE_RETRACTED = 5
uint8 fsm_state

uint8 ARM_PUMP_STATE_OFF = 0
uint8 ARM_PUMP_STATE_SUCK = 1
uint8 ARM_PUMP_STATE_BLOW = 2
uint8 arm_pump

bool left_storage
bool right_storage

##### 以下是调试信息，不用于实际控制 #####

amorph_msgs/Pid[14] pids
geometry_msgs/Pose custom_controller_pose
bool last_ik_success
