# Robot arms states
bool power_state
bool servo_state
bool estoped
bool collision_state
bool  inpos_left
bool  inpos_right
bool  drag_left
bool  drag_right
bool  inerror_left
bool  inerror_right

# External-axis states
bool[] ext_is_powered
bool[] ext_is_powering
bool[] ext_is_enabled
bool[] ext_is_enabling
bool[] ext_is_inpos
bool[] ext_is_on_limit

# AGV states
int32 agv_motion_state
int32 agv_wheel_state

bool agv_enabled

bool agv_idle
bool agv_manual_move
bool agv_navigation
bool agv_disabled
bool agv_obstacle_avoidance
bool agv_unknown

bool agv_wheel_idle
bool agv_estop
bool agv_collision               # 1: Collision detected (touching edge)
bool agv_release_brake          # 1: Brake is released, AGV can be manually pushed
bool agv_reset
bool agv_pausing
bool agv_stoping
bool agv_charging
bool agv_moving
bool agv_soft_pause
bool agv_soft_resume
bool agv_soft_estop
bool agv_wheel_unknown