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

#AGV states
uint8 agv_state_id
bool agv_idle
bool agv_manual_move
bool agv_navigation
bool agv_disabled
bool agv_unknown
bool agv_all_drive_enabled
uint8[] agv_drive_state
uint16[] agv_drive_error_code
bool agv_estoping
bool agv_touching                       # 1: Collision detected
bool agv_release_brake                  # 1: Brake is released, enable is disconnected, AGV can be manually pushed, state_id = 1
bool agv_safety_board_heartbeat         # 1: SCB is in abnormal state
bool agv_reset
bool agv_pausing
bool agv_stoping
bool agv_charging