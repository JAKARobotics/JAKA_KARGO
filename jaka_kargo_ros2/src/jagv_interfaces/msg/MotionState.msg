# 0:idle, 1:hand_move, 2:auto_move, 3: disabled, -1: unknown
uint8 state_id
# drive_state[i] = AGV_wheel[i]; drive_state[i] == 0: disable, 1: enable
uint8[4] drive_state
# drive_error_code[i] = AGV_wheel_errcode[i]
uint16[4] drive_error_code
#bit0: estoping bit1: touching bit2：release_brake 3: safety_board_heartbeat 4: reset 5: pausing 6: stoping 7: charging
uint32 wheel_states