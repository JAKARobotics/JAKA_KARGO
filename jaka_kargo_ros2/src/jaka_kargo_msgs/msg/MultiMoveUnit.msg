int32 motion_unit_type     # 0=robot, 1=ext
int32 motion_unit_id       # robot_id or ext_id
int32 move_type            # 0=JOINT_MOVE, 1=LINEAR_MOVE, 2=CIRCULAR_MOVE
int32 move_mode            # 0=ABS, 1=INCR, 2=CONTINUE, 3=STOP

# JOINT_MOVE
float64[] movej_end_pos    # robot: full joint list, ext: use first value only
float64 movej_j_vel
float64 movej_j_acc
float64 movej_j_jerk
float64 movej_blend_tol

# LINEAR_MOVE  [x,y,z,rx,ry,rz]
float64[6] movel_end_pos
float64 movel_vel
float64 movel_acc
float64 movel_jerk
float64 movel_ori_vel
float64 movel_ori_acc
float64 movel_ori_jerk
float64 movel_blend_tol

# CIRCULAR_MOVE  [x,y,z,rx,ry,rz]
float64[6] movec_mid_pos
float64[6] movec_end_pos
float64 movec_vel
float64 movec_acc
float64 movec_jerk
float64 movec_ori_vel
float64 movec_ori_acc
float64 movec_ori_jerk
float64 movec_blend_tol
float64 movec_circle_cnt
int32 movec_circle_mode