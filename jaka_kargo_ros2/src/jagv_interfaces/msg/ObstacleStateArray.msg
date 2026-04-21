# 多传感器障碍物状态汇总
# 固定索引: 0=dual_laser, 1=front_depth_camera, 2=back_depth_camera, 3=lidar_3d
string[] sensor_sources
bool[] has_obstacles
float64[] distances
geometry_msgs/Point[] positions
