include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                                --地图坐标系名字，运行cartographer，会自动把map加到tf的头部位置
  tracking_frame = "base_link",                     --将所有传感器数据转换到这个坐标系下，如果有imu，最好设置成imulink，因为imu数据的频率高
  -- base_link改为odom,发布map到odom之间的位姿态
  published_frame = "odom",                         --设置为tf树最顶层的坐标系名字
  odom_frame = "odom",
  provide_odom_frame = false,                       -- true改为false，不用提供里程计数据，tf树为map->footprint_link
                                                    -- 如果为true，则tf树为map->odom->footprint_link
  publish_frame_projected_to_2d = true,             -- false改为true，发布2D位姿
  use_odometry = true,                              -- false改为true，使用里程计数据，如果没有launch中没有remap会订阅/odom话题，
  use_nav_sat = false,                              -- 是否使用GPS数据
  use_landmarks = false,                            -- 是否使用landmark数据
  num_laser_scans = 1,                              -- 0改为1,使用一个雷达
  num_multi_echo_laser_scans = 0,                   -- 1改为0，不使用多波雷达
  num_subdivisions_per_laser_scan = 1,              -- 一帧数据分成几个子帧处理，1就是不分
  num_point_clouds = 0,                             -- 是否使用点云数据
  lookup_transform_timeout_sec = 0.2,               -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,                  -- 发布submap的时间间隔
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,            -- 发布轨迹的时间间隔
  rangefinder_sampling_ratio = 1.,                  -- 传感器数据的采样频率-雷达
  odometry_sampling_ratio = 1.,                     -- odom
  fixed_frame_pose_sampling_ratio = 1.,             -- gps
  imu_sampling_ratio = 1.,                          -- imu 这里1是每来一帧数据都会用，0.5就是每两帧数据用一次                   
  landmarks_sampling_ratio = 1.,
}


-- false改为true，启动2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- 0改成0.10,比机器人半径小的都忽略
TRAJECTORY_BUILDER_2D.min_range = 0.10
-- 30改成3.5,限制在雷达最大扫描范围内，越小一般越精确些
TRAJECTORY_BUILDER_2D.max_range = 30
-- 5改成3,传感器数据超出有效范围最大值
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- true改成false,不使用IMU数据，大家可以开启，然后对比下效果
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- false改成true,使用实时回环检测来进行前端的扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- 1.0改成0.1,提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 0.55改成0.65,Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.min_score = 0.75
--0.6改成0.7,全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10

-- 设置0可关闭全局SLAM
-- POSE_GRAPH.optimize_every_n_nodes = 0

return options
