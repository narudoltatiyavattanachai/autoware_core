# Detailed Autoware Core API Structure

## sensing/
├── autoware_gnss_poser/
│   ├── src/gnss_poser_node.cpp → defines `GNSSPoser` class
│   │   ├── subscribes: `fix` (NavSatFix), `autoware_orientation` (GnssInsOrientationStamped)
│   │   ├── publishes: `gnss_pose` (PoseStamped), `gnss_pose_cov` (PoseWithCovarianceStamped)
│   │   └── functions: `is_fixed()`, `callback_nav_sat_fix()`, `publish_tf()`
│   ├── include/autoware/gnss_poser/gnss_poser_node.hpp → core interface definitions
│   └── launch/gnss_poser.launch.xml → node configuration

├── autoware_vehicle_velocity_converter/
│   ├── src/vehicle_velocity_converter_node.cpp
│   │   ├── subscribes: `velocity_status` (VehicleReport)
│   │   └── publishes: `twist_with_covariance` (TwistWithCovarianceStamped)
│   └── launch/vehicle_velocity_converter.launch.xml
        └── params: frame_id, speed_scale_factor, velocity_stddev_xx, angular_velocity_stddev_zz

├── autoware_downsample_filters/
│   ├── src/random_downsample_filter/random_downsample_filter_node.hpp
│   │   ├── subscribes: pointcloud input
│   │   ├── publishes: downsampled pointcloud
│   │   └── params: sample_num (number of points to keep)
│   └── src/voxel_grid_downsample_filter/voxel_grid_downsample_filter_node.hpp
        ├── subscribes: pointcloud input
        └── publishes: voxel-downsampled pointcloud

## perception/
├── autoware_ground_filter/
│   ├── src/node.cpp → defines `GroundFilterComponent`
│   │   ├── subscribes: `input` (PointCloud2)
│   │   ├── publishes: `output` (PointCloud2)
│   │   └── functions: `faster_filter()`, `classifyPointCloud()`, `convertPointcloud()`
│   ├── src/ground_filter.cpp → core ground detection algorithm
│   │   └── functions: `process()`, `gridBasedFilter()`, `recheckGroundCluster()`
│   └── launch/ground_filter.launch.xml → configures filter parameters
        └── params: elevation_grid_mode, radial_divider_angle_deg, grid_size_m

├── autoware_euclidean_cluster_object_detector/
│   ├── src/euclidean_cluster_node.hpp → clustering node
│   │   ├── subscribes: pointcloud input
│   │   ├── publishes: detected object clusters
│   │   └── functions: `detectClusters()`, `cluster()`
│   └── src/euclidean_cluster.cpp → clustering implementation
        └── functions: `clusterSegment()`, `filterClusters()`

## planning/
├── autoware_path_generator/
│   ├── src/node.cpp → defines `PathGenerator` node
│   │   ├── subscribes: `~/input/route`, `~/input/vector_map`, `~/input/odometry`
│   │   ├── publishes: `~/output/path`, `~/output/turn_indicators_cmd`
│   │   └── functions: `generate_path()`, `plan_path()`, `update_current_lanelet()`
│   ├── include/autoware/path_generator/utils.hpp → utility interfaces
│   │   └── functions: `get_turn_signal()`, `refine_goal()`, `get_path_bounds()`
│   └── src/utils.cpp → path utilities implementation

├── autoware_mission_planner/
│   ├── src/mission_planner_node.cpp
│   │   ├── subscribes: `~/input/vector_map`, `~/input/modified_goal`
│   │   ├── publishes: `~/output/route`
│   │   └── services: `~/input/clear_route`
│   └── include/autoware/mission_planner/mission_planner.hpp
        └── functions: `plan()`, `planRoute()`, `planPath()`

├── motion_velocity_planner/
│   ├── autoware_motion_velocity_planner/
│   │   ├── src/node.hpp → declares `MotionVelocityPlannerNode`
│   │   │   ├── subscribes: trajectory input, dynamic objects, pointcloud, traffic signals
│   │   │   ├── publishes: modified trajectory with velocity, velocity limits
│   │   │   └── services: `LoadPlugin`, `UnloadPlugin` for modular velocity planning
│   │   ├── src/node.cpp → velocity planning implementation
│   │   │   └── functions: `update_planner_data()`, `generate_trajectory()`, `smooth_trajectory()`
│   │   └── src/planner_manager.cpp → manages velocity planning modules
│   │       └── functions: `generateTrajectory()`, `registerPlugin()`
│   └── autoware_motion_velocity_obstacle_stop_module/
        └── src/ → obstacle detection and velocity planning for safety

## localization/
├── autoware_ekf_localizer/
│   ├── src/ekf_localizer.cpp → defines `EKFLocalizer` node
│   │   ├── subscribes: `~/input/odometry`, `~/input/imu`, `~/input/twist_with_covariance`
│   │   ├── publishes: `~/output/odometry`, `~/output/twist_with_covariance`
│   │   └── functions: `predict()`, `measurementUpdate()`, `updateKalmanFilter()`
│   ├── src/ekf_module.cpp → core EKF implementation
│   └── src/covariance.cpp → covariance matrix management
        └── functions: `initCovariance()`, `createProcessNoiseCovariance()`

├── autoware_gyro_odometer/
│   └── src/gyro_odometer_node.cpp → computes odometry
        ├── subscribes: `~/input/twist`, `~/input/imu`
        ├── publishes: `~/output/odometry`
        └── functions: `gyroOdometryCallback()`, `updatePosition()`

## map/
├── autoware_map_loader/
│   ├── src/lanelet2_map_loader/lanelet2_map_loader_node.cpp
│   │   ├── subscribes: `~/input/map_projector_info`
│   │   ├── publishes: `~/output/lanelet2_map` (LaneletMapBin)
│   │   └── functions: `load_map()`, `create_map_bin_msg()`
│   └── src/pointcloud_map_loader/pointcloud_map_loader_node.cpp
        ├── publishes: `~/output/pointcloud_map`, `~/output/pointcloud_map_metadata`
        └── functions: `load()`, `publish()`, `loadPCD()`

├── autoware_lanelet2_map_visualizer/
    └── src/lanelet2_map_visualization_node.hpp
        ├── subscribes: `~/input/lanelet2_map` (LaneletMapBin)
        ├── publishes: visualization markers for RViz
        └── functions: `createVisualization()`, `visualize_regulatory_elements()`

## common/
├── autoware_lanelet2_utils/
│   └── include/autoware/lanelet2_utils/
        └── functions: `getClosestLanelet()`, `getConflictingLanelets()`, `lineStringWithWidthToPolygon()`

├── autoware_motion_utils/
│   └── include/autoware/motion_utils/
        └── functions: `findNearestIndex()`, `calcLateralOffset()`, `resampleTrajectory()`

├── autoware_object_recognition_utils/
    └── include/autoware/object_recognition_utils/
        └── functions: `calcBoundingBoxFromPoints()`, `classifyBySize()`, `filterObjectsByClassification()`

## testing/
├── autoware_test_utils/
│   ├── test_map/ → provides standard test maps
│   │   ├── common/ → general testing map with multiple lane types
│   │   ├── 2km_test/ → straight road test map
│   │   └── road_shoulders/ → map for testing shoulder detection
│   └── include/autoware/test_utils/
        └── functions: `loadMapFromFile()`, `createTestTrajectory()`

└── autoware_planning_test_manager/
    └── src/autoware_planning_test_manager.cpp
        └── class: `PlanningInterfaceTestManager` for testing planning modules