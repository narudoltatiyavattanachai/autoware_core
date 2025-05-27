# Autoware Core API Structure

## src/sensing/
├── autoware_gnss_poser/
│   ├── src/gnss_poser_node.cpp → defines `GNSSPoser`, subscribes to `fix`, publishes `gnss_pose`
│   ├── include/autoware/gnss_poser/gnss_poser_node.hpp → declares `is_fixed()`, `publish_tf()`
│   └── launch/gnss_poser.launch.xml → configures GNSS poser

├── autoware_vehicle_velocity_converter/
│   ├── src/vehicle_velocity_converter_node.cpp → converts VehicleReport to TwistWithCovarianceStamped
│   └── launch/vehicle_velocity_converter.launch.xml → configures conversion parameters

├── autoware_downsample_filters/
│   ├── src/random_downsample_filter/random_downsample_filter_node.hpp → implements random downsampling
│   ├── src/voxel_grid_downsample_filter/voxel_grid_downsample_filter_node.hpp → voxel-based downsampling
│   └── launch/ → contains filter configuration files

├── autoware_crop_box_filter/
    └── include/autoware/crop_box_filter/crop_box_filter_node.hpp → filters pointcloud by region

## src/perception/
├── autoware_ground_filter/
│   ├── src/node.cpp → defines `GroundFilterComponent`, subscribes to pointcloud
│   ├── src/ground_filter.cpp → implements `process()`, `gridBasedFilter()`
│   └── launch/ground_filter.launch.xml → configures filter parameters

├── autoware_euclidean_cluster_object_detector/
│   ├── src/euclidean_cluster_node.hpp → defines clustering algorithm
│   ├── src/voxel_grid_based_euclidean_cluster_node.hpp → optimized clustering
│   └── src/euclidean_cluster.cpp → implements `clusterSegment()`, `filterClusters()`

└── autoware_perception_objects_converter/
    └── src/ → converts between perception object representations

## src/planning/
├── autoware_path_generator/
│   ├── src/node.cpp → defines `PathGenerator`, subscribes to `route`, publishes `path`
│   ├── include/autoware/path_generator/utils.hpp → defines `get_turn_signal()`, `refine_goal()`
│   └── src/utils.cpp → implements path generation utilities

├── autoware_mission_planner/
│   ├── src/mission_planner_node.cpp → publishes `route`, offers `clear_route` service
│   └── include/autoware/mission_planner/mission_planner.hpp → defines `plan()`, `planRoute()`

├── autoware_route_handler/
│   ├── src/route_handler.cpp → library for calculating routes on lanelet maps
│   └── include/autoware/route_handler/ → defines route interfaces

├── autoware_planning_topic_converter/
│   ├── README.md → explains converter components between planning message types
│   └── src/ → implements converters like `PathToTrajectory` as composable nodes

├── motion_velocity_planner/
│   ├── autoware_motion_velocity_planner/
│   │   ├── src/node.hpp → declares `MotionVelocityPlannerNode` with publishers and subscribers
│   │   ├── src/node.cpp → implements velocity planning with `generate_trajectory()`
│   │   └── srv/ → defines `LoadPlugin.srv` and `UnloadPlugin.srv` service interfaces
│   └── autoware_motion_velocity_obstacle_stop_module/
        └── src/ → implements obstacle detection and stopping

├── behavior_velocity_planner/
    ├── autoware_behavior_velocity_planner/
    │   └── src/node.cpp → implements behavior-based velocity planning
    └── autoware_behavior_velocity_stop_line_module/
        └── src/ → implements stop line detection and handling

## src/localization/
├── autoware_ekf_localizer/
│   ├── src/ekf_localizer.cpp → defines `EKFLocalizer`, implements sensor fusion
│   ├── src/ekf_module.cpp → implements `initCovariance()`, `updateKalmanFilter()`
│   ├── src/covariance.cpp, mahalanobis.cpp → mathematical components
│   └── launch/ → configuration for EKF localizer

├── autoware_gyro_odometer/
│   └── src/gyro_odometer_node.cpp → computes vehicle odometry with `updatePosition()`

├── autoware_twist2accel/
│   └── src/ → calculates acceleration from velocity measurements

└── autoware_stop_filter/
    └── src/ → detects when vehicle has stopped

## src/map/
├── autoware_map_loader/
│   ├── src/lanelet2_map_loader/lanelet2_map_loader_node.cpp → loads HD maps
│   ├── src/pointcloud_map_loader/pointcloud_map_loader_node.cpp → loads pointcloud maps
│   └── launch/ → launch configurations for map loaders

├── autoware_map_projection_loader/
│   └── src/ → loads map projection for GNSS-to-map transformation

└── autoware_lanelet2_map_visualizer/
    └── src/lanelet2_map_visualization_node.hpp → publishes map visualization markers

## src/control/
├── autoware_simple_pure_pursuit/
│   ├── src/ → implements pure pursuit steering algorithm
│   └── include/ → defines controller interfaces and parameters

└── autoware_core_control/
    └── src/ → contains core control interfaces and algorithms

## src/common/
├── autoware_lanelet2_utils/
│   └── include/ → defines `getClosestLanelet()`, `getConflictingLanelets()`

├── autoware_component_interface_specs/
│   └── include/ → standardized component interface definitions

├── autoware_interpolation/
│   └── include/ → interpolation functions for trajectory points

├── autoware_motion_utils/
│   └── include/ → defines `findNearestIndex()`, `calcLateralOffset()`

├── autoware_object_recognition_utils/
│   └── include/ → defines `calcBoundingBoxFromPoints()`, `classifyBySize()`

├── autoware_vehicle_info_utils/
│   └── include/ → utilities for accessing standardized vehicle parameters

├── autoware_trajectory/
│   └── include/ → defines trajectory data structures and algorithms

└── autoware_global_parameter_loader/
    └── include/ → utilities for loading global parameters

## testing/
├── autoware_test_utils/
│   ├── test_map/ → test maps (common, 2km straight, intersection, etc.)
│   └── README.md → documents test map usage

├── autoware_planning_test_manager/
│   └── src/autoware_planning_test_manager.cpp → provides `PlanningInterfaceTestManager`

└── autoware_testing/
    └── include/ → utilities for test infrastructure