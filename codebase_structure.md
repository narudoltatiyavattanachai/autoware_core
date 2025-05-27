# Autoware Core Codebase Structure

## sensing/
├── autoware_gnss_poser/
│   ├── src/gnss_poser_node.cpp → defines `GNSSPoser` node, subscribes to `fix` (NavSatFix)
│   ├── include/autoware/gnss_poser/gnss_poser_node.hpp → declares `is_fixed()`, `publish_tf()`
│   └── launch/gnss_poser.launch.xml → configures GNSS poser parameters

├── autoware_vehicle_velocity_converter/
│   ├── src/vehicle_velocity_converter_node.cpp → converts VehicleReport to TwistWithCovarianceStamped
│   └── launch/vehicle_velocity_converter.launch.xml → configures conversion parameters

├── autoware_downsample_filters/
│   ├── src/random_downsample_filter/random_downsample_filter_node.hpp → downsamples using random selection
│   ├── src/voxel_grid_downsample_filter/voxel_grid_downsample_filter_node.hpp → voxel-based downsampling
│   └── launch/ → launch files with configurable filter parameters

├── autoware_crop_box_filter/
    └── include/autoware/crop_box_filter/crop_box_filter_node.hpp → node to clip pointcloud regions

## perception/
├── autoware_ground_filter/
│   ├── src/node.cpp → defines `GroundFilterComponent`, subscribes to pointcloud, publishes filtered cloud
│   ├── src/ground_filter.cpp → implements ground detection algorithms
│   └── launch/ground_filter.launch.xml → configures filter parameters

├── autoware_euclidean_cluster_object_detector/
│   ├── src/euclidean_cluster_node.hpp → clusters pointcloud into distinct objects
│   ├── src/voxel_grid_based_euclidean_cluster_node.hpp → optimized clustering using voxel grid
│   └── src/euclidean_cluster.cpp → implements core clustering algorithm

└── autoware_perception_objects_converter/
    └── src/ → converts between different perception object representations

## planning/
├── autoware_path_generator/
│   ├── src/node.cpp → defines `PathGenerator`, subscribes to route/map/odom, publishes path/signals
│   ├── include/autoware/path_generator/utils.hpp → defines functions like `get_turn_signal()`, `refine_goal()`
│   └── src/utils.cpp → implements path utility functions for route planning

├── autoware_mission_planner/
│   ├── src/mission_planner_node.cpp → manages high-level routing decisions
│   └── include/autoware/mission_planner/ → defines routing interfaces

├── autoware_route_handler/
│   ├── src/route_handler.cpp → implements functions for route calculation on lanelet maps
│   └── include/autoware/route_handler/ → defines route planning interfaces

├── autoware_planning_topic_converter/
│   ├── README.md → explains converter tools between planning message types
│   └── src/ → implements interfaces like `PathToTrajectory` as composable ROS nodes

├── motion_velocity_planner/
│   ├── autoware_motion_velocity_planner/
│   │   ├── src/node.hpp → declares `MotionVelocityPlannerNode` with publishers/subscribers
│   │   ├── src/node.cpp → implements velocity planning for trajectories
│   │   └── srv/ → defines LoadPlugin.srv and UnloadPlugin.srv service interfaces
│   └── autoware_motion_velocity_obstacle_stop_module/
        └── src/ → implements obstacle detection and stopping

├── behavior_velocity_planner/
    ├── autoware_behavior_velocity_planner/
    │   └── src/node.cpp → implements behavior-based velocity planning
    └── autoware_behavior_velocity_stop_line_module/
        └── src/ → implements stop line detection and handling

## localization/
├── autoware_ekf_localizer/
│   ├── src/ekf_localizer.cpp → defines `EKFLocalizer`, implements sensor fusion
│   ├── src/ekf_module.cpp → core EKF implementation for localization
│   ├── src/covariance.cpp, mahalanobis.cpp → mathematical components
│   └── launch/ → configuration files for the EKF localizer

├── autoware_gyro_odometer/
│   └── src/gyro_odometer_node.cpp → computes vehicle odometry using IMU and vehicle velocity

├── autoware_twist2accel/
│   └── src/ → calculates acceleration from velocity measurements

└── autoware_stop_filter/
    └── src/ → implements filter to detect when vehicle has stopped

## map/
├── autoware_map_loader/
│   ├── src/lanelet2_map_loader/lanelet2_map_loader_node.cpp → loads Lanelet2 HD map data
│   ├── src/pointcloud_map_loader/pointcloud_map_loader_node.cpp → loads pointcloud map data
│   └── launch/ → launch configurations for map loaders

├── autoware_map_projection_loader/
│   └── src/ → loads map projection parameters for GNSS-to-map coordinate transformation

└── autoware_lanelet2_map_visualizer/
    └── src/lanelet2_map_visualization_node.hpp → visualizes Lanelet2 map elements in RViz

## control/
├── autoware_simple_pure_pursuit/
│   ├── src/ → implements pure pursuit algorithm for steering control
│   └── include/ → defines controller interfaces and parameters

└── autoware_core_control/
    └── src/ → contains core control algorithms and interfaces

## common/
├── autoware_lanelet2_utils/
│   └── include/ → utility functions for working with Lanelet2 map data

├── autoware_component_interface_specs/
│   └── include/ → standardized interface definitions for components

├── autoware_interpolation/
│   └── include/ → interpolation functions for path/trajectory points

├── autoware_motion_utils/
│   └── include/ → utility functions for motion planning and trajectory handling

├── autoware_object_recognition_utils/
│   └── include/ → utilities for classifying and tracking objects

├── autoware_vehicle_info_utils/
│   └── include/ → utilities for accessing vehicle parameters

├── autoware_trajectory/
│   └── include/ → trajectory data structures and algorithms

└── autoware_global_parameter_loader/
    └── include/ → utilities for loading global parameters

## testing/
├── autoware_test_utils/
│   ├── test_map/ → contains test maps for unit testing (common, 2km straight, etc.)
│   └── README.md → documents available maps and usage examples

├── autoware_planning_test_manager/
│   └── src/autoware_planning_test_manager.cpp → framework for testing planning modules

└── autoware_testing/
    └── include/ → utilities for test infrastructure