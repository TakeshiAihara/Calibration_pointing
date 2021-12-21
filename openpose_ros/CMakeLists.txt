cmake_minimum_required(VERSION 3.12.0)
project(openpose_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++14)
add_definitions(${PCL_DEFINITIONS})
find_package(catkin REQUIRED COMPONENTS
  openpose_ros_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  message_generation
  cv_bridge
  std_msgs
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

#find_package(CUDA REQUIRED) # Uncomment this line if you have trouble compiling
find_package(OpenCV REQUIRED)
find_package(GFlags)
find_package(Glog)
find_package(OpenPose REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${CAFFE_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/openpose_ros_node.cpp src/openpose.cpp src/openpose_ros_io.cpp
                                    src/openpose_flags.cpp)
add_executable(detect_poses src/detect_poses.cpp)
add_executable(tf_poses_publisher src/tf_publisher.cpp)
add_executable(tabletop_trash src/tabletop_trash.cpp)
add_executable(calibration_pointing src/calibration_pointing.cpp)
#add_executable(calibrate_action src/calibrate_action.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenPose_LIBS}
  ${GFLAGS_LIBRARY}
  ${GLOG_LIBRARY}
)
target_link_libraries(detect_poses ${catkin_LIBRARIES})
target_link_libraries(tf_poses_publisher ${catkin_LIBRARIES})
target_link_libraries(tabletop_trash ${catkin_LIBRARIES})
target_link_libraries(calibration_pointing ${catkin_LIBRARIES})
#target_link_libraries(calibrate_action ${catkin_LIBRARIES})
