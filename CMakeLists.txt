cmake_minimum_required(VERSION 3.14)
project(oa_licalib)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

#SET(CXX_MARCH native)
#set(CMAKE_CXX_FLAGS "-ftemplate-backtrace-limit=0  -march=${CXX_MARCH} -O3 -DBASALT_USE_CHOLMOD -DBASALT_DISABLE_ASSERTS -DNDEBUG")
#set(CMAKE_CXX_FLAGS " -march=${CXX_MARCH} -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")
#add_definitions(-O3 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})
#set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen")
set(CERES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-ceres-solver")

find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
set(STD_CXX_FS stdc++fs)

find_package(Ceres REQUIRED)
#include_directories(${CERES_INCLUDE_DIRS})
include_directories(${CERES_INCLUDES})

message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Found ceres headers in: ${CERES_INCLUDE_DIRS}")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(velodyne_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
#find_package(eigen_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
#find_package(tf_conversions REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
#find_package(roslib REQUIRED)
find_package(Boost REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(oa_licalib_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})

set(PANGOLIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-pangolin")
find_package(Pangolin REQUIRED)


include_directories(
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/thirdparty/
        sensor_msgs
        std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${PCL_LIBRARIES}
        ${Pangolin_INCLUDE_DIRS}
)

include_directories(thirdparty/Sophus)

# Set link libraries used by all binaries
set(thirdparty_libraries
        Boost
        PCL
        OpenCV
        ${YAML_CPP_LIBRARIES}
        Pangolin
        Ceres
        )

################ Library #######################
#ament_auto_add_library(lib_calib
#        "src/trajectory/se3_trajectory.cpp"
#        "src/trajectory/trajectory_estimator.cpp"
#        "src/trajectory/trajectory_viewer.cpp"
#        "src/calib/lidar_localization.cpp"
#        "src/calib/lidar_ndt_odometry.cpp"
#        "src/calib/surfel_association.cpp"
#        "src/calib/inertial_initializer.cpp"
#        "src/calib/scan_undistortion.cpp"
#        )


set (USED_LIBRARIES
        rclcpp
        velodyne_msgs
        angles
        rclpy
        geometry_msgs
        nav_msgs
        pcl_conversions
        pcl_ros
        PCL
        sensor_msgs
        std_msgs
        tf2
        ndt_omp_ros2
        tf2_eigen
        rosbag2
        ${thirdparty_libraries}
        oa_licalib_msgs
        )
add_executable(lib_calib src/trajectory/se3_trajectory.cpp src/trajectory/trajectory_estimator.cpp src/trajectory/trajectory_viewer.cpp src/calib/lidar_localization.cpp src/calib/lidar_ndt_odometry.cpp src/calib/surfel_association.cpp src/calib/inertial_initializer.cpp src/calib/scan_undistortion.cpp)
ament_target_dependencies(lib_calib ${USED_LIBRARIES})
target_link_libraries(lib_calib ${USED_LIBRARIES})
target_compile_options(lib_calib PRIVATE -Wall -Wextra)
#add_dependencies(lib_calib  ${PROJECT_NAME})
################# Node ###################

add_executable(li_calib_node src/app/li_calib_node.cpp src/calib/calib_helper.cpp)
ament_target_dependencies(li_calib_node ${USED_LIBRARIES} )
target_link_libraries(li_calib_node ${thirdparty_libraries})
target_compile_options(li_calib_node PRIVATE -Wall -Wextra)

#add_dependencies(li_calib_node  ${PROJECT_NAME})

add_executable(map_evaluation_tools src/app/map_evaluation_tools.cpp)
ament_target_dependencies(map_evaluation_tools ${USED_LIBRARIES} )
target_link_libraries(map_evaluation_tools ${thirdparty_libraries})
target_compile_options(map_evaluation_tools PRIVATE -Wall -Wextra)

ament_export_dependencies(
        rclcpp
        rclpy
        geometry_msgs
        nav_msgs
        pcl_conversions
        pcl_ros
        PCL
        sensor_msgs
        std_msgs
        tf2
        ndt_omp_ros2
        tf2_eigen
)

ament_export_include_directories(
        include
)
ament_package()