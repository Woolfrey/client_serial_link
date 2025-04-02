cmake_minimum_required(VERSION 3.8)
project(serial_link_action_client)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                       List build dependencies                                    #
####################################################################################################

# ROS2 packages
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_link_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Location of header files:
include_directories(include)

####################################################################################################
#                                            Executables                                           #
####################################################################################################

# Trajectory tracking
add_executable(trajectory_tracking_client src/nodes/trajectory_tracking_client.cpp
                                          src/track_cartesian_trajectory.cpp
                                          src/track_joint_trajectory.cpp
                                          src/utilities.cpp)
                                          
ament_target_dependencies(trajectory_tracking_client "action_msgs"
                                                     "ament_cmake"
                                                     "rclcpp"
                                                     "rclcpp_action"
                                                     "serial_link_interfaces"
                                                     "std_msgs")
                                                     
# Follow transform
add_executable(follow_transform_client src/nodes/follow_transform_client.cpp
                                       src/follow_transform.cpp
                                       src/track_joint_trajectory.cpp
                                       src/utilities.cpp)
                                          
ament_target_dependencies(follow_transform_client "action_msgs"
                                                  "ament_cmake"
                                                  "rclcpp"
                                                  "rclcpp_action"
                                                  "serial_link_interfaces"
                                                  "std_msgs")
                                              
# Follow twist
add_executable(follow_twist_client src/nodes/follow_twist_client.cpp
                                   src/follow_twist.cpp
                                   src/track_joint_trajectory.cpp
                                   src/utilities.cpp)
                                          
ament_target_dependencies(follow_twist_client "action_msgs"
                                              "ament_cmake"
                                              "rclcpp"
                                              "rclcpp_action"
                                              "serial_link_interfaces"
                                              "std_msgs")
                                              
                                              

####################################################################################################
#                                 Install files so ROS2 can find them                              #
####################################################################################################

install(TARGETS follow_transform_client
                follow_twist_client
                trajectory_tracking_client
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
