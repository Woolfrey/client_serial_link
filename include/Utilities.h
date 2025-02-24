/**
 * @file    Utilities.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Useful functions for use in action clients.
 * 
 * @details This header file contains forward declarations of functions that are useful in action clients.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include "TrackCartesianTrajectory.h"
#include "TrackJointTrajectory.h"
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ library
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 C++ action library
#include "serial_link_interfaces/msg/joint_trajectory_point.hpp"
#include "serial_link_interfaces/msg/cartesian_trajectory_point.hpp"

/**
 * @brief Get the tolerances on the trajectory tracking error
 * @return A std::vector<double> object of tolerances.
 */
std::vector<double>
load_joint_tolerances();

/**
 * @brief Store pre-defined joint trajectories and save them in a std::map.
 * @return A std::map where the key is the trajectory name specified in the YAML file.
 */
std::map<std::string, std::vector<serial_link_interfaces::msg::JointTrajectoryPoint>>
load_joint_configurations();

/**
 * @brief Store pre-defined Cartesian trajectories in a std::map
 * @return A std::map where the key is the trajectory name specified in the YAML file.
 */
std::map<std::string,std::vector<serial_link_interfaces::msg::CartesianTrajectoryPoint>>
load_endpoint_poses();

/**
 * @brief This function manages the asynchronous cancellation sequence.
 * @param activeClient A pointer to the action client that is currently running.
 * @return True if/when successful, false if there was a problem.
 */
bool
stop_robot(ActionClientInterface *activeClient);

