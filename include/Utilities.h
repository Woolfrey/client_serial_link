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
 * @see https://github.com/Woolfrey/software_robot_library for more information on the SerialLinkBase class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include "client/CartesianTrajectoryClient.h"
#include "client/JointTrajectoryClient.h"
#include "rclcpp/rclcpp.hpp"                                                                        // ROS2 C++ library
#include "rclcpp_action/rclcpp_action.hpp"                                                          // ROS2 C++ action library
#include "RobotLibrary/SerialLinkBase.h"

/**
 * @brief Store pre-defined joint trajectories and save them in a std::map.
 * @return A std::map where the key is the trajectory name specified in the YAML file.
 */
std::map<std::string, std::vector<serial_link_action_server::msg::JointTrajectoryPoint>>
load_joint_configurations();

/**
 * @brief Store pre-defined Cartesian trajectories in a std::map
 * @return A std::map where the key is the trajectory name specified in the YAML file.
 */
std::map<std::string,std::vector<serial_link_action_server::msg::CartesianTrajectoryPoint>>
load_endpoint_poses();

/**
 * @brief This function manages the asynchronous cancellation sequence.
 * @param activeClient A pointer to the action client that is currently running.
 * @return True if/when successful, false if there was a problem.
 */
bool
stop_robot(ActionClientInterface *activeClient);

