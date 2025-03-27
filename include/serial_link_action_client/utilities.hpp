/**
 * @file    utilities.hpp
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

#include <serial_link_action_client/track_cartesian_trajectory.hpp>
#include <serial_link_action_client/track_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ library
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 C++ action library
#include <serial_link_interfaces/msg/joint_trajectory_point.hpp>
#include <serial_link_interfaces/msg/cartesian_trajectory_point.hpp>

namespace serial_link_action_client {

/**
 * @brief Get the tolerances on joint position error in joint feedback control.
 * @return A std::vector<double> object of tolerances.
 */
std::vector<double>
load_joint_error_tolerances(const std::shared_ptr<rclcpp::Node> &node);

/**
 * @brief Get the tolerances for the position & orientation error in Cartesian feedback control.
 * @return A std::array<double,2> containing the maximum (norm) of the position error, and (norm) of the orientation error.
 */
std::array<double,2>
load_pose_error_tolerances(const std::shared_ptr<rclcpp::Node> &node);

/**
 * @brief Store pre-defined joint trajectories and save them in a std::map.
 * @return A std::map where the key is the trajectory name specified in the YAML file.
 */
std::map<std::string, std::vector<serial_link_interfaces::msg::JointTrajectoryPoint>>
load_joint_configurations(const std::shared_ptr<rclcpp::Node> &node);

/**
 * @brief Store pre-defined Cartesian trajectories in a std::map
 * @return A std::map where the key is the trajectory name specified in the YAML file.
 */
std::map<std::string,std::vector<serial_link_interfaces::msg::CartesianTrajectoryPoint>>
load_endpoint_poses(const std::shared_ptr<rclcpp::Node> &node);

/**
 * @brief This function manages the asynchronous cancellation sequence.
 * @param activeClient A pointer to the action client that is currently running.
 * @return True if/when successful, false if there was a problem.
 */
bool
stop_robot(std::shared_ptr<ActionClientInterface> activeClient);

}
