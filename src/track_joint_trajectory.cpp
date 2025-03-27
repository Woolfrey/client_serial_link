/**
 * @file    TrackJointTrajectory.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   An action client for the TrackJointTrajectory action.
 * 
 * @details This class acts as the client implementation of the TrackJointTrajectory action
 *          defined in the serial_link_interfaces package.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_action_client/track_joint_trajectory.hpp>

namespace serial_link_action_client {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Processes the result of an action                               //                           
////////////////////////////////////////////////////////////////////////////////////////////////////
void
TrackJointTrajectory::result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            if(_verbose)
            {
                std::string performanceResults = "";
                
                int jointNum = 1;
                
                for(auto stats : result.result->position_error)
                {
                    performanceResults += "Joint " + std::to_string(jointNum) + ":\n"
                                          "   - Mean:      " + std::to_string(stats.mean) + "\n"
                                          "   - Std. dev.: " + std::to_string(sqrt(stats.variance)) + "\n"
                                          "   - Min.:      " + std::to_string(stats.min) + "\n"
                                          "   - Max.:      " + std::to_string(stats.max) + "\n";
                    
                    ++jointNum;
                }
                
                RCLCPP_INFO(_node->get_logger(),
                            "Joint trajectory tracking complete. Position error:\n%s",
                            performanceResults.c_str());
            }
            else
            {
                RCLCPP_INFO(_node->get_logger(), "Joint trajectory tracking complete.");
            }
            
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            RCLCPP_INFO(_node->get_logger(), "Joint trajectory tracking was canceled.");
            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_INFO(_node->get_logger(), "Joint trajectory tracking was aborted: %s", result.result->message.c_str());
            break;
        }
        default:
        {
            RCLCPP_WARN(_node->get_logger(), "Unknown result code.");
            break;
        }
    }
}

}
