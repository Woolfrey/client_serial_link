/**
 * @file    TrackCartesianTrajectory.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   An action client for the TrackCartesianTrajectory action.
 * 
 * @details This class acts as the client implementation of the TrackCartesianTrajectory action
 *          defined in the serial_link_interfaces package.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_action_client/track_cartesian_trajectory.hpp>

namespace serial_link_action_client {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //                           
////////////////////////////////////////////////////////////////////////////////////////////////////
TrackCartesianTrajectory::TrackCartesianTrajectory(std::shared_ptr<rclcpp::Node> clientNode,
                          const std::string &actionName,
                          bool verbose)
: ActionClientBase(clientNode, actionName),
  _verbose(verbose)
{
    // Override the result callback in the base class
    _options.result_callback = std::bind
    (
        &TrackCartesianTrajectory::result_callback,                                                 // Name of the method
        this,                                                                                       // Attach this node
        std::placeholders::_1                                                                       // I don't know what this does
    );
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Processes the result of an action                               //                           
////////////////////////////////////////////////////////////////////////////////////////////////////
void
TrackCartesianTrajectory::result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            if(_verbose)
            {
                std::string performanceResults =
                "Position error (mm) :\n"
                "   - Mean:      " + std::to_string(result.result->position_error.mean*1000) + "\n"
                "   - Std. dev.: " + std::to_string(sqrt(result.result->position_error.variance)*1000) + "\n"
                "   - Min:       " + std::to_string(result.result->position_error.min*1000) + "\n"
                "   - Max:       " + std::to_string(result.result->position_error.max*1000) + "\n"
                "Orientation error (deg) :\n"
                "   - Mean:      " + std::to_string(result.result->orientation_error.mean*180/M_PI) + "\n"
                "   - Std. dev.: " + std::to_string(sqrt(result.result->orientation_error.variance)*180/M_PI) + "\n"
                "   - Min:       " + std::to_string(result.result->orientation_error.min*180/M_PI) + "\n"
                "   - Max:       " + std::to_string(result.result->orientation_error.max*180/M_PI);            
     
                RCLCPP_INFO(_node->get_logger(),
                            "Cartesian trajectory tracking complete.\n%s",
                            performanceResults.c_str());
            }
            else
            {
                RCLCPP_INFO(_node->get_logger(), "Cartesian trajectory tracking complete.");
            }
            
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            RCLCPP_INFO(_node->get_logger(), "Cartesian trajectory tracking was canceled.");
            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_INFO(_node->get_logger(), "Cartesian trajectory tracking was aborted: %s", result.result->message.c_str());
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
