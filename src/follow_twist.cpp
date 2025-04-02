/**
 * @file    follow_twist.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Implementation of the client for the FollowTwist action.
 * 
 * @details This class acts as a client for the FollowTwist action. It overrides the virtual methods
 *          defined in the ActionClientBase class.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_action_client/follow_twist.hpp>

namespace serial_link_action_client {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //                           
////////////////////////////////////////////////////////////////////////////////////////////////////
FollowTwist::FollowTwist(std::shared_ptr<rclcpp::Node> clientNode,
                         const std::string &actionName,
                         bool verbose)
: ActionClientBase(clientNode, actionName),
  _verbose(verbose)
{
    // Override the result callback in the base class
    _options.result_callback = std::bind
    (
        &FollowTwist::result_callback,                                                              // Name of the method
        this,                                                                                       // Attach this node
        std::placeholders::_1                                                                       // I don't know what this does
    );
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Processes the result of an action                               //                           
////////////////////////////////////////////////////////////////////////////////////////////////////
void
FollowTwist::result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
{                
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_WARN(_node->get_logger(),
                        "This condition should never be called since the FollowTwist.action has no end conditions. "
                        "How did that happen (o_O) ?");
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            if(_verbose)
            {
                std::string performanceResults =
                "Linear velocity error (m/s) :\n"
                "   - Mean:      " + std::to_string(result.result->linear_error.mean) + "\n"
                "   - Std. dev.: " + std::to_string(sqrt(result.result->linear_error.variance)) + "\n"
                "   - Min:       " + std::to_string(result.result->linear_error.min) + "\n"
                "   - Max:       " + std::to_string(result.result->linear_error.max) + "\n"
                "Angular velocity error (rpm) :\n"
                "   - Mean:      " + std::to_string(result.result->angular_error.mean*60/(2*M_PI)) + "\n"
                "   - Std. dev.: " + std::to_string(sqrt(result.result->angular_error.variance)*60/(2*M_PI)) + "\n"
                "   - Min:       " + std::to_string(result.result->angular_error.min*60/(2*M_PI)) + "\n"
                "   - Max:       " + std::to_string(result.result->angular_error.max*60/(2*M_PI));
                
                RCLCPP_INFO(_node->get_logger(), "Follow twist action ended.\n%s", performanceResults.c_str());
            }
            else
            {
                RCLCPP_INFO(_node->get_logger(), "Follow twist action ended.");
            }
            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_INFO(_node->get_logger(), "Follow twist tracking was aborted: %s", result.result->message.c_str());
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
