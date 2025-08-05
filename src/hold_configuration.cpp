/**
 * @file    hold_configuration.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0
 * @brief   Source code for the HoldConfiguration action client class.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_action_client/hold_configuration.hpp>

namespace serial_link_action_client {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
HoldConfiguration::HoldConfiguration(std::shared_ptr<rclcpp::Node> clientNode,
                                     const std::string &actionName,
                                     bool verbose)
: ActionClientBase(clientNode, actionName),
  _verbose(verbose)
{
    // Override the result callback in the base class
    _defaultOptions.result_callback = std::bind
    (
        &HoldConfiguration::result_callback,                                                // Name of the method
        this,                                                                               // Attach this node
        std::placeholders::_1                                                               // I don't know what this does
    );
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Processes the result of an action                               //                           
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HoldConfiguration::result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(_node->get_logger(),
                        "This case should never be called because the HoldConfiguration action has no end condition. "
                        "How did that happen??? (ー_ーゞ");                         
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            if(_verbose)
            {
                std::string performanceResults = "";
                
                int jointNum = 0;

                for(auto stats : result.result->position_error)
                {
                    ++jointNum;
                    
                    performanceResults += "Joint " + std::to_string(jointNum) + ":\n"
                                          "   - Mean:      " + std::to_string(stats.mean) + "\n"
                                          "   - Std. dev.: " + std::to_string(sqrt(stats.variance)) + "\n"
                                          "   - Min.:      " + std::to_string(stats.min) + "\n"
                                          "   - Max.:      " + std::to_string(stats.max) + "\n";
                    
                }
                
                RCLCPP_INFO(_node->get_logger(),
                            "Hold configuration action cancelled. Position error:\n%s",
                            performanceResults.c_str());
            }
            
            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_INFO(_node->get_logger(), "Hold configuration action was aborted: %s", result.result->message.c_str());
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
