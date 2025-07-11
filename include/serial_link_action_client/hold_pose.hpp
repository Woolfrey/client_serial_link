/**
 * @file    hold_pose.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0
 * @brief   An class for the HoldPose action client.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#ifndef HOLD_POSE_CLIENT_H
#define HOLD_POSE_CLIENT_H

#include <serial_link_action_client/action_client_base.hpp>
#include <serial_link_interfaces/action/hold_pose.hpp>
#include <rclcpp/rclcpp.hpp>

namespace serial_link_action_client {

class HoldPose : public serial_link_action_client::ActionClientBase<serial_link_interfaces::action::HoldPose>
{
    public:
    
        using Action = serial_link_interfaces::action::HoldPose;                                    // For brevity
    
        /**
         * @brief Constructor.
         * @param clientNode A pointer to  the client node.
         * @param actionName The name of the action being advertised by the server.
         */
        HoldPose(std::shared_ptr<rclcpp::Node> clientNode,
                 const std::string &actionName,
                 bool verbose = false);
                                  
    private:
       
        bool _verbose = false;                                                                      ///< Controls detail of results
        
        /**
         * @brief This method executes after an action is completed.
         *        It overrides the method defined in the base class.
         * @result The result portion of the action.
         */
        void
        result_callback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result);   
};

}

#endif

