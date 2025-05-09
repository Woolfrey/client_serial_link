/**
 * @file    follow_transform.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   An action client for the FollowTransform action.
 * 
 * @details This class acts as the client implementation of the FollowTransform action
 *          defined in the serial_link_interfaces package.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 *
 * @see https://github.com/Woolfrey/interface_serial_link
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#ifndef FOLLOW_TWIST_CLIENT_H
#define FOLLOW_TWIST_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <serial_link_action_client/action_client_base.hpp>
#include <serial_link_interfaces/action/follow_transform.hpp>

namespace serial_link_action_client {

class FollowTransform : public serial_link_action_client::ActionClientBase<serial_link_interfaces::action::FollowTransform>
{
    public:
    
        using Action = serial_link_interfaces::action::FollowTransform;                             // For brevity
    
        /**
         * @brief Constructor.
         * @param clientNode A pointer to the client node.
         * @param actionName The name of the action being advertised by the server.
         */
        FollowTransform(std::shared_ptr<rclcpp::Node> clientNode,
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

