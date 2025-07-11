/**
 * @file    hold_configuration.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0
 * @brief   An action client for the HoldConfiguration action.
 * 
 * @details This class acts as the client implementation of the HoldConfiguration action
 *          defined in the serial_link_interfaces package.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 * @see https://github.com/Woolfrey/interface_serial_link
 */
 
#ifndef HOLD_CONFIGURATION_CLIENT_H
#define HOLD_CONFIGURATION_CLIENT_H

#include <serial_link_action_client/action_client_base.hpp>
#include <serial_link_interfaces/action/hold_configuration.hpp>
#include <rclcpp/rclcpp.hpp>

namespace serial_link_action_client {

class HoldConfiguration : public serial_link_action_client::ActionClientBase<serial_link_interfaces::action::HoldConfiguration>
{
    public:
    
        using Action = serial_link_interfaces::action::HoldConfiguration;                           // For brevity
    
        /**
         * @brief Constructor.
         * @param clientNode A pointer to the client node for this server.
         * @param actionName The name of the action being advertised by the server.
         */
        HoldConfiguration(std::shared_ptr<rclcpp::Node> clientNode,
                          const std::string &actionName,
                          bool verbose = false);
                            
    private:
        
        bool _verbose = false;                                                                      ///< Used to control detail of results     
        
        /**
         * @brief This method executes after an action is completed.
         *        It overrides the method defined in the base class.
         * @param result The result portion of the action.
         */
        void
        result_callback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result);
};

}

#endif
