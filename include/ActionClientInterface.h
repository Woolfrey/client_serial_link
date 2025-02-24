/**
 * @file    ActionClientInterface.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Provides basic interfaces for interacting with different actions.
 * 
 * @details This class provides basic interfaces for interacting with common attributes across all
 *          different types of actions. It means that multiple action clients of different types
 *          can be better managed & coordinates.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#ifndef ACTION_CLIENT_INTERFACE_H
#define ACTION_CLIENT_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/**
 * @brief A class that provides basic interfaces to all actions clients.
 */
class ActionClientInterface
{
    public:
    
        /**
         * @brief Destructor.
         */
        virtual
        ~ActionClientInterface() = default;
        
        /**
         * @brief Gets the status of the action that is running.
         *        This is a virtual method and must be defined in any derived class.
         * @see https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html
         * 0 = Unknown
         * 1 = Accepted
         * 2 = Executing
         * 3 = Canceling
         * 4 = Succeeded
         * 5 = Canceled
         * 6 = Aborted
         * @return An int8_t for the status.
         */
        virtual
        int8_t
        status() const = 0;
        
        /**
         * @brief This cancels an action that is currently executing.
         *        This is a virtual method and must be defined in any derived class.
         * @return True if successful, false if there was a problem.
         */
        virtual
        bool
        cancel_action() = 0;
        
        /**
         * @brief Checks to see if the action is currently running.
         *        This is a virtual method and must be defined in any derived class.
         * @return True if the action is accepted, currently running, or in the process of cancelling.
         */
        virtual
        bool
        is_running() const = 0;
};                                                                                                  // Semicolon required after class declaration

#endif
