/**
 * @file    ActionClientBase.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Provides structure and basic interfaces to all action clients.
 * 
 * @details This class elaborates on the fundamental methods required for sending goals & receiving
 *          results in a ROS2 action client. It provides common structure to all actions so they can
 *          be coordinated under a single client executable.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the KinematicTree class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#ifndef ACTION_CLIENT_BASE_H
#define ACTION_CLIENT_BASE_H

#include <client/ActionClientInterface.h>
#include <memory>
#include <string>
#include <action_msgs/srv/cancel_goal.hpp>

/**
 * @brief Provides structure for and basic interfaces to all action clients.
 */
template <class Action>
class ActionClientBase : public ActionClientInterface
{
    public:
        
        using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;                                 ///< For easier referencing

        /**
         * @brief Constructor.
         * @param clientNode A pointer to the client node.
         * @param actionName Must match what is advertised by the server.
         */
        ActionClientBase(std::shared_ptr<rclcpp::Node> clientNode,
                         const std::string &actionName);

        /**
         * @brief Sends a goal to the server to perform a given action.
         * @param goal The goal field of the action to be sent to the server.
         * @return Returns true of the goal is accepted, false if not.
         */
        bool
        send_goal(const typename Action::Goal::SharedPtr &goal);

        /**
         * @brief Asks the action to cancel.
         * @return Returns true if the cancellation request is accepted, false for any other reason.
         */
        bool
        cancel_action() override;
        
        /**
         * @brief This overrides the method defined in the base class.
         * @return An int8_t for the status.
         * @details 0 = Unknown
         *          1 = Accepted
         *          2 = Executing
         *          3 = Canceling
         *          4 = Succeeded
         *          5 = Canceled
         *          6 = Aborted
         * @see https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html
         */
        int8_t
        status() const override
        {
            if(_goalHandle) return _goalHandle->get_status();
            else            return 0;
        }
        
        /**
         * @brief Checks to see if an action is currently active.
         * @return True if accepted (about to start), currently executing, or in the process of canceling.
         */
        bool
        is_running() const override
        {
            if(status() == 1    
            or status() == 2
            or status() == 3)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
 
    protected:
        
        std::shared_ptr<rclcpp::Node> _node;                                                        ///< Pointer to client node.
        
        typename rclcpp_action::Client<Action>::SendGoalOptions _options;                           ///< These are used to set callback functions        
       
        typename rclcpp_action::Client<Action>::SharedPtr _actionClient;                            ///< This is the foundation of the class

        typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr _goalHandle;                    ///< Current goal handle
          
        /**
         * @brief This method executes after sending a goal, and receiving the response from the server.
         * @param GoalHandle A pointer to the goal handle associated with the action.
         */
        void
        handle_response(const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goalHandle);
        
        /**
         * This method executes when an action is finished and the server returns the result.
         * @param result The result portion of the associated goal field.
         */
        void
        handle_result(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result);
            
        /**
         * This method executes after an action server has completed the cancellation process.
         */
        void
        cancel_callback(const typename rclcpp_action::Client<Action>::CancelResponse::SharedPtr response);
};

#endif
