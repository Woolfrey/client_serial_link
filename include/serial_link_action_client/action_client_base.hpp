/**
 * @file    action_client_base.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
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
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#ifndef ACTION_CLIENT_BASE_H
#define ACTION_CLIENT_BASE_H

#include <serial_link_action_client/action_client_interface.hpp>
#include <memory>
#include <string>
#include <action_msgs/srv/cancel_goal.hpp>

namespace serial_link_action_client {

/**
 * @brief Provides structure for all action clients.
 */
template <class Action>
class ActionClientBase : public serial_link_action_client::ActionClientInterface
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
         * @brief Send goal to the server using custom callbacks.
         * @param goal The goal field of the action.
         * @param goalResponseCallback The function that is executed after receiving the response from the server.
         * @param feedbackCallback The function that executes when feedback is received.
         * @param resultCallback The function that executes when the action ends.
         * @return False if the server is not available.
         */
        bool
        send_goal(const typename Action::Goal::SharedPtr &goal,
                  std::function<void(typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr)> goalResponseCallback = nullptr,
                  std::function<void(typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr, const typename Action::Feedback::ConstSharedPtr &)> feedbackCallback = nullptr,
                  std::function<void(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &)> resultCallback = nullptr,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

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
        status() const override;
        
        /**
         * @brief Checks to see if an action is currently active.
         * @return True if accepted (about to start), currently executing, or in the process of canceling.
         */
        bool
        is_running() const override;

    protected:

        std::shared_ptr<rclcpp::Node> _node;                                                        ///< Pointer to client node.
        
        typename rclcpp_action::Client<Action>::SendGoalOptions _defaultOptions;                    ///< These are used to set callback functions        
       
        typename rclcpp_action::Client<Action>::SharedPtr _actionClient;                            ///< This is the foundation of the class

        typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr _goalHandle;                    ///< Current goal handle
        
        /**
         * @brief This method executes after sending a goal, and receiving the response from the server.
         * @param GoalHandle A pointer to the goal handle associated with the action.
         */
        void
        goal_response_callback(const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goalHandle);
        
        /**
         * This method executes when an action is finished and the server returns the result.
         * @param result The result portion of the associated goal field.
         */
        void
        result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result);
            
        /**
         * This method executes after an action server has completed the cancellation process.
         */
        void
        cancel_callback(const typename rclcpp_action::Client<Action>::CancelResponse::SharedPtr response);
};

}

#include <serial_link_action_client/action_client_base.tpp>

#endif
