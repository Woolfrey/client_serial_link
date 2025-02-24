/**
 * @file    ActionClientBase.cpp
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
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <ActionClientBase.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
ActionClientBase<Action>::ActionClientBase(std::shared_ptr<rclcpp::Node> clientNode,
                                           const std::string &actionName)
                                           : ActionClientInterface(),
                                             _node(clientNode),
                                             _actionClient(rclcpp_action::create_client<Action>(_node, actionName))
{
    // Attach the response callback after an action request is sent
    _options.goal_response_callback = std::bind
    (
        &ActionClientBase::handle_response,                                                         // Name of the method
        this,                                                                                       // Attach this node
        std::placeholders::_1                                                                       // I don't know what this does
    );

    // Attach the result callback for when an action is finished
    _options.result_callback = std::bind
    (
        &ActionClientBase::handle_result,                                                           // Name of the method
        this,                                                                                       // Attach this node
        std::placeholders::_1                                                                       // I don't know what this does
    );
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Send an action request to the server                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
bool
ActionClientBase<Action>::send_goal(const typename Action::Goal::SharedPtr &goal)
{
    auto goalHandleFuture = _actionClient->async_send_goal(*goal, _options);                        // Send request, and link callback methods
    
    if (goalHandleFuture.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready)
    {
        return goalHandleFuture.get() ? true : false;                                               // We want a fast return
    }
    else
    {
        RCLCPP_INFO(_node->get_logger(), "Failed to receive response from server after 500 ms.");
        
        return false;
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Processes the response to an action request                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionClientBase<Action>::handle_response(const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goalHandle)
{
    if(goalHandle)                                                                                  // Not a null pointer
    {
        _goalHandle = goalHandle;                                                                   // Save it internally
    }
    else
    {
        RCLCPP_INFO(_node->get_logger(), "Action request rejected by the server.");
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Executes when an action is finished.                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionClientBase<Action>::handle_result(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(_node->get_logger(), "Action completed.");
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            RCLCPP_INFO(_node->get_logger(), "Action canceled.");
            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_ERROR(_node->get_logger(), "Action aborted.");
            break;
        }
        default:
        {
            RCLCPP_WARN(_node->get_logger(), "Unknown result code.");
            break;
        }
    }
} 

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Cancel the action that is in progress                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
bool
ActionClientBase<Action>::cancel_action()
{
    auto cancelFuture = _actionClient->async_cancel_goal
    (
        _goalHandle,
        [this](const typename rclcpp_action::Client<Action>::CancelResponse::SharedPtr response)
        {
            this->cancel_callback(response);
        }
    );

    if (cancelFuture.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready)
    {
        return true;
    }
    else
    {
        RCLCPP_WARN(_node->get_logger(), "Request for action cancellation timed out.");
        return false;
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Executes after cancelling                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionClientBase<Action>::cancel_callback(const typename rclcpp_action::Client<Action>::CancelResponse::SharedPtr response)
{
    switch (response->return_code)
    {
        case action_msgs::srv::CancelGoal::Response::ERROR_REJECTED:
            RCLCPP_ERROR(_node->get_logger(), "Cancel rejected.");
            break;
        case action_msgs::srv::CancelGoal::Response::ERROR_UNKNOWN_GOAL_ID:
            RCLCPP_ERROR(_node->get_logger(), "Unknown goal ID.");
            break;
        case action_msgs::srv::CancelGoal::Response::ERROR_GOAL_TERMINATED:
            RCLCPP_ERROR(_node->get_logger(), "Goal already terminated.");
            break;
        default:
            RCLCPP_INFO(_node->get_logger(), "Cancel succeeded.");
            break;
    }
}

