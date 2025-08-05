/**
 * @file    action_client_base.tpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    March 2025
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

#include <serial_link_action_client/action_client_base.hpp>

namespace serial_link_action_client {

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
    _defaultOptions.goal_response_callback = std::bind
    (
        &ActionClientBase::goal_response_callback,                                                  // Name of the method
        this,                                                                                       // Attach this node
        std::placeholders::_1                                                                       // I don't know what this does
    );

    // Attach the result callback for when an action is finished
    _defaultOptions.result_callback = std::bind
    (
        &ActionClientBase::result_callback,                                                         // Name of the method
        this,                                                                                       // Attach this node
        std::placeholders::_1                                                                       // I don't know what this does
    );
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Send an action request to the server, with custom callback options             //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
bool
ActionClientBase<Action>::send_goal(const typename Action::Goal::SharedPtr &goal,
                                    std::function<void(typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr)> goalResponseCallback,
                                    std::function<void(typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr, const typename Action::Feedback::ConstSharedPtr &)> feedbackCallback,
                                    std::function<void(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &)> resultCallback,
                                    std::chrono::milliseconds timeout)
{
    typename rclcpp_action::Client<Action>::SendGoalOptions options;

    // Use default callbacks if user hasn't supplied one
    options.goal_response_callback = goalResponseCallback
                                   ? goalResponseCallback
                                   : _defaultOptions.goal_response_callback;

    if (feedbackCallback) options.feedback_callback = feedbackCallback;
    
    options.result_callback = resultCallback
                            ? resultCallback
                            : _defaultOptions.result_callback;                     

    // Wait for the action server
    if (not _actionClient->wait_for_action_server(timeout))
    {
        RCLCPP_ERROR(_node->get_logger(), "Server not available within %ld ms.", timeout.count());
        
        return false;
    }

    _actionClient->async_send_goal(*goal, options);                                                 // Send goal asynchronously

    return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Processes the response to an action request                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionClientBase<Action>::goal_response_callback(const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goalHandle)
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
ActionClientBase<Action>::result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
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
 //                                   Get the current goal status                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
int8_t
ActionClientBase<Action>::status() const
{
    if(_goalHandle) return _goalHandle->get_status();
    else            return 0;
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

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Check to see if the current action is running                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
bool
ActionClientBase<Action>::is_running() const
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

} // namespace

