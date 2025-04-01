/**
 * @file    follow_twist_client.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    March 2025
 * @version 1.0
 * @brief   An action client and user interface for controlling a robot through the FollowTwist action.
 * 
 * @details This executable launches clients for a TrackJointTrajectory & FollowTwist actions.
 *          Joint configurations are loaded which can be sent to the robot to arrange it in
 *          pre-set positions. Then the robot can switch over to a real-time velocity control topic.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ library
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 C++ action library
#include <serial_link_action_client/follow_twist.hpp>                                               // Action client class
#include <serial_link_action_client/track_joint_trajectory.hpp>                                     // Action client class
#include <serial_link_action_client/utilities.hpp>                                                  // Helper functions
#include <thread>                                                                                   // Threading (duh!)

// These make code easier to read:
using FollowTwistAction     = serial_link_interfaces::action::FollowTwist;
using JointTrajectoryAction = serial_link_interfaces::action::TrackJointTrajectory;
using JointTrajectoryPoint  = serial_link_interfaces::msg::JointTrajectoryPoint;

  /////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{  
    using namespace serial_link_action_client;                                                      // ActionClientInterface, FollowTwist, TrackJointTrajectory
    
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
   
    std::shared_ptr<rclcpp::Node> clientNode = rclcpp::Node::make_shared("serial_link_client");     // Create client node
    
    // Create dictionary for joint configurations
    std::map<std::string, std::vector<JointTrajectoryPoint>> jointConfigurations = load_joint_configurations(clientNode);

    // Load parameters
    double linearVelocityTolerance  = clientNode->declare_parameter("tolerance.twist.linear",  0.5);
    double angularVelocityTolerance = clientNode->declare_parameter("tolerance.twist.angular", 0.1);
    double timeout = clientNode->declare_parameter("tolerance.timeout", 0.10);
    std::vector<double> jointTrackingTolerances = clientNode->declare_parameter<std::vector<double>>("tolerance.joint", {0.0});
    std::string twistTopicName = clientNode->declare_parameter("twist_topic_name", "twist_command");
    
    // Create clients and attach to node
    auto followTwistClient     = std::make_shared<FollowTwist>(clientNode, "follow_twist", true);
    
    auto jointTrajectoryClient = std::make_shared<TrackJointTrajectory>(clientNode, "track_joint_trajectory", true);
 
    std::shared_ptr<ActionClientInterface> activeClient = nullptr;                                  // To keep track of active client

    std::thread{[clientNode]() { rclcpp::spin(clientNode); }}.detach();                             // Spin the node in a separate thread so we can continue
  
    while (rclcpp::ok())
    {
        // Get user input
        RCLCPP_INFO(clientNode->get_logger(), "Enter a command. Type 'options' to a see a list. "
                                              "Use 'follow' to activate velocity control of endpoint. "
                                              "Press Enter to cancel.");
        std::string commandPrompt;
        std::getline(std::cin, commandPrompt);
        
        if (commandPrompt == "options")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Available joint commands:");
            for (const auto &config : jointConfigurations)
            {
                RCLCPP_INFO(clientNode->get_logger(), "- %s", config.first.c_str());
            }
        }
        else if (commandPrompt == "close")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Shutting down.");
            
            stop_robot(activeClient);

            break;
        }
        else if(commandPrompt == "cancel" or  commandPrompt == "")
        {
            stop_robot(activeClient);
        }
        else if(commandPrompt == "follow")
        {
            if (activeClient != nullptr and activeClient->is_running()) stop_robot(activeClient);
            
            RCLCPP_INFO(clientNode->get_logger(), "Following twist command.");
            
            auto goal = std::make_shared<FollowTwistAction::Goal>();
            goal->topic_name = twistTopicName;
            goal->linear_tolerance = linearVelocityTolerance;
            goal->angular_tolerance = angularVelocityTolerance;
            goal->timeout = timeout;
            
            followTwistClient->send_goal(goal);
            
            activeClient = followTwistClient;
        }
        else
        {
            auto iterator = jointConfigurations.find(commandPrompt);                                // Find the named configuration
            
            if (iterator != jointConfigurations.end())
            {   
                if (activeClient != nullptr and activeClient->is_running()) stop_robot(activeClient);
                
                auto goal = std::make_shared<JointTrajectoryAction::Goal>();                        // Generate goal object
                
                goal->points = iterator->second;                                                    // Attach the joint trajectory
                goal->tolerances = jointTrackingTolerances;
                
                RCLCPP_INFO(clientNode->get_logger(), "Moving to `%s` configuration(s).", commandPrompt.c_str()); // Inform user

                jointTrajectoryClient->send_goal(goal);                                             // Send request to client
                
                activeClient = jointTrajectoryClient;                                             
            }
        }
    }
 
    rclcpp::shutdown();                                                                             // Shut down

    return 0;
}
