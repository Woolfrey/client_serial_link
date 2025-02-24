/**
 * @file    trajectory_tracking_client.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Launches clients for interacting with Cartesian and joint trajectory action servers.
 * 
 * @details This executables launches clients for interacting with the Cartesian & joint trajectory
 *          tracking action servers in the serial_link_action_server package. It loads pre-defined
 *          endpoint poses, and joint configurations, which are sent as goals to the server. The
 *          server will then generate trajectories and perform realtime feedback control.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include "TrackCartesianTrajectory.h"
#include "TrackJointTrajectory.h"
#include <random>                                                                                   // For generating random numbers
#include "rclcpp/rclcpp.hpp"                                                                        // ROS2 C++ library
#include "rclcpp_action/rclcpp_action.hpp"                                                          // ROS2 C++ action library
#include <thread>                                                                                   // Threading (duh!)
#include "Utilities.h"                                                                              // Useful functions

using JointTrajectoryPoint      = serial_link_interfaces::msg::JointTrajectoryPoint;
using CartesianTrajectoryPoint  = serial_link_interfaces::msg::CartesianTrajectoryPoint;
using CartesianTrajectoryAction = serial_link_interfaces::action::TrackCartesianTrajectory;
using JointTrajectoryAction     = serial_link_interfaces::action::TrackJointTrajectory;

  /////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
   
    auto clientNode = rclcpp::Node::make_shared("trajectory_tracking_client");                      // Create client node and advertise its name
    auto jointConfigurations = load_joint_configurations();                                         // From the parameter server
    auto endpointPoses = load_endpoint_poses();                                                     // From the parameter server
    auto joint_tolerances = load_joint_tolerances();
    
    // Create the action clients, attach to node
    TrackJointTrajectory jointTrajectoryClient(clientNode, "track_joint_trajectory", true);
    TrackCartesianTrajectory cartesianTrajectoryClient(clientNode, "track_cartesian_trajectory", true);  
    ActionClientInterface *activeClient = nullptr;                                                  // Use this to keep track of which action is running           
   
    std::thread{[clientNode]() { rclcpp::spin(clientNode); }}.detach();                             // Spin the node in a separate thread so we can continue
  
    while (rclcpp::ok())
    {
        // Get user input
        RCLCPP_INFO(clientNode->get_logger(), "Enter a command. Type 'options' to a see a list.");
        std::string commandPrompt;
        std::getline(std::cin, commandPrompt);
        
        if (commandPrompt == "options")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Available joint commands:");
            for (const auto &config : jointConfigurations)
            {
                RCLCPP_INFO(clientNode->get_logger(), "- %s", config.first.c_str());
            }
            RCLCPP_INFO(clientNode->get_logger(), "Available Cartesian commands:");
            for(const auto &pose : endpointPoses)
            {
                RCLCPP_INFO(clientNode->get_logger(), "- %s", pose.first.c_str());
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
        else
        {
            auto iterator = jointConfigurations.find(commandPrompt);
            
            if (iterator != jointConfigurations.end())
            {   
                if (activeClient != nullptr and activeClient->is_running())
                {
                    stop_robot(activeClient);
                }
                
                auto goal = std::make_shared<JointTrajectoryAction::Goal>();                        // Generate goal object
                
                goal->points = iterator->second;                                                    // Attach the joint trajectory
                goal->tolerances = joint_tolerances;
                
                RCLCPP_INFO(clientNode->get_logger(), "Moving to `%s` configuration(s).", commandPrompt.c_str()); // Inform user

                jointTrajectoryClient.send_goal(goal);                                              // Send request to client
                
                activeClient = &jointTrajectoryClient;                                             
            }
            else
            {
                auto iterator = endpointPoses.find(commandPrompt);
                
                if (iterator != endpointPoses.end())
                {   
                    if (activeClient != nullptr and activeClient->is_running())
                    {
                        stop_robot(activeClient);
                    }
                    
                    auto goal = std::make_shared<CartesianTrajectoryAction::Goal>();                // Generate goal object
                    
                    goal->points = iterator->second;                                                // Attach the joint trajectory
                    
                    RCLCPP_INFO(clientNode->get_logger(), "Moving `%s` .", commandPrompt.c_str());  // Inform user

                    cartesianTrajectoryClient.send_goal(goal);                                      // Send request to client
                    
                    activeClient = &cartesianTrajectoryClient;                                             
                }
                else
                {
                    RCLCPP_WARN(clientNode->get_logger(), "Unknown command. Type 'options' to see a list.");
                }
            }
        }
    }

    rclcpp::shutdown();                                                                             // Shut down

    return 0;
}
