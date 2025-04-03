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

#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ library
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 C++ action library
#include <serial_link_action_client/track_cartesian_trajectory.hpp>                                 // Action client class
#include <serial_link_action_client/track_joint_trajectory.hpp>                                     // Action client class
#include <serial_link_action_client/utilities.hpp>                                                  // Helper functions
#include <thread>                                                                                   // Threading (duh!)

// These make code easier to read:
using CartesianTrajectoryAction = serial_link_interfaces::action::TrackCartesianTrajectory;
using CartesianTrajectoryPoint  = serial_link_interfaces::msg::CartesianTrajectoryPoint;
using JointTrajectoryAction     = serial_link_interfaces::action::TrackJointTrajectory;
using JointTrajectoryPoint      = serial_link_interfaces::msg::JointTrajectoryPoint;

  /////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{  
    using namespace serial_link_action_client;                                                      // ActionClientInterface, TrackCartesianTrajectory, TrackJointTrajectory
    
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
   
    std::shared_ptr<rclcpp::Node> clientNode = rclcpp::Node::make_shared("serial_link_client");     // Create client node
    
    // Load trajectory waypoints and put them in a dictionary
    std::map<std::string, std::vector<JointTrajectoryPoint>> jointConfigurations = load_joint_configurations(clientNode);
    std::map<std::string, std::vector<CartesianTrajectoryPoint>> endpointPoses   = load_endpoint_poses(clientNode);

    // Load tolerances on trajectory tracking
    double positionErrorTolerance = clientNode->declare_parameter("tolerance.pose.position", 0.05);
    double orientationErrorTolerance = clientNode->declare_parameter("tolerance.pose.orientation", 0.1);
    std::vector<double> jointErrorTolerances = clientNode->declare_parameter<std::vector<double>>("tolerance.joint", {0.0});
  
    // Create clients and attach to node
    auto cartesianTrajectoryClient = std::make_shared<TrackCartesianTrajectory>(clientNode, "track_cartesian_trajectory", true);
    auto jointTrajectoryClient     = std::make_shared<TrackJointTrajectory>(clientNode, "track_joint_trajectory", true);
 
    std::shared_ptr<ActionClientInterface> activeClient = nullptr;                                  // To keep track of active client

    std::thread{[clientNode]() { rclcpp::spin(clientNode); }}.detach();                             // Spin the node in a separate thread so we can continue
  
    while (rclcpp::ok())
    {
        // Get user input
        RCLCPP_INFO(clientNode->get_logger(), "Enter a command. Type 'options' to a see a list. Press Enter to cancel.");
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
            auto iterator = jointConfigurations.find(commandPrompt);                                // Find the named configuration
            
            if (iterator != jointConfigurations.end())
            {   
                if (activeClient != nullptr and activeClient->is_running()) stop_robot(activeClient);
                
                auto goal = std::make_shared<JointTrajectoryAction::Goal>();                        // Generate goal object
                
                goal->points     = iterator->second;                                                // Attach the joint trajectory
                goal->tolerances = jointErrorTolerances;
                
                RCLCPP_INFO(clientNode->get_logger(), "Moving to `%s` configuration(s).", commandPrompt.c_str()); // Inform user

                jointTrajectoryClient->send_goal(goal);                                             // Send request to client
                
                activeClient = jointTrajectoryClient;                                             
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
                    goal->position_tolerance = positionErrorTolerance;
                    goal->orientation_tolerance = orientationErrorTolerance;
                    
                    RCLCPP_INFO(clientNode->get_logger(), "Moving `%s` .", commandPrompt.c_str());  // Inform user

                    cartesianTrajectoryClient->send_goal(goal);                                     // Send request to client
                    
                    activeClient = cartesianTrajectoryClient;                                             
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
