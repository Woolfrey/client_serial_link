/**
 * @file    Utilities.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Useful functions for use in action clients.
 * 
 * @details This source file elaborates on the forward declarations in the associated header file
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the SerialLinkBase class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <Utilities.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Put joint trajectory points in to a searchable std::map                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::map<std::string, std::vector<serial_link_action_server::msg::JointTrajectoryPoint>>
load_joint_configurations()
{
    using JointTrajectoryPoint = serial_link_action_server::msg::JointTrajectoryPoint;              // Makes referencing easier
    
    std::map<std::string,std::vector<JointTrajectoryPoint>> jointConfigurations;                    // We want to return this
    
    auto configNode = rclcpp::Node::make_shared("joint_configurations");                            // Create node to host parameters
    
    // Acquire number of joints
    configNode->declare_parameter("number_of_joints", 0);
    int numJoints;
    configNode->get_parameter("number_of_joints", numJoints);
    
    // Get a list of configuration names
    configNode->declare_parameter("names", std::vector<std::string>{});                             // Declare name list  
    std::vector<std::string> names = {};                                                            // Storage location for names  
    configNode->get_parameter("names", names);                                                      // Get the values from the server
    
    // Cycle through all the named configurations, extract waypoints
    for(auto name : names)
    {
        // Declare the position, time parameters
        configNode->declare_parameter(name+".positions", std::vector<double>{});
        configNode->declare_parameter(name+".times", std::vector<double>{});
        
        // Create storage location
        std::vector<double> positions;
        std::vector<double> times;
        
        // Obtain from parameter server
        configNode->get_parameter(name+".positions", positions);
        configNode->get_parameter(name+".times", times);
        
        // Ensure dimensions are correct
        if (positions.size() % numJoints != 0)
        {
            throw std::invalid_argument
            (
                "Size of position array (" + std::to_string(positions.size()) + 
                ") not divisible by number of joints (" + std::to_string(numJoints) + ")."
            );
        }
     
        // Ensure number of waypoints matches number of times
        if(positions.size()/numJoints != times.size())
        {
            throw std::invalid_argument
            (
                "Number of waypoints (" + std::to_string(positions.size()/numJoints) + ") "
                "does not match number of times (" + std::to_string(times.size()) + ")."
            );
        }
        
        // Put the positions & times together to define a trajectory
        std::vector<JointTrajectoryPoint> points;
        
        for (size_t i = 0; i < times.size(); ++i)
        {
            JointTrajectoryPoint point;
            
            point.time = times[i];
            
            std::vector<double> joint_positions(positions.begin() + i * numJoints,
                                                positions.begin() + (i + 1) * numJoints);
            
            point.position = joint_positions;                                                       // Assign the extracted positions
            
            points.push_back(point);                                                                // Add the point to the trajectory
        }
        
        jointConfigurations.emplace(name, points);
    }
    
    return jointConfigurations;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Put Cartesian trajectory points in to a searchable std::map                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::map<std::string,std::vector<serial_link_action_server::msg::CartesianTrajectoryPoint>>
load_endpoint_poses()
{
    using CartesianTrajectoryPoint = serial_link_action_server::msg::CartesianTrajectoryPoint;      // For easier referencing
    
    std::map<std::string, std::vector<CartesianTrajectoryPoint>> endpointPoses;                     // We want to return this
    
    auto configNode = rclcpp::Node::make_shared("endpoint_poses");                                  // Name must match the YAML file
    
    // Get a list of names
    configNode->declare_parameter("names", std::vector<std::string>{});                             // Declare name list  
    std::vector<std::string> names = {};                                                            // Storage location for names  
    configNode->get_parameter("names", names);                                                      // Get the values from the server
    
    for(auto name : names)
    {
        // Declare the position, time parameters
        configNode->declare_parameter(name+".poses", std::vector<double>{});
        configNode->declare_parameter(name+".times", std::vector<double>{});
        configNode->declare_parameter(name+".reference", "");
        
        // Create storage location
        std::vector<double> poses;
        std::vector<double> times;
        std::string reference;
        
        // Obtain from parameter server
        configNode->get_parameter(name+".poses", poses);
        configNode->get_parameter(name+".times", times);
        configNode->get_parameter(name+".reference", reference);
        
        // Ensure dimensions are correct
        if (poses.size() % 6 != 0)
        {
            throw std::invalid_argument
            (
                "Size of pose array (" + std::to_string(poses.size()) + ") not divisible by 6."
            );
        }
     
        // Ensure number of waypoints matches number of times
        if(poses.size()/6 != times.size())
        {
            throw std::invalid_argument
            (
                "Number of waypoints (" + std::to_string(poses.size()/6) + ") "
                "does not match number of times (" + std::to_string(times.size()) + ")."
            );
        }
        
        int code;
        if(reference == "absolute") code = 0;
        if(reference == "local"   ) code = 1;
        if(reference == "relative") code = 2;
        
        // Put the positions & times together to define a trajectory
        std::vector<CartesianTrajectoryPoint> points;
        
        for (size_t i = 0; i < times.size(); ++i)
        {
            CartesianTrajectoryPoint point;
            
            point.reference = code;
     
            point.time = times[i];
            
            std::vector<double> pose(poses.begin() + i * 6, poses.begin() + (i + 1) * 6);
            
            point.pose.position.x = pose[0];
            point.pose.position.y = pose[1];
            point.pose.position.z = pose[2];
                       
            // Convert pose[3], pose[4], pose[5] (roll, pitch, yaw) to quaternion
            double cy = cos(pose[5] * 0.5);
            double sy = sin(pose[5] * 0.5);
            double cp = cos(pose[4] * 0.5);
            double sp = sin(pose[4] * 0.5);
            double cr = cos(pose[3] * 0.5);
            double sr = sin(pose[3] * 0.5);

            point.pose.orientation.w = cr * cp * cy + sr * sp * sy;
            point.pose.orientation.x = sr * cp * cy - cr * sp * sy;
            point.pose.orientation.y = cr * sp * cy + sr * cp * sy;
            point.pose.orientation.z = cr * cp * sy - sr * sp * cy;
            
            points.push_back(point);                                                                // Add the point to the trajectory
        }
        
        endpointPoses.emplace(name, points);                                                        // Add to the map
    }
    
    return endpointPoses;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Stops the active action server from running                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool
stop_robot(ActionClientInterface *activeClient)
{
    if (activeClient == nullptr)
    {
        return true;                                                                                // No active action
    }

    // Possible return codes:
    // 0 = Unknown
    // 1 = Accepted
    // 2 = Executing
    // 3 = Canceling
    // 4 = Succeeded
    // 5 = Canceled
    // 6 = Aborted

    if (activeClient->status() == 0)
    {
        return false; // A problem
    }

    // If running, cancel
    if (activeClient->status() == 1
    or  activeClient->status() == 2)
    {
        activeClient->cancel_action();
    }

    // Wait until action is resolved
    while (activeClient->status() != 4 &&
           activeClient->status() != 5 &&
           activeClient->status() != 6)
    {      
        std::this_thread::sleep_for(std::chrono::milliseconds(10));                                 // Wait for 10ms
    }

    return true;
}

