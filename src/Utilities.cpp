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
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <Utilities.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Load tolerances for joint feedback error from parameter file                //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double>
load_joint_error_tolerances(const std::shared_ptr<rclcpp::Node> &node)
{
    std::string nodeName = node->get_name();
    
    node->declare_parameter("joint_error_tolerances", std::vector<double>{});                       // We need to declare before we can get
    
    if (not node->has_parameter("joint_error_tolerances"))
    {
        throw std::invalid_argument("Parameter 'joint_error_tolerances' not found in the '" + nodeName + "' node. "
                                    "Does the name in the YAML file match the node name?");
    }
    
    return node->get_parameter("joint_error_tolerances").as_double_array();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Load tolerances for Cartesian feedback error from parameter file             //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::array<double,2>
load_pose_error_tolerances(const std::shared_ptr<rclcpp::Node> &node)
{
    std::string nodeName = node->get_name();
    
    node->declare_parameter("pose_error_tolerance.position", 0.1);
    node->declare_parameter("pose_error_tolernace.orientation", 0.05);
    
    double position = node->get_parameter("pose_error_tolerance.position").as_double();
    double orientation = node->get_parameter("pose_error_tolernace.orientation").as_double();
    
    return {position, orientation};
}
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Put joint trajectory points in to a searchable std::map                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::map<std::string, std::vector<serial_link_interfaces::msg::JointTrajectoryPoint>>
load_joint_configurations(const std::shared_ptr<rclcpp::Node> &node)
{
    using JointTrajectoryPoint = serial_link_interfaces::msg::JointTrajectoryPoint;                 // Makes referencing easier
    
    std::string nodeName = node->get_name();
    
    int numJoints = 0;                                                                              // Number of actuated joints
    std::vector<std::string> jointConfigNames;                                                      // So we can search the YAML file by name
    std::map<std::string, std::vector<JointTrajectoryPoint>> jointConfigurations;                   // This is what we want to return
   
    // Declare parameters we intend to load
    node->declare_parameter("number_of_joints", 0);                                                 // We need to declare before we can get
    node->declare_parameter("joint_config_names", std::vector<std::string>{});                      // Need to declare before we can get
       
    // Get the number of joints 
    if(not node->has_parameter("number_of_joints"))
    {
        throw std::invalid_argument("Could not find parameter 'number_of_joints' for the '" + nodeName + "' node. "
                                    "Does the name in the YAML file match the node name?");
    }
    else
    {
        node->get_parameter("number_of_joints", numJoints);                                         // Save it
    }
    
    // Get a list of the configuration names
    if(not node->has_parameter("joint_config_names"))
    {
        throw std::invalid_argument("Could not find parameter `joint_config_names` for the '" + nodeName + "' node. "
                                    "Does the name in the YAML file match the node name?");
    }
    else
    {
        node->get_parameter("joint_config_names", jointConfigNames);
    }
     
    // Search through the names and extract the waypoints
    for(const auto &name : jointConfigNames)
    {
        // Need to declare in advance
        node->declare_parameter(name+".positions", std::vector<double>{});
        node->declare_parameter(name+".times", std::vector<double>{});
        
        if(!node->has_parameter(name+".positions")
        or !node->has_parameter(name+".times"))
        {
            throw std::invalid_argument("No 'positions' and/or 'times' associated with the '" + name + "' joint configuration.");
        }
        
        std::vector<double> times = node->get_parameter(name+".times").as_double_array();
        std::vector<double> positions = node->get_parameter(name+".positions").as_double_array();
        
        // Ensure dimensions are correct
        if (positions.size() % numJoints != 0)
        {
            throw std::invalid_argument("Size of position array (" + std::to_string(positions.size()) + ") "
                                        "is not divisible by the number of joints (" + std::to_string(numJoints) + ").");
        }
            
        // Ensure number of waypoints matches number of times
        if(positions.size()/numJoints != times.size())
        {
            throw std::invalid_argument("Number of waypoints (" + std::to_string(positions.size()) + " "
                                        "does not match the number of times (" + std::to_string(times.size()) + ").");
        }
            
        // Combine the positions & times to define a trajectory
        std::vector<JointTrajectoryPoint> points;
    
        for (size_t i = 0; i < times.size(); ++i)
        {
            JointTrajectoryPoint point;
            
            point.time = times[i];
            
            std::vector<double> joint_positions(positions.begin() + i * numJoints,
                                                positions.begin() + (i + 1) * numJoints);
            
            point.position = joint_positions;                                                   // Assign the extracted positions
            
            points.push_back(point);                                                            // Add the point to the trajectory
        }
            
        jointConfigurations.emplace(name, points);
    }

    return jointConfigurations;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Put Cartesian trajectory points in to a searchable std::map                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::map<std::string,std::vector<serial_link_interfaces::msg::CartesianTrajectoryPoint>>
load_endpoint_poses(const std::shared_ptr<rclcpp::Node> &node)
{
    using CartesianTrajectoryPoint = serial_link_interfaces::msg::CartesianTrajectoryPoint;         // For easier referencing
    
    std::string nodeName = node->get_name();
    
    std::vector<std::string> poseNames;
    std::map<std::string, std::vector<CartesianTrajectoryPoint>> endpointPoses;                     // We want to return this
    
    // Declare what parameters we intend to load
    node->declare_parameter("pose_names", std::vector<std::string>{});                              // Need to declare before we can get
            
    // Get a list of names
    if(not node->has_parameter("pose_names"))
    {
        throw std::invalid_argument("Could not find 'pose_names' parameters for the '" + nodeName + "' node. "
                                    "Does the name in the YAML file match the node name?");
    }
    else
    {
        node->get_parameter("pose_names", poseNames);                                               // Now get them
        
        for(const auto &name : poseNames)
        {
            // Declare in advance
            node->declare_parameter(name+".poses", std::vector<double>{});
            node->declare_parameter(name+".times", std::vector<double>{});
            node->declare_parameter(name+".reference", "");
            
            if (!node->has_parameter(name+".poses")
            or  !node->has_parameter(name+".times")
            or  !node->has_parameter(name+".reference"))
            {
                throw std::invalid_argument("Could not find 'poses', 'times', and/or 'reference' "
                                            "for the '" + name + "' field.");
            }
            
            // Get the parameters
            std::vector<double> poses = node->get_parameter(name+".poses").as_double_array();
            std::vector<double> times = node->get_parameter(name+".times").as_double_array();
            std::string reference = node->get_parameter(name+".reference").as_string();
            
            // Ensure there are 6 elements for pose (3 for position, 3 for orientation)
            if(poses.size() % 6 != 0)
            {
                throw std::invalid_argument("Size of pose array (" + std::to_string(poses.size()) + ") "
                                            "for '" + name + "' not divisible by 6.");
            }
            
            // Ensure the number of waypoints matches the number of times
            if(poses.size()/6 != times.size())
            {
                throw std::invalid_argument("Number of waypoints (" + std::to_string(poses.size()/6) + ") "
                                            "does not match the number of times (" + std::to_string(times.size()) + ").");
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
                
                point.reference_frame = code;
         
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

