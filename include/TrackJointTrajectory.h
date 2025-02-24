/**
 * @file    TrackJointTrajectory.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   An action client for the TrackJointTrajectory action.
 * 
 * @details This class acts as the client implementation of the TrackJointTrajectory action
 *          defined in the serial_link_interfaces package.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#ifndef JOINT_TRAJECTORY_CLIENT_H
#define JOINT_TRAJECTORY_CLIENT_H

#include "ActionClientBase.h"
#include "serial_link_interfaces/action/track_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

class TrackJointTrajectory : public ActionClientBase<serial_link_interfaces::action::TrackJointTrajectory>
{
    public:
    
        using Action = serial_link_interfaces::action::TrackJointTrajectory;                        // For brevity
    
        /**
         * @brief Constructor.
         * @param clientNode A pointer to the client node for this server.
         * @param actionName The name of the action being advertised by the server.
         */
        TrackJointTrajectory(std::shared_ptr<rclcpp::Node> clientNode,
                              const std::string &actionName,
                              bool verbose = false)
                             : ActionClientBase(clientNode, actionName),
                               _verbose(verbose)
        {
            // Override the result callback in the base class
            _options.result_callback = std::bind
            (
                &TrackJointTrajectory::result_callback,                                            // Name of the method
                this,                                                                               // Attach this node
                std::placeholders::_1                                                               // I don't know what this does
            );
        }
                            
    private:
        
        bool _verbose = false;                                                                      ///< Used to control detail of results     
        
        /**
         * @brief This method executes after an action is completed.
         *        It overrides the method defined in the base class.
         * @param result The result portion of the action.
         */
        void
        result_callback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result);
};

#endif
