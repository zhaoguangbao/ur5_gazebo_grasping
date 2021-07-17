//
// Created by zhao on 2021/7/16.
//


#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>

#include <vector>
#include <iostream>

#include <ros/ros.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;


int main(int argc, char** argv) {
    ros::init(argc, argv, "test_movegroup");
    ros::NodeHandle node_handle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(2);
    spinner.start();

    const std::string PLANNING_GROUP = "arm";
    const std::string GRIPPER_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    move_group_interface.setNamedTarget("home_j");

    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group_interface.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        move_group_interface.execute(my_plan);
    else
        ROS_WARN("Plan Failed...");

    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = -tau / 6;
    move_group_interface.setJointValueTarget(joint_group_positions);

    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    ros::Duration(2).sleep();

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        move_group_interface.execute(my_plan);
    else
        ROS_WARN("Plan Failed...");

    std::string frame_="kinect2_rgb_optical_frame";
    std::string grasp_frame_="/grasp_link";
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ROS_INFO("wait for transform....");
    // grasp_frame_ represented in base_link
    while(ros::ok())
    {
        try{
            listener.waitForTransform("base_link", grasp_frame_, ros::Time(0), ros::Duration(0.0));
            listener.lookupTransform("base_link", grasp_frame_,
                                     ros::Time(0), transform);
            auto pos=transform.getOrigin();
            auto pose=transform.getRotation();
            std::cout<<pos.x()<<'\t'<<pos.y()<<'\t'<<pos.z()<<'\n';
            std::cout<<pose.x()<<'\t'<<pose.y()<<'\t'<<pose.z()<<'\t'<<pose.w()<<'\n';
            // move robot
            move_group_interface.setPositionTarget(pos.x(), pos.y(), pos.z()+0.4);
            success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                move_group_interface.execute(my_plan);
                ROS_INFO("Plan Success...");
                break;
            }
            else
            {
                ROS_WARN("Plan Failed...");
            }
        }catch (const tf::LookupException& e)
        {
            ROS_WARN("Transform Failed: %s...", e.what());
            ros::Duration(2).sleep();
        }
    }

    ros::waitForShutdown();
    return 0;
}

