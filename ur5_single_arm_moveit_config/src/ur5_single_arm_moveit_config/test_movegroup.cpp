//
// Created by zhao on 2021/7/16.
//


#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>

#include <vector>
#include <iostream>

#include <ros/ros.h>

//    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
//    std::vector<double> joint_group_positions;
//    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//    joint_group_positions[0] = -tau / 6;
//    move_group_interface.setJointValueTarget(joint_group_positions);
//
//    move_group_interface.setMaxVelocityScalingFactor(0.05);
//    move_group_interface.setMaxAccelerationScalingFactor(0.05);
//
//    ros::Duration(2).sleep();
//
//    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;


int main(int argc, char** argv) {
    ros::init(argc, argv, "test_movegroup");
    ros::NodeHandle node_handle;
    bool bgoHome=false;
    if(argc==2)
    {
        bgoHome=(bool)argv[1];
    }
//    node_handle.param("gohome", bgoHome, false);

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(2);
    spinner.start();

    const std::string PLANNING_GROUP = "arm";
    const std::string GRIPPER_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper_group_interface(GRIPPER_GROUP);
    moveit::core::RobotStatePtr arm_current_state = move_group_interface.getCurrentState();
    moveit::core::RobotStatePtr gripper_current_state = move_group_interface.getCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;

    move_group_interface.setNamedTarget("home");
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        move_group_interface.execute(my_plan);
    else
        ROS_WARN("Plan Failed...");
    ros::Duration(1.0).sleep();
    gripper_group_interface.setNamedTarget("open");
    success = (gripper_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        gripper_group_interface.execute(my_plan);

    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

//    std::string frame_="kinect2_rgb_optical_frame";
    std::string grasp_frame_="/grasp_link";
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ROS_INFO("wait for transform....");
    // grasp_frame_ represented in base_link
    while(ros::ok() && !bgoHome)
    {
        try{
            listener.waitForTransform("base_link", grasp_frame_, ros::Time(0), ros::Duration(0.0));
            listener.lookupTransform("base_link", grasp_frame_,
                                     ros::Time(0), transform);

            move_group_interface.setNamedTarget("home_j");
            success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                move_group_interface.execute(my_plan);
            else
                ROS_WARN("Plan Failed...");

            auto pos=transform.getOrigin();
            auto pose=transform.getRotation();
            std::cout<<pos.x()<<'\t'<<pos.y()<<'\t'<<pos.z()<<'\n';
            std::cout<<pose.x()<<'\t'<<pose.y()<<'\t'<<pose.z()<<'\t'<<pose.w()<<'\n';
            tf::Quaternion q;
            q.setW(0.707); q.setX(0.0); q.setY(0.707); q.setZ(0.0);
            transform.setRotation(q);
            // move robot
//            move_group_interface.setPositionTarget(pos.x(), pos.y(), pos.z()+0.15);

            geometry_msgs::Pose grasp_target_pose;
            grasp_target_pose.orientation.w = q.w();
            grasp_target_pose.orientation.x = q.x();
            grasp_target_pose.orientation.y = q.y();
            grasp_target_pose.orientation.z = q.z();
            grasp_target_pose.position.x = pos.x();
            grasp_target_pose.position.y = pos.y();
            grasp_target_pose.position.z = pos.z()+0.16;
            move_group_interface.setPoseTarget(grasp_target_pose);
            success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                move_group_interface.execute(my_plan);

            grasp_target_pose.position.z = pos.z()+0.10;
            move_group_interface.setPoseTarget(grasp_target_pose);
            success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                move_group_interface.execute(my_plan);
                ROS_INFO("Plan Success...");
                std::vector<double> gripper_joint_positions;
                gripper_current_state->copyJointGroupPositions(joint_model_group, gripper_joint_positions);
                gripper_joint_positions[2]=0.38;
                gripper_group_interface.setJointValueTarget(gripper_joint_positions);
                success = (gripper_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                    gripper_group_interface.execute(my_plan);

                geometry_msgs::Pose current_pose=move_group_interface.getCurrentPose().pose;
                current_pose.position.z+=0.2;
                move_group_interface.setPoseTarget(current_pose);
                success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                    move_group_interface.execute(my_plan);
                break;
            }
            else
            {
                ROS_WARN("Plan Failed...");
                break;
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

