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

enum eRunType{
    eGrasp=0,
    eKinect=1,
    eRealsense=2
};

class GraspDemo{
public:
    GraspDemo():
            move_group_interface(PLANNING_GROUP),
            gripper_group_interface(GRIPPER_GROUP),
            arm_current_state(move_group_interface.getCurrentState()),
            gripper_current_state(gripper_group_interface.getCurrentState())
    {

    }

    // grasp
    void goToGrasp()
    {
        tf::StampedTransform transform;
        ROS_INFO("wait for transform....");
        while(ros::ok())
        {
            try{
                listener.waitForTransform("base_link", grasp_frame_, ros::Time(0), ros::Duration(0.0));
                listener.lookupTransform("base_link", grasp_frame_,
                                         ros::Time(0), transform);

                auto grasp_target_pose=getTargetPose(transform);
                goTargetPose(grasp_target_pose);

                grasp_target_pose.position.z = grasp_target_pose.position.z-0.06;
                move_group_interface.setPoseTarget(grasp_target_pose);
                success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    move_group_interface.execute(my_plan);
                    ROS_INFO("Plan Success...");
                    closeGripper();

                    geometry_msgs::Pose current_pose=move_group_interface.getCurrentPose().pose;
                    current_pose.position.z+=0.2;
                    goTargetPose(current_pose);
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

    }
    // go home, pre grasp for kinect
    void goHome()
    {
        move_group_interface.setNamedTarget("home");
        success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            move_group_interface.execute(my_plan);
        else
            ROS_WARN("Plan Failed...");
        openGripper();
        ros::Duration(1.0).sleep();
    }
    // go home_j, pre grasp for realsense
    void goHomeJ()
    {
        move_group_interface.setNamedTarget("home_j");
        success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            move_group_interface.execute(my_plan);
        else
            ROS_WARN("Plan Failed...");
        ros::Duration(1.0).sleep();
    }
private:
    geometry_msgs::Pose getTargetPose(tf::StampedTransform& transform)
    {
        auto pos=transform.getOrigin();
        auto pose=transform.getRotation();
        std::cout<<pos.x()<<'\t'<<pos.y()<<'\t'<<pos.z()<<'\n';
        std::cout<<pose.x()<<'\t'<<pose.y()<<'\t'<<pose.z()<<'\t'<<pose.w()<<'\n';
        tf::Quaternion q;
        q.setW(0.707); q.setX(0.0); q.setY(0.707); q.setZ(0.0);
        transform.setRotation(q);
        // move robot
        // move_group_interface.setPositionTarget(pos.x(), pos.y(), pos.z()+0.15);

        geometry_msgs::Pose grasp_target_pose;
        grasp_target_pose.orientation.w = q.w();
        grasp_target_pose.orientation.x = q.x();
        grasp_target_pose.orientation.y = q.y();
        grasp_target_pose.orientation.z = q.z();
        grasp_target_pose.position.x = pos.x();
        grasp_target_pose.position.y = pos.y();
        grasp_target_pose.position.z = pos.z()+0.16;

        return grasp_target_pose;
    }
    bool goTargetPose(const geometry_msgs::Pose& target_pose)
    {
        move_group_interface.setPoseTarget(target_pose);
        success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            move_group_interface.execute(my_plan);
            return true;
        }
        else
        {
            ROS_WARN("Plan Failed...");
            return false;
        }
    }
    bool closeGripper()
    {
        const moveit::core::JointModelGroup* joint_model_group = gripper_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        std::vector<double> gripper_joint_positions;
        gripper_current_state->copyJointGroupPositions(joint_model_group, gripper_joint_positions);
        gripper_joint_positions[2]=0.4;
        gripper_group_interface.setJointValueTarget(gripper_joint_positions);
        success = (gripper_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            gripper_group_interface.execute(my_plan);
        return success;
    }
    bool openGripper()
    {
        gripper_group_interface.setNamedTarget("open");
        success = (gripper_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            gripper_group_interface.execute(my_plan);
        return success;
    }
private:
    const std::string PLANNING_GROUP{"arm"};
    const std::string GRIPPER_GROUP{"gripper"};

//    ros::NodeHandle node_handle;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    moveit::planning_interface::MoveGroupInterface gripper_group_interface;
    moveit::core::RobotStatePtr arm_current_state;
    moveit::core::RobotStatePtr gripper_current_state;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success{false};

    const std::string grasp_frame_{"/grasp_link"};
    tf::TransformListener listener;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_movegroup");
    ros::NodeHandle node_handle("~");
    int runtype=eGrasp;

    // node_handle.getParam("run_type", runtype);
    node_handle.param<int>("run_type", runtype, eGrasp);

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(2);
    spinner.start();
    GraspDemo graspDemo;

    switch (runtype) {
        case eGrasp:
            graspDemo.goHome();
            graspDemo.goHomeJ();
            graspDemo.goToGrasp();
            break;
        case eKinect:
            graspDemo.goHome();
            break;
        case eRealsense:
            graspDemo.goHome();
            graspDemo.goHomeJ();
            break;
        default:
            break;
    }
    ROS_INFO("finish...");
    // ros::waitForShutdown();
    return 0;
}

