//
// Created by zhao on 2021/7/18.
//

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/builtin_string.h>

int main(int argc, char* argv[]) {
    /*
    const std::string FRAME = "world";
    const std::string filename="/home/zhao/Downloads/004_sugar_box_berkeley_meshes/004_sugar_box/poisson/nontextured.pcd";

    ros::init(argc, argv, "test_pcd2rviz");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(filename, *cloud);

    // Visualize the point cloud in rviz.
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_indexed", 1);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = FRAME;
    std::cout<<msg.width<<'\n';
    ros::Rate rate(1);
    ROS_INFO("...");
    while (ros::ok())
    {
        //publishing point cloud data
        cloud_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
     */
    ros::init(argc, argv, "test_pcd2rviz");
    ros::NodeHandle nh("~");

    // ros::NodeHandle nh;
    // ros::Publisher cloud_pub = nh.advertise<std_msgs::String>("cloud_indexed", 1);
    // topic list /cloud_indexed
    // ros::NodeHandle nh("~");
    // ros::Publisher cloud_pub = nh.advertise<std_msgs::String>("cloud_indexed", 1);
    // topic list /test_pcd2rviz/cloud_indexed

    ros::Publisher cloud_pub = nh.advertise<std_msgs::String>("cloud_indexed", 1);
    std_msgs::String msg;
    msg.data="test";
    ros::Rate rate(1);
    ROS_INFO("...");
    while (ros::ok())
    {
        //publishing point cloud data
        cloud_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}

