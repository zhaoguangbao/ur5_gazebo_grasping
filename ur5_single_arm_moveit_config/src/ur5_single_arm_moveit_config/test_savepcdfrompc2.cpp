//
// Created by zhao on 2021/8/13.
//

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //do stuff with temp_cloud here
    static bool bsave=true;
    if(bsave)
    {
        bsave=false;
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZRGBA> ("/home/zhao/table_scene_rgba.pcd", *temp_cloud, false); // binary = false
        // pcl::io::savePCDFileASCII (filename, *cloud);
        ROS_INFO("Done");
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pc2_listener");

    ros::NodeHandle node;
    ros::Subscriber sub_handler=node.subscribe("/kinect_V2/depth_registered/points", 10, cloud_cb);
    ROS_INFO("wait for msg...");

    ros::spin();
    return 0;
};

