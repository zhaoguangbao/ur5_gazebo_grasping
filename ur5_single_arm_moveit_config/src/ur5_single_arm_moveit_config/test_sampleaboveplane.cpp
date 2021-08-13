//
// Created by zhao on 2021/8/13.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>

#include <gpd/util/cloud.h>
#include <gpd/util/config_file.h>
#include <gpd/candidate/candidates_generator.h>



gpd::util::Cloud* cloud_camera_ = nullptr;
gpd::candidate::CandidatesGenerator::Parameters params_;
std::string config_filename="/home/zhao/git_ws/gpd/cfg/eigen_params.cfg";

ros::Publisher pub;


void read_params(const std::string &filename)
{
    gpd::util::ConfigFile config_file(filename);
    config_file.ExtractKeys();
    // Create object to generate grasp candidates.
    params_.num_samples_ =
            config_file.getValueOfKey<int>("num_samples", 1000);
    params_.num_threads_ =
            config_file.getValueOfKey<int>("num_threads", 1);
    params_.remove_statistical_outliers_ =
            config_file.getValueOfKey<bool>("remove_outliers", false);
    params_.sample_above_plane_ =
            config_file.getValueOfKey<bool>("sample_above_plane", false);
    params_.voxelize_ =
            config_file.getValueOfKey<bool>("voxelize", true);
    params_.voxel_size_ =
            config_file.getValueOfKey<double>("voxel_size", 0.003);
    params_.normals_radius_ =
            config_file.getValueOfKey<double>("normals_radius", 0.03);
    params_.refine_normals_k_ =
            config_file.getValueOfKey<int>("refine_normals_k", 0);
    params_.workspace_ =
            config_file.getValueOfKeyAsStdVectorDouble("workspace", "-1 1 -1 1 -1 1");
}

void preprocessed(gpd::util::Cloud& cloud)
{
    printf("Processing cloud with %zu points.\n",
           cloud.getCloudOriginal()->size());

    cloud.removeNans();

    cloud.filterWorkspace(params_.workspace_);

    if (params_.voxelize_) {
        cloud.voxelizeCloud(params_.voxel_size_);
    }

    cloud.calculateNormals(params_.num_threads_, params_.normals_radius_);

    if (params_.refine_normals_k_ > 0) {
        cloud.refineNormals(params_.refine_normals_k_);
    }

    if (params_.sample_above_plane_) {
        cloud.sampleAbovePlane();
    }

    cloud.subsample(params_.num_samples_);
}


void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
    Eigen::Matrix3Xd view_points(3,1);
    view_points<<0.0, 0.0, 0.0;
    if(!cloud_camera_)
        delete cloud_camera_;
    cloud_camera_ = nullptr;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    //do stuff with temp_cloud here
    cloud_camera_ = new gpd::util::Cloud(temp_cloud, 0, view_points);
    preprocessed(*cloud_camera_);

    // convert
    // get processed cloud
//    std::vector<int> indices=cloud_camera_->getSampleIndices();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices=cloud_camera_->getSampleIndices();
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(cloud_camera_->getCloudProcessed());
    extract.setIndices(inliers);
    extract.setNegative(false); //
    extract.filter(*processed_cloud);

    ROS_INFO("processed size: %zu --> %zu", temp_cloud->size(), processed_cloud->size());
    pcl::toPCLPointCloud2(*processed_cloud, pcl_pc2);
    sensor_msgs::PointCloud2 processed_spc2;
    pcl_conversions::fromPCL(pcl_pc2, processed_spc2);
    pub.publish(processed_spc2);

    // save
    static bool bsave=true;
    if(bsave)
    {
        bsave=false;
        pcl::PCDWriter writer;
        processed_cloud->height=1;
        processed_cloud->width=processed_cloud->size();
        writer.write<pcl::PointXYZRGBA> ("/home/zhao/processed2.pcd", *processed_cloud, false); // binary = false
        // pcl::io::savePCDFileASCII (filename, *cloud);
        ROS_INFO("Done");
    }

    ROS_INFO("Done");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pc2_listener");

    ros::NodeHandle node;

    read_params(config_filename);
    ros::Subscriber sub_handler=node.subscribe("/kinect_V2/depth_registered/points", 10, cloud_cb);

    pub=node.advertise<sensor_msgs::PointCloud2>("/table_top_points", 10);
    ROS_INFO("wait for msg...");

    ros::spin();
    return 0;
};
