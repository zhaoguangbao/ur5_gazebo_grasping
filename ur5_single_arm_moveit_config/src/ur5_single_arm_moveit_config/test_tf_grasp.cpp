//
// Created by zhao on 2021/7/17.
//

#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>

#include <tf/transform_broadcaster.h>

class PubGraspTF{
public:
    // publish grasp pos
    void grasp_callback(const gpd_ros::GraspConfigListConstPtr & msg)
    {
        frame_=msg->header.frame_id;
        std::vector<gpd_ros::GraspConfig> grasps=msg->grasps;
        if(!grasps.empty()){
            std::cout<<grasps[0].score<<grasps[0].width<<grasps[0].sample<<'\n';
            std::cout<<grasps[0].position<<'\n';
            std::cout<<grasps[0].approach<<'\n';
            std::cout<<grasps[0].binormal<<'\n';
            std::cout<<grasps[0].axis<<'\n';

            transform.setOrigin( tf::Vector3(grasps[0].position.x, grasps[0].position.y, grasps[0].position.z) );
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.setRotation(q);

            bstart=true;
        }
        std::cout<<"...."<<'\n';
    }
    PubGraspTF()
    {
        graspConfigSub=node_handle.subscribe("/detect_grasps/clustered_grasps", 10, &PubGraspTF::grasp_callback, this);
    }
    void Run()
    {
        ROS_INFO("Run ...");
        ros::Rate rate(100);
        while(ros::ok())
        {
            if(bstart)
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_, grasp_frame_));
                rate.sleep();
            }
//            else
//            {
//                ROS_INFO("...");
//            }
            ros::spinOnce();
        }
    }
public:
    ros::NodeHandle node_handle;
    ros::Subscriber graspConfigSub;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    std::string frame_{"kinect2_rgb_optical_frame"};
    std::string grasp_frame_{"grasp_link"};
    bool bstart{false};
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "test_tf_grasp");
//    ros::NodeHandle node_handle;
    PubGraspTF pubgrasptf;
    pubgrasptf.Run();

//    ros::spin();
    return 0;
}

