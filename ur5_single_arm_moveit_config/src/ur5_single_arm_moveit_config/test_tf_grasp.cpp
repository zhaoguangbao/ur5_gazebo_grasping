//
// Created by zhao on 2021/7/17.
//

#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>

#include <tf/transform_broadcaster.h>

void detect_grasps_callback(const gpd_ros::GraspConfigList& l)
{
    gpd_ros::GraspConfig g = l.grasps[0];

    double coefficient = M_PI / 180;
    double roll, pitch, yaw;

    tf::Matrix3x3 m(
            g.approach.x, g.binormal.x, g.axis.x,
            g.approach.y, g.binormal.y, g.axis.y,
            g.approach.z, g.binormal.z, g.axis.z);

    m.getEulerYPR(yaw, pitch, roll);

    std::cout << m.getRow(0).getX() << m.getRow(0).getY() << m.getRow(0).getZ() << std::endl;
    std::cout << m.getRow(1).getX() << m.getRow(1).getY() << m.getRow(1).getZ() << std::endl;
    std::cout << m.getRow(2).getX() << m.getRow(2).getY() << m.getRow(2).getZ() << std::endl;

    geometry_msgs::Point point;
    point.x = g.sample.x;
    point.y = g.sample.y;
    point.z = g.sample.z;
    ROS_INFO("output xyz rpy: %f  %f  %f  %f  %f  %f", point.x ,point.y ,point.z, roll, pitch, yaw);

    geometry_msgs::PoseStamped target_pose_stamped;
    target_pose_stamped.pose.position.x = point.x;
    target_pose_stamped.pose.position.y = point.y;
    target_pose_stamped.pose.position.z = point.z;
    target_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}


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
            const double deg2rad = 180 / M_PI;
//            q.setRPY(0, 0, 0);
//            q.setRPY(21*deg2rad, 90*deg2rad, 180*deg2rad);
            q.setRPY(21, 90, 180);
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
        ros::Rate rate(10);
        while(ros::ok())
        {
            if(bstart)
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_, grasp_frame_));
                rate.sleep();
            }
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

