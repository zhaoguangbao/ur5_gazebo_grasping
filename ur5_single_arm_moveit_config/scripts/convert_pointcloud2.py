#! /usr/bin/env python2.7
from __future__ import print_function

import numpy as np
import rospy
import pcl
import pcl.pcl_visualization
# from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import pointclouds


def remove_table_points(points_voxel_, vis=False):
    xy_unique = np.unique(points_voxel_[:, 0:2], axis=0)
    new_points_voxel_ = points_voxel_
    pre_del = np.zeros([1])
    for i in range(len(xy_unique)):
        tmp = []
        for j in range(len(points_voxel_)):
            if np.array_equal(points_voxel_[j, 0:2], xy_unique[i]):
                tmp.append(j)
        print(len(tmp))
        if len(tmp) < 3:
            tmp = np.array(tmp)
            pre_del = np.hstack([pre_del, tmp])
    if len(pre_del) != 1:
        pre_del = pre_del[1:]
        new_points_voxel_ = np.delete(points_voxel_, pre_del, 0)
    print("Success delete [[ {} ]] points from the table!".format(len(points_voxel_) - len(new_points_voxel_)))
    return new_points_voxel_


def remove_white_pixel(msg, points_, vis=False):
    points_with_c_ = pointclouds.pointcloud2_to_array(msg)
    points_with_c_ = pointclouds.split_rgb_field(points_with_c_)
    r = np.asarray(points_with_c_['r'], dtype=np.uint32)
    g = np.asarray(points_with_c_['g'], dtype=np.uint32)
    b = np.asarray(points_with_c_['b'], dtype=np.uint32)
    rgb_colors = np.vstack([r, g, b]).T
    # rgb = rgb_colors.astype(np.float) / 255
    ind_good_points_ = np.sum(rgb_colors[:] < 210, axis=-1) == 3
    ind_good_points_ = np.where(ind_good_points_ == 1)[0]
    new_points_ = points_[ind_good_points_]
    return new_points_


def cal_grasp(msg):
    points_ = pointclouds.pointcloud2_to_xyz_array(msg)
    points_ = points_.astype(np.float32)
    remove_white = False
    if remove_white:
        points_ = remove_white_pixel(msg, points_, vis=True)

    remove_points = False
    if remove_points:
        points_ = remove_table_points(points_, vis=True)
    point_cloud = pcl.PointCloud(points_)
    print(points_.size, '...')
    # visual = pcl.pcl_visualization.PCLVisualizering()
    # visual.AddPointCloud(point_cloud)
    # while not visual.WasStopped():
    #     visual.Spin()

    # norm = point_cloud.make_NormalEstimation()
    # norm.set_KSearch(30)  # critical parameter when calculating the norms
    # normals = norm.compute()
    # surface_normal = normals.to_array()
    # surface_normal = surface_normal[:, 0:3]
    # vector_p2cam = cam_pos_ - points_
    # vector_p2cam = vector_p2cam / np.linalg.norm(vector_p2cam, axis=1).reshape(-1, 1)
    # tmp = np.dot(vector_p2cam, surface_normal.T).diagonal()
    # angel = np.arccos(np.clip(tmp, -1.0, 1.0))
    # wrong_dir_norm = np.where(angel > np.pi * 0.5)[0]
    # tmp = np.ones([len(angel), 3])
    # tmp[wrong_dir_norm, :] = -1
    # surface_normal = surface_normal * tmp
    # select_point_above_table = 0.010
    # #  modify of gpg: make it as a parameter. avoid select points near the table.
    # points_for_sample = points_[np.where(points_[:, 2] > select_point_above_table)[0]]
    # if len(points_for_sample) == 0:
    #     rospy.loginfo("Can not seltect point, maybe the point cloud is too low?")
    #     return [], points_, surface_normal


def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message

        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    # points_list = []
    points_list = pointclouds.pointcloud2_to_array(ros_cloud, split_rgb=True)
    print(points_list.shape)

    # for data in PointCloud2.read_points(ros_cloud, skip_nans=True):
    #     points_list.append([data[0], data[1], data[2], data[3]])
    points_list = points_list.astype(np.float32)
    pcl_data = pcl.PointCloud(points_list)

    return pcl_data


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # arr = pointclouds.pointcloud2_to_xyz_array(data)
    # print(arr.shape, type(arr))  # (307200, 3) <class 'numpy.ndarray'>

    # arr = pointclouds.split_rgb_field(data)
    # print(arr.shape, type(arr))

    # ret_arr = pointclouds.pointcloud2_to_array(data)
    # print(ret_arr.shape, type(ret_arr))
    #
    # arr = pointclouds.pointcloud2_to_dtype(data)
    # print(type(arr))
    # arr_np = np.array(arr)
    # print(arr_np.shape)
    # print(arr_np)
    # print('-'*10)

    # error
    # fields = pointclouds.arr_to_fields(arr_np)
    # fields_np = np.array(fields)
    # print(fields_np.shape)
    ros_to_pcl(data)

    pass


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/kinect_V2/depth_registered/points", PointCloud2, cal_grasp)
    rospy.loginfo('wait for msg...')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
