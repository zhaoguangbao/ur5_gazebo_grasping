mainly from [UR5+robotiq_85_gripper GAZEBO模拟视觉抓取平台仿真](https://blog.csdn.net/harrycomeon/article/details/107073020)

platform

ubuntu 18.04 + ros melodic

[problem and solution](https://blog.csdn.net/m0_47989004/article/details/118520102?spm=1001.2014.3001.5502)

run

```bash
roslaunch ur5_single_arm_moveit_config test_ur5_gazebo.launch
roslaunch ur5_single_arm_moveit_config ur5_moveit_planning_execution.launch
rosrun ur5_single_arm_moveit_config grasping_demo1.py
```

```bash
roslaunch ur5_single_arm_moveit_config test_ur5_gazebo.launch
roslaunch ur5_single_arm_moveit_config ur5_moveit_planning_execution.launch
roslaunch find_object_2d find_object_3d_kinect2.launch
rosrun opencv tf_listener.py object_frame:="/object_frame should be modified"
rosrun ur5_single_arm_moveit_config grasping_demo_vision2.py
```

## note for myself

git push -u master master
git push

