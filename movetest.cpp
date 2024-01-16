#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/collision_detection/collision_robot.h>
//#include <moveit/collision_detection/collision_tools/collision_tools.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>




int main(int argc, char** argv) {
    ros::init(argc, argv, "movetest");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    geometry_msgs::Pose pose_msg;
    // 设置MoveIt
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::MoveGroupInterface move_group2("gripper");
    

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "world";
    move_group.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    move_group.allowReplanning(true);
    move_group2.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    move_group.setGoalPositionTolerance(10000);
    move_group.setGoalOrientationTolerance(10000);

    //设置允许的最大速度和加速度
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setMaxVelocityScalingFactor(0.2);

    move_group.setPlanningTime(20.0);  // 设置规划时间

    // 等待MoveIt和控制器准备完毕
    ros::Duration(1.0).sleep();

    //ros::Rate loop_rate(10);


    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);
    
    /*
    move_group2.setNamedTarget("open");
    move_group2.move();
    sleep(1);*/

    /*
    double targetPose[4] = {0.0, 0.5, -0.1, -0.4};
    std::vector<double> joint_group_positions(4);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];


    move_group.setJointValueTarget(joint_group_positions);
    move_group.move();
    sleep(1);

    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);

    
    ROS_INFO("success0");
    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);


    */    

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    // double roll = 0.0;
    // double pitch = 0.0;
    // double yaw = 0.0;
    // tf2::Quaternion orientation;
    // orientation.setRPY(roll, pitch, yaw);
    // target_pose.orientation = tf2::toMsg(orientation);

    target_pose.position.x = 0.28;
    target_pose.position.y = 0;
    target_pose.position.z = 0.18;
    ROS_INFO("success1");

    // 设置机器臂当前的状态作为运动初始状态
    move_group.setStartStateToCurrentState();

    move_group.setPoseTarget(target_pose);
    ROS_INFO("success2");

    // 获取当前机械臂状态
    robot_state::RobotState current_state = *move_group.getCurrentState();

    std::string end_effector_link = move_group.getEndEffectorLink();
    ROS_INFO("End effector link: %s", end_effector_link.c_str());

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
    auto success = move_group.plan(my_plan);
    ROS_INFO("success3");

    if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("success!");
        move_group.execute(my_plan);
        sleep(1);
    } else {
        if (my_plan.trajectory_.joint_trajectory.points.empty())
        {
            ROS_ERROR("No trajectory points. Planning failed.");
        }
        else
        {
            ROS_INFO("The plan failed for an unknown reason.");
        }
        ROS_WARN("Failed to plan the trajectory.");
    }
    ROS_INFO("success!!!");

    return 0;
}
