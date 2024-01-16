#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

// 定义全局变量，用于存储目标位姿信息
geometry_msgs::Pose pose_msg;

cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) <<
        392.716970864628, 0, 243.0185503073266,
        0, 387.129884776694, 215.7363588792927,
        0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);

// 回调函数用于处理RGB图像数据并获取目标位姿
void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // 将ROS消息转换为OpenCV格式
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        
        //识别aruco的位置并打上坐标轴
        cv::Mat frame = cv_ptr->image;
        if (!frame.empty()) {
            std::vector< int > markerIds;
            std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
            cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds, cv::Scalar(0, 255, 0));
            //识别aruco的位置

            ROS_DEBUG("markerIds size is %d", (int)markerIds.size());

            // cv::Mat imageCopy;
            std::vector<cv::Vec3d> rvecs, tvecs;
            // 第一个参数(MarkerCornersONE): Marker 四个角的坐标(图片坐标系为基)
            // 第二个参数(landpad_det_len * ....): Marker的实际大小
            // 第三, 四个参数(camera_matrix, distortion_coefficients)为相机的参数, 相机畸变参数
            // 最后两个参数为输出, 旋转向量, 偏移向量(以相机坐标系为基)
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.050, cameraMatrix, distCoeffs, rvecs, tvecs);// 求解旋转矩阵rvecs和平移矩阵tvecs
            ROS_DEBUG("rvecs size is %d", (int)rvecs.size());
            if ((int)rvecs.size() > 0) {
                ROS_INFO("markerId is %d", (int)markerIds[0]);
                
                //pose_msg.header.frame_id = "charge_camera_link";
                //pose_msg.header.stamp = ros::Time(0);
                pose_msg.position.x = tvecs[0][0];
                pose_msg.position.y = tvecs[0][1];
                pose_msg.position.z = tvecs[0][2];
                //pose_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(tvecs[0][0], tvecs[0][1], tvecs[0][2]);

                ROS_INFO("Target Pose: x=%f, y=%f, z=%f", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
            }
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "capture");
    ros::NodeHandle nh;

    // 订阅RGB图像消息
    ros::Subscriber rgb_sub = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, rgbImageCallback);


    // 设置MoveIt
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    move_group.setPlanningTime(10.0);  // 设置规划时间

    // 等待MoveIt和控制器准备完毕
    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(10);

    //初始化pose_msg
    pose_msg.position.x = 0.01;
    pose_msg.position.y = 0.01;
    pose_msg.position.z = 0.01;
    pose_msg.orientation.w=1.0;

    //move_group.setNamedTarget("home");
    //move_group.go();

    while (ros::ok()){
    ROS_INFO("started");
        if(pose_msg.position.x || pose_msg.position.y || pose_msg.position.z){
            // 设置目标位姿
            geometry_msgs::Pose target_pose=pose_msg;
            ROS_INFO("Moveit Target Pose: x=%f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
            // 在这里设置目标位姿信息，根据相机识别的目标位姿赋值

            
            // 执行规划
            move_group.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::core::MoveItErrorCode success = move_group.plan(my_plan);
            //int success=move_group.plan(my_plan);
            

            // 控制机械臂移动到目标位姿
            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {ROS_INFO("success!");
                move_group.execute(my_plan);
            } else {
                ROS_WARN("Failed to plan the trajectory.");
            }
            pose_msg.position.x = 0;
            pose_msg.position.y = 0;
            pose_msg.position.z = 0;
            pose_msg.orientation.w=1.0;
        }
        ros::spinOnce();  // 持续监听ROS话题
        loop_rate.sleep();
    }
    return 0;
}
