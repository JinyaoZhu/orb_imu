
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../ORB_SLAM2/include/System.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <Eigen/Dense>


ros::Publisher  pose_pub;
ros::Publisher  imu_pub;

class EkfFusion
{
public:
    EkfFusion(){}

    void CamPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        Eigen::Vector3f cam_t(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
        Eigen::Quaternionf cam_q(msg->pose.orientation.w,msg->pose.orientation.x,
                             msg->pose.orientation.y,msg->pose.orientation.z);
	
	cam_R_wc_ = cam_q.toRotationMatrix();
	
	Eigen::Matrix4f T_bc; //transformation matrix of camera frame wrt. body frame
	
	T_bc << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0;
	
	Eigen::Matrix4f T_cb;
	T_cb = T_bc.inverse();
	
	Eigen::Matrix3f R_cb = T_cb.block<3,3>(0,0);
	//Eigen::Vector3f t_cb = T_cb.block<3,1>(0,3);

        
	
	Eigen::Vector3f acc_w = cam_R_wc_*(R_cb*acc_b_) + Eigen::Vector3f(0,9.81,0);
	
//         geometry_msgs::PoseStamped pose_msg;
//         pose_msg.pose.position.x = cam_t(0);
//         pose_msg.pose.position.y = cam_t(1);
//         pose_msg.pose.position.z = cam_t(2);
//         pose_msg.pose.orientation.x = cam_q.x();
//         pose_msg.pose.orientation.y = cam_q.y();
//         pose_msg.pose.orientation.z = cam_q.z();
//         pose_msg.pose.orientation.w = cam_q.w();
//         pose_msg.header = msg->header;
//         pose_pub.publish(pose_msg);
	sensor_msgs::Imu imu_msg;
        imu_msg.linear_acceleration.x = acc_w(0);
        imu_msg.linear_acceleration.y = acc_w(1);
        imu_msg.linear_acceleration.z = acc_w(2);
        imu_msg.header = msg->header;
        imu_pub.publish(imu_msg);
    }

    void ImuCallback(const sensor_msgs::ImuConstPtr& msg)
    {
        Eigen::Vector3f imu_acc(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
        Eigen::Vector3f imu_omega(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);

	acc_b_ = 0.8*acc_b_ + 0.2*imu_acc;
	
//         sensor_msgs::Imu imu_msg;
//         imu_msg.angular_velocity.x = imu_omega(0);
//         imu_msg.angular_velocity.y = imu_omega(1);
//         imu_msg.angular_velocity.z = imu_omega(2);
//         imu_msg.linear_acceleration.x = imu_acc(0);
//         imu_msg.linear_acceleration.y = imu_acc(1);
//         imu_msg.linear_acceleration.z = imu_acc(2);
//         imu_msg.header = msg->header;
//         imu_pub.publish(imu_msg);
    }
    
    Eigen::Matrix3f cam_R_wc_;
    Eigen::Vector3f acc_b_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_fusion");
    ros::start();

    EkfFusion ekf;

    ros::NodeHandle nodeHandler;

    pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/ekf_pose",1);
    imu_pub  = nodeHandler.advertise<sensor_msgs::Imu>("/ekf_imu",1);
    ros::Subscriber cam_sub = nodeHandler.subscribe("/cam_pose", 1, &EkfFusion::CamPoseCallback, &ekf);
    ros::Subscriber imu_sub = nodeHandler.subscribe("/imu", 1, &EkfFusion::ImuCallback, &ekf);
    
    ros::spin();

    ros::shutdown();

    return 0;
}


