#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>


class DriveController {
private:    
    ros::NodeHandle nh_;

    ros::Publisher odom_pub_;
    ros::Publisher cmd_vel_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber reset_sub_;

    tf::TransformBroadcaster tf_broadcaster_;

    double r_, lx_, ly_, ticks_;
    std::string frame_id_, child_frame_id_;
    int reset_timeout_;

    std::vector<float> reset_X_, reset_P_, prev_X_, prev_P_;

public:
    DriveController();

private:
    std::vector<float> ticks2rads(std::vector<int64_t> T);

    void odomCallback(const std_msgs::Int64MultiArray::ConstPtr& msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void resetOdomCallback(const std_msgs::Bool::ConstPtr& msg);
};


DriveController::DriveController() : nh_("~") {
    std::string odom_pub_topic, odom_sub_topic, cmd_vel_pub_topic, cmd_vel_sub_topic, reset_sub_topic;

    nh_.param<std::string>("odom_pub_topic", odom_pub_topic, "");
    nh_.param<std::string>("odom_sub_topic", odom_sub_topic, "");
    nh_.param<std::string>("cmd_vel_pub_topic", cmd_vel_pub_topic, "");
    nh_.param<std::string>("cmd_vel_sub_topic", cmd_vel_sub_topic, "");
    nh_.param<std::string>("reset_sub_topic", reset_sub_topic, "");
    nh_.param<double>("r", r_, 0.0);
    nh_.param<double>("lx", lx_, 0.0);
    nh_.param<double>("ly", ly_, 0.0);
    nh_.param<double>("ticks", ticks_, 0.0);
    nh_.param<std::string>("frame_id", frame_id_, "");
    nh_.param<std::string>("child_frame_id", child_frame_id_, "");
    nh_.param<int>("reset_timeout", reset_timeout_, 0);

    ROS_INFO("odom_pub_topic: '%s'", odom_pub_topic.c_str());
    ROS_INFO("odom_sub_topic: '%s'", odom_sub_topic.c_str());
    ROS_INFO("cmd_vel_pub_topic: '%s'", cmd_vel_pub_topic.c_str());
    ROS_INFO("cmd_vel_sub_topic: '%s'", cmd_vel_sub_topic.c_str());
    ROS_INFO("reset_sub_topic: '%s'", reset_sub_topic.c_str());
    ROS_INFO("r: %f", r_);
    ROS_INFO("lx: %f", lx_);
    ROS_INFO("ly: %f", ly_);
    ROS_INFO("ticks: %f", ticks_);
    ROS_INFO("frame_id: '%s'", frame_id_.c_str());
    ROS_INFO("child_frame_id: '%s'", child_frame_id_.c_str());
    ROS_INFO("reset_timeout: %lu", static_cast<unsigned long>(reset_timeout_));

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_pub_topic, 10);
    cmd_vel_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(cmd_vel_pub_topic, 10);

    odom_sub_ = nh_.subscribe<std_msgs::Int64MultiArray>(
        odom_sub_topic, 10, boost::bind(&DriveController::odomCallback, this, _1));

    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
        cmd_vel_sub_topic, 10, boost::bind(&DriveController::cmdVelCallback, this, _1));
    
    reset_sub_ = nh_.subscribe<std_msgs::Bool>(
        reset_sub_topic, 10, boost::bind(&DriveController::resetOdomCallback, this, _1));

    reset_X_ = {0, 0, 0, 0};
    reset_P_ = {0, 0, 0, 0};
    prev_X_ = reset_X_;
    prev_P_ = reset_P_;
}


std::vector<float> DriveController::ticks2rads(std::vector<int64_t> T) {
    std::vector<float> P;
    for (size_t i = 0; i < T.size(); i++) {
        P.push_back(2*M_PI * T[i] / ticks_);
    }

    return P;
}


void DriveController::odomCallback(const std_msgs::Int64MultiArray::ConstPtr& msg) {
    auto odom = nav_msgs::Odometry();

    odom.header.frame_id = frame_id_;
    odom.header.stamp = ros::Time::now();
    odom.child_frame_id = child_frame_id_;

    std::vector<float> P = ticks2rads(msg->data);

    std::vector<float> X;
    X.resize(3);

    float dp0 = P[0] - prev_P_[0];
    float dp1 = P[1] - prev_P_[1];
    float dp2 = P[2] - prev_P_[2];
    float dp3 = P[3] - prev_P_[3];

    float dx = (+dp0 + dp1 + dp2 + dp3) * r_ / 4;
    float dy = (-dp0 + dp1 + dp2 - dp3) * r_ / 4;
    float dY = (-dp0 + dp1 - dp2 + dp3) * r_ / 4 / (lx_ + ly_);

    X[0] = prev_X_[0] + dx * std::cos(prev_X_[2]) - dy * std::sin(prev_X_[2]);
    X[1] = prev_X_[1] + dx * std::sin(prev_X_[2]) + dy * std::cos(prev_X_[2]);
    X[2] = prev_X_[2] + dY;

    prev_X_ = X;
    prev_P_ = P;

    odom.pose.pose.position.x = X[0];
    odom.pose.pose.position.y = X[1];
    odom.pose.pose.position.z = 0;

    tf::Quaternion q;
    q.setRPY(0, 0, X[2]);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_.publish(odom);

    auto transform = geometry_msgs::TransformStamped();

    transform.header.stamp = odom.header.stamp;
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;

    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;

    transform.transform.rotation.x = odom.pose.pose.orientation.x;
    transform.transform.rotation.y = odom.pose.pose.orientation.y;
    transform.transform.rotation.z = odom.pose.pose.orientation.z;
    transform.transform.rotation.w = odom.pose.pose.orientation.w;

    tf_broadcaster_.sendTransform(transform);
}


void DriveController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float wz = msg->angular.z;

    std::vector<float> W;
    W.resize(4);

    W[0] = (vx - vy - (lx_ + ly_) * wz) / r_;
    W[1] = (vx + vy + (lx_ + ly_) * wz) / r_;
    W[2] = (vx + vy - (lx_ + ly_) * wz) / r_;
    W[3] = (vx - vy + (lx_ + ly_) * wz) / r_;

    std_msgs::Float32MultiArray cmd_vel_msg;
    cmd_vel_msg.data = W;

    cmd_vel_pub_.publish(cmd_vel_msg);
}


void DriveController::resetOdomCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data == false) {
        return;
    }

    ros::Duration(reset_timeout_).sleep();

    prev_X_ = reset_X_;
    prev_P_ = reset_P_;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "drive_controller");

    DriveController drive_controller;

    ros::spin();

    return 0;
}
