#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "robot_msgs/msg/u_int8_vector.hpp"

#define GLOBAL_VELS_NUM 3


using std::placeholders::_1;


class DriveController : public rclcpp::Node {
private:
    rclcpp::Publisher<robot_msgs::msg::UInt8Vector>::SharedPtr serial_pub_;
    rclcpp::Subscription<robot_msgs::msg::UInt8Vector>::SharedPtr serial_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

    size_t pose_num_, pose_size_, vel_num_, vel_size_;
    double r_, lx_, ly_, ticks_;
    std::string frame_id_, child_frame_id_;

public:
    DriveController();

private:
    std::vector<float> calcForwardKinematics(std::vector<float> V);
    std::vector<float> calcInverseKinematics(std::vector<float> W);

    std::vector<float> ticks2rads(std::vector<int64_t> P);
    std::vector<float> calcGlobalPose(std::vector<int64_t> P);
    
    void odomCallback(const robot_msgs::msg::UInt8Vector& msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist& msg);
};


DriveController::DriveController() : Node("drive_controller") {
    this->declare_parameter("odom_pub", "");
    this->declare_parameter("cmd_vel_sub", "");
    this->declare_parameter("serial_pub", "");
    this->declare_parameter("serial_sub", "");
    this->declare_parameter("r", 0.0);
    this->declare_parameter("lx", 0.0);
    this->declare_parameter("ly", 0.0);
    this->declare_parameter("ticks", 0.0);
    this->declare_parameter("frame_id", "");
    this->declare_parameter("child_frame_id", "");
    this->declare_parameter("pose_num", 0);
    this->declare_parameter("pose_size", 0);
    this->declare_parameter("vel_num", 0);
    this->declare_parameter("vel_size", 0);

    std::string odom_pub_topic = this->get_parameter("odom_pub").as_string();
    std::string cmd_vel_sub_topic = this->get_parameter("cmd_vel_sub").as_string();
    std::string serial_pub_topic = this->get_parameter("serial_pub").as_string();
    std::string serial_sub_topic = this->get_parameter("serial_sub").as_string();
    r_ = this->get_parameter("r").as_double();
    lx_ = this->get_parameter("lx").as_double();
    ly_ = this->get_parameter("ly").as_double();
    ticks_ = this->get_parameter("ticks").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();
    pose_num_ = this->get_parameter("pose_num").as_int();
    pose_size_ = this->get_parameter("pose_size").as_int();
    vel_num_ = this->get_parameter("vel_num").as_int();
    vel_size_ = this->get_parameter("vel_size").as_int();

    RCLCPP_INFO(this->get_logger(), "serial_pub_topic: '%s'", serial_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_sub_topic: '%s'", serial_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_pub_topic: '%s'", odom_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "vel_sub_topic: '%s'", cmd_vel_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "r: %lf", r_);
    RCLCPP_INFO(this->get_logger(), "lx: %lf", lx_);
    RCLCPP_INFO(this->get_logger(), "ly: %lf", ly_);
    RCLCPP_INFO(this->get_logger(), "ticks: %lf", ticks_);
    RCLCPP_INFO(this->get_logger(), "frame_id: '%s'", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "child_frame_id: '%s'", child_frame_id_.c_str());
    
    RCLCPP_INFO(this->get_logger(), "vel_num: %ld", pose_num_);
    RCLCPP_INFO(this->get_logger(), "vel_size: %ld", pose_size_);
    RCLCPP_INFO(this->get_logger(), "vel_num: %ld", vel_num_);
    RCLCPP_INFO(this->get_logger(), "vel_size: %ld", vel_size_);

    serial_pub_ = this->create_publisher<robot_msgs::msg::UInt8Vector>(serial_pub_topic, 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_pub_topic, 10);

    serial_sub_ = this->create_subscription<robot_msgs::msg::UInt8Vector>(serial_sub_topic, 10, std::bind(&DriveController::odomCallback, this, _1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_sub_topic, 10, std::bind(&DriveController::cmdVelCallback, this, _1));

    tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}


std::vector<float> DriveController::calcForwardKinematics(std::vector<float> V) {
    if (V.size() != GLOBAL_VELS_NUM) {
        RCLCPP_FATAL(this->get_logger(), "Wrong cmd_vel_size");
        rclcpp::shutdown();
    }

    float vx = V[0];
    float vy = V[1];
    float wz = V[2];

    std::vector<float> W;
    W.resize(vel_num_);

    W[0] = (vx - vy - (lx_ + ly_) * wz) / r_;
    W[1] = (vx + vy + (lx_ + ly_) * wz) / r_;
    W[2] = (vx + vy - (lx_ + ly_) * wz) / r_;
    W[3] = (vx - vy + (lx_ + ly_) * wz) / r_;

    return W;
}


std::vector<float> DriveController::calcInverseKinematics(std::vector<float> W) {
    if (W.size() != vel_num_) {
        RCLCPP_FATAL(this->get_logger(), "Wrong odom_velocities_size");
        rclcpp::shutdown();
    }

    float w0 = W[0];
    float w1 = W[1];
    float w2 = W[2];
    float w3 = W[3];

    std::vector<float> V;
    V.resize(GLOBAL_VELS_NUM);

    V[0] = (+w0 + w1 + w2 + w3) * r_ / 4;
    V[1] = (-w0 + w1 + w2 - w3) * r_ / 4;
    V[2] = (-w0 + w1 - w2 + w3) * r_ / 4 / (lx_ + ly_);

    return V;
}


std::vector<float> DriveController::ticks2rads(std::vector<int64_t> P) {
    std::vector<float> R;
    for (size_t i = 0; i < P.size(); i++) {
        R.push_back(2 * 3.14 * P[i] / ticks_);
    }

    return R;
 }


std::vector<float> DriveController::calcGlobalPose(std::vector<int64_t> P) {
    if (P.size() != pose_num_) {
        RCLCPP_FATAL(this->get_logger(), "Wrong odom_poses_size");
        rclcpp::shutdown();
    }

    std::vector<float> R = ticks2rads(P);

    float p0 = R[0];
    float p1 = R[1];
    float p2 = R[2];
    float p3 = R[3];

    std::vector<float> X;
    X.resize(GLOBAL_VELS_NUM);

    X[0] = (+p0 + p1 + p2 + p3) * r_ / 4;
    X[1] = (-p0 + p1 + p2 - p3) * r_ / 4;
    X[2] = (-p0 + p1 - p2 + p3) * r_ / 4 / (lx_ + ly_);

    return X;
}


void DriveController::odomCallback(const robot_msgs::msg::UInt8Vector& msg) {
    std::vector<int64_t> P;
    std::vector<float> W;

    for (size_t i = 0; i < pose_num_; i++) {
        int64_t pose;
        std::memcpy(&pose, msg.data.data() + i * pose_size_, pose_size_);
        P.push_back(pose);
        std::cout << pose << " ";
    }
    std::cout << std::endl;

    for (size_t i = 0; i < pose_num_; i++) {
        float w;
        std::memcpy(&w, msg.data.data() + pose_num_ * pose_size_ + i * vel_size_, vel_size_);
        W.push_back(w);
    }

    auto odom = nav_msgs::msg::Odometry();

    odom.header.frame_id = frame_id_;
    odom.header.stamp = this->get_clock()->now();
    odom.child_frame_id = child_frame_id_;

    std::vector<float> X = calcGlobalPose(P);

    odom.pose.pose.position.x = X[0];
    odom.pose.pose.position.y = X[1];
    odom.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, X[2]);

    std::vector<float> V = calcInverseKinematics(W);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
/*
    odom.twist.twist.linear.x = V[0];
    odom.twist.twist.linear.y = V[1];
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = V[2];
*/
    odom_pub_->publish(odom);

    auto transform = geometry_msgs::msg::TransformStamped();

    transform.header.stamp = odom.header.stamp;
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;

    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.x = odom.pose.pose.orientation.x;
    transform.transform.rotation.y = odom.pose.pose.orientation.y;
    transform.transform.rotation.z = odom.pose.pose.orientation.z;
    transform.transform.rotation.w = odom.pose.pose.orientation.w;

    tf2_broadcaster_->sendTransform(transform);

    //std::cout << "ODOM: " << V[0] << " " << V[1] << " " << V[2] << " " << W[0] << " " << W[1] << " " << W[2] << " " << W[3] << std::endl;
}


void DriveController::cmdVelCallback(const geometry_msgs::msg::Twist& msg) {

    std::vector<float> V;
    V.push_back(msg.linear.x);
    V.push_back(msg.linear.y);
    V.push_back(msg.angular.z);

    std::vector<float> W = calcForwardKinematics(V);

    auto serial_msg = robot_msgs::msg::UInt8Vector();
    serial_msg.data.resize(vel_num_ * vel_size_);

    for (size_t i = 0; i < W.size(); i++) {
        std::memcpy(serial_msg.data.data() + i * vel_size_, W.data() + i, vel_size_);
    }

    serial_pub_->publish(serial_msg);

    //std::cout << "CMD_VEL: " << V[0] << " " << V[1] << " " << V[2] << " " << W[0] << " " << W[1] << " " << W[2] << " " << W[3] << std::endl;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveController>()); 
    rclcpp::shutdown();

    return 0;
}
