#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "robot_msgs/msg/u_int8_vector.hpp"
#include "robot_msgs/msg/float32_vector.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


class DriveController : public rclcpp::Node {
private:
    const size_t GLOBAL_VELS_NUM_ = 3;
    const size_t GLOBAL_POSES_NUM_ = 3;

    const float M_2PI_ = 6.28;
    
    size_t reset_timeout_;
    float kp_, ki_, kd_;

    const size_t PID_NUM_ = 3;
    const size_t PID_SIZE_ = 4;

    rclcpp::Publisher<robot_msgs::msg::UInt8Vector>::SharedPtr serial_pub_;
    rclcpp::Subscription<robot_msgs::msg::UInt8Vector>::SharedPtr serial_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_pub_;
    rclcpp::Subscription<robot_msgs::msg::Float32Vector>::SharedPtr pid_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;


    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

    size_t cmd_size_, pose_num_, pose_size_, vel_num_, vel_size_;
    size_t target_num_, target_size_;
    double r_, lx_, ly_, ticks_;
    std::string frame_id_, child_frame_id_;

    std::vector<float> reset_X_, reset_P_, prev_X_, prev_P_;

public:
    DriveController();

private:
    std::vector<float> calcForwardKinematics(std::vector<float> V);
    std::vector<float> calcInverseKinematics(std::vector<float> W);

    std::vector<float> ticks2rads(std::vector<int64_t> T);
    std::vector<int64_t> rads2ticks(std::vector<float> P);

    std::vector<float> calcGlobalPose(std::vector<int64_t> P);
    std::vector<int64_t> calcLocalPose(std::vector<float> X);
    
    void odomCallback(const robot_msgs::msg::UInt8Vector& msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist& msg);
    void pidCallback(const robot_msgs::msg::Float32Vector& msg);
    void resetOdomCallback(const std_msgs::msg::Bool& msg);
};


DriveController::DriveController() : Node("drive_controller") {
    this->declare_parameter("odom_pub", "");
    this->declare_parameter("cmd_vel_sub", "");
    this->declare_parameter("serial_pub", "");
    this->declare_parameter("serial_sub", "");
    this->declare_parameter("pid_sub", "");
    this->declare_parameter("reset_sub", "");
    this->declare_parameter("r", 0.0);
    this->declare_parameter("lx", 0.0);
    this->declare_parameter("ly", 0.0);
    this->declare_parameter("ticks", 0.0);
    this->declare_parameter("frame_id", "");
    this->declare_parameter("child_frame_id", "");
    this->declare_parameter("reset_timeout", 0);
    this->declare_parameter("kp", 0.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("cmd_size", 0);
    this->declare_parameter("pose_num", 0);
    this->declare_parameter("pose_size", 0);
    this->declare_parameter("vel_num", 0);
    this->declare_parameter("vel_size", 0);
    this->declare_parameter("target_num", 0);
    this->declare_parameter("target_size", 0);

    std::string odom_pub_topic = this->get_parameter("odom_pub").as_string();
    std::string cmd_vel_sub_topic = this->get_parameter("cmd_vel_sub").as_string();
    std::string serial_pub_topic = this->get_parameter("serial_pub").as_string();
    std::string serial_sub_topic = this->get_parameter("serial_sub").as_string();
    std::string pid_sub_topic = this->get_parameter("pid_sub").as_string();
    std::string reset_sub_topic = this->get_parameter("reset_sub").as_string();
    r_ = this->get_parameter("r").as_double();
    lx_ = this->get_parameter("lx").as_double();
    ly_ = this->get_parameter("ly").as_double();
    ticks_ = this->get_parameter("ticks").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();
    reset_timeout_ = this->get_parameter("reset_timeout").as_int();
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    cmd_size_ = this->get_parameter("cmd_size").as_int();
    pose_num_ = this->get_parameter("pose_num").as_int();
    pose_size_ = this->get_parameter("pose_size").as_int();
    vel_num_ = this->get_parameter("vel_num").as_int();
    vel_size_ = this->get_parameter("vel_size").as_int();
    target_num_ = this->get_parameter("target_num").as_int();
    target_size_ = this->get_parameter("target_size").as_int();

    RCLCPP_INFO(this->get_logger(), "odom_pub_topic: '%s'", odom_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "vel_sub_topic: '%s'", cmd_vel_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_pub_topic: '%s'", serial_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_sub_topic: '%s'", serial_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "pid_sub_topic: '%s'", pid_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "reset_sub_topic: '%s'", reset_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "r: %lf", r_);
    RCLCPP_INFO(this->get_logger(), "lx: %lf", lx_);
    RCLCPP_INFO(this->get_logger(), "ly: %lf", ly_);
    RCLCPP_INFO(this->get_logger(), "ticks: %lf", ticks_);
    RCLCPP_INFO(this->get_logger(), "frame_id: '%s'", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "child_frame_id: '%s'", child_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "reset_timeout: %ld", reset_timeout_);
    RCLCPP_INFO(this->get_logger(), "kp: %lf", kp_);
    RCLCPP_INFO(this->get_logger(), "ki: %lf", ki_);
    RCLCPP_INFO(this->get_logger(), "kd: %lf", kd_);
    RCLCPP_INFO(this->get_logger(), "cmd_size: %ld", cmd_size_);
    RCLCPP_INFO(this->get_logger(), "vel_num: %ld", pose_num_);
    RCLCPP_INFO(this->get_logger(), "vel_size: %ld", pose_size_);
    RCLCPP_INFO(this->get_logger(), "vel_num: %ld", vel_num_);
    RCLCPP_INFO(this->get_logger(), "vel_size: %ld", vel_size_);
    RCLCPP_INFO(this->get_logger(), "vel_num: %ld", target_num_);
    RCLCPP_INFO(this->get_logger(), "vel_size: %ld", target_size_);

    serial_pub_ = this->create_publisher<robot_msgs::msg::UInt8Vector>(serial_sub_topic, 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_pub_topic, 10);

    pose_pub_ = this->create_publisher<std_msgs::msg::Int64>("/dbg/pose", 10);
    target_pub_ = this->create_publisher<std_msgs::msg::Float32>("/dbg/target", 10);

    serial_sub_ = this->create_subscription<robot_msgs::msg::UInt8Vector>(serial_pub_topic, 10, std::bind(&DriveController::odomCallback, this, _1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_sub_topic, 10, std::bind(&DriveController::cmdVelCallback, this, _1));

    pid_sub_ = this->create_subscription<robot_msgs::msg::Float32Vector>(pid_sub_topic, 10, std::bind(&DriveController::pidCallback, this, _1));
    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(reset_sub_topic, 10, std::bind(&DriveController::resetOdomCallback, this, _1));

    tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    reset_X_ = {0, 0, 0, 0};
    reset_P_ = {0, 0, 0, 0};

    prev_X_ = reset_X_;
    prev_P_ = reset_P_;
}


std::vector<float> DriveController::calcForwardKinematics(std::vector<float> V) {
    if (V.size() != GLOBAL_VELS_NUM_) {
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
    V.resize(GLOBAL_VELS_NUM_);

    V[0] = (+w0 + w1 + w2 + w3) * r_ / 4;
    V[1] = (-w0 + w1 + w2 - w3) * r_ / 4;
    V[2] = (-w0 + w1 - w2 + w3) * r_ / 4 / (lx_ + ly_);

    return V;
}


std::vector<float> DriveController::ticks2rads(std::vector<int64_t> T) {
    std::vector<float> P;
    for (size_t i = 0; i < T.size(); i++) {
        P.push_back(M_2PI_ * T[i] / ticks_);
    }

    return P;
 }


std::vector<int64_t> DriveController::rads2ticks(std::vector<float> P) {
    std::vector<int64_t> T;
    for (size_t i = 0; i < P.size(); i++) {
        T.push_back(P[i] * ticks_ / M_2PI_);
    }

    return T;
 }


std::vector<float> DriveController::calcGlobalPose(std::vector<int64_t> T) {
    if (T.size() != pose_num_) {
        RCLCPP_FATAL(this->get_logger(), "Wrong odom_poses_size");
        rclcpp::shutdown();
    }

    std::vector<float> P = ticks2rads(T);

    std::vector<float> X;
    X.resize(GLOBAL_VELS_NUM_);

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

    return X;
}


std::vector<int64_t> DriveController::calcLocalPose(std::vector<float> X) {
    if (X.size() != GLOBAL_POSES_NUM_) {
        RCLCPP_FATAL(this->get_logger(), "Wrong global_poses_num");
        rclcpp::shutdown();
    }

    float x = X[0];
    float y = X[1];
    float z = X[2];

    std::vector<float> P;
    P.resize(vel_num_);

    P[0] = (x - y - (lx_ + ly_) * z) / r_;
    P[1] = (x + y + (lx_ + ly_) * z) / r_;
    P[2] = (x + y - (lx_ + ly_) * z) / r_;
    P[3] = (x - y + (lx_ + ly_) * z) / r_;

    std::vector<int64_t> T = rads2ticks(P);

    return T;
}


void DriveController::odomCallback(const robot_msgs::msg::UInt8Vector& msg) {
    std::vector<int64_t> P;
    std::vector<float> W;
    std::vector<float> T;

    for (size_t i = 0; i < pose_num_; i++) {
        int64_t pose;
        std::memcpy(&pose, msg.data.data() + i * pose_size_, pose_size_);
        P.push_back(pose);
    }

    for (size_t i = 0; i < pose_num_; i++) {
        float w;
        std::memcpy(&w, msg.data.data() + pose_num_ * pose_size_ + i * vel_size_, vel_size_);
        W.push_back(w);
    }

    for (size_t i = 0; i < target_num_; i++) {
        float t;
        std::memcpy(&t, msg.data.data() + pose_num_ * pose_size_ + vel_num_ * vel_size_ + i * target_size_, target_size_);
        T.push_back(t);
    }

    auto pose_msg = std_msgs::msg::Int64();
    auto target_msg = std_msgs::msg::Float32();

    pose_msg.data = P[0];
    target_msg.data = T[0];

    pose_pub_->publish(pose_msg);
    target_pub_->publish(target_msg);

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

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
/*
    std::vector<float> V = calcInverseKinematics(W);

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
    transform.transform.translation.z = odom.pose.pose.position.z;

    transform.transform.rotation.x = odom.pose.pose.orientation.x;
    transform.transform.rotation.y = odom.pose.pose.orientation.y;
    transform.transform.rotation.z = odom.pose.pose.orientation.z;
    transform.transform.rotation.w = odom.pose.pose.orientation.w;

    tf2_broadcaster_->sendTransform(transform);
}


void DriveController::cmdVelCallback(const geometry_msgs::msg::Twist& msg) {
    auto serial_msg = robot_msgs::msg::UInt8Vector();
    serial_msg.data.resize(cmd_size_);

    std::vector<float> V;
    V.push_back(msg.linear.x);
    V.push_back(msg.linear.y);
    V.push_back(msg.angular.z);

    std::vector<float> W = calcForwardKinematics(V);

    for (size_t i = 0; i < W.size(); i++) {
        std::memcpy(serial_msg.data.data() + i * vel_size_, W.data() + i, vel_size_);
    }
    
    std::memcpy(serial_msg.data.data() + vel_num_ * vel_size_ + PID_SIZE_ * 0, &kp_, PID_SIZE_);
    std::memcpy(serial_msg.data.data() + vel_num_ * vel_size_ + PID_SIZE_ * 1, &ki_, PID_SIZE_);
    std::memcpy(serial_msg.data.data() + vel_num_ * vel_size_ + PID_SIZE_ * 2, &kd_, PID_SIZE_);

    serial_pub_->publish(serial_msg);
}


void DriveController::pidCallback(const robot_msgs::msg::Float32Vector& msg) {
    kp_ = msg.data[0];
    ki_ = msg.data[1];
    kd_ = msg.data[2];

    cmdVelCallback(geometry_msgs::msg::Twist());
}


void DriveController::resetOdomCallback(const std_msgs::msg::Bool& msg) {
    if (msg.data == false) {
        return;
    }

    auto serial_msg = robot_msgs::msg::UInt8Vector();
    serial_msg.data.resize(cmd_size_);

    size_t reset_idx = vel_num_ * vel_size_ + PID_NUM_ * PID_SIZE_;
    serial_msg.data[reset_idx] = 1;

    serial_pub_->publish(serial_msg);

    sleep(reset_timeout_);
    prev_X_ = reset_X_;
    prev_P_ = reset_P_;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveController>()); 
    rclcpp::shutdown();

    return 0;
}
