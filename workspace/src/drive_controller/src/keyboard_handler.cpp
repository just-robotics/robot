#include <cstdint>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include "robot_msgs/msg/float32_vector.hpp"


class KeyboardHandler : public rclcpp::Node {
private:
    rclcpp::Publisher<robot_msgs::msg::Float32Vector>::SharedPtr k_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_publisher_;
    rclcpp::TimerBase::SharedPtr parse_timer_;

    size_t reset_timeout_;
    float kp_, kd_, ki_;

public:
    KeyboardHandler();

    void print_k();

private:
    void reset();
    void publish_k();
    void set_kp(float kp);
    void set_kd(float kd);
    void set_ki(float ki);

    bool isPosFloat(std::string str);
    void parse();
};


KeyboardHandler::KeyboardHandler() : Node("keyboard_handler") {
    std::cout << "# Enter to change kp: kp <kp_float_value>" << std::endl;
    std::cout << "# Enter to change ki: ki <ki_float_value>" << std::endl;
    std::cout << "# Enter to change kd: kd <kd_float_value>" << std::endl;
    std::cout << "# Enter to reset: r" << std::endl;

    this->declare_parameter("pid_pub", "");
    this->declare_parameter("reset_pub", "");
    this->declare_parameter("reset_timeout", 0);
    this->declare_parameter("kp", 0.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);

    std::string pid_pub_topic = this->get_parameter("pid_pub").as_string();
    std::string reset_pub_topic = this->get_parameter("reset_pub").as_string();
    reset_timeout_ = this->get_parameter("reset_timeout").as_int();
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();

    RCLCPP_INFO(this->get_logger(), "pid_pub_topic: '%s'", pid_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "reset_pub_topic: '%s'", reset_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "reset_timeout: %ld", reset_timeout_);
    RCLCPP_INFO(this->get_logger(), "kp: %lf", kp_);
    RCLCPP_INFO(this->get_logger(), "ki: %lf", ki_);
    RCLCPP_INFO(this->get_logger(), "kd: %lf", kd_);

    k_publisher_ = this->create_publisher<robot_msgs::msg::Float32Vector>(pid_pub_topic, 10);
    reset_publisher_ = this->create_publisher<std_msgs::msg::Bool>(reset_pub_topic, 10);

    using namespace std::chrono_literals;
    parse_timer_ = this->create_wall_timer(100ms, std::bind(&KeyboardHandler::parse, this));

    print_k();
}


void KeyboardHandler::print_k() {
    std::cout << "# kp = " << kp_ << "; ki = " << ki_ << "; kd = " << kd_ << std::endl;
}


void KeyboardHandler::reset() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    reset_publisher_->publish(msg);
    sleep(reset_timeout_);
    std::cout << "# reset sent" << std::endl;
}


void KeyboardHandler::publish_k() {
    auto msg = robot_msgs::msg::Float32Vector();
    msg.data.push_back(kp_);
    msg.data.push_back(ki_);
    msg.data.push_back(kd_);

    k_publisher_->publish(msg);

    print_k();
}


void KeyboardHandler::set_kp(float kp) {
    kp_ = kp;
    publish_k();
}


void KeyboardHandler::set_ki(float ki) {
    ki_ = ki;
    publish_k();
}


void KeyboardHandler::set_kd(float kd) {
    kd_ = kd;
    publish_k();
}


bool KeyboardHandler::isPosFloat(std::string str) {
    if (str.size() == 0) {
        return false;
    }

    if (str[0] != '+') {
        if (!std::isdigit(str[0])) {
            return false;
        }
    }

    bool dot = false;

    auto it = str.begin() + 1;

    while (it != str.end()) {
        if (std::isdigit(*it)) {
            it++;
            continue;
        }
        if (*it == '.' && !dot) {
            it++;
            dot = true;
            continue;
        }
        if (*it == 'f' && (it + 1) == str.end() && dot) {
            it++;
            continue;
        }
        return false;
    }
    return true;
}


void KeyboardHandler::parse() {
    std::string reset_cmd = "r";
    std::string kp_cmd = "kp";
    std::string ki_cmd = "ki";
    std::string kd_cmd = "kd";

    std::string input;

    std::cin >> input;

    if (std::cin.fail()) {
        return;
    }

    if (input == kp_cmd) {
        std::cin >> input;
        if (isPosFloat(input)) {
            return set_kp(std::stof(input));
        }
    }

    if (input == ki_cmd) {
        std::cin >> input;
        if (isPosFloat(input)) {
            return set_ki(std::stof(input));
        }
    }

    if (input == kd_cmd) {
        std::cin >> input;
        if (isPosFloat(input)) {
            return set_kd(std::stof(input));
        }
    }

    if (input == reset_cmd) {
        return reset();
    }

    std::cout << "# wrong input" << std::endl;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardHandler>());
    rclcpp::shutdown();

    return 0;
}
