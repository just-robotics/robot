#include <cstdint>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"


#define MIN_MOTOR_ID 0
#define MAX_MOTOR_ID 3


class PidPublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr k_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr motor_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr start_publisher_;

    rclcpp::TimerBase::SharedPtr parse_timer_;

    float kp_, kd_, ki_;
    uint8_t motor_;

public:
    PidPublisher();

private:
    void selectMotor(uint8_t motor_);
    void start_rotation();

    void publish_k();
    void set_kp(float kp);
    void set_kd(float kd);
    void set_ki(float ki);

    bool isPosFloat(std::string str);
    void parse();
};


PidPublisher::PidPublisher() : Node("pid_publisher") {
    k_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pid_regulator/pid", 10);
    motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("/pid_regulator/motor_id", 10);
    start_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("/pid_regulator/start_rotation", 10);

    using namespace std::chrono_literals;
    parse_timer_ = this->create_wall_timer(100ms, std::bind(&PidPublisher::parse, this));

    kp_ = 1.0;
    kd_ = 0.0;
    ki_ = 0.0;
    motor_ = 0;
}


void PidPublisher::selectMotor(uint8_t motor) {
    if (motor_ == motor) {
        return;
    }

    motor_ = motor;
    auto msg = std_msgs::msg::UInt8();
    msg.data = motor_;
    motor_publisher_->publish(msg);
}


void PidPublisher::start_rotation() {
    start_publisher_->publish(std_msgs::msg::UInt8());
}


void PidPublisher::publish_k() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.push_back(kp_);
    msg.data.push_back(kd_);
    msg.data.push_back(ki_);

    k_publisher_->publish(msg);

    std::cout << "# kp = " << kp_ << "; kd = " << kd_ << "; ki = " << ki_ << std::endl;
}


void PidPublisher::set_kp(float kp) {
    kp_ = kp;
    publish_k();
}


void PidPublisher::set_kd(float kd) {
    kd_ = kd;
    publish_k();
}


void PidPublisher::set_ki(float ki) {
    ki_ = ki;
    publish_k();
}


bool PidPublisher::isPosFloat(std::string str) {
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


void PidPublisher::parse() {

    std::string motor_cmd = "m";
    std::string reset_cmd = "reset";
    std::string kp_cmd = "kp";
    std::string kd_cmd = "kd";
    std::string ki_cmd = "ki";
    std::string start_cmd = "s";

    std::string input;

    std::cin >> input;

    if (std::cin.fail()) {
        return;
    }

    if (input == motor_cmd) {
        std::cin >> input;
        int motor = std::stoi(input);
        if (MIN_MOTOR_ID <= motor && motor <= MAX_MOTOR_ID) {
            std::cout << "# motor " << motor << " selected" << std::endl;
            return selectMotor(motor);
        }
    }

    if (input == kp_cmd) {
        std::cin >> input;
        if (isPosFloat(input)) {
            return set_kp(std::stof(input));
        }
    }

    if (input == kd_cmd) {
        std::cin >> input;
        if (isPosFloat(input)) {
            return set_kd(std::stof(input));
        }
    }

    if (input == ki_cmd) {
        std::cin >> input;
        if (isPosFloat(input)) {
            return set_ki(std::stof(input));
        }
    }

    if (input == start_cmd) {
        return start_rotation();
    }

    std::cout << "# wrong input" << std::endl;
}


int main(int argc, char** argv) {
    std::cout << "Enter to change motor: m <motor_id>" << std::endl;
    std::cout << "Enter to change kp: kp <kp_float_value>" << std::endl;
    std::cout << "Enter to change kd: kd <kd_float_value>" << std::endl;
    std::cout << "Enter to change ki: ki <ki_float_value>" << std::endl;
    std::cout << "Enter to start rotation: s" << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidPublisher>());
    rclcpp::shutdown();

    return 0;
}
