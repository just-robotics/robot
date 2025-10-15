#include <ros.h>
#include <std_msgs/Int64MultiArray.h>

#include "config.h"
#include "motor.h"


uint64_t start_time;

int64_t odom_array[MOTORS];
std_msgs::Int64MultiArray odom_msg;

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32MultiArray> cmd_vel_sub("/robot1/drive_controller/velocities", &Motor::callback);
ros::Subscriber<std_msgs::Bool> reset_sub("/robot1/drive_controller/reset", &Motor::resetMotors);
ros::Publisher odom_pub("/robot1/drive_controller/arduino_odom", &odom_msg);


void setup() {    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Motor::init();

    odom_msg.data_length = MOTORS;
    odom_msg.data = odom_array;

    nh.initNode();
    nh.advertise(odom_pub);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(reset_sub);

    start_time = millis();
}


void loop() {
    nh.spinOnce();
    
    Motor::spinMotors();

    if (millis() - start_time >= 20) {
        Motor::fillOdomMsg(odom_msg);
        odom_pub.publish(&odom_msg);
        start_time = millis();
    }
}
