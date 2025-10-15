/*
 * rosserial::std_msgs::Float64MultiArray Test
 * Receives a Float64 input, subtracts 1.0, and publishes it as an array of 4 elements
 */

#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

// массив из 4 элементов
float x_array[4];
std_msgs::Float64MultiArray test;

ros::Publisher p("my_topic", &test);

void messageCb(const std_msgs::Float64& msg) {
  float x = msg.data;

  // заполняем массив 4 элементами
  for (int i = 0; i < 4; i++) {
    x_array[i] = x - 1.0;  // уменьшаем на 1.0
  }

  // обновляем данные сообщения
  test.data_length = 4;
  test.data = x_array;

  digitalWrite(13, HIGH - digitalRead(13));   // моргаем светодиодом
  p.publish(&test);
}

ros::Subscriber<std_msgs::Float64> s("your_topic", &messageCb);

void setup() {
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
}

void loop() {
  nh.spinOnce();
}
