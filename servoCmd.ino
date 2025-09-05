#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

Servo servo;
int servoPin = 8;  

void servoCallback(const std_msgs::Int32& msg) {
  int angle = msg;  
  servo.Write(angle);
}

ros::Subscriber<std_msgs::Int32> sub("servo_cmd", &servoCallback);

void setup() {
  myServo.attach(servoPin);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}