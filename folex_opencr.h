//

#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "src/dynamixel_driver.h"


#define JOINT_NUM 12

double present_time = 0.0;
double previous_time = 0.0;
double control_time = 2;

uint8_t joint_num = JOINT_NUM;


void targetJointCallback(const sensor_msgs::JointState &msg);

ros::NodeHandle nh;

ros::Subscriber<sensor_msgs::JointState> target_joint_sub("target_joint", targetJointCallback);

sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("opencr/joint_state", &joint_state_msg);

Folex::DynamixelDriver dynamixel_driver;
