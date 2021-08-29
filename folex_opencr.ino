/******************************************************************************
* Copyright 2021 Hyunwook Choi (Daniel Choi)
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
******************************************************************************/

#include <string>

#include "folex_opencr.h"


void setup()
{
  // // Debug serial
  // Serial.begin(115200);
  // while (!Serial)
  // {
  //   // Wait
  // }

  // ROS
  nh.initNode();

  nh.subscribe(target_joint_sub);
  
  nh.advertise(joint_state_pub);

  initJointState();

  dynamixel_driver.initialize();
  dynamixel_driver.enable();
  dynamixel_driver.resetPosition();
}

void loop()
{
  publishJointStates();

  nh.spinOnce();

  delay(1);
}


void initJointState()
{
  static char *joint_state_names[] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                                      "joint7", "joint8", "joint9", "joint10", "joint11", "joint12"};

  joint_state_msg.name = joint_state_names;

  joint_state_msg.name_length = joint_num;
  joint_state_msg.position_length = joint_num;
  joint_state_msg.velocity_length = joint_num;
}

void updateJointStates()
{
  static float joint_state_pos[20] = {0.0, };
  static float joint_state_vel[20] = {0.0, };

  float present_joint_position[joint_num];
  float present_joint_velocity[joint_num];

  dynamixel_driver.getPresentPosition(present_joint_position);

  for (uint8_t i = 0; i < joint_num; i++)
  {
    joint_state_pos[i] = present_joint_position[i];
  }

  // Save data in JointState message
  joint_state_msg.position = joint_state_pos;
  joint_state_msg.velocity = joint_state_vel;
}

void publishJointStates()
{
  updateJointStates();
  joint_state_msg.header.stamp = nh.now();
  joint_state_pub.publish(&joint_state_msg);
}


void targetJointCallback(const sensor_msgs::JointState &msg)
{
  for (uint8_t i = 0; i < 3; i++)
  {
    dynamixel_driver.writeValueGoalVelocity(i, msg.velocity[i]);
    dynamixel_driver.writeValueGoalPosition(i, msg.position[i]);
  }
}