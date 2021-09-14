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

#ifndef DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_DRIVER_H

#include <Arduino.h>    // DEBUG, LED
#include <DynamixelSDK.h>


// Joint number
#define JOINT_0     0
#define JOINT_1     1
#define JOINT_2     2
#define JOINT_3     3
#define JOINT_4     4
#define JOINT_5     5
#define JOINT_6     6
#define JOINT_7     7
#define JOINT_8     8
#define JOINT_9     9
#define JOINT_10    10
#define JOINT_11    11
#define JOINT_NUM   12

/* DYNAMIXEL AX-12A */
// http://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table
// Model number
#define AX_12A                    12
// Control table address
#define ADDR_AX_TORQUE_ENABLE     24
#define ADDR_AX_GOAL_POSITION     30
#define ADDR_AX_MOVING_SPEED      32
#define ADDR_AX_PRESENT_POSITION  36
#define ADDR_AX_PRESENT_SPEED     38
// Origin degree
#define ORIGIN_AX_DEGREE          150

/* DYNAMIXEL XL430-W250-T */
// http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table
// Model number
#define XL430_W250                1060
// Control table address
#define ADDR_XL_TORQUE_ENABLE     64
#define ADDR_XL_PROFILE_VELOCITY  112
#define ADDR_XL_GOAL_POSITION     116
#define ADDR_XL_PRESENT_POSITION  132
// Origin degree
#define ORIGIN_XL_DEGREE          180

/* POSITION LIMITS */
// To-Do

/* VELOCITY LIMITS */
#define LIMIT_AX_VELOCITY         120
#define LIMIT_XL_VELOCITY         120

// Dynamixel parameters
#define BAUDRATE                  1000000
#define DEVICENAME                ""          // OpenCR : Empty
#define PROTOCOL_VERSION          1.0

// Dynamixel torque control
#define TORQUE_DISABLE            0
#define TORQUE_ENABLE             1

// Data Size
#define BYTE                      1
#define WORD                      2
#define DWORD                     4


namespace Folex
{
  class DynamixelDriver
  {
  private:
    uint32_t baudrate;
    float protocol_version;
    uint8_t joint_num = JOINT_NUM;

    // DYNAMIXEL position value per degree
    float dxl_ax_vpd = 1023.0F / 300.0F;
    float dxl_xl_vpd = 4095.0F / 360.0F;

    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupSyncRead *groupSyncRead_;

    // <ID, Model Number>
    std::map<uint8_t, uint16_t> all_joint;
    std::map<uint8_t, uint16_t> hip_joint;
    std::map<uint8_t, uint16_t> upper_leg_joint;
    std::map<uint8_t, uint16_t> lower_leg_joint;

    std::map<uint16_t, uint8_t> dxl_ax_address_array;  // <key, value> <Address, Size>
    std::map<uint16_t, uint8_t> dxl_xl_address_array;  // <key, value> <Address, Size>


  public:
    DynamixelDriver();
    ~DynamixelDriver();

    bool initialize();
    void close();
    bool setTorque(uint8_t id, bool onoff);
    bool enable();
    bool disable();

    bool readPresentPosition(uint8_t id);
    bool getPresentPosition(float *value);
    // bool writeGoalPosition(uint8_t id, uint32_t value);

    void setGoalPositionFromSerial(uint32_t data[]);
    void setGoalVelocityFromSerial(uint32_t data[]);


    bool writeValueGoalPosition(uint8_t id, uint32_t value);
    bool writeValueGoalVelocity(uint8_t id, uint32_t value);
    bool writeValue(uint8_t id, uint16_t address, uint32_t value);
    void resetPosition();

    // CONVERT FUNCTIONS
    void convertRadianToValue(float (&data)[12]);
    float convertValueToRadian(uint8_t id, uint32_t value);
    uint16_t convertRpmToValue(uint16_t dxl_model, double rpm);

    void test();
    void gaitTestFirst();
    void gaitTestSecond();
    void gaitTrot();
  };
}

#endif // DYNAMIXEL_DRIVER_H
