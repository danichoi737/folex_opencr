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

#include "dynamixel_driver.h"


Folex::DynamixelDriver::DynamixelDriver() : baudrate(BAUDRATE), protocol_version(PROTOCOL_VERSION)
{
  // Joint-Dynamixel mapping
  // All joint
  all_joint.insert(std::make_pair(JOINT_0, XL430_W250));     // LEFT-FRONT
  all_joint.insert(std::make_pair(JOINT_1, AX_12A));
  all_joint.insert(std::make_pair(JOINT_2, AX_12A));
  all_joint.insert(std::make_pair(JOINT_3, XL430_W250));     // RIGHT-FRONT
  all_joint.insert(std::make_pair(JOINT_4, AX_12A));
  all_joint.insert(std::make_pair(JOINT_5, AX_12A));
  all_joint.insert(std::make_pair(JOINT_6, XL430_W250));     // LEFT_HIND
  all_joint.insert(std::make_pair(JOINT_7, AX_12A));
  all_joint.insert(std::make_pair(JOINT_8, AX_12A));
  all_joint.insert(std::make_pair(JOINT_9, XL430_W250));     // RIGHT-HIND
  all_joint.insert(std::make_pair(JOINT_10, AX_12A));
  all_joint.insert(std::make_pair(JOINT_11, AX_12A));

  // Hip joint
  hip_joint.insert(std::make_pair(JOINT_0, XL430_W250));
  hip_joint.insert(std::make_pair(JOINT_3, XL430_W250));
  hip_joint.insert(std::make_pair(JOINT_6, XL430_W250));
  hip_joint.insert(std::make_pair(JOINT_9, XL430_W250));

  // Upper leg joint
  upper_leg_joint.insert(std::make_pair(JOINT_1, AX_12A));
  upper_leg_joint.insert(std::make_pair(JOINT_4, AX_12A));
  upper_leg_joint.insert(std::make_pair(JOINT_7, AX_12A));
  upper_leg_joint.insert(std::make_pair(JOINT_10, AX_12A));

  // Lower leg joint
  lower_leg_joint.insert(std::make_pair(JOINT_2, AX_12A));
  lower_leg_joint.insert(std::make_pair(JOINT_5, AX_12A));
  lower_leg_joint.insert(std::make_pair(JOINT_8, AX_12A));
  lower_leg_joint.insert(std::make_pair(JOINT_11, AX_12A));

  // Dynamixel Address and Size
  dxl_ax_address_array.insert(std::make_pair(ADDR_AX_TORQUE_ENABLE, BYTE));
  dxl_ax_address_array.insert(std::make_pair(ADDR_AX_GOAL_POSITION, WORD));
  dxl_ax_address_array.insert(std::make_pair(ADDR_AX_MOVING_SPEED, WORD));
  dxl_xl_address_array.insert(std::make_pair(ADDR_XL_TORQUE_ENABLE, BYTE));
  dxl_xl_address_array.insert(std::make_pair(ADDR_XL_GOAL_POSITION, DWORD));
  dxl_xl_address_array.insert(std::make_pair(ADDR_XL_PROFILE_VELOCITY, DWORD));
}

Folex::DynamixelDriver::~DynamixelDriver()
{
  close();
}

/*******************************************************************
** INITIALIZE AND CLOSE FUNCTIONS
********************************************************************/

bool Folex::DynamixelDriver::initialize()
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    return false;
  }

  // Set port baud rate
  if (portHandler_->setBaudRate(baudrate) == false)
  {
    return false;
  }

  Serial.println("Initialize Dynamixel driver completed.");
  return true;
}

void Folex::DynamixelDriver::close()
{
  // Close port
  portHandler_->closePort();
}


/*******************************************************************
** SET FUNCTIONS
********************************************************************/

bool Folex::DynamixelDriver::setTorque(uint8_t id, bool onoff)
{
  bool result = false;
  uint8_t error = 0;

  if (all_joint.find(id)->second == AX_12A)
  {
    result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_AX_TORQUE_ENABLE, onoff, &error);
  }
  else if (all_joint.find(id)->second == XL430_W250)
  {
    result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL_TORQUE_ENABLE, onoff, &error);
  }
  else
  {
    // error
  }

  return result;
}


/*******************************************************************
** ENABLE AND DISABLE FUNCTIONS
********************************************************************/

bool Folex::DynamixelDriver::enable()
{
  bool result = false;
  std::map<uint8_t, uint16_t>::iterator it_joint;

  for (it_joint = all_joint.begin(); it_joint != all_joint.end(); it_joint++)
  {
    result = setTorque(it_joint->first, TORQUE_ENABLE);
  }

  return result;
}

bool Folex::DynamixelDriver::disable()
{
  bool result = false;
  std::map<uint8_t, uint16_t>::iterator it_joint;

  for (it_joint = all_joint.begin(); it_joint != all_joint.end(); it_joint++)
  {
    result = setTorque(it_joint->first, TORQUE_DISABLE);
  }

  return result;
}


/*******************************************************************
** READ FUNCTIONS
********************************************************************/

bool Folex::DynamixelDriver::readPresentPosition(uint8_t id)
{
  bool result = false;
  uint8_t error = 0;

  uint16_t read_value_ax = 0;
  uint32_t read_value_xl = 0;

  uint32_t present_position = 0;

  if (all_joint.find(id)->second == AX_12A)
  {
    result = packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_AX_PRESENT_POSITION, &read_value_ax, &error);
    present_position = read_value_ax;
  }
  else if (all_joint.find(id)->second == XL430_W250)
  {
    result = packetHandler_->read4ByteTxRx(portHandler_, id, ADDR_XL_PRESENT_POSITION, &read_value_xl, &error);
    present_position = read_value_xl;
  }
  else
  {
    // error
  }

  // DEBUG
  if (result == true)
  {
    Serial.println(packetHandler_->getTxRxResult(result));
  }
  else
  {
    Serial.println(present_position);
  }

  return result;
}

/*******************************************************************
** GET FUNCTIONS
********************************************************************/

bool Folex::DynamixelDriver::getPresentPosition(float *value)
{
  bool result = false;
  uint8_t error = 0;

  uint16_t read_value_ax = 0;
  uint32_t read_value_xl = 0;

  for (uint8_t id = 0; id < joint_num; id++)
  {
    if (all_joint.find(id)->second == AX_12A)
    {
      result = packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_AX_PRESENT_POSITION, &read_value_ax, &error);
      // value[id] = convertValueToRadian(id, read_value_ax);
      value[id] = read_value_ax;
    }
    else if (all_joint.find(id)->second == XL430_W250)
    {
      result = packetHandler_->read4ByteTxRx(portHandler_, id, ADDR_XL_PRESENT_POSITION, &read_value_xl, &error);
      // value[id] = convertValueToRadian(id, read_value_xl);
      value[id] = read_value_xl;
    }
    else
    {
      // error
    }

    // DEBUG
    Serial.println(value[id]);
  }

  return result;
}


/*******************************************************************
** WRITE FUNCTIONS
********************************************************************/

bool Folex::DynamixelDriver::writeValueGoalPosition(uint8_t id, uint32_t value)
{
  bool result = false;
  uint8_t error = 0;

  if (all_joint.find(id)->second == AX_12A)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value);
  }
  else if (all_joint.find(id)->second == XL430_W250)
  {
    writeValue(id, ADDR_XL_GOAL_POSITION, value);
  }
  else
  {
    // error
  }

  return result;
}

bool Folex::DynamixelDriver::writeValueGoalVelocity(uint8_t id, uint32_t value)
{
  bool result = false;
  uint8_t error = 0;

  if (all_joint.find(id)->second == AX_12A)
  {
    writeValue(id, ADDR_AX_MOVING_SPEED, value);
  }
  else if (all_joint.find(id)->second == XL430_W250)
  {
    writeValue(id, ADDR_XL_PROFILE_VELOCITY, value);
    setTorque(id, true);  // Apply velocity
  }
  else
  {
    // error    
  }

  return result;
}

bool Folex::DynamixelDriver::writeValue(uint8_t id, uint16_t address, uint32_t value)
{
  bool result = false;
  uint8_t error = 0;

  if (all_joint.find(id)->second == AX_12A)
  {
    switch (dxl_ax_address_array.find(address)->second)
    {
    case BYTE:
      result = packetHandler_->write1ByteTxRx(portHandler_, id, address, value, &error);
      break;

    case WORD:
      result = packetHandler_->write2ByteTxRx(portHandler_, id, address, value, &error);
      break;
    
    default:
      result = false;
      break;
    }
  }
  else if (all_joint.find(id)->second == XL430_W250)
  {
    switch (dxl_xl_address_array.find(address)->second)
    {
    case BYTE:
      result = packetHandler_->write1ByteTxRx(portHandler_, id, address, value, &error);
      break;

    case WORD:
      result = packetHandler_->write2ByteTxRx(portHandler_, id, address, value, &error);
      break;

    case DWORD:
      result = packetHandler_->write4ByteTxRx(portHandler_, id, address, value, &error);
      break;
    
    default:
      result = false;
      break;
    }
  }
  else
  {
    // error
  }

  return result;
}


/*******************************************************************
** SET FUNCTIONS
********************************************************************/

void Folex::DynamixelDriver::setGoalPositionFromSerial(uint32_t data[])
{
  for (uint8_t i = 0; i < 12; i++)
  {
    writeValueGoalPosition(i, data[i]);
  }
}

void Folex::DynamixelDriver::setGoalVelocityFromSerial(uint32_t data[])
{
  for (uint8_t i = 0; i < 12; i++)
  {
    // if (data[i] > 0 && data[i] < LIMIT_AX_VELOCITY)
    // {
    //   // To-Do
    // }

    writeValueGoalVelocity(i, data[i]);
  }
}


/*******************************************************************
** RESET FUNCTIONS
********************************************************************/

void Folex::DynamixelDriver::resetPosition()
{
  // Set RPM
  double init_rpm = 5.0;
  std::map<uint8_t, uint16_t>::iterator it_dxl_array;
  
  for (uint8_t id = 0; id < 12; id++)
  {
    if (all_joint.find(id)->second == AX_12A)
    {
      writeValue(id, ADDR_AX_MOVING_SPEED, 20);
    }
    else if (all_joint.find(id)->second == XL430_W250)
    {
      writeValue(id, ADDR_XL_PROFILE_VELOCITY, 20);
    }
    else
    {
      // error
    }   
  }

  // Move to origin
  for (uint8_t id = 0; id < 12; id++)
  {
    if (all_joint.find(id)->second == AX_12A)
    {
      writeValue(id, ADDR_AX_GOAL_POSITION, 512);
    }
    else if (all_joint.find(id)->second == XL430_W250)
    {
      writeValue(id, ADDR_XL_GOAL_POSITION, 2048);
    }
    else
    {
      // error
    } 
  }
}


/*******************************************************************
** CONVERT FUNCTIONS
********************************************************************/

float Folex::DynamixelDriver::convertValueToRadian(uint8_t id, uint32_t value)
{
  float radian = 0.0;



  return radian;
}


