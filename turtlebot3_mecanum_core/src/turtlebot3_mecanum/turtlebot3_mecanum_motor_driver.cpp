/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */
// MAJOR CHANGES for mecanum

#include "../../include/turtlebot3_mecanum/turtlebot3_mecanum_motor_driver.h"

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  //left_wheel_id_(DXL_LEFT_ID),
  //right_wheel_id_(DXL_RIGHT_ID)
  left_front_wheel_id_(DXL_LEFT_FRONT_ID),
  right_front_wheel_id_(DXL_RIGHT_FRONT_ID),
  left_rear_wheel_id_(DXL_LEFT_REAR_ID),
  right_rear_wheel_id_(DXL_RIGHT_REAR_ID)
{
  torque_ = false;
  dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY; // USING motors of burger
}

/*********************************************************************************/
/*********************************************************************************/

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
}

/*********************************************************************************/
/*********************************************************************************/

bool Turtlebot3MotorDriver::init(String turtlebot3)
{
  DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }

  // Enable Dynamixel Torque
  setTorque(true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  
  if (turtlebot3 == "Burger")
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
  else if (turtlebot3 == "Waffle or Waffle Pi")
    dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
  else
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY; // In other cases use burger

  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

/*********************************************************************************/
/*********************************************************************************/

bool Turtlebot3MotorDriver::setTorque(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  torque_ = onoff;

  /***********************/
  // LEFT FRONT WHEEL
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_LEFT_FRONT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  /***********************/
  // RIGHT FRONT WHEEL
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_RIGHT_FRONT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }
  
  /***********************/
  // LEFT REAR WHEEL
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_LEFT_REAR_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  /***********************/
  // RIGHT REAR WHEEL
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_RIGHT_REAR_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  /*****/
  return true;
}

/*********************************************************************************/
/*********************************************************************************/

bool Turtlebot3MotorDriver::getTorque()
{
  return torque_;
}

/*********************************************************************************/
/*********************************************************************************/

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
  DEBUG_SERIAL.end();
}

/*********************************************************************************/
/*********************************************************************************/

bool Turtlebot3MotorDriver::readEncoder(int32_t &left_front_value, int32_t &right_front_value, int32_t &left_rear_value, int32_t &right_rear_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  //char log_msg[50];


  /***************************************************************/
  // Set parameter
  // Left front
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_front_wheel_id_);
  if (dxl_addparam_result != true){
    //sprintf(log_msg, "Problem adding param left_front_value");
    //nh.loginfo(log_msg);
    return false;
  }

  // Right front
  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_front_wheel_id_);
  if (dxl_addparam_result != true){
    //sprintf(log_msg, "Problem adding param right_front_value");
    //nh.loginfo(log_msg);
    return false;
  }

  // Left rear
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_rear_wheel_id_);
  if (dxl_addparam_result != true){
    //sprintf(log_msg, "Problem adding param left_rear_value");
    //nh.loginfo(log_msg);
    return false;
  }

  // Right rear
  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_rear_wheel_id_);
  if (dxl_addparam_result != true){
    //sprintf(log_msg, "Problem adding param right_rear_value");
    //nh.loginfo(log_msg);
    return false;
  }

  /***************************************************************/
  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    //sprintf(log_msg, "Problem with sync read");
    //nh.loginfo(log_msg);
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
  }

  /***************************************************************/
  // Check if groupSyncRead data of Dynamixels are available
  // Left front
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true){
    //sprintf(log_msg, "left_front_wheel_id_ not available");
    //nh.loginfo(log_msg);
    return false;
  }

  // Right front
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true){
    //sprintf(log_msg, "right_front_wheel_id_ not available");
    //nh.loginfo(log_msg);
    return false;
  }

  // Left rear
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true){
    //sprintf(log_msg, "left_rear_wheel_id_ not available");
    //nh.loginfo(log_msg);
    return false;
  }

  // Right rear
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true){
    //sprintf(log_msg, "right_rear_wheel_id_ not available");
    //nh.loginfo(log_msg);
    return false;
  }
  /***************************************************************/
  // Get data
  left_front_value  = groupSyncReadEncoder_->getData(left_front_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_front_value = groupSyncReadEncoder_->getData(right_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  left_rear_value  = groupSyncReadEncoder_->getData(left_rear_wheel_id_,    ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_rear_value = groupSyncReadEncoder_->getData(right_rear_wheel_id_,   ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  /***************************************************************/
  groupSyncReadEncoder_->clearParam();
  return true;
}

/*********************************************************************************/
/*********************************************************************************/

bool Turtlebot3MotorDriver::writeVelocity(int64_t left_front_value, int64_t right_front_value, int64_t left_rear_value, int64_t right_rear_value )
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t left_front_data_byte[4] = {0, };
  uint8_t right_front_data_byte[4] = {0, };
  uint8_t left_rear_data_byte[4] = {0, };
  uint8_t right_rear_data_byte[4] = {0, };

  /***********************/
  // LEFT FRONT WHEEL

  left_front_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_front_value));
  left_front_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_front_value));
  left_front_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_front_value));
  left_front_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_front_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_front_wheel_id_, (uint8_t*)&left_front_data_byte);
  if (dxl_addparam_result != true){
    return false;
  }

  /***********************/
  // RIGHT FRONT WHEEL

  right_front_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_front_value));
  right_front_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_front_value));
  right_front_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_front_value));
  right_front_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_front_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_front_wheel_id_, (uint8_t*)&right_front_data_byte);
  if (dxl_addparam_result != true){
    return false;
  }

  /***********************/
  // LEFT REAR WHEEL

  left_rear_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_rear_value));
  left_rear_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_rear_value));
  left_rear_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_rear_value));
  left_rear_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_rear_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_rear_wheel_id_, (uint8_t*)&left_rear_data_byte);
  if (dxl_addparam_result != true){
    return false;
  }

  /***********************/
  // RIGHT REAR WHEEL

  right_rear_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_rear_value));
  right_rear_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_rear_value));
  right_rear_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_rear_value));
  right_rear_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_rear_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_rear_wheel_id_, (uint8_t*)&right_rear_data_byte);
  if (dxl_addparam_result != true){
    return false;
  }

  /*************/
  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  /*************/
  groupSyncWriteVelocity_->clearParam();
  return true;
}

/*********************************************************************************/
/*********************************************************************************/


bool Turtlebot3MotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity_cmd[4]; // Changed from length 2 to 4

  float lin_vel_X = value[LINEAR_X];
  float lin_vel_Y = value[LINEAR_Y]; // ADDED line
  float ang_vel   = value[ANGULAR];

  float scale = 1.0;
  float aux_scale = 0.0;

  /**************/
  // MODEL

  wheel_velocity_cmd[LEFT_FRONT]  = (31.2500)*lin_vel_X  + (-31.2500)*(lin_vel_Y)  + (-7.0312)*(ang_vel);
  wheel_velocity_cmd[RIGHT_FRONT] = (31.2500)*lin_vel_X  + (31.2500)*(lin_vel_Y)   + (7.0312)*(ang_vel);
  wheel_velocity_cmd[LEFT_REAR]   = (31.2500)*lin_vel_X  + (31.2500)*(lin_vel_Y)   + (-7.0312)*(ang_vel);
  wheel_velocity_cmd[RIGHT_REAR]  = (31.2500)*lin_vel_X  + (-31.2500)*(lin_vel_Y)  + (7.0312)*(ang_vel);

  /**************/
  // Scale vel to motor data

  wheel_velocity_cmd[LEFT_FRONT]  = wheel_velocity_cmd[LEFT_FRONT]  * VELOCITY_CONSTANT_VALUE;
  wheel_velocity_cmd[RIGHT_FRONT] = wheel_velocity_cmd[RIGHT_FRONT] * VELOCITY_CONSTANT_VALUE;
  wheel_velocity_cmd[LEFT_REAR]   = wheel_velocity_cmd[LEFT_REAR]   * VELOCITY_CONSTANT_VALUE;
  wheel_velocity_cmd[RIGHT_REAR]  = wheel_velocity_cmd[RIGHT_REAR]  * VELOCITY_CONSTANT_VALUE;

  /**************/
  // Constraints

  if(abs(wheel_velocity_cmd[LEFT_FRONT]) > 0.1){
    aux_scale = dynamixel_limit_max_velocity_/abs(wheel_velocity_cmd[LEFT_FRONT]);
    if( aux_scale < scale ){ 
      scale = aux_scale;
    }
  }

  if(abs(wheel_velocity_cmd[RIGHT_FRONT]) > 0.1){
    aux_scale = dynamixel_limit_max_velocity_/abs(wheel_velocity_cmd[RIGHT_FRONT]);
    if( aux_scale < scale ){ 
      scale = aux_scale;
    }
  }

  if(abs(wheel_velocity_cmd[LEFT_REAR]) > 0.1){
    aux_scale = dynamixel_limit_max_velocity_/abs(wheel_velocity_cmd[LEFT_REAR]);
    if( aux_scale < scale ){ 
      scale = aux_scale;
    }
  }

  if(abs(wheel_velocity_cmd[RIGHT_REAR]) > 0.1){
    aux_scale = dynamixel_limit_max_velocity_/abs(wheel_velocity_cmd[RIGHT_REAR]);
    if( aux_scale < scale ){ 
      scale = aux_scale;
    }
  }

  if(scale < 1.0){
    wheel_velocity_cmd[LEFT_FRONT]  = wheel_velocity_cmd[LEFT_FRONT]  * scale;
    wheel_velocity_cmd[RIGHT_FRONT] = wheel_velocity_cmd[RIGHT_FRONT] * scale;
    wheel_velocity_cmd[LEFT_REAR]   = wheel_velocity_cmd[LEFT_REAR]   * scale;
    wheel_velocity_cmd[RIGHT_REAR]  = wheel_velocity_cmd[RIGHT_REAR]  * scale;
  }

  // Safe check
  wheel_velocity_cmd[LEFT_FRONT]  = constrain(wheel_velocity_cmd[LEFT_FRONT]  , -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  wheel_velocity_cmd[RIGHT_FRONT] = constrain(wheel_velocity_cmd[RIGHT_FRONT] , -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  wheel_velocity_cmd[LEFT_REAR]   = constrain(wheel_velocity_cmd[LEFT_REAR]   , -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  wheel_velocity_cmd[RIGHT_REAR]  = constrain(wheel_velocity_cmd[RIGHT_REAR]  , -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

  /**************/
  // Write Vel

  dxl_comm_result = writeVelocity( (int64_t)wheel_velocity_cmd[LEFT_FRONT], (int64_t)wheel_velocity_cmd[RIGHT_FRONT], (int64_t)wheel_velocity_cmd[LEFT_REAR], (int64_t)wheel_velocity_cmd[RIGHT_REAR] );
  if (dxl_comm_result == false){
    return false;
  }

  return true;
}
