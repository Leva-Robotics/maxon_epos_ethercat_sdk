// clang-format off
/*
** Copyright 2021 Robotic Systems Lab - ETH Zurich:
** Linghao Zhang, Jonas Junger, Lennart Nachtigall
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its contributors
**    may be used to endorse or promote products derived from this software without
**    specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// clang-format on

#include <cstdint>
#include <vector>

#define OD_INDEX_RX_PDO_ASSIGNMENT uint16_t(0x1c12)
#define OD_INDEX_TX_PDO_ASSIGNMENT uint16_t(0x1c13)

#define OD_INDEX_TX_PDO_MAPPING_1 uint16_t(0x1A00)
#define OD_INDEX_TX_PDO_MAPPING_2 uint16_t(0x1A01)
#define OD_INDEX_TX_PDO_MAPPING_3 uint16_t(0x1A02)
#define OD_INDEX_TX_PDO_MAPPING_4 uint16_t(0x1A03)

#define OD_INDEX_RX_PDO_MAPPING_1 uint16_t(0x1600)
#define OD_INDEX_RX_PDO_MAPPING_2 uint16_t(0x1601)
#define OD_INDEX_RX_PDO_MAPPING_3 uint16_t(0x1602)
#define OD_INDEX_RX_PDO_MAPPING_4 uint16_t(0x1603)

#define OD_INDEX_ERROR_REGISTER (0x1001)
#define OD_INDEX_ERROR_HISTORY (0x1003)
#define OD_INDEX_DIAGNOSIS (0x10F3)
#define OD_INDEX_5VDC_SUPPLY (0x2200)
#define OD_INDEX_MOTOR_DATA (0x3001)
#define OD_INDEX_GEAR_DATA (0x3003)
#define OD_INDEX_ERROR_CODE (0x603F)
#define OD_INDEX_CONTROLWORD (0x6040)
#define OD_INDEX_STATUSWORD (0x6041)
#define OD_INDEX_QUICKSTOP_OPTION_CODE (0x605A)
#define OD_INDEX_SHUTDOWN_OPTION_CODE (0x605B)
#define OD_INDEX_DISABLE_OPERATION_OPTION_CODE (0x605C)
#define OD_INDEX_FAULT_REACTION_OPTION_CODE (0x605E)
#define OD_INDEX_MODES_OF_OPERATION (0x6060)
#define OD_INDEX_MODES_OF_OPERATION_DISPLAY (0x6061)
#define OD_INDEX_POSITION_ACTUAL (0x6064)
#define OD_INDEX_FOLLOW_ERROR_WINDOW (0x6065)
#define OD_INDEX_VELOCITY_DEMAND (0x606B)
#define OD_INDEX_SENSOR_SSI (0x3012)
#define OD_INDEX_HALL_SENSOR (0x301A)
#define OD_INDEX_VELOCITY_ACTUAL (0x606C)
#define OD_INDEX_TARGET_TORQUE (0x6071)
#define OD_INDEX_MOTOR_RATED_TORQUE (0x6076)
#define OD_INDEX_TORQUE_ACTUAL (0x6077)
#define OD_INDEX_CURRENT_CONTROL_PARAM (0x30A0)
#define OD_INDEX_POSITION_CONTROL_PARAM (0x30A1)
#define OD_INDEX_VELOCITY_CONTROL_PARAM (0x30A2)
#define OD_INDEX_CURRENT_ACTUAL (0x30D1)
#define OD_INDEX_TARGET_POSITION (0x607A)
#define OD_INDEX_MAX_PROFILE_VELOCITY (0x607F)
#define OD_INDEX_MAX_MOTOR_SPEED (0x6080)
#define OD_INDEX_PROFILE_VELOCITY (0x6081)
#define OD_INDEX_MOTION_PROFILE_TYPE (0x6086)
#define OD_INDEX_OFFSET_POSITION (0x60B0)
#define OD_INDEX_OFFSET_VELOCITY (0x60B1)
#define OD_INDEX_OFFSET_TORQUE (0x60B2)
#define OD_INDEX_INTERPOLATION_TIME_PERIOD (0x60C2)
#define OD_INDEX_DIGITAL_INPUTS (0x60FD)
#define OD_INDEX_TARGET_VELOCITY (0x60FF)
#define OD_INDEX_ABORT_CONNECTION_OPTION_CODE (0x6007)
#define OD_INDEX_RESET_DEFAULT_PARAMETERS (0x1011)
#define OD_INDEX_MAX_SYSTEM_SPEED (0x3000)
#define OD_INDEX_CURRENT_DEMAND (0x30D0)
#define OD_INDEX_POSITION_DEMAND (0x6062)
#define OD_INDEX_SOFT_LIMIT (0x607D)

//SI unit registers
#define OD_INDEX_SI_UNIT_ACCELERATION (0x60AA)
#define OD_INDEX_SI_UNIT_POSITION (0x60A8)
#define OD_INDEX_SI_UNIT_VELOCITY (0x60A9)


//Acceleration related registers
#define OD_INDEX_MAX_ACCELERATION (0x60C5)
#define OD_INDEX_PROFILE_ACCELERATION (0x6083)
#define OD_INDEX_PROFILE_DECELERATION (0x6084)
#define OD_INDEX_QUICKSTOP_DECELERATION (0x6085)


//Position related registers
#define OD_INDEX_POSITION_RANGE_LIMIT (0x607B)
#define OD_INDEX_SOFTWARE_POSITION_LIMIT (0x607D)

//TEMPERATURE RELATED REGISTERS
#define OD_INDEX_TEMPERATURE (0x3201)
#define OD_INDEX_I2T (0x3200)

//VOLTAGE RELATED REGISTERS
#define OD_INDEX_PSU_VOLTAGE (0x2200)

//HOMING RELATED REGISTERS
#define OD_INDEX_HOME_REFERENCE_STATE (0x30B5)
#define OD_INDEX_HOME_POSITION (0x30B0)
#define OD_INDEX_HOME_METHOD (0x6098)
#define OD_INDEX_HOME_OFFSET (0x3673)

//IMPACT MITIGATION RELATED REGISTERS
#define OD_INDEX_IMPACT_MITIGATION (0x3670)


// anydrive5 specific object dictionary values

#define OD_INDEX_JOINT_TORQUE_EST (0x3672)
#define OD_INDEX_JOINT_VELOCITY_ACTUAL (0x606C)
#define OD_INDEX_JOINT_POSITION_ACTUAL (0x6064)
#define OD_INDEX_JOINT_CURRENT_ACTUAL (0x30D1)

#define OD_INDEX_TARGET_JOINT_TORQUE (0x34C3)
#define OD_INDEX_TARGET_JOINT_VELOCITY (0x60FF)
#define OD_INDEX_TARGET_JOINT_POSITION (0x607A)


//Jvtc parameters locations

#define OD_INDEX_JVPT_TORQUE_CONTROL_PARAM (0x3671)
#define OD_INDEX_JVPT_PARAMETERS (0x34C6)


//homing parameters

#define OD_HOMING_METHOD (0x6098)

//store param

#define OD_STORE_PARAM (0x1010)