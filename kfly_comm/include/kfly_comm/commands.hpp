//          Copyright Emil Fresk 2016 - 2017
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <cstdint>

namespace kfly_comm
{
/* Command are from the view of KFly. */
enum class commands : uint8_t
{
  /**
   * @brief   The zero command is not allowed.
   */
  None = 0,
  /**
   * @brief   ACK command.
   */
  ACK = 1,
  /**
   * @brief   Ping command.
   */
  Ping = 2,
  /**
   * @brief   Send debug message command.
   */
  DebugMessage = 3,
  /**
   * @brief   Get running mode command.
   */
  GetRunningMode = 4,
  /**
   * @brief   Manage Subscriptions command.
   */
  ManageSubscriptions = 5,

  /*===============================================*/
  /* Info, ID and Save commands.                   */
  /*===============================================*/

  /**
   * @brief   Get system strings.
   */
  GetSystemStrings = 16,
  /**
   * @brief   Get system status.
   */
  GetSystemStatus = 17,
  /**
   * @brief   Set the user definable ID strings.
   */
  SetDeviceStrings = 18,
  /**
   * @brief   Save all settings to flash.
   */
  SaveToFlash = 19,
  /**
   * @brief   Erase flash.
   */
  EraseFlash = 20,

  /*===============================================*/
  /* Controller specific commands.                 */
  /*===============================================*/

  /**
   * @brief   Motor override command for ESC calibration.
   */
  MotorOverride = 23,
  /**
   * @brief   Get controller references (quaternion and rate).
   */
  GetControllerReferences = 24,
  /**
   * @brief   Get control signals (actuator desired).
   */
  GetControlSignals = 25,
  /**
   * @brief   Get contreller settings.
   */
  GetControllerLimits = 26,
  /**
   * @brief   Set controller settings.
   */
  SetControllerLimits = 27,
  /**
   * @brief   Get Arm Settings.
   */
  GetArmSettings = 28,
  /**
   * @brief   Set Arm Settings.
   */
  SetArmSettings = 29,
  /**
   * @brief   Get rate controller data.
   */
  GetRateControllerData = 30,
  /**
   * @brief   Set rate controller data.
   */
  SetRateControllerData = 31,
  /**
   * @brief   Get attitude controller data.
   */
  GetAttitudeControllerData = 32,
  /**
   * @brief   Set attitude controller data.
   */
  SetAttitudeControllerData = 33,
  /**
   * @brief   Get velocity controller data.
   */
  GetVelocityControllerData = 34,
  /**
   * @brief   Set velocity controller data.
   */
  SetVelocityControllerData = 35,
  /**
   * @brief   Get position controller data.
   */
  GetPositionControllerData = 36,
  /**
   * @brief   Set position controller data.
   */
  SetPositionControllerData = 37,
  /**
   * @brief   Get channel mixer.
   */
  GetChannelMix = 39,
  /**
   * @brief   Set channel mixer.
   */
  SetChannelMix = 40,

  /*===============================================*/
  /* RC specific commands.                         */
  /*===============================================*/

  /**
   * @brief   Get RC input settings.
   */
  GetRCInputSettings = 41,
  /**
   * @brief   Set RC input settings.
   */
  SetRCInputSettings = 42,
  /**
   * @brief   Get RC input settings.
   */
  GetRCOutputSettings = 43,
  /**
   * @brief   Set RC input settings.
   */
  SetRCOutputSettings = 44,
  /**
   * @brief   Get RC values.
   */
  GetRCValues = 45,

  /*===============================================*/
  /* Sensor specific commands.                     */
  /*===============================================*/

  /**
   * @brief   Get calibrated IMU data.
   */
  GetIMUData = 46,
  /**
   * @brief   Get raw IMU data.
   */
  GetRawIMUData = 47,
  /**
   * @brief   Get IMU calibration.
   */
  GetIMUCalibration = 48,
  /**
   * @brief   Set IMU calibration.
   */
  SetIMUCalibration = 49,

  /*===============================================*/
  /* Estimation specific commands.                 */
  /*===============================================*/

  /**
   * @brief   Get rate estimation.
   */
  GetEstimationRate = 50,
  /**
   * @brief   Get attitude estimation.
   */
  GetEstimationAttitude = 51,
  /**
   * @brief   Get velocity estimation.
   */
  GetEstimationVelocity = 52,
  /**
   * @brief   Get position estimation.
   */
  GetEstimationPosition = 53,
  /**
   * @brief   Get all states.
   */
  GetEstimationAllStates = 54,
  /**
   * @brief   Reset estimation.
   */
  ResetEstimation = 55,

  /*===============================================*/
  /* Filter specific commands.                     */
  /*===============================================*/

  /**
   * @brief   Get control filters.
   */
  GetControlFilters = 56,
  /**
   * @brief   Set control filters.
   */
  SetControlFilters = 57,

  /*===============================================*/
  /* Computer control specific commands.           */
  /*===============================================*/

  /**
   * @brief   New control reference from the PC side.
   */
  ComputerControlReference = 126,

  /*===============================================*/
  /* Motion capture frame specific commands.       */
  /*===============================================*/

  /**
   * @brief   New motion capture measurement.
   */
  MotionCaptureMeasurement = 127
};
}
