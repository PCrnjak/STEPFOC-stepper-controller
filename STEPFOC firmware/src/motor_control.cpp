/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    motor_control.cpp
 * @brief   This file provides code for utilities for motor control
 * @author Petar Crnjak
 ******************************************************************************
 * @attention
 *
 * Copyright (c) Source robotics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "utils.h"
#include <Arduino.h>
#include <stdio.h>
#include "iodefs.h"
#include "hw_init.h"
#include "motor_control.h"
#include "common.h"
#include <SPI.h>
#include "common.h"
#include "foc.h"

#define TIMING_DEBUG 0

SPISettings MT6816_settings(16000000, MSBFIRST, SPI_MODE3);

/// @todo phase order working in any orientation
/// @todo feedforwards for cascade control and PD control
/// @todo cogging compensation
/// @todo encoder non linearity compensation

/// @brief  Collect all sensor data
void Collect_data()
{

  uint16_t result;
  uint16_t result1;
  uint16_t result2;

  /*Get position, current1, current2, vbus*/
  /*position sensor packs its data in 2 8 bit data packets */
  /*First we send 0x8300 as a response we get one packet*/
  /*Then we send 0c8400 as a response we get second packet*/
  /*Between requesting packets we need small delay and we use ADC readings as a delay*/
  /*Reading sense1,2 and vbus takes around 10us */
  /*After we get both packets they are unpacked and we get position value as 14 bit, no mag bit and parity check bit */
  SPI.beginTransaction(MT6816_settings);
  digitalWriteFast(CSN, LOW);
  result1 = SPI.transfer16(0x8300);
  digitalWriteFast(CSN, HIGH);

  controller.Sense1_Raw = ADC_CHANNEL_6_READ_SENSE1();
  controller.Sense2_Raw = ADC_CHANNEL_8_READ_SENSE2();

  controller.VBUS_RAW = ADC_CHANNEL_4_READ_VBUS();
  result1 = result1 << 8;
  digitalWriteFast(CSN, LOW);
  result2 = SPI.transfer16(0x8400);
  digitalWriteFast(CSN, HIGH);
  result = result1 | result2;
  controller.Position_Raw = (result >> 2);
  SPI.endTransaction();
  /***********************************/

  /* Check for magnet on MT6816*/
  if ((result & MT6816_NO_MAGNET_BIT) == MT6816_NO_MAGNET_BIT)
  { // no magnet or not detecting enough flux density
    controller.Magnet_warrning = true;
    controller.encoder_error = 1;
  }
  else
  { // Detected magnet
    controller.Magnet_warrning = false;
  }
  /***********************************/

  /* Parity check, works wierd and inconsistent, skip for now
 if(parityCheck(result)){
    controller.MT6816_parity_check_pass = 1;
 }else{
    controller.MT6816_parity_check_pass = 0;
 }
*/

  /* Transform RAW current and voltage to mV, mA.*/

  controller.Sense1_mA = -Get_current_mA(controller.Sense1_Raw);
  controller.Sense2_mA = Get_current_mA(controller.Sense2_Raw);

  controller.VBUS_mV = Get_voltage_mA(controller.VBUS_RAW);
  /***********************************/

  /* Handle encoder full rotation overflow*/
  if (controller.Position_Raw - controller.Old_Position_Raw > CPR2)
  {
    controller.ROTATIONS = controller.ROTATIONS - 1;
  }
  else if (controller.Position_Raw - controller.Old_Position_Raw < -CPR2)
  {
    controller.ROTATIONS = controller.ROTATIONS + 1;
  }
  /***********************************/

  /* Motor position with multiple rotations in encoder ticks*/
  controller.Position_Ticks = controller.Position_Raw + (controller.ROTATIONS * CPR);
  /***********************************/

  /* Calculate velocity in ticks/s */
  controller.Velocity = (controller.Position_Ticks - controller.Old_Position_Ticks) / LOOP_TIME;
  /* Moving average filter on the velocity */
  controller.Velocity_Filter = movingAverage(controller.Velocity);
  /***********************************/

  /*  Save position values for next cycle*/
  controller.Old_Position_Ticks = controller.Position_Ticks;
  controller.Old_Position_Raw = controller.Position_Raw;
  /***********************************/

  // Position_RAD = qfp_fmul(Position_Raw, RAD_CONST);

  /* Caclculate electrical angle */
  // controller.Electric_Angle = RAD_CONST * fmod((controller.Position_Raw * NPP),CPR);
  controller.Electric_Angle = RAD_CONST * ((controller.Position_Raw * controller.pole_pairs) % CPR);

  // add offset
  controller.Electric_Angle += controller.theta_offset;

  // Normalize angle between 0 and 2Pi
  if (controller.Electric_Angle < 0)
    controller.Electric_Angle = (controller.Electric_Angle + PI2);
  else if (controller.Electric_Angle > PI2)
    controller.Electric_Angle = (controller.Electric_Angle - PI2);
  /***********************************/

  /****  End measurment here *********/
}

/// @brief  Collect all sensor data2
void Collect_data2()
{

  uint16_t result;
  uint16_t result1;
  uint16_t result2;

  /*Get position, current1, current2, vbus*/
  /*position sensor packs its data in 2 8 bit data packets */
  /*First we send 0x8300 as a response we get one packet*/
  /*Then we send 0c8400 as a response we get second packet*/
  /*Between requesting packets we need small delay and we use ADC readings as a delay*/
  /*Reading sense1,2 and vbus takes around 10us */
  /*After we get both packets they are unpacked and we get position value as 14 bit, no mag bit and parity check bit */
  SPI.beginTransaction(MT6816_settings);
  digitalWriteFast(CSN, LOW);
  result1 = SPI.transfer16(0x8300);
  digitalWriteFast(CSN, HIGH);

  controller.Sense1_Raw = ADC_CHANNEL_6_READ_SENSE1();
  controller.Sense2_Raw = ADC_CHANNEL_8_READ_SENSE2();

  controller.VBUS_RAW = ADC_CHANNEL_4_READ_VBUS();
  result1 = result1 << 8;
  digitalWriteFast(CSN, LOW);
  result2 = SPI.transfer16(0x8400);
  digitalWriteFast(CSN, HIGH);
  result = result1 | result2;
  controller.Position_Raw = result >> 2;
  SPI.endTransaction();
  /***********************************/

  /* Check for magnet on MT6816*/
  if ((result & MT6816_NO_MAGNET_BIT) == MT6816_NO_MAGNET_BIT)
  { // no magnet or not detecting enough flux density
    controller.Magnet_warrning = true;
    controller.encoder_error = 1;
  }
  else
  { // Detected magnet
    controller.Magnet_warrning = false;
  }
  /***********************************/

  /* Parity check, works wierd and inconsistent, skip for now
 if(parityCheck(result)){
    controller.MT6816_parity_check_pass = 1;
 }else{
    controller.MT6816_parity_check_pass = 0;
 }
*/

  /* Transform RAW current and voltage to mV, mA.*/

  controller.Sense1_mA = -Get_current_mA(controller.Sense1_Raw);
  controller.Sense2_mA = Get_current_mA(controller.Sense2_Raw);

  controller.VBUS_mV = Get_voltage_mA(controller.VBUS_RAW);
  /***********************************/

  /* Handle encoder full rotation overflow*/
  if (controller.Position_Raw - controller.Old_Position_Raw > CPR2)
  {
    controller.ROTATIONS = controller.ROTATIONS - 1;
  }
  else if (controller.Position_Raw - controller.Old_Position_Raw < -CPR2)
  {
    controller.ROTATIONS = controller.ROTATIONS + 1;
  }
  /***********************************/

  /* Motor position with multiple rotations in encoder ticks*/
  controller.Position_Ticks = controller.Position_Raw + (controller.ROTATIONS * CPR);
  /***********************************/

  /* Calculate velocity in ticks/s */
  controller.Velocity = (controller.Position_Ticks - controller.Old_Position_Ticks) / LOOP_TIME;
  /* Moving average filter on the velocity */
  controller.Velocity_Filter = movingAverage(controller.Velocity);
  /***********************************/

  /*  Save position values for next cycle*/
  controller.Old_Position_Ticks = controller.Position_Ticks;
  controller.Old_Position_Raw = controller.Position_Raw;
  /***********************************/

  // Position_RAD = qfp_fmul(Position_Raw, RAD_CONST);

  /* Caclculate electrical angle */
  // controller.Electric_Angle = RAD_CONST * fmod((controller.Position_Raw * NPP),CPR);
  controller.Electric_Angle = RAD_CONST * ((controller.Position_Raw * controller.pole_pairs) % CPR);

  // Normalize angle between 0 and 2Pi
  if (controller.Electric_Angle < 0)
    controller.Electric_Angle = (controller.Electric_Angle + PI2);
  else if (controller.Electric_Angle > PI2)
    controller.Electric_Angle = (controller.Electric_Angle - PI2);
  /***********************************/

  /****  End measurment here *********/
}

/// @brief  Get first encoder reading when powered on
void Get_first_encoder()
{
  uint16_t result;
  uint16_t result1;
  uint16_t result2;

  /*Get position, current1, current2, vbus*/
  /*position sensor packs its data in 2 8 bit data packets */
  /*First we send 0x8300 as a response we get one packet*/
  /*Then we send 0c8400 as a response we get second packet*/
  /*Between requesting packets we need small delay and we use ADC readings as a delay*/
  /*Reading sense1,2 and vbus takes around 10us */
  /*After we get both packets they are unpacked and we get position value as 14 bit, no mag bit and parity check bit */
  SPI.beginTransaction(MT6816_settings);
  digitalWriteFast(CSN, LOW);
  result1 = SPI.transfer16(0x8300);
  digitalWriteFast(CSN, HIGH);

  delayMicroseconds(100);
  result1 = result1 << 8;
  digitalWriteFast(CSN, LOW);
  result2 = SPI.transfer16(0x8400);
  digitalWriteFast(CSN, HIGH);
  result = result1 | result2;
  controller.Position_Raw = result >> 2;
  SPI.endTransaction();
  /***********************************/

  /* Check for magnet on MT6816*/
  if ((result & MT6816_NO_MAGNET_BIT) == MT6816_NO_MAGNET_BIT)
  { // no magnet or not detecting enough flux density
    controller.Magnet_warrning = true;
    controller.encoder_error = 1;
  }
  else
  { // Detected magnet
    controller.Magnet_warrning = false;
  }
  /***********************************/

  /* Motor position with multiple rotations in encoder ticks*/
  controller.Position_Ticks = controller.Position_Raw;
  /***********************************/

  /*  Save position values for next cycle*/
  controller.Old_Position_Ticks = controller.Position_Ticks;
  controller.Old_Position_Raw = controller.Position_Raw;
  /***********************************/
}

/// @brief Interrupt callback routine for FOC mode
void IT_callback(void)
{
#if (TIMING_DEBUG > 0)
  int c = micros();
#endif

  /*Sample temperature every 15000 ticks; If enabled*/
  if (controller.Thermistor_on_off == 1)
  {
    static int temperature_tick = 0;
    if (temperature_tick >= 15000)
    {
      controller.TEMP_RAW = ADC_CHANNEL_5_READ_TEMP() >> 4;
      controller.TEMP_DEG = Temp_table[controller.TEMP_RAW];
      temperature_tick = 0;
    }
    temperature_tick = temperature_tick + 1;
  }
  /***********************************/

  /* Check for errors, If errors are present go idle  */
    /* If motor controller is gripper */
  if (controller.I_AM_GRIPPER == 1)
  {
    Gripper.temperature_error = controller.temperature_error;
    Gripper.timeout_error = controller.Watchdog_error;
    Gripper.position_ticks = controller.Position_Raw - 16383;
    if (Gripper.estop_status == 1)
    {
      Gripper.estop_error = 1;
      controller.Error = 1;
    }

    Gripper.position = mapAndConstrain(controller.Position_Ticks, Gripper.max_closed_position, Gripper.max_open_position);
  }

  ///  Check termistor
  if (controller.Thermistor_on_off == 1)
  {
    if (controller.TEMP_DEG < controller.Min_temperature || controller.TEMP_DEG > controller.Max_temperature)
    {
      controller.temperature_error = 1;
      controller.Error = 1;
    }
  }

  /// Check Vbus
  if (controller.VBUS_mV < controller.Min_Vbus || controller.VBUS_mV > controller.Max_Vbus)
  // if (controller.VBUS_mV > controller.Max_Vbus)
  {
    controller.Vbus_error = 1;
    controller.Error = 1;
  }

  /// Encoder error
  if (controller.encoder_error == 1)
  {
    controller.Error = 1;
  }

  /// Speed error
  if (abs(controller.Velocity_Filter) > PID.Velocity_limit_error)
  {
    controller.Velocity_error = 1;
    controller.Error = 1;
  }

  /// Not calibrated
  if (controller.Calibrated == 0)
  {
    controller.Error = 1;
  }

  /// Estop error
  if (controller.ESTOP_error == 1)
  {
    controller.Error = 1;
  }

  /// Watchdog error
  if (controller.Watchdog_error == 1)
  {
    controller.Error = 1;
  }

  /// @todo Current error

  /* Get all data needed for FOC */
  Collect_data();
  /***********************************/

  /* Calculate Iq and Id currents at this moment*/
  sine_cosine_calc(controller.Electric_Angle);
  park_transform(controller.Sense1_mA, controller.Sense2_mA, &FOC.Id, &FOC.Iq);

  /// If there is no error driver can operate normaly
  /// If error is active we will immediately go to idle
  if (controller.Error == 0)
  {
    switch (controller.Controller_mode)
    {
    case 0:
      /// Sleep mode input.Logic high to enable device;
      Brake_Coast();

      break;
    case 1:
      Position_mode();
      break;
    case 2:
      Velocity_mode();
      break;
    case 3:
      Torque_mode();
      break;
    case 4:
      PD_mode();
      break;
    case 5:
      Open_loop_speed_stepper(controller.Open_loop_speed, controller.Open_loop_voltage);
      break;
    case 6:
      Voltage_Torque_mode();
      break;
    case 7:
      Calibrate_gripper();
      break;
    case 8:
      Hall_Indexing_mode();
      break;
    case 9:
      Calibrate_Angle_Offset();
      break;
    case 10:
      Open_loop_move_steps(controller.gosteps, controller.Open_loop_voltage, controller.microstep, controller.step_interval);
      break;
    case 11:
      Gripper_mode();
      break;

    default:
      /// Idle
      break;
    }
  }
  else /// If there is error driver is disabled and in sleep mode
  {
    controller.Controller_mode = 0;
    Brake_Coast();
  }

#if (TIMING_DEBUG > 0)
  int c2 = micros();
  controller.execution_time = c2 - c;
#endif
}

/// @brief Non-blocking self-calibration of theta_offset
/// @return CALIB_IN_PROGRESS (0), CALIB_DONE (1), or CALIB_ABORTED (-1)
/// @hack this one runs standalone
int Calibrate_Angle_Offset()
{
  enum CalibResult
  {
    CALIB_IN_PROGRESS = 0,
    CALIB_DONE = 1,
    CALIB_ABORTED = -1
  };
  const int MAX_CALIB_CYCLES = 800000; // limit to prevent infinite loop, Around 2 minutes

  static int state = 0;
  static int counter = 0;
  static float forward_accum = 0;
  static float reverse_accum = 0;
  static float forward_avg = 0;
  static float reverse_avg = 0;
  static int last_error_sign = 0;
  static float theta_nudge = 0.025f;
  static int initial_sign_done = 0; // flag if initial sign test is done
  static int cycle_counter = 0;

  // --- config ---
  const int settle_cycles = 4000;
  const int measure_cycles = 200;
  const float error_threshold = 800.0f;
  const float min_velocity = 1000.0f;

  // check max cycles
  cycle_counter++;
  if (cycle_counter > MAX_CALIB_CYCLES)
  {
    controller.Controller_mode = 0;
    // reset for next time
    state = 0;
    counter = 0;
    forward_accum = 0;
    reverse_accum = 0;
    forward_avg = 0;
    reverse_avg = 0;
    last_error_sign = 0;
    theta_nudge = 0.025f;
    initial_sign_done = 0;
    cycle_counter = 0;
    return CALIB_DONE;
  }

  Torque_mode();

  switch (state)
  {
  case 0: // forward spin
    PID.Id_setpoint = 0;
    PID.Iq_setpoint = controller.calibration_offset_current;
    counter = 0;
    forward_accum = 0;
    state = 1;
    break;

  case 1: // forward settle
    counter++;
    if (counter >= settle_cycles)
    {
      counter = 0;
      state = 2;
    }
    break;

  case 2: // forward measure
    counter++;
    forward_accum += controller.Velocity_Filter;
    if (counter >= measure_cycles)
    {
      forward_avg = forward_accum / measure_cycles;
      controller.Velocity_fwd = forward_avg;
      counter = 0;

      PID.Iq_setpoint = -controller.calibration_offset_current;
      reverse_accum = 0;
      state = 3;
    }
    break;

  case 3: // reverse settle
    counter++;
    if (counter >= settle_cycles)
    {
      counter = 0;
      state = 4;
    }
    break;

  case 4: // reverse measure
    counter++;
    reverse_accum += controller.Velocity_Filter;
    if (counter >= measure_cycles)
    {
      reverse_avg = reverse_accum / measure_cycles;
      controller.Velocity_bwd = reverse_avg;

      PID.Iq_setpoint = 0;
      PID.Id_setpoint = 0;

      // determine initial theta_offset sign
      if (!initial_sign_done)
      {
        if (fabs(forward_avg) > fabs(reverse_avg))
          theta_nudge = -fabs(theta_nudge);
        else
          theta_nudge = fabs(theta_nudge);

        initial_sign_done = 1;
      }

      state = 5; // go to normal iterative nudging
    }
    break;

  case 5: // iterative nudging
  {
    float error = forward_avg + reverse_avg;
    if (fabs(forward_avg) < min_velocity || fabs(reverse_avg) < min_velocity)
    {
      error = (forward_avg + reverse_avg > 0) ? 10000.0f : -10000.0f;
    }

    if (fabs(error) > error_threshold)
    {
      int error_sign = (error > 0) ? 1 : -1;
      if (last_error_sign != 0 && error_sign != last_error_sign)
      {
        theta_nudge = -theta_nudge;
      }

      controller.theta_offset += theta_nudge;

      if (controller.theta_offset > PI2)
        controller.theta_offset -= PI2;
      if (controller.theta_offset < 0)
        controller.theta_offset += PI2;

      last_error_sign = error_sign;

      // restart measurement cycle
      state = 0;
      counter = 0;
      forward_accum = 0;
      reverse_accum = 0;
    }
    else
    {
      state = 6; // done
    }
  }
  break;

  case 6: // finished
    controller.Controller_mode = 0;
    // reset for next time
    state = 0;
    counter = 0;
    forward_accum = 0;
    reverse_accum = 0;
    forward_avg = 0;
    reverse_avg = 0;
    last_error_sign = 0;
    theta_nudge = 0.025f;
    initial_sign_done = 0;
    cycle_counter = 0;
    return CALIB_DONE;
  }

  return CALIB_IN_PROGRESS;
}

/// @brief Non-blocking self-calibration of theta_offset
/// @return CALIB_IN_PROGRESS (0), CALIB_DONE (1), or CALIB_ABORTED (-1)
/// @hack this one runs in calibration loop
int Calibrate_Angle_Offset2()
{
  enum CalibResult
  {
    CALIB_IN_PROGRESS = 0,
    CALIB_DONE = 1,
    CALIB_ABORTED = -1
  };
  const int MAX_CALIB_CYCLES = 800000; // limit to prevent infinite loop, Around 2 minutes

  static int state = 0;
  static int counter = 0;
  static float forward_accum = 0;
  static float reverse_accum = 0;
  static float forward_avg = 0;
  static float reverse_avg = 0;
  static int last_error_sign = 0;
  static float theta_nudge = 0.025f;
  static int initial_sign_done = 0; // flag if initial sign test is done
  static int cycle_counter = 0;

  // --- config ---
  const int settle_cycles = 4000;
  const int measure_cycles = 200;
  const float error_threshold = 800.0f;
  const float min_velocity = 1000.0f;

  // check max cycles
  cycle_counter++;
  if (cycle_counter > MAX_CALIB_CYCLES)
  {
    // reset for next time
    state = 0;
    counter = 0;
    forward_accum = 0;
    reverse_accum = 0;
    forward_avg = 0;
    reverse_avg = 0;
    last_error_sign = 0;
    theta_nudge = 0.025f;
    initial_sign_done = 0;
    cycle_counter = 0;
    return CALIB_DONE;
  }

  Torque_mode();

  switch (state)
  {
  case 0: // forward spin
    PID.Id_setpoint = 0;
    PID.Iq_setpoint = controller.calibration_offset_current;
    counter = 0;
    forward_accum = 0;
    state = 1;
    break;

  case 1: // forward settle
    counter++;
    if (counter >= settle_cycles)
    {
      counter = 0;
      state = 2;
    }
    break;

  case 2: // forward measure
    counter++;
    forward_accum += controller.Velocity_Filter;
    if (counter >= measure_cycles)
    {
      forward_avg = forward_accum / measure_cycles;
      controller.Velocity_fwd = forward_avg;
      counter = 0;

      PID.Iq_setpoint = -controller.calibration_offset_current;
      reverse_accum = 0;
      state = 3;
    }
    break;

  case 3: // reverse settle
    counter++;
    if (counter >= settle_cycles)
    {
      counter = 0;
      state = 4;
    }
    break;

  case 4: // reverse measure
    counter++;
    reverse_accum += controller.Velocity_Filter;
    if (counter >= measure_cycles)
    {
      reverse_avg = reverse_accum / measure_cycles;
      controller.Velocity_bwd = reverse_avg;

      PID.Iq_setpoint = 0;
      PID.Id_setpoint = 0;

      // determine initial theta_offset sign
      if (!initial_sign_done)
      {
        if (fabs(forward_avg) > fabs(reverse_avg))
          theta_nudge = -fabs(theta_nudge);
        else
          theta_nudge = fabs(theta_nudge);

        initial_sign_done = 1;
      }

      state = 5; // go to normal iterative nudging
    }
    break;

  case 5: // iterative nudging
  {
    float error = forward_avg + reverse_avg;
    if (fabs(forward_avg) < min_velocity || fabs(reverse_avg) < min_velocity)
    {
      error = (forward_avg + reverse_avg > 0) ? 10000.0f : -10000.0f;
    }

    if (fabs(error) > error_threshold)
    {
      int error_sign = (error > 0) ? 1 : -1;
      if (last_error_sign != 0 && error_sign != last_error_sign)
      {
        theta_nudge = -theta_nudge;
      }

      controller.theta_offset += theta_nudge;

      if (controller.theta_offset > PI2)
        controller.theta_offset -= PI2;
      if (controller.theta_offset < 0)
        controller.theta_offset += PI2;

      last_error_sign = error_sign;

      // restart measurement cycle
      state = 0;
      counter = 0;
      forward_accum = 0;
      reverse_accum = 0;
    }
    else
    {
      state = 6; // done
    }
  }
  break;

  case 6: // finished
    // reset for next time
    state = 0;
    counter = 0;
    forward_accum = 0;
    reverse_accum = 0;
    forward_avg = 0;
    reverse_avg = 0;
    last_error_sign = 0;
    theta_nudge = 0.025f;
    initial_sign_done = 0;
    cycle_counter = 0;
    return CALIB_DONE;
  }

  return CALIB_IN_PROGRESS;
}

/// @brief run calibration routine
/// Will exit if there is an error and report it
void Update_IT_callback_calib()
{

  /// Flags
  static int calib_step_pole_pair = 0;
  static int calib_step_magnet = 0;
  static int calib_step_voltage = 0;
  static int calib_step_resistance = 0;
  static int calib_step_inductance = 0;
  static int calib_step_delay = 0;
  static int calib_dir_pole_pair = 0;
  static int Phase_order_step = 0;
  static int KV_step = 0;

  static int tick_cnt = 0;
  static float resistance_voltage = 0;

  static int check_Vbus = 0;
  static int check_magnet = 0;

  volatile float EMA_a_R = 0.05; //  0.05 with 2V
  static float Res_avg = 0;
  static int Resistance_accumulator = 0;

  static int delay = 200; // Delay between resistance and inductance mesurement

  static int Inductor_start = 0;
  static float inductance_accumulator = 0.0;
  static int inductance_step = 0;
  static int inductance_sample_size = 100000;
  static int inductance_tick = 0;
  static int inductance_tick_2 = 0;

  static int Phase_step = 0;
  static int Phase_ticks = 0;
  static int Phase_start = 0;
  static int64_t _speed_accumulator = 0;
  static float Voltage_multiplier[4] = {1, 1, 1};
  static int64_t Controller_velocity_RPM[4] = {0, 0, 0};
  static int Spin_duration = 18500; // In interrupt ticks = 8500 * 200us = 1.7s
  static int Start_position_phase = 0;

  static int KV_ticks = 0;
  static int KV_duration = 5000;
  static int64_t KV_speed_accumulator = 0;
  static int64_t KV_current_accumulator = 0;

  // Open loop
  static float theta_SIM = 0;
  static int velocity_step = 0;
  static int open_loop_start = 0;
  static int start_position = 0;
  static int move_stop = 0;
  static int tick_tick = 0;
  static int open_loop_delay = 0;
  static float open_loop_step = 0.087266;

  /////////////////////////////////////////////////////////////////////
  /*
Check if you entered pole pair of the stepper
*/
  if (calib_step_pole_pair == 0 && controller.Calib_error == 0)
  {

    // If there is error
    if (controller.pole_pairs != 50 && controller.pole_pairs != 75 && controller.pole_pairs != 100 && controller.pole_pairs != 125)
    {
      controller.PP_status = 1;
      controller.Calib_error = 1;
    }
    else // If there is no error
    {
      controller.PP_status = 2;
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Check if there is magnet
*/
  if (calib_step_magnet == 0 && controller.Calib_error == 0)
  {
    controller.Calibration = 1;
    Collect_data();
    // If there is error
    if (controller.Magnet_warrning == 1)
    {
      controller.Magnet_cal_status = 1;
      calib_step_magnet = 0;
      controller.Calib_error = 1;
    }
    else // If there is no error
    {
      controller.Magnet_cal_status = 2;
      calib_step_magnet = 1;
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Check if the Vbus is in the range
*/
  if (calib_step_voltage == 0 && calib_step_magnet == 1 && controller.Calib_error == 0)
  {
    controller.VBUS_RAW = ADC_CHANNEL_4_READ_VBUS();
    controller.VBUS_mV = Get_voltage_mA(controller.VBUS_RAW);
    // If there is error
    if (controller.VBUS_mV < controller.Min_Vbus || controller.VBUS_mV > controller.Max_Vbus)
    {
      controller.Vbus_cal_status = 1;
      calib_step_voltage = 0;
      controller.Calib_error = 1;
    }
    else // If there is no error
    {
      controller.Vbus_cal_status = 2;
      calib_step_voltage = 1;
      resistance_voltage = controller.Resistance_calc_voltage;
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Check if you entered a valid resistance
*/
  if (calib_step_resistance == 0 && calib_step_voltage == 1 && controller.Calib_error == 0)
  {
    // If resistance is 0, ie. if you didnt change initial resistance via serial terminal
    if (controller.Total_Resistance == 0 && controller.Resistance == 0)
    {
      controller.Resistance_cal_status = 1;
      controller.Total_Resistance = 0;
      controller.Resistance = 0;
      controller.Calib_error = 1;
    }
    else
    {
      controller.Resistance_cal_status = 2;
      calib_step_resistance = 1;
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Small delay between resistance mesure and inductance mesure to drain the coil
*/
  if (calib_step_delay == 0 && calib_step_resistance == 1 && controller.Calib_error == 0)
  {
    tick_cnt = tick_cnt + 1;
    if (tick_cnt == delay)
    {
      calib_step_delay = 1;
    }
  }

  /////////////////////////////////////////////////////////////////////
  /// Check if you entered a valid Inductance
  if (calib_step_delay == 1 && calib_step_inductance == 0 && controller.Calib_error == 0)
  {
    // If Inductance is 0, ie. if you didnt change initial inductance via serial terminal
    if (controller.Inductance == 0)
    {
      controller.Inductance_cal_status = 1;
      calib_step_inductance = 1;
      controller.Calib_error = 1;
    }
    else
    {
      controller.Inductance_cal_status = 2;
      controller.Calib_error = 0;
      calib_step_inductance = 1;

      PID.Ki = (float)1 - (float)exp(-controller.Resistance * LOOP_TIME / controller.Inductance);
      controller.Crossover_frequency = ((float)controller.current_control_bandwidth) / (float)(LOOP_FREQ / PI2);
      PID.Kp_iq = (controller.Resistance * (float)(controller.Crossover_frequency / (float)(PID.Ki))) / 7; // devide by 7 is found empirically to give decent results ¯\_(ツ)_/¯ 
      PID.Ki_iq = ((float)PID.Ki * (float)PID.Kp_iq);
      PID.Kp_id = PID.Kp_iq;
      PID.Ki_id = PID.Ki_iq;
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Open loop spin; check if we spin in good direction, if not fail and prompt user to switch phases
*/
  if (calib_step_inductance == 1 && calib_dir_pole_pair == 0 && controller.Calib_error == 0)
  {
    tick_tick = tick_tick + 1;
    Collect_data();
    // Set initial position and speed
    if (open_loop_start == 0)
    {
      open_loop_start = 1;
    }

    // Read the position here after it becomes stable; we introduce small delay = 20000 * LOOP_TIME
    if (tick_tick == 20000 && open_loop_delay == 0 && open_loop_start == 1)
    {
      // Collect_data();
      start_position = controller.Position_Ticks;
      open_loop_delay = 1;
    }

    // Spin the motor Open loop for "number_of_elec_rotations"
    if (open_loop_delay == 1 && move_stop == 0)
    {

      // float speed_el, int voltage
      Open_loop_speed_stepper(300, 5000);
      if (tick_tick == 40000)
      {
        move_stop = 1;
      }
    }

    // After we spun the motor
    if (move_stop == 1)
    {
      // Collect_data();
      int end_position = controller.Position_Ticks;
      // If our end position is smaller than the start
      if (end_position < start_position)
      {

        controller.Open_loop_cal_status = 1;
        controller.Calib_error = 1;
        calib_dir_pole_pair = 0;
      }
      else
      {

        controller.Open_loop_cal_status = 2;
        calib_dir_pole_pair = 1;
        controller.Calib_error = 0;
      }
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Check if the phase order is correct
*/
  if (calib_dir_pole_pair == 1 && Phase_order_step == 0 && controller.Calib_error == 0)
  {
    Phase_ticks = Phase_ticks + 1;
    switch (Phase_step)
    {
    case 0: // Small delay where the motor is stopped

      if (Phase_ticks == 10000)
      {
        Phase_step = 1;
        Phase_ticks = 0;
      }
      else
      {
        Collect_data2();
        sine_cosine_calc(controller.Electric_Angle);
        park_transform(controller.Sense1_mA, controller.Sense2_mA, &FOC.Id, &FOC.Iq);
        PID.Uq_setpoint = 0;
        Voltage_Torque_mode();
      }

      break;

    case 1: // Collect the initial position

      Collect_data2();
      start_position = controller.Position_Ticks;
      Phase_step = 2;

      break;

    case 2: // Spin the motor using voltage torque mode
      if (Phase_ticks < Spin_duration)
      {
        Collect_data2();
        /* Calculate Iq and Id currents at this moment*/
        sine_cosine_calc(controller.Electric_Angle);
        park_transform(controller.Sense1_mA, controller.Sense2_mA, &FOC.Id, &FOC.Iq);
        PID.Uq_setpoint = 3000;
        Voltage_Torque_mode();
      }
      else
      {
        // perform checks
        int end_position = controller.Position_Ticks;
        int delta = end_position - start_position;
        int half_rotation_ticks = CPR / 2;

        if (delta > half_rotation_ticks)
        {
          // ✅ spun forward, enough distance
          Phase_step = 3;
        }
        else
        {
          // ❌ wrong direction or just vibrating
          controller.Calib_error = 1;
          PID.Uq_setpoint = 0;
          Voltage_Torque_mode();
          Phase_ticks = 0;
          Phase_start = 1;
          Phase_step = 0;
          Phase_order_step = 0;
          controller.Kt_cal_status = 1;
          controller.Phase_order_status = 1;
        }
      }

      break;

    case 3: // Small delay
      Phase_step = 4;
      controller.theta_offset = 0;
      break;

    case 4: // Perform angle offset search
            /* Get all data needed for FOC */
      Collect_data();
      /***********************************/

      /* Calculate Iq and Id currents at this moment*/
      sine_cosine_calc(controller.Electric_Angle);
      park_transform(controller.Sense1_mA, controller.Sense2_mA, &FOC.Id, &FOC.Iq);
      controller.temp_var_offset_calib = Calibrate_Angle_Offset2();
      if (controller.temp_var_offset_calib == 0)
      {
        Phase_step = 4;
      }
      else if (controller.temp_var_offset_calib == 1)
      {
        Phase_step = 5;
      }
      else if (controller.temp_var_offset_calib == -1)
      {
        Phase_ticks = 0;
        Phase_start = 1;
        Phase_step = 0;
        Phase_order_step = 0;
        controller.Kt_cal_status = 1;
        controller.Phase_order_status = 1;
        // controller.Calib_error = 1;
      }
      break;

    case 5: // End, previous steps were good, go to the Kt calculation
      Phase_ticks = 0;
      Phase_start = 1;
      Phase_step = 0;
      Phase_order_step = 1;
      controller.Kt_cal_status = 2;
      controller.Phase_order_status = 2;
      controller.Calib_error = 0;
      break;
    }
  }

  /// Calculate Kt, KV and Flux linkage
  if (Phase_order_step == 1 && KV_step == 0 && controller.Calib_error == 0)
  {

    if (KV_ticks <= KV_duration)
    {
      PID.Iq_current_limit = 1600;
      PID.Iq_setpoint = 500;
      Collect_data();
      Torque_mode();
      sine_cosine_calc(controller.Electric_Angle);
      park_transform(controller.Sense1_mA, controller.Sense2_mA, &FOC.Id, &FOC.Iq);
      KV_ticks = KV_ticks + 1;
      float rpm_speed = (controller.Velocity_Filter * 60) / 16384;
      KV_speed_accumulator = KV_speed_accumulator + rpm_speed;
      KV_current_accumulator = KV_current_accumulator + FOC.Iq;
    }
    else
    {

      float rpm_filtered = KV_speed_accumulator / KV_duration;
      float current_filtered = KV_current_accumulator / KV_duration;
      int load_correction = PID.Iq_setpoint / (PID.Iq_setpoint - current_filtered);
      /// ((controller.pole_pairs * 4) / controller.pole_pairs ) is just to reming that we need to multiple by 4; 1.8 deg stepper = 200 steps / 4 = 50 pole pairs
      /// Thus 4 is the magic number
      float KV_value = (rpm_filtered / (controller.VBUS_mV / 1000)) * ((controller.pole_pairs * 4) / controller.pole_pairs);
      controller.KV = KV_value;
      controller.Kt = float(8.27) / controller.KV;
      controller.flux_linkage = (float)2 / 3 * (float)controller.Kt / (float)controller.pole_pairs;

      controller.temp1 = KV_value;
      pwm_set(PWM_CH1, PWM_MAX, 13);
      pwm_set(PWM_CH2, PWM_MAX, 13);
      pwm_set(PWM_CH3, PWM_MAX, 13);
      pwm_set(PWM_CH4, PWM_MAX, 13);
      KV_step = 1;
      controller.KV_status = 2;
    }
  }

  // If we get to last step without error
  if (KV_step == 1)
  {
    // Detach calibration interrupt routine
    Ticker_detach(TIM3);
    controller.Calibration = 3;
    controller.Calibrated = 1;
    // Reset all flags
    calib_step_magnet = 0;
    calib_step_voltage = 0;
    calib_step_resistance = 0;
    calib_step_inductance = 0;
    calib_step_delay = 0;
    Phase_order_step = 0;
    calib_dir_pole_pair = 0;
    tick_cnt = 0;
    check_Vbus = 0;
    check_magnet = 0;
    Res_avg = 0;
    Resistance_accumulator = 0;
    Inductor_start = 0;
    inductance_accumulator = 0.0;
    inductance_step = 0;
    inductance_tick = 0;
    inductance_tick_2 = 0;
    Phase_step = 0;
    Phase_ticks = 0;
    Phase_start = 0;
    _speed_accumulator = 0;
    theta_SIM = 0;
    velocity_step = 0;
    open_loop_start = 0;
    start_position = 0;
    move_stop = 0;
    tick_tick = 0;
    open_loop_delay = 0;
    resistance_voltage = 0;
    KV_ticks = 0;
    KV_speed_accumulator = 0;
    KV_current_accumulator = 0;
    KV_step = 0;
  }

  // If there is error
  if (controller.Calib_error == 1)
  {
    // Detach calibration interrupt routine
    Ticker_detach(TIM3);
    controller.Calibration = 2;
    controller.Calibrated = 0;
    // Reset all flags
    calib_step_magnet = 0;
    calib_step_voltage = 0;
    calib_step_resistance = 0;
    calib_step_inductance = 0;
    calib_step_delay = 0;
    Phase_order_step = 0;
    calib_dir_pole_pair = 0;
    tick_cnt = 0;
    check_Vbus = 0;
    check_magnet = 0;
    Res_avg = 0;
    Resistance_accumulator = 0;
    Inductor_start = 0;
    inductance_accumulator = 0.0;
    inductance_step = 0;
    inductance_tick = 0;
    inductance_tick_2 = 0;
    Phase_step = 0;
    Phase_ticks = 0;
    Phase_start = 0;
    _speed_accumulator = 0;
    theta_SIM = 0;
    velocity_step = 0;
    open_loop_start = 0;
    start_position = 0;
    move_stop = 0;
    tick_tick = 0;
    open_loop_delay = 0;
    resistance_voltage = 0;
    KV_ticks = 0;
    KV_speed_accumulator = 0;
    KV_current_accumulator = 0;
    KV_step = 0;
  }
}

/// @brief Reports data when in calibration mode
/// @param Serialport
void Calib_report(Stream &Serialport)
{

  static bool print_flag[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static bool PP_flag = 0;

  if (controller.Calibration != 0)
  {
    if (controller.Calibration == 1 && print_flag[0] == 0)
    {
      Serialport.print("----------------");
      Serialport.println(" ");
      Serialport.print("Calibration mode");
      Serialport.println(" ");
      Serialport.println("----------------");

      print_flag[0] = 1;
    }

    /// Pole pair status
    if (controller.PP_status != 0 && PP_flag == 0)
    {
      if (controller.PP_status == 1)
      {
        Serialport.print("Pole pair test: Failed");
        Serialport.println(" ");
        Serialport.print("For 1.8 deg stepper enter #PP 50 ");
        Serialport.println(" ");
        Serialport.print("For 1.2 deg stepper enter #PP 75");
        Serialport.println(" ");
        Serialport.print("For 0.9 deg stepper enter #PP 100");
        Serialport.println(" ");
        Serialport.print("For 0.72 deg stepper enter #PP 125");
        Serialport.println(" ");
      }
      else if (controller.PP_status == 2)
      {
        Serialport.print("Pole pair test: Passed");
        Serialport.println(" ");
      }
      PP_flag = 1;
    }

    /// Magnet test status
    if (controller.Magnet_cal_status != 0 && print_flag[1] == 0)
    {
      if (controller.Magnet_cal_status == 1)
      {
        Serialport.print("Magnet test: Failed");
        Serialport.println(" ");
      }
      else if (controller.Magnet_cal_status == 2)
      {
        Serialport.print("Magnet test: Passed");
        Serialport.println(" ");
      }
      print_flag[1] = 1;
    }

    // Vbus test status
    if (controller.Vbus_cal_status != 0 && print_flag[2] == 0)
    {
      if (controller.Vbus_cal_status == 1)
      {
        Serialport.print("Vbus test: Failed");
        Serialport.println(" ");
      }
      else if (controller.Vbus_cal_status == 2)
      {
        Serialport.print("Vbus test: Passed");
        Serialport.println(" ");
      }
      print_flag[2] = 1;
    }

    // Resistance test status
    if (controller.Resistance_cal_status != 0 && print_flag[3] == 0)
    {
      if (controller.Resistance_cal_status == 1)
      {
        Serialport.print("Resistance test: Failed");
        Serialport.println(" ");
        Serialport.print("Please enter your stepper resistance by using #R command");
        Serialport.println(" ");
      }
      else if (controller.Resistance_cal_status == 2)
      {
        Serialport.print("Resistance = ");
        Serialport.print(controller.Resistance, 6);
        Serialport.print("Ohm");
        Serialport.println(" ");
      }
      print_flag[3] = 1;
    }

    // Inductance test status
    if (controller.Inductance_cal_status != 0 && print_flag[4] == 0)
    {
      if (controller.Inductance_cal_status == 1)
      {
        Serialport.print("Inductance test: Failed");
        Serialport.println(" ");
        Serialport.print("Please enter your stepper inductance by using #L command");
        Serialport.println(" ");
      }
      else if (controller.Inductance_cal_status == 2)
      {
        Serialport.print("Inductance = ");
        Serialport.print(controller.Inductance, 6);
        Serialport.print("H");
        Serialport.println(" ");

        Serialport.print("Kp_iq = ");
        Serialport.print(PID.Kp_iq, 6);
        Serialport.println(" ");

        Serialport.print("Ki_iq = ");
        Serialport.print(PID.Ki_iq, 6);
        Serialport.println(" ");
      }
      // Print Ki Kp za struje i zadani bandwidth
      print_flag[4] = 1;
    }

    // Open loop status
    if (controller.Open_loop_cal_status != 0 && print_flag[5] == 0)
    {
      if (controller.Open_loop_cal_status == 1)
      {
        Serialport.println("Open loop: Failed");
        Serialport.println("Switch motor phases and try again!");
        Serialport.println(" ");
      }
      else if (controller.Open_loop_cal_status == 2)
      {
        Serialport.println("Open loop direction: good ");
        Serialport.println(" ");
      }
      // Settings used
      print_flag[5] = 1;
    }

    // Phase order test status
    if (controller.Phase_order_status != 0 && print_flag[6] == 0)
    {
      if (controller.Phase_order_status == 1)
      {
        Serialport.println(" ");
        Serialport.println("Phase order is wrong!");
        Serialport.println("Switch motor phases and try again!");
      }
      else if (controller.Phase_order_status == 2)
      {
        Serialport.print("Angle offset is: ");
        Serialport.print(controller.theta_offset);
        Serialport.println(" ");
        Serialport.print("Phase order is good!");
        Serialport.println(" ");
      }
      print_flag[6] = 1;
    }

    // Kt call
    if (controller.KV_status != 0 && print_flag[7] == 0)
    {
      if (controller.KV_status == 2)
      {
        Serialport.print("KV is: ");
        Serialport.print(controller.KV, 5);
        Serialport.println(" ");
        Serialport.print("Kt is: ");
        Serialport.print(controller.Kt, 5);
        Serialport.println(" ");
        Serialport.print("Flux linkage is: ");
        Serialport.print(controller.flux_linkage, 6);
        Serialport.println(" ");
        Serialport.print("----------------");
      }

      print_flag[7] = 1;
    }

    // If calib failed
    // Set clalibration status to 0 (go outside of calibration) set calibrated flag to 0 and go idle.
    // Also report error.
    if (controller.Calibration == 2 && print_flag[8] == 0)
    {
      Serialport.print("Calibration failed!");
      Serialport.println(" ");
      Serialport.print("----------------");
      Serialport.println(" ");
      controller.Controller_mode = 0;
      // Settings used
      print_flag[8] = 1;
      Ticker_detach(TIM3);
      Ticker_init(TIM3, LOOP_FREQ, IT_callback);
      controller.Error = 0;
      controller.Calibrated = 0;

      controller.Calibration = 0;
      controller.Calib_error = 0;
      controller.Magnet_cal_status = 0;
      controller.Vbus_cal_status = 0;
      controller.Resistance_cal_status = 0;
      controller.Inductance_cal_status = 0;
      controller.Open_loop_cal_status = 0;
      controller.Kt_cal_status = 0;
      controller.Phase_order_status = 0;
      controller.KV_status = 0;
      for (int i = 0; i < sizeof(print_flag) / sizeof(print_flag[0]); i++)
      {
        print_flag[i] = 0; // Set each element to 0
      }
    }

    // If calib was success
    // Set clalibration status to 0 (go outside of calibration)
    // set calibrated flag to 1 and enter closed loop mode (Idle) if there are no errors.
    if (controller.Calibration == 3 && print_flag[9] == 0)
    {

      Serialport.print("Calibration success!");
      Serialport.println(" ");
      Serialport.print("Save config with #Save");
      Serialport.println(" ");
      Serialport.print("Going to idle!");
      Serialport.println(" ");
      Serialport.print("----------------");
      Serialport.println(" ");
      controller.Controller_mode = 0;
      // Settings used
      print_flag[9] = 1;
      Ticker_detach(TIM3);
      Ticker_init(TIM3, LOOP_FREQ, IT_callback);
      controller.Error = 0;
      controller.Calibrated = 1;

      controller.Calibration = 0;
      controller.Calib_error = 0;
      controller.Magnet_cal_status = 0;
      controller.Vbus_cal_status = 0;
      controller.Resistance_cal_status = 0;
      controller.Inductance_cal_status = 0;
      controller.Open_loop_cal_status = 0;
      controller.Kt_cal_status = 0;
      controller.Phase_order_status = 0;
      controller.KV_status = 0;
      for (int i = 0; i < sizeof(print_flag) / sizeof(print_flag[0]); i++)
      {
        print_flag[i] = 0; // Set each element to 0
      }
    }
  }
}

/// @brief How LED acts depending on control mode and errors
/// @param ms clock with ms resolution
void LED_status(uint32_t ms)
{
  static uint32_t last_time = 0;
  static uint32_t last_time_LED = 0;
  static uint32_t BLINK_cnt = 0;
  static uint32_t BLINK_delay_cnt = 0;
  static int led_state = 0;

  if (controller.Calibration == 1)
  {
    if ((ms - last_time_LED) > 50) // run every x ms
    {
      led_state = !led_state;
      digitalWriteFast(LED, led_state);
      last_time_LED = ms;
    }
  }

  else if (controller.Error == 1)
  {
    digitalWriteFast(LED, HIGH);
  }

  else if (controller.Error == 0 && controller.Calibrated == 1)
  {
    if (BLINK_cnt <= 6)
    {
      if ((ms - last_time_LED) > 115) // run every x ms
      {
        led_state = !led_state;
        digitalWriteFast(LED, led_state);
        last_time_LED = ms;
        BLINK_cnt = BLINK_cnt + 1;
      }
    }
    else
    {
      digitalWriteFast(LED, LOW);
      if ((ms - last_time_LED) > 400) // run every x ms
      {

        digitalWriteFast(LED, LOW);
        last_time_LED = ms;
        BLINK_cnt = 0;
      }
    }
  }
}

/// @brief Encoder parity check. Not used at this moment.
/// @param data
/// @return
bool parityCheck(uint16_t data)
{
  data ^= data >> 8;
  data ^= data >> 4;
  data ^= data >> 2;
  data ^= data >> 1;

  return (~data) & 1;
}

/// @brief Get current in mA from RAW encoder ticks. 0A is equal to ADC_MIDPOINT
/// @param adc_value RAW ADC value
/// @return Current in mA
int Get_current_mA(int adc_value)
{

  // Formula is Vout = Vref + (Vsense * GAIN)
  // int current = qfp_fmul((adc_value - 2048), CURRENT_SENSE_CONSTANT_mV);

  int current = qfp_fmul((adc_value - 2048), CURRENT_SENSE_CONSTANT_mV);
  return current;
}

int Get_ADC_Value(int current_mA)
{
  // Rearranged formula: adc_value = (current / CURRENT_SENSE_CONSTANT_mV) + 2048
  int adc_value = (current_mA / CURRENT_SENSE_CONSTANT_mV) + 2048;

  // Clamp the result within the range of a 12-bit ADC
  adc_value = constrain(adc_value, 0, 4095);

  return adc_value;
}

/// @brief Get current in A from RAW encoder ticks. 0A is equal to ADC_MIDPOINT
/// @param adc_value RAW ADC value
/// @return Current in A
float Get_current(int adc_value)
{

  // Formula is Vout = Vref + (Vsense * GAIN)
  float current = qfp_fmul((adc_value - 2048), CURRENT_SENSE_CONSTANT);

  return current;
}

/// @brief Max Voltage is 40V, 10K and 870 ohm resistor in series. For 3v3 on ADC pin we need 41.231V on input.
/// @brief 41231 mV = 4095 ADC ticks => (41231/4095) = 10.06 = 10. Voltage_mv = ADC_tick * 10
/// @param adc_value RAW ADC value
/// @return Voltage in mv
int Get_voltage_mA(int adc_value)
{

  int voltage = qfp_fmul(adc_value, 10);
  return voltage;
}

/// @brief  FOC control but with Ud and Uq voltages; useful for checking if your motor is
/// connected correctly and parametrization of the motor
void Voltage_Torque_mode()
{

  float Ud_setpoint = PID.Ud_setpoint; //(PID.Kp_id * Id_error + PID.Id_errSum);
  float Uq_setpoint = PID.Uq_setpoint; //(PID.Kp_iq * Iq_error + PID.Iq_errSum);

  int voltage_limit_var = PID.Voltage_limit;
  if (PID.Voltage_limit == 0)
  {
    voltage_limit_var = controller.VBUS_mV;
  }
  else if (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV)
  {
    voltage_limit_var = PID.Voltage_limit;
  }
  else
  {
    voltage_limit_var = controller.VBUS_mV;
  }

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, (voltage_limit_var));        // Normalizira Ud i Uq na vbus value
  inverse_park_transform(Ud_setpoint, Uq_setpoint, &FOC.Ua, &FOC.Ub); // Tu dobije Ualpha i Ubeta koji se kreću od -Vbus do Vbus?
  FOC.PWM1 = PWM_MAX;
  FOC.PWM2 = PWM_MAX;
  FOC.PWM3 = PWM_MAX;
  FOC.PWM4 = PWM_MAX;

  FOC.Ua = constrain(FOC.Ua, -controller.VBUS_mV, controller.VBUS_mV);
  FOC.Ub = constrain(FOC.Ub, -controller.VBUS_mV, controller.VBUS_mV);

  if (FOC.Ua > 0)
    FOC.PWM3 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM4 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);

  if (FOC.Ub > 0)
    FOC.PWM2 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM1 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);

  // write to PWM hardware
  Phase_order();
}

/// @brief  FOC cascade torque/current mode
void Torque_mode()
{

  /* Clamp Iq to current limit*/
  if (PID.Iq_setpoint > PID.Iq_current_limit)
    PID.Iq_setpoint = PID.Iq_current_limit;
  else if (PID.Iq_setpoint < -PID.Iq_current_limit)
    PID.Iq_setpoint = -PID.Iq_current_limit;

  /* Current PIDS*/
  float Id_error = PID.Id_setpoint - FOC.Id;
  float Iq_error = PID.Iq_setpoint - FOC.Iq;

  PID.Id_errSum = PID.Id_errSum + Id_error * PID.Ki_id;
  PID.Iq_errSum = PID.Iq_errSum + Iq_error * PID.Ki_iq;

  int voltage_limit_var = PID.Voltage_limit;
  if (PID.Voltage_limit == 0)
  {
    voltage_limit_var = controller.VBUS_mV;
  }
  else if (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV)
  {
    voltage_limit_var = PID.Voltage_limit;
  }
  else
  {
    voltage_limit_var = controller.VBUS_mV;
  }

  /* Handle integral windup*/
  limit_norm(&PID.Id_errSum, &PID.Iq_errSum, voltage_limit_var);

  float Ud_setpoint = (PID.Kp_id * Id_error + PID.Id_errSum);
  float Uq_setpoint = (PID.Kp_iq * Iq_error + PID.Iq_errSum);

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, (voltage_limit_var));        // Normalizira Ud i Uq na vbus value
  inverse_park_transform(Ud_setpoint, Uq_setpoint, &FOC.Ua, &FOC.Ub); // Tu dobije Ualpha i Ubeta koji se kreću od -Vbus do Vbus?
  FOC.PWM1 = PWM_MAX;
  FOC.PWM2 = PWM_MAX;
  FOC.PWM3 = PWM_MAX;
  FOC.PWM4 = PWM_MAX;

  FOC.Ua = constrain(FOC.Ua, -controller.VBUS_mV, controller.VBUS_mV);
  FOC.Ub = constrain(FOC.Ub, -controller.VBUS_mV, controller.VBUS_mV);

  if (FOC.Ua > 0)
    FOC.PWM3 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM4 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);

  if (FOC.Ub > 0)
    FOC.PWM2 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM1 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);

  // write to PWM hardware
  Phase_order();
}

/// @todo feedforwards
/// @brief Impedance PD controller
/// @todo add cogging comp
void PD_mode()
{
  // PID.Iq_setpoint = PID.KP *() + PID.KD*() + current_feedforward + cogging_comp
  PID.Iq_setpoint = PID.KP * (PID.Position_setpoint - controller.Position_Ticks) + PID.KD * (PID.Velocity_setpoint - controller.Velocity_Filter) + PID.Feedforward_current;
  // PID.Iq_setpoint = PID.KP * (0 - controller.Position_Ticks) + PID.KD * (0 - controller.Velocity_Filter);

  /* Clamp Iq to current limit*/

  if (PID.Iq_setpoint > PID.Iq_current_limit)
    PID.Iq_setpoint = PID.Iq_current_limit;
  else if (PID.Iq_setpoint < -PID.Iq_current_limit)
    PID.Iq_setpoint = -PID.Iq_current_limit;

  /* Current PIDS*/
  float Id_error = PID.Id_setpoint - FOC.Id;
  float Iq_error = PID.Iq_setpoint - FOC.Iq;

  PID.Id_errSum = PID.Id_errSum + Id_error * PID.Ki_id;
  PID.Iq_errSum = PID.Iq_errSum + Iq_error * PID.Ki_iq;

  int voltage_limit_var = PID.Voltage_limit;
  if (PID.Voltage_limit == 0)
  {
    voltage_limit_var = controller.VBUS_mV;
  }
  else if (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV)
  {
    voltage_limit_var = PID.Voltage_limit;
  }
  else
  {
    voltage_limit_var = controller.VBUS_mV;
  }

  /* Handle integral windup*/
  limit_norm(&PID.Id_errSum, &PID.Iq_errSum, voltage_limit_var);

  // float Ud_setpoint = 0; //(PID.Kp_id * Id_error + PID.Id_errSum);
  // float Uq_setpoint = 8000; //(PID.Kp_iq * Iq_error + PID.Iq_errSum);

  float Ud_setpoint = (PID.Kp_id * Id_error + PID.Id_errSum);
  float Uq_setpoint = (PID.Kp_iq * Iq_error + PID.Iq_errSum);

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, (voltage_limit_var));        // Normalizira Ud i Uq na vbus value
  inverse_park_transform(Ud_setpoint, Uq_setpoint, &FOC.Ua, &FOC.Ub); // Tu dobije Ualpha i Ubeta koji se kreću od -Vbus do Vbus?
  FOC.PWM1 = PWM_MAX;
  FOC.PWM2 = PWM_MAX;
  FOC.PWM3 = PWM_MAX;
  FOC.PWM4 = PWM_MAX;

  FOC.Ua = constrain(FOC.Ua, -controller.VBUS_mV, controller.VBUS_mV);
  FOC.Ub = constrain(FOC.Ub, -controller.VBUS_mV, controller.VBUS_mV);

  if (FOC.Ua > 0)
    FOC.PWM3 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM4 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);

  if (FOC.Ub > 0)
    FOC.PWM2 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM1 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);

  // write to PWM hardware
  Phase_order();
}

/// @todo feedforwards
/// @brief  FOC cascade velocity mode
/// @todo add cogging comp
void Velocity_mode()
{

  /*Clamp velocity to velocity limit*/
  if (PID.Velocity_setpoint > PID.Velocity_limit)
    PID.Velocity_setpoint = PID.Velocity_limit;
  else if (PID.Velocity_setpoint < -PID.Velocity_limit)
    PID.Velocity_setpoint = -PID.Velocity_limit;

  /* Velocity PIDS*/
  float velocity_error = PID.Velocity_setpoint - controller.Velocity_Filter;
  PID.V_errSum = PID.V_errSum + velocity_error * PID.Ki_v;

  /* Clamp integral term*/
  if (PID.V_errSum > PID.Iq_current_limit)
    PID.V_errSum = PID.Iq_current_limit;
  else if (PID.V_errSum < -PID.Iq_current_limit)
    PID.V_errSum = -PID.Iq_current_limit;

  /* Calculate Iq setpoint*/
  PID.Iq_setpoint = PID.Kp_v * velocity_error + PID.V_errSum + PID.Feedforward_current; // + cogging_comp

  /* Clamp Iq to current limit*/

  if (PID.Iq_setpoint > PID.Iq_current_limit)
    PID.Iq_setpoint = PID.Iq_current_limit;
  else if (PID.Iq_setpoint < -PID.Iq_current_limit)
    PID.Iq_setpoint = -PID.Iq_current_limit;

  /* Current PIDS*/
  float Id_error = PID.Id_setpoint - FOC.Id;
  float Iq_error = PID.Iq_setpoint - FOC.Iq;

  PID.Id_errSum = PID.Id_errSum + Id_error * PID.Ki_id;
  PID.Iq_errSum = PID.Iq_errSum + Iq_error * PID.Ki_iq;

  int voltage_limit_var = PID.Voltage_limit;
  if (PID.Voltage_limit == 0)
  {
    voltage_limit_var = controller.VBUS_mV;
  }
  else if (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV)
  {
    voltage_limit_var = PID.Voltage_limit;
  }
  else
  {
    voltage_limit_var = controller.VBUS_mV;
  }

  /* Handle integral windup*/
  limit_norm(&PID.Id_errSum, &PID.Iq_errSum, voltage_limit_var);

  // float Ud_setpoint = 0; //(PID.Kp_id * Id_error + PID.Id_errSum);
  // float Uq_setpoint = 8000; //(PID.Kp_iq * Iq_error + PID.Iq_errSum);

  float Ud_setpoint = (PID.Kp_id * Id_error + PID.Id_errSum);
  float Uq_setpoint = (PID.Kp_iq * Iq_error + PID.Iq_errSum);

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, (voltage_limit_var));        // Normalizira Ud i Uq na vbus value
  inverse_park_transform(Ud_setpoint, Uq_setpoint, &FOC.Ua, &FOC.Ub); // Tu dobije Ualpha i Ubeta koji se kreću od -Vbus do Vbus?
  FOC.PWM1 = PWM_MAX;
  FOC.PWM2 = PWM_MAX;
  FOC.PWM3 = PWM_MAX;
  FOC.PWM4 = PWM_MAX;

  FOC.Ua = constrain(FOC.Ua, -controller.VBUS_mV, controller.VBUS_mV);
  FOC.Ub = constrain(FOC.Ub, -controller.VBUS_mV, controller.VBUS_mV);

  if (FOC.Ua > 0)
    FOC.PWM3 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM4 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);

  if (FOC.Ub > 0)
    FOC.PWM2 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM1 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);

  // write to PWM hardware
  Phase_order();
}

/// @todo feedforwards
/// @brief FOC cascaded position mode
/// @todo add cogging comp
void Position_mode()
{
  /* Positon PIDS*/
  float position_error = PID.Position_setpoint - controller.Position_Ticks;
  PID.P_errSum = PID.Kp_p * position_error;
  PID.Velocity_setpoint = PID.P_errSum + PID.Feedforward_speed;

  /*Clamp velocity to velocity limit*/
  if (PID.Velocity_setpoint > PID.Velocity_limit)
    PID.Velocity_setpoint = PID.Velocity_limit;
  else if (PID.Velocity_setpoint < -PID.Velocity_limit)
    PID.Velocity_setpoint = -PID.Velocity_limit;

  /* Velocity PIDS*/
  float velocity_error = PID.Velocity_setpoint - controller.Velocity_Filter;
  PID.V_errSum = PID.V_errSum + velocity_error * PID.Ki_v;

  /* Clamp integral term*/
  if (PID.V_errSum > PID.Iq_current_limit)
    PID.V_errSum = PID.Iq_current_limit;
  else if (PID.V_errSum < -PID.Iq_current_limit)
    PID.V_errSum = -PID.Iq_current_limit;

  /* Calculate Iq setpoint*/
  PID.Iq_setpoint = PID.Kp_v * velocity_error + PID.V_errSum + PID.Feedforward_current; // + cogging_comp

  /* Clamp Iq to current limit*/
  if (PID.Iq_setpoint > PID.Iq_current_limit)
    PID.Iq_setpoint = PID.Iq_current_limit;
  else if (PID.Iq_setpoint < -PID.Iq_current_limit)
    PID.Iq_setpoint = -PID.Iq_current_limit;

  /* Current PIDS*/
  float Id_error = PID.Id_setpoint - FOC.Id;
  float Iq_error = PID.Iq_setpoint - FOC.Iq;

  PID.Id_errSum = PID.Id_errSum + Id_error * PID.Ki_id;
  PID.Iq_errSum = PID.Iq_errSum + Iq_error * PID.Ki_iq;

  int voltage_limit_var = PID.Voltage_limit;
  if (PID.Voltage_limit == 0)
  {
    voltage_limit_var = controller.VBUS_mV;
  }
  else if (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV)
  {
    voltage_limit_var = PID.Voltage_limit;
  }
  else
  {
    voltage_limit_var = controller.VBUS_mV;
  }

  /* Handle integral windup*/
  limit_norm(&PID.Id_errSum, &PID.Iq_errSum, voltage_limit_var);

  // float Ud_setpoint = 0; //(PID.Kp_id * Id_error + PID.Id_errSum);
  // float Uq_setpoint = 8000; //(PID.Kp_iq * Iq_error + PID.Iq_errSum);

  float Ud_setpoint = (PID.Kp_id * Id_error + PID.Id_errSum);
  float Uq_setpoint = (PID.Kp_iq * Iq_error + PID.Iq_errSum);

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, (voltage_limit_var));        // Normalizira Ud i Uq na vbus value
  inverse_park_transform(Ud_setpoint, Uq_setpoint, &FOC.Ua, &FOC.Ub); // Tu dobije Ualpha i Ubeta koji se kreću od -Vbus do Vbus?
  FOC.PWM1 = PWM_MAX;
  FOC.PWM2 = PWM_MAX;
  FOC.PWM3 = PWM_MAX;
  FOC.PWM4 = PWM_MAX;

  FOC.Ua = constrain(FOC.Ua, -controller.VBUS_mV, controller.VBUS_mV);
  FOC.Ub = constrain(FOC.Ub, -controller.VBUS_mV, controller.VBUS_mV);

  if (FOC.Ua > 0)
    FOC.PWM3 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM4 = (int)PWM_MAX * map_float(abs(FOC.Ua), 0, controller.VBUS_mV, 1, 0);

  if (FOC.Ub > 0)
    FOC.PWM2 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);
  else
    FOC.PWM1 = (int)PWM_MAX * map_float(abs(FOC.Ub), 0, controller.VBUS_mV, 1, 0);

  // write to PWM hardware
  Phase_order();
}

/// @brief Spin the motor in the open loop
/// @param speed Speed in TICKS/S
/// @param trigger_value
void Hall_Indexing_mode()
{

  // Read hall sensor
  controller.additional2_var = digitalReadFast(ADDITIONAL2);

  // If hall sensor value is same as the trigger value we set
  if (controller.additional2_var == controller.trigger_value)
  {
    if (controller.hall_trigger == 1)
    {
      PID.Position_setpoint = controller.Position_Ticks;
      controller.hall_trigger = 0;
    }
    Position_mode();
  }
  else
  {

    Velocity_mode();
    controller.hall_trigger = 1;
  }
}

/// @brief Spin the stepper motor in open loop
/// @param speed_el Electrical speed in RAD/s; for steppers this is step frequency in rad/s
/// @param voltage Drive voltage in millivolts
void Open_loop_speed_stepper(float speed_el, int voltage)
{
  static float theta_SIM = 0;

  // Advance rotor electrical angle
  theta_SIM += speed_el * LOOP_TIME;
  theta_SIM = fmod(theta_SIM, PI2);
  if (theta_SIM < 0)
    theta_SIM += PI2;

  // Generate 2-phase excitation
  float Ua = voltage * sinf(theta_SIM);
  float Ub = voltage * cosf(theta_SIM);

  // Reset PWM values
  FOC.PWM1 = PWM_MAX;
  FOC.PWM2 = PWM_MAX;
  FOC.PWM3 = PWM_MAX;
  FOC.PWM4 = PWM_MAX;

  // Phase A drive (Ua -> CH1/CH2)
  if (Ua > 0)
    FOC.PWM3 = (int)(PWM_MAX * map_float(fabs(Ua), 0, controller.VBUS_mV, 1, 0));
  else
    FOC.PWM4 = (int)(PWM_MAX * map_float(fabs(Ua), 0, controller.VBUS_mV, 1, 0));

  // Phase B drive (Ub -> CH3/CH4)
  if (Ub > 0)
    FOC.PWM1 = (int)(PWM_MAX * map_float(fabs(Ub), 0, controller.VBUS_mV, 1, 0));
  else
    FOC.PWM2 = (int)(PWM_MAX * map_float(fabs(Ub), 0, controller.VBUS_mV, 1, 0));

  // Apply to hardware
  Phase_order();
}

/// @brief Step the motor open loop with selectable speed and microstepping.
/// @param steps Number of steps to move (positive = forward, negative = reverse)
/// @param voltage Drive voltage in millivolts
/// @param microsteps_per_cycle Number of microsteps per electrical cycle
/// @param step_interval_us Delay between microsteps in microseconds
void Open_loop_move_steps(int steps, int voltage, int microsteps_per_cycle, int step_interval_us)
{
  static int step_index = 0;
  static int target_steps = 0;
  static int done_steps = 0;
  static int step_timer = 0;
  static int step_interval_ticks = 1;
  static int dir = 1;

  // Convert interval (µs) to loop ticks
  if (step_interval_us < LOOP_TIME * 1e6)
    step_interval_us = (int)(LOOP_TIME * 1e6); // enforce minimum = 1 loop tick

  step_interval_ticks = (int)(step_interval_us / (LOOP_TIME * 1e6));
  if (step_interval_ticks < 1)
    step_interval_ticks = 1;

  // Start a new move if requested
  if (steps != 0 && target_steps == 0)
  {
    target_steps = abs(steps);
    done_steps = 0;
    dir = (steps > 0) ? 1 : -1;
    step_timer = 0;
  }

  // If there's no active move, return immediately
  if (target_steps == 0)
  {
    controller.Controller_mode = 0;
    return;
  }

  // Wait until the step interval has elapsed
  if (++step_timer < step_interval_ticks)
    return;

  step_timer = 0;

  // Advance the step index
  step_index += dir;
  if (step_index >= microsteps_per_cycle)
    step_index = 0;
  if (step_index < 0)
    step_index = microsteps_per_cycle - 1;

  // Compute the excitation
  float theta = (PI2 * step_index) / microsteps_per_cycle;
  float Ua = voltage * sinf(theta);
  float Ub = voltage * cosf(theta);

  // Set the PWMs to their initial state
  FOC.PWM1 = PWM_MAX;
  FOC.PWM2 = PWM_MAX;
  FOC.PWM3 = PWM_MAX;
  FOC.PWM4 = PWM_MAX;

  // Phase A
  if (Ua > 0)
    FOC.PWM3 = (int)(PWM_MAX * map_float(fabs(Ua), 0, controller.VBUS_mV, 1, 0));
  else
    FOC.PWM4 = (int)(PWM_MAX * map_float(fabs(Ua), 0, controller.VBUS_mV, 1, 0));

  // Phase B
  if (Ub > 0)
    FOC.PWM1 = (int)(PWM_MAX * map_float(fabs(Ub), 0, controller.VBUS_mV, 1, 0));
  else
    FOC.PWM2 = (int)(PWM_MAX * map_float(fabs(Ub), 0, controller.VBUS_mV, 1, 0));

  Phase_order();

  // One step has been completed
  done_steps++;

  // Stop if the move is complete
  if (done_steps >= target_steps)
  {
    step_index = 0;
    target_steps = 0;
    done_steps = 0;
    step_timer = 0;
    step_interval_ticks = 1;
    dir = 1;
    controller.open_loop_move_done = 1;
    controller.Controller_mode = 0;
  }
}

void Phase_order()
{

  pwm_set(PWM_CH1, FOC.PWM1, 13);
  pwm_set(PWM_CH2, FOC.PWM2, 13);
  pwm_set(PWM_CH3, FOC.PWM3, 13);
  pwm_set(PWM_CH4, FOC.PWM4, 13);
}

void Brake_Coast()
{

  if (controller.brake_coast == 1)
  {
    // Brake
    pwm_set(PWM_CH1, PWM_MAX, 13);
    pwm_set(PWM_CH2, PWM_MAX, 13);
    pwm_set(PWM_CH3, PWM_MAX, 13);
    pwm_set(PWM_CH4, PWM_MAX, 13);
  }
  else
  {
    // Coast
    pwm_set(PWM_CH1, 0, 13);
    pwm_set(PWM_CH2, 0, 13);
    pwm_set(PWM_CH3, 0, 13);
    pwm_set(PWM_CH4, 0, 13);
  }
}


#define POSITION_TOLERANCE 50
#define POSITION_HYSTERESIS (POSITION_TOLERANCE * 2)
#define VELOCITY_CONTACT_THRESHOLD 500.0f
#define CURRENT_CONTACT_RATIO 0.8f

/// @brief Gripper mode
void Gripper_mode()
{
  // --- Reset position flag if a new command was received ---
  if (Gripper.Same_command == 0)
  {
    Gripper.At_position = 0;
  }

  // --- Transform setpoints into motor-readable params ---
  PID.Iq_current_limit = Gripper.current_setpoint;
  PID.Position_setpoint = map(Gripper.position_setpoint, 0, 255, Gripper.max_open_position, Gripper.max_closed_position);
  int vel_setpoint = map(Gripper.speed_setpoint, 0, 255, Gripper.min_speed, Gripper.max_speed);

  // --- Handle speed direction ---
  if (PID.Position_setpoint > controller.Position_Ticks)
  {
    PID.Velocity_setpoint = vel_setpoint;
  }
  else if (PID.Position_setpoint < controller.Position_Ticks)
  {
    PID.Velocity_setpoint = -vel_setpoint;
  }

  // --- Main control condition: valid GOTO command ---
  if (Gripper.action_status == 1 && Gripper.calibrated == 1 && Gripper.activated == 1 && controller.I_AM_GRIPPER == 1)
  {
    int pos_error = abs(PID.Position_setpoint - controller.Position_Ticks);

    // ✅ If we're within tolerance → mark as at position
    if (pos_error < POSITION_TOLERANCE)
    {
      Gripper.At_position = 1;
    }
    // ✅ Only reset if error grows significantly (hysteresis)
    else if (pos_error > POSITION_HYSTERESIS)
    {
      Gripper.At_position = 0;
    }

    // --- Control logic ---
    if (Gripper.At_position)
    {
      // ✅ Once in position → stay in position mode
      Position_mode();
      Gripper.object_detection_status = 3; // at position
    }
    else
    {
      // ✅ Approach target using velocity control
      Velocity_mode();

      float current_abs = fabs(FOC.Iq);
      float velocity_abs = fabs(controller.Velocity_Filter);

      if (velocity_abs > VELOCITY_CONTACT_THRESHOLD)
      {
        // Still moving → no object
        Gripper.object_detection_status = 0;
      }
      else
      {
        // Slowed down → check current for contact
        if (current_abs > CURRENT_CONTACT_RATIO * PID.Iq_current_limit)
        {
          Gripper.object_detection_status = (FOC.Iq > 0) ? 2 : 1;
        }
        else
        {
          Gripper.object_detection_status = 0;
        }
      }
    }
  }
  else
  {
    // --- Idle or disabled state ---
    PID.Velocity_setpoint = 0;
    Velocity_mode();

    float current_abs = fabs(FOC.Iq);
    float velocity_abs = fabs(controller.Velocity_Filter);

    // ✅ Detect contact even when idle (optional)
    if (velocity_abs < VELOCITY_CONTACT_THRESHOLD && current_abs > CURRENT_CONTACT_RATIO * PID.Iq_current_limit)
    {
      Gripper.object_detection_status = (FOC.Iq > 0) ? 2 : 1;
    }
    else
    {
      Gripper.object_detection_status = 0; // free motion
    }
  }

  // ✅ Remember we processed this command
  Gripper.Same_command = 1;
}


/// @brief We need to calibrate the gripper before it can be used
/// Calibration moves gripper to fully open position; maps the value
/// Move to the fully closed position and map the value. Positions are detected
/// If large enough current is detected. After this is complete set Gripper.calibrated = 1
void Calibrate_gripper()
{

  static int tick_gripper_cnt = 0;
  static bool grip_cal_1 = 0;
  static bool grip_cal_2 = 0;
  static int grip_delay_1 = 0;
  static int grip_delay_tick_1 = 0;

  int current_limit = 690;
  int speed_limit = 20; // Lower this if it does not home
  Gripper.In_calibration = 1;

  tick_gripper_cnt = tick_gripper_cnt + 1;

  /// Fully close the gripper (This is step two)
  if (grip_cal_1 == 1 && grip_cal_2 == 0 && grip_delay_1 == 1)
  {
    /// Transform setpoints into motor redable params
    PID.Iq_current_limit = current_limit;
    /// PID.Position_setpoint = map(Gripper.position_setpoint, 0, 255, Gripper.max_open_position, Gripper.max_closed_position);
    int vel_sepoint = map(speed_limit, 0, 255, Gripper.min_speed, Gripper.max_speed);
    PID.Velocity_setpoint = -vel_sepoint;
    Velocity_mode();

    /// If it is not moving and current is around the setpoint

    if (isAroundValue(abs(controller.Velocity_Filter), 0, 700) && isAroundValue(abs(FOC.Iq), PID.Iq_current_limit, 30))
    {
      Gripper.max_closed_position = controller.Position_Ticks;
      // Gripper.max_closed_position =  Gripper.position_ticks;
      grip_cal_2 = 1;
    }
  }
  /***********************************/

  /// Fully open the gripper (This is step one)
  if (grip_cal_1 == 0)
  {
    /// Transform setpoints into motor redable params
    PID.Iq_current_limit = current_limit;
    /// PID.Position_setpoint = map(Gripper.position_setpoint, 0, 255, Gripper.max_open_position, Gripper.max_closed_position);
    int vel_sepoint = map(speed_limit, 0, 255, Gripper.min_speed, Gripper.max_speed);
    PID.Velocity_setpoint = vel_sepoint;
    Velocity_mode();
    /// If it is not moving and current is around the setpoint

    if (isAroundValue(abs(controller.Velocity_Filter), 0, 700) && isAroundValue(abs(FOC.Iq), PID.Iq_current_limit, 30))
    {
      Gripper.max_open_position = controller.Position_Ticks;
      // Gripper.max_open_position = Gripper.position_ticks;
      grip_cal_1 = 1;
      /// record the posiiton, switch speed setpoint sign
    }
  }
  /***********************************/

  /// Small delay between direction change
  if (grip_cal_1 == 1 && grip_delay_1 == 0)
  {
    // digitalWriteFast(SLEEP, LOW);
    // digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;
    grip_delay_tick_1 = grip_delay_tick_1 + 1;
    PID.Velocity_setpoint = 0;
    Velocity_mode();
    if (grip_delay_tick_1 == 100)
    {
      grip_delay_tick_1 = 0;
      grip_delay_1 = 1;
    }
  }
  /***********************************/

  /// At the end reset everything
  if (grip_cal_1 == 1 && grip_cal_2 == 1 && grip_delay_1 == 1)
  {

    Gripper.calibrated = 1;

    grip_cal_1 = 0;
    grip_cal_2 = 0;

    grip_delay_1 = 0;
    grip_delay_tick_1 = 0;
    tick_gripper_cnt = 0;
    // digitalWriteFast(SLEEP, LOW);
    // digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;
    controller.Controller_mode = 0;
    Gripper.In_calibration = 0;
  }
  /***********************************/

  /// Calibration timed out
  /// during whole calibration we keep track of tick_gripper_cnt
  /// One tick is equal to interrupt time.
  if (tick_gripper_cnt >= 150000)
  {
    Gripper.calibrated = 0;

    grip_cal_1 = 0;
    grip_cal_2 = 0;

    grip_delay_1 = 0;
    grip_delay_tick_1 = 0;
    tick_gripper_cnt = 0;
    // digitalWriteFast(SLEEP, LOW);
    // digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;
    controller.Controller_mode = 0;
    Gripper.In_calibration = 0;
  }
}
