/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

/* Include guard */
#ifndef SPEED_PID_CONTROLLER_HPP
#define SPEED_PID_CONTROLLER_HPP

#include <brain/globalsv.hpp>
#include <chrono>
#include <drivers/speedingmotor.hpp>
#include <mbed.h>
#include <utils/task.hpp>

namespace periodics {
/**
 * @brief Closed-loop PID controller for velocity regulation.
 *
 * Uses IMU-based velocity estimation to maintain constant speed
 * regardless of battery voltage drop or friction variations.
 *
 * Features:
 * - Serial-configurable PID gains (no reflashing needed)
 * - Anti-windup with integral clamping
 * - Feedforward compensation for faster response
 * - Enable/disable toggle for fallback to open-loop
 *
 * Serial Commands:
 * - #pidGains:Kp;Ki;Kd;;   - Set gains (scaled x100)
 * - #pidTarget:speed;;     - Set target speed in mm/s
 * - #pidEnable:0|1;;       - Enable/disable PID
 * - #pidStatus:0;;         - Query current state
 */
class CSpeedPIDController : public utils::CTask {
public:
  /* Constructor */
  CSpeedPIDController(std::chrono::milliseconds f_period,
                      UnbufferedSerial &f_serialPort,
                      drivers::ISpeedingCommand &f_speedingControl);

  /* Destructor */
  ~CSpeedPIDController();

  /* Serial callback for PID gains configuration */
  void serialCallbackPIDGainsCommand(char const *a, char *b);

  /* Serial callback for target speed */
  void serialCallbackPIDTargetCommand(char const *a, char *b);

  /* Serial callback for enable/disable */
  void serialCallbackPIDEnableCommand(char const *a, char *b);

  /* Serial callback for status query */
  void serialCallbackPIDStatusCommand(char const *a, char *b);

private:
  /* PID control loop - runs periodically */
  virtual void _run();

  /* Clamp value to range */
  int32_t clamp(int32_t value, int32_t min, int32_t max);

  /* Reference to Serial object */
  UnbufferedSerial &m_serialPort;

  /* Reference to speed motor driver */
  drivers::ISpeedingCommand &m_speedingControl;

  /* PID gains (scaled x100 for integer math) */
  int32_t m_Kp; // Proportional gain
  int32_t m_Ki; // Integral gain
  int32_t m_Kd; // Derivative gain

  /* PID state variables */
  int32_t m_targetSpeed; // Target speed in mm/s
  int32_t m_integral;    // Accumulated integral term
  int32_t m_prevError;   // Previous error for derivative
  int32_t m_integralMax; // Anti-windup clamp limit

  /* Timing */
  uint16_t m_dt; // Delta time in ms

  /* Enable flag */
  bool m_isEnabled;

  /**
   * Derivative Low-Pass Filter
   *
   * Purpose: Reduces noise amplification from IMU derivative calculations.
   * The filter smooths the derivative term using exponential moving average.
   *
   * To DISABLE: Set m_useDerivativeFilter = false (uses raw derivative)
   * To ENABLE:  Set m_useDerivativeFilter = true  (applies EMA filter)
   *
   * Filter coefficient (m_derivativeFilterAlpha):
   *   - Higher value (e.g., 70) = more smoothing, slower response
   *   - Lower value  (e.g., 30) = less smoothing, faster response
   */
  bool m_useDerivativeFilter;      // Toggle: true = filtered, false = raw
  int32_t m_derivativeFiltered;    // Filtered derivative state
  int32_t m_derivativeFilterAlpha; // Filter coefficient (0-100, default 70)

  /* Scale factor for fixed-point math */
  static const int32_t PID_SCALE = 100;

  /* Speed limits (mm/s) - per BFMC regulations */
  static const int32_t SPEED_MIN = -500;
  static const int32_t SPEED_MAX = 500;

}; // class CSpeedPIDController

}; // namespace periodics

#endif // SPEED_PID_CONTROLLER_HPP
