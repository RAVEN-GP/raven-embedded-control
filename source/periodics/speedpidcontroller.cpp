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

#include <periodics/speedpidcontroller.hpp>

namespace periodics {

/**
 * @brief Constructor - initializes PID with BFMC-tuned default gains.
 *
 * Default values based on Reely TC-04 chassis + Quickrun Fusion SE motor:
 * - Kp = 300 (3.0 actual) - responsive speed correction
 * - Ki = 30  (0.3 actual) - compensate battery voltage drop
 * - Kd = 5   (0.05 actual) - minimal to avoid noise amplification
 */
CSpeedPIDController::CSpeedPIDController(
    std::chrono::milliseconds f_period, UnbufferedSerial &f_serialPort,
    drivers::ISpeedingCommand &f_speedingControl)
    : utils::CTask(f_period), m_serialPort(f_serialPort),
      m_speedingControl(f_speedingControl), m_Kp(300) // Default Kp = 3.0
      ,
      m_Ki(30) // Default Ki = 0.3
      ,
      m_Kd(5) // Default Kd = 0.05
      ,
      m_targetSpeed(0), m_integral(0), m_prevError(0),
      m_integralMax(20000) // Anti-windup: max ±200 mm/s correction
      ,
      m_dt(f_period.count()), m_isEnabled(false),
      m_useDerivativeFilter(true) // OPTIONAL: Set to false to disable filter
      ,
      m_derivativeFiltered(0),
      m_derivativeFilterAlpha(70) // EMA coefficient: 70% old + 30% new
{}

CSpeedPIDController::~CSpeedPIDController() {}

/**
 * @brief Clamp value to specified range.
 */
int32_t CSpeedPIDController::clamp(int32_t value, int32_t min, int32_t max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

/**
 * @brief Main PID control loop - executes at configured period.
 *
 * PID Algorithm:
 *   error = target - current
 *   P = Kp * error
 *   I = Ki * ∫error dt  (with anti-windup)
 *   D = Kd * d(error)/dt
 *   output = feedforward + P + I + D
 *
 * Uses feedforward (target speed) + PID correction for faster response.
 */
void CSpeedPIDController::_run() {
  // Only run if enabled and in KL30 (motor control active)
  if (!m_isEnabled || uint8_globalsV_value_of_kl != 30) {
    return;
  }

  // Read current velocity from IMU global (scaled x1000 in imu.cpp)
  // m_velocityX is in mm/s * 1000, we want mm/s
  int32_t currentVelocity = int32_globalsV_velocity_x_mms / 1000;

  // Calculate error (positive = too slow, need more speed)
  int32_t error = m_targetSpeed - currentVelocity;

  // --- Proportional term ---
  int32_t P = (m_Kp * error) / PID_SCALE;

  // --- Integral term with anti-windup ---
  m_integral += (error * m_dt) / 1000; // Integrate over time (dt in ms)
  m_integral = clamp(m_integral, -m_integralMax, m_integralMax);
  int32_t I = (m_Ki * m_integral) / PID_SCALE;

  // --- Derivative term ---
  int32_t derivative = ((error - m_prevError) * 1000) / m_dt; // Scale for ms->s

  /**
   * DERIVATIVE LOW-PASS FILTER (Optional)
   *
   * Reduces noise amplification from IMU sensor jitter.
   * Uses Exponential Moving Average (EMA): filtered = α*old + (1-α)*new
   *
   * To DISABLE this filter: set m_useDerivativeFilter = false in constructor
   * To adjust smoothing: change m_derivativeFilterAlpha (0-100)
   *   - Higher = more smoothing (slower response)
   *   - Lower  = less smoothing (faster response, more noise)
   */
  int32_t D;
  if (m_useDerivativeFilter) {
    // Apply EMA filter: filtered = (alpha * old + (100-alpha) * new) / 100
    m_derivativeFiltered = (m_derivativeFilterAlpha * m_derivativeFiltered +
                            (100 - m_derivativeFilterAlpha) * derivative) /
                           100;
    D = (m_Kd * m_derivativeFiltered) / PID_SCALE;
  } else {
    // Raw derivative (no filtering)
    D = (m_Kd * derivative) / PID_SCALE;
  }
  m_prevError = error;

  // --- Feedforward + PID output ---
  // Feedforward: use target speed as base
  // PID correction: adjusts for disturbances (battery, friction)
  int32_t output = m_targetSpeed + P + I + D;

  // Clamp to BFMC speed limits
  output = clamp(output, SPEED_MIN, SPEED_MAX);

  // Apply to motor
  m_speedingControl.setSpeed(static_cast<int>(output));
}

/**
 * @brief Serial callback to configure PID gains at runtime.
 *
 * Format: #pidGains:Kp;Ki;Kd;;\r\n
 * Gains are scaled x100 (e.g., Kp=300 means actual Kp=3.0)
 *
 * Example: #pidGains:300;30;5;;\r\n sets Kp=3.0, Ki=0.3, Kd=0.05
 */
void CSpeedPIDController::serialCallbackPIDGainsCommand(char const *a,
                                                        char *b) {
  int kp, ki, kd;
  uint8_t parsed = sscanf(a, "%d;%d;%d", &kp, &ki, &kd);

  if (parsed == 3 && kp >= 0 && ki >= 0 && kd >= 0) {
    m_Kp = kp;
    m_Ki = ki;
    m_Kd = kd;

    // Reset integral to prevent jumps when gains change
    m_integral = 0;

    sprintf(b, "%d;%d;%d", m_Kp, m_Ki, m_Kd);
  } else {
    sprintf(b, "syntax error (Kp;Ki;Kd)");
  }
}

/**
 * @brief Serial callback to set target speed.
 *
 * Format: #pidTarget:speed_mm_s;;\r\n
 *
 * Example: #pidTarget:200;;\r\n sets target to 200 mm/s
 */
void CSpeedPIDController::serialCallbackPIDTargetCommand(char const *a,
                                                         char *b) {
  int target;
  uint8_t parsed = sscanf(a, "%d", &target);

  if (parsed == 1) {
    // Clamp to valid range
    m_targetSpeed = clamp(target, SPEED_MIN, SPEED_MAX);

    // Reset integral when target changes
    m_integral = 0;
    m_prevError = 0;

    sprintf(b, "%d", m_targetSpeed);
  } else {
    sprintf(b, "syntax error");
  }
}

/**
 * @brief Serial callback to enable/disable PID control.
 *
 * Format: #pidEnable:0|1;;\r\n
 *
 * When disabled, speed commands go directly to motor (open-loop).
 * When enabled, PID corrects for velocity errors.
 */
void CSpeedPIDController::serialCallbackPIDEnableCommand(char const *a,
                                                         char *b) {
  uint8_t enable;
  uint8_t parsed = sscanf(a, "%hhu", &enable);

  if (parsed == 1) {
    if (uint8_globalsV_value_of_kl == 30) {
      m_isEnabled = (enable >= 1);

      // Reset PID state on enable/disable
      m_integral = 0;
      m_prevError = 0;
      m_derivativeFiltered = 0; // Reset filter state

      sprintf(b, "%d", m_isEnabled ? 1 : 0);
    } else {
      sprintf(b, "kl 30 required");
    }
  } else {
    sprintf(b, "syntax error");
  }
}

/**
 * @brief Serial callback to query current PID state.
 *
 * Format: #pidStatus:0;;\r\n
 *
 * Response: enabled;target;currentVel;error;Kp;Ki;Kd
 */
void CSpeedPIDController::serialCallbackPIDStatusCommand(char const *a,
                                                         char *b) {
  int32_t currentVelocity = int32_globalsV_velocity_x_mms / 1000;
  int32_t error = m_targetSpeed - currentVelocity;

  sprintf(b, "%d;%d;%d;%d;%d;%d;%d", m_isEnabled ? 1 : 0, m_targetSpeed,
          currentVelocity, error, m_Kp, m_Ki, m_Kd);
}

}; // namespace periodics
