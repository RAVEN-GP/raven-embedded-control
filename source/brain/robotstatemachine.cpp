/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/


#include <brain/robotstatemachine.hpp>

#define scale_ds_to_ms 100

namespace brain{

    /**
     * @brief CRobotStateMachine Class constructor
     * 
     * @param f_period              period for controller execution in seconds
     * @param f_serialPort          reference to serial communication object
     * @param f_steeringControl     reference to steering motor control interface
     * @param f_speedingControl     reference to brushless motor control interface
     */
    CRobotStateMachine::CRobotStateMachine(
            std::chrono::milliseconds                      f_period,
            UnbufferedSerial&             f_serialPort,
            drivers::ISteeringCommand&    f_steeringControl,
            drivers::ISpeedingCommand&    f_speedingControl
        ) 
        : utils::CTask(f_period)
        , m_serialPort(f_serialPort)
        , m_steeringControl(f_steeringControl)
        , m_speedingControl(f_speedingControl)
        , m_state(0)
        , m_ticksRun(0)
        , m_targetTime(0)
        , m_period((uint16_t)(f_period.count()))
        , m_speed(0)
        , m_steering(0)
        , m_calibON(false)
        , m_deadManSwitch(f_speedingControl, 500)
    {
    }

    /** @brief  CRobotStateMachine class destructor
     */
    CRobotStateMachine::~CRobotStateMachine()
    {
    };

    /** \brief  _Run method contains the main application logic, where it controls the lower lever drivers (dc motor and steering) based the given command and state.
     * It has three main states: 
     *  - 1 - move state -> control the motor rotation speed by giving a speed reference, which is then converted to PWM
     *  - 2 - steering state -> trigger the steering of the motor
     *  - 3 - Brake state -> make the motor enter into a brake state. 
     *  - 4 - State responsible for configuring the vehicle's speed and steering over a specified duration.         
     */
    void CRobotStateMachine::_run()
    {   
        char buffer[100];
        switch(m_state)
        {
            // speed state - control the dc motor rotation speed and the steering angle. 
            case 1:
                m_deadManSwitch.applySafeSpeed(m_speed); // Apply speed through safety wrapper (dead man switch)
                snprintf(buffer, sizeof(buffer), "@speed:%d;;\r\n", m_speed);
                m_serialPort.write(buffer, strlen(buffer));
                m_state = 0;
                break;

            // Steering state
            case 2:
                m_steeringControl.setAngle(m_steering); // control the steering angle
                snprintf(buffer, sizeof(buffer), "@steer:%d;;\r\n", m_steering);
                m_serialPort.write(buffer, strlen(buffer));
                m_state = 0;
                break;

            // Brake state
            case 3:
                m_steeringControl.setAngle(m_steering); // control the steering angle 
                m_speedingControl.setBrake();
                snprintf(buffer, sizeof(buffer), "@brake:1;;\r\n");
                m_serialPort.write(buffer, strlen(buffer));
                m_state = 0;
                break;

            // State responsible for configuring the vehicle's speed and steering over a specified duration.
            case 4:
                // If the accumulated ticks exceed the target time, stop the movement and deactivate the task.
                if(m_ticksRun >= m_targetTime+m_period)
                {
                    m_deadManSwitch.applySafeSpeed(0); // Force speed to zero through safety wrapper
                    m_steeringControl.setAngle(0);
                    m_state = 0;

                    if(!m_calibON) m_serialPort.write("@vcd:0;0;0;;\r\n", 15);
                    else{
                       m_serialPort.write("@vcdCalib:0;0;;\r\n", 18);
                       m_calibON = false;
                    } 
                    
                }
                else
                {
                    // Otherwise, increment the tick counter.
                    m_ticksRun += m_period;
                }
                break;
        }
    }

    /** \brief  Serial callback method for speed command
     *
     * Serial callback method setting controller to value received for dc motor control values. 
     * In the case of pid activated, the dc motor control values has to be express in meter per second, otherwise represent the duty cycle of PWM signal in percent. 
     * The steering angle has to express in degree, where the positive values marks the right direction and the negative values noticed the left turning direction.
     *
     * @param a                   string to read data 
     * @param b                   string to write data 
     * 
     */
    void CRobotStateMachine::serialCallbackSPEEDcommand(char const * a, char * b)
    {
        int l_speed;
        uint32_t l_res = sscanf(a,"%d",&l_speed);
        if (1 == l_res)
        {
            if(uint8_globalsV_value_of_kl == 30)
            {
                // Reset dead man switch watchdog on valid command
                m_deadManSwitch.resetWatchdog();

                m_state = 1;
                m_speed = m_speedingControl.inRange(l_speed);

            }
            else{
                sprintf(b,"kl 30 is required!!");
            }
        }
        else
        {
            sprintf(b,"syntax error");
        }
    }

    /** \brief  Serial callback method for steering command
     *
     * Serial callback method setting controller to value received for steering angle.
     * The steering angle has to express in degree, where the positive values marks the right direction and the negative values noticed the left turning direction.
     *
     * @param a                   string to read data 
     * @param b                   string to write data 
     * 
     */
    void CRobotStateMachine::serialCallbackSTEERcommand(char const * a, char * b)
    {
        int l_angle;
        uint32_t l_res = sscanf(a,"%d",&l_angle);
        if (1 == l_res)
        {
            if(uint8_globalsV_value_of_kl == 30)
            {
                // Reset dead man switch watchdog on valid command
                m_deadManSwitch.resetWatchdog();

                m_state = 2;
                m_steering = m_steeringControl.inRange(l_angle);
            }
            else{
                sprintf(b,"kl 30 is required!!");
            }
        }
        else
        {
            sprintf(b,"syntax error");
        }
    }

    /** \brief  Serial callback actions for brake command
     *
     * This method aims to change the state of controller to brake and sets the steering angle to the received value. 
     *
     * @param a                   string to read data 
     * @param b                   string to write data
     * 
     */
    void CRobotStateMachine::serialCallbackBRAKEcommand(char const * a, char * b)
    {
        int l_angle;
        uint32_t l_res = sscanf(a,"%d",&l_angle);
        if(1 == l_res)
        {
            // Reset dead man switch watchdog on valid command
            m_deadManSwitch.resetWatchdog();
            
            m_state = 3;
            m_steering = m_steeringControl.inRange(l_angle);

        }
        else
        {
            sprintf(b,"syntax error");
        }
    }

    /** \brief  Serial callback actions for brake command
     *
     * This method aims to change the state of controller to brake and sets the steering angle to the received value. 
     *
     * @param a                   string to read data 
     * @param b                   string to write data
     * 
     */
    void CRobotStateMachine::serialCallbackVCDcommand(char const * message, char * response)
    {
        int speed, steer;
        uint8_t time_deciseconds;

        uint8_t parsed = sscanf(message, "%d;%d;%hhu", &speed, &steer, &time_deciseconds);

        if(uint8_globalsV_value_of_kl != 30){
            sprintf(response,"kl 30 is required!!");
            return;
        }

        m_targetTime = time_deciseconds;

        if(parsed == 3 && speed < 501 && speed > -501 && steer < 233 && steer > -233)
        {
            // Reset dead man switch watchdog on valid command
            m_deadManSwitch.resetWatchdog();

            sprintf(response, "%d;%d;%d", speed, steer, time_deciseconds);

            m_ticksRun = 0;

            m_targetTime = time_deciseconds * scale_ds_to_ms;

            m_state = 4;

            m_steeringControl.setAngle(steer);
            m_speedingControl.setSpeed(speed);
        }
        else
        {
            sprintf(response, "something went wrong");
        }
    }

    /** \brief  Serial callback actions for brake command
     *
     * This method aims to change the state of controller to brake and sets the steering angle to the received value. 
     *
     * @param a                   string to read data 
     * @param b                   string to write data
     * 
     */
    void CRobotStateMachine::serialCallbackVCDCalibcommand(char const * message, char * response)
    {
        int speed, steer;
        uint8_t time_deciseconds;

        uint8_t parsed = sscanf(message, "%d;%d;%hhu", &speed, &steer, &time_deciseconds);

        if(uint8_globalsV_value_of_kl != 30){
            sprintf(response,"kl 30 is required!!");
            return;
        }

        m_targetTime = time_deciseconds;

        if(parsed == 3 && speed < 501 && speed > -501 && steer < 273 && steer > -273)
        {
            // Reset dead man switch watchdog on valid command
            m_deadManSwitch.resetWatchdog();

            m_ticksRun = 0;

            m_targetTime = time_deciseconds * scale_ds_to_ms;

            m_state = 4;

            m_steeringControl.setAngle(steer);
            m_speedingControl.setSpeed(speed);

            sprintf(response, "%d;%d", m_speedingControl.pwm_value, m_steeringControl.pwm_value);

            m_calibON = true;
        }
        else
        {
            sprintf(response, "something went wrong");
        }
    }

    void CRobotStateMachine::serialCallbackSteerLimitscommand(char const * message, char * response)
    {
        uint8_t msg = 0;

        (void)sscanf(message,"%hhu",&msg);

        sprintf(response, "%d;%d", m_steeringControl.get_lower_limit(), m_steeringControl.get_upper_limit());
    }

    void CRobotStateMachine::serialCallbackAlivecommand(char const * message, char * response)
    {
        uint8_t alive = 0;

        (void)sscanf(message,"%hhu",&alive);

        sprintf(response,"1");
    }

    void CRobotStateMachine::serialCallbackMPCcommand(char const * message, char * response)
    {
        // Format options:
        // 1) "#mpc:STEER;;"              -> steering only
        // 2) "#mpc:SPEED;STEER;;"        -> speed + steering
        //
        // Units:
        // - SPEED is in the same units as existing speed commands (mm/s in your setup)
        // - STEER is in the same units as existing steer commands (e.g. 120 = 12.0 deg if your stack uses *10)

        if(uint8_globalsV_value_of_kl != 30){
            sprintf(response, "kl 30 is required!!");
            return;
        }

        // Reset dead man switch watchdog on valid command
        m_deadManSwitch.resetWatchdog();

        int speed = 0;
        int steer = 0;

        // Try parse "speed;steer"
        int parsed = sscanf(message, "%d;%d", &speed, &steer);

        // If only one number given, treat it as steer
        if(parsed == 1){
            steer = speed;
            speed = 0;
            parsed = 1;
        }

        if(parsed != 1 && parsed != 2){
            sprintf(response, "syntax error");
            return;
        }

        // Clamp to safe ranges using existing drivers
        steer = m_steeringControl.inRange(steer);

        // OPTIONAL speed set
        if(parsed == 2){
            speed = m_speedingControl.inRange(speed);
        }

        // --- Anti-jitter: limit how fast steering can change per command ---
        // (Keeps things smooth even if the high-level controller is noisy)
        const int MAX_STEER_STEP = 12; // tune: smaller = smoother, larger = faster response

        if(!m_mpc_hasLastSteer){
            m_mpc_lastSteer = steer;
            m_mpc_hasLastSteer = true;
        } else {
            int diff = steer - m_mpc_lastSteer;
            if(diff >  MAX_STEER_STEP) steer = m_mpc_lastSteer + MAX_STEER_STEP;
            if(diff < -MAX_STEER_STEP) steer = m_mpc_lastSteer - MAX_STEER_STEP;
            m_mpc_lastSteer = steer;
        }

        // Apply immediately (no state machine timing)
        m_steeringControl.setAngle(steer);
        if(parsed == 2){
            m_speedingControl.setSpeed(speed);
        }

        // Reply
        if(parsed == 2){
            sprintf(response, "%d;%d", speed, steer);
        } else {
            sprintf(response, "%d", steer);
        }
    }


}; // namespace brain