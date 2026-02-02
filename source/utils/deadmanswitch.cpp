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

#include <utils/deadmanswitch.hpp>

namespace utils
{
    /**
     * @brief Constructor for dead man switch
     * @param f_speedingControl Reference to speed control interface
     * @param f_timeout_ms Timeout period in milliseconds (default: 500ms)
     */
    CDeadManSwitch::CDeadManSwitch(
        drivers::ISpeedingCommand& f_speedingControl,
        uint32_t f_timeout_ms
    )
        : m_speedingControl(f_speedingControl)
        , m_timeout_ms(f_timeout_ms)
        , m_started(false)
        , m_triggered(false)
    {
    }

    /**
     * @brief Destructor
     */
    CDeadManSwitch::~CDeadManSwitch()
    {
    }

    /**
     * @brief Reset the watchdog timer (called whenever a valid command is received)
     */
    void CDeadManSwitch::resetWatchdog()
    {
        // Initialize watchdog on first reset
        if (!m_started) {
            m_watchdog.start();
            m_started = true;
        }

        // Reset the timer
        m_watchdog.reset();

        // Clear the triggered flag
        m_triggered = false;
    }

    /**
     * @brief Apply speed with safety check. Forces speed to zero if timeout exceeded.
     * @param speed The desired speed value
     */
    void CDeadManSwitch::applySafeSpeed(int speed)
    {
        // Initialize watchdog on first call if not already started
        if (!m_started) {
            m_watchdog.start();
            m_started = true;
        }

        // Check if timeout exceeded (convert milliseconds to microseconds)
        if (m_watchdog.elapsed_time().count() > (int32_t)(m_timeout_ms * 1000)) {
            // Timeout exceeded: force speed to zero (dead man switch engaged)
            m_speedingControl.setSpeed(0);
            m_triggered = true;
        } else {
            // Within timeout window: apply requested speed
            m_speedingControl.setSpeed(speed);
            m_triggered = false;
        }
    }

    /**
     * @brief Check if the watchdog has triggered (timeout exceeded)
     * @return True if timeout has been exceeded, false otherwise
     */
    bool CDeadManSwitch::isTriggered()
    {
        return m_triggered;
    }

}; // namespace utils
