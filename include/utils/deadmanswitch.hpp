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

#ifndef DEAD_MAN_SWITCH_HPP
#define DEAD_MAN_SWITCH_HPP

#include <mbed.h>
#include <drivers/speedingmotor.hpp>

namespace utils
{
    /**
     * @brief Dead man switch watchdog timer that forces motor speed to zero
     *        if no command is received within the specified timeout period.
     *        This is a critical safety feature to prevent runaway vehicles.
     */
    class CDeadManSwitch
    {
        public:
            /**
             * @brief Constructor for dead man switch
             * @param f_speedingControl Reference to speed control interface
             * @param f_timeout_ms Timeout period in milliseconds (default: 500ms)
             */
            CDeadManSwitch(
                drivers::ISpeedingCommand& f_speedingControl,
                uint32_t f_timeout_ms = 500
            );

            /**
             * @brief Destructor
             */
            ~CDeadManSwitch();

            /**
             * @brief Reset the watchdog timer (called whenever a valid command is received)
             */
            void resetWatchdog();

            /**
             * @brief Apply speed with safety check. Forces speed to zero if timeout exceeded.
             * @param speed The desired speed value
             */
            void applySafeSpeed(int speed);

            /**
             * @brief Get the current timeout value
             * @return Timeout in milliseconds
             */
            uint32_t getTimeout() const { return m_timeout_ms; }

            /**
             * @brief Set a new timeout value
             * @param f_timeout_ms New timeout in milliseconds
             */
            void setTimeout(uint32_t f_timeout_ms) { m_timeout_ms = f_timeout_ms; }

            /**
             * @brief Check if the watchdog has triggered (timeout exceeded)
             * @return True if timeout has been exceeded, false otherwise
             */
            bool isTriggered();

        private:
            /* Reference to speed control interface */
            drivers::ISpeedingCommand& m_speedingControl;

            /* Watchdog timer */
            Timer m_watchdog;

            /* Timeout value in milliseconds */
            uint32_t m_timeout_ms;

            /* Flag to track if watchdog has been started */
            bool m_started = false;

            /* Flag to track if watchdog has triggered */
            bool m_triggered = false;
    };

}; // namespace utils

#endif // DEAD_MAN_SWITCH_HPP
