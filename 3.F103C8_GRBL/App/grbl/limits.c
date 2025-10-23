/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2018-2019 Thomas Truong
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "grbl.h"

// Homing configuration constants (matching original GRBL behavior)
#ifndef HOMING_AXIS_SEARCH_SCALAR
#define HOMING_AXIS_SEARCH_SCALAR  2.0f  // Must be >1 to ensure limit trigger
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
#define HOMING_AXIS_LOCATE_SCALAR  5.0f  // Must be >1 to clear limit switch
#endif
#ifndef N_HOMING_LOCATE_CYCLE
#define N_HOMING_LOCATE_CYCLE      1     // Number of fine locate cycles
#endif

// Static homing data (tracks non-blocking state)
static homing_data_t homing_data = {.state = HOMING_STATE_IDLE};

// Initialize limits module (original logic preserved)
void limits_init() {
#ifdef STM32
    // Configure limit pins and interrupts based on settings
    if (bit_isfalse(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
        limits_disable();
    } else {
        limits_enable();
    }
#elif ATMEGA328P
    LIMIT_DDR &= ~(LIMIT_MASK);  // Set limit pins as input
#ifdef DISABLE_LIMIT_PIN_PULL_UP
    LIMIT_PORT &= ~(LIMIT_MASK);  // Normal low (external pull-down required)
#else
    LIMIT_PORT |= (LIMIT_MASK);   // Enable internal pull-up (normal high)
#endif

    // Enable/disable limit interrupt based on settings
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
        LIMIT_PCMSK |= LIMIT_MASK;  // Enable specific limit pins for PCINT
        PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
    } else {
        limits_disable();
    }

#ifdef ENABLE_SOFTWARE_DEBOUNCE
    MCUSR &= ~(1<<WDRF);         // Clear watchdog reset flag
    WDTCSR |= (1<<WDCE) | (1<<WDE); // Enable watchdog configuration
    WDTCSR = (1<<WDP0);         // Set watchdog timeout (~32ms)
#endif
#endif
}

// Disable hard limits interrupt (original logic preserved)
void limits_disable() {
#ifdef STM32F1
    NVIC_DisableIRQ(EXTI15_10_IRQn);
#endif
#ifdef STM32F4
    NVIC_DisableIRQ(EXTI15_10_IRQn);
#endif
#ifdef ATMEGA328P
    LIMIT_PCMSK &= ~LIMIT_MASK;  // Disable specific limit pins
    PCICR &= ~(1 << LIMIT_INT);  // Disable Pin Change Interrupt
#endif
}

// Enable hard limits interrupt (original logic preserved)
void limits_enable() {
#ifdef STM32F4
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    EnableLimitsINT();  // Enable SPI-expander limits (if used)
#endif
#ifdef ATMEGA328P
    LIMIT_PCMSK |= LIMIT_MASK;  // Enable specific limit pins
    PCICR |= (1 << LIMIT_INT);  // Enable Pin Change Interrupt
#endif
}

// Get limit switch state (original logic preserved)
uint8_t limits_get_state() {
    uint8_t limit_state = 0;
#ifdef STM32
    uint16_t pin_state = 0;
#ifdef STM32F1
    pin_state = GPIO_ReadInputData(LIM_GPIO_Port);
#endif
#ifdef STM32F4
    pin_state = GetLimitsState();  // SPI-expander limit read (F4 specific)
#endif

#ifdef INVERT_LIMIT_PIN_MASK
    pin_state ^= INVERT_LIMIT_PIN_MASK;
#endif
    if (bit_isfalse(settings.flags, BITFLAG_INVERT_LIMIT_PINS)) {
        pin_state ^= LIM_MASK;
    }

    // Map pin state to axis bitmask
    for (uint8_t idx = 0; idx < N_AXIS; idx++) {
        if (pin_state & limit_pin_mask[idx]) {
            limit_state |= (1 << idx);
        }
    }
#elif ATMEGA328P
    uint8_t pin_state = (LIMIT_PIN & LIMIT_MASK);
#ifdef INVERT_LIMIT_PIN_MASK
    pin_state ^= INVERT_LIMIT_PIN_MASK;
#endif
    if (bit_isfalse(settings.flags, BITFLAG_INVERT_LIMIT_PINS)) {
        pin_state ^= LIMIT_MASK;
    }

    // Map pin state to axis bitmask
    for (uint8_t idx = 0; idx < N_AXIS; idx++) {
        if (pin_state & get_limit_pin_mask(idx)) {
            limit_state |= (1 << idx);
        }
    }
#endif
    return limit_state;
}

// STM32 Limit Interrupt Handler (original logic preserved)
#ifdef STM32
void HandleLimitIT(void) {
    // Ignore if already in alarm state
    if (sys.state == STATE_ALARM) return;

    // Trigger system reset on hard limit violation
    if (!(sys_rt_exec_alarm)) {
#ifdef HARD_LIMIT_FORCE_STATE_CHECK
        if (limits_get_state()) {
            mc_reset();
            system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
        }
#else
        mc_reset();
        system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
#endif
    }
}
#endif

// ATMEGA328P Limit Interrupt Handler (original logic preserved)
#ifdef ATMEGA328P
#ifndef ENABLE_SOFTWARE_DEBOUNCE
ISR(LIMIT_INT_vect) {
    if (sys.state == STATE_ALARM) return;
    if (!(sys_rt_exec_alarm)) {
#ifdef HARD_LIMIT_FORCE_STATE_CHECK
        if (limits_get_state()) {
            mc_reset();
            system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
        }
#else
        mc_reset();
        system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
#endif
    }
}
#else
// Software debounce using watchdog timer
ISR(LIMIT_INT_vect) {
    if (!(WDTCSR & (1<<WDIE))) {
        WDTCSR |= (1<<WDIE);
    }
}

ISR(WDT_vect) {
    WDTCSR &= ~(1<<WDIE);  // Disable watchdog
    if (sys.state == STATE_ALARM) return;
    if (!(sys_rt_exec_alarm) && limits_get_state()) {
        mc_reset();
        system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
    }
}
#endif
#endif

// Initialize non-blocking homing parameters
static void homing_init(uint8_t cycle_mask) {
    // Reset homing data structure
    memset(&homing_data, 0, sizeof(homing_data_t));
    homing_data.state = HOMING_STATE_INIT;
    homing_data.cycle_mask = cycle_mask;
    
    // Calculate total homing cycles (approach + retract + locate)
    homing_data.remaining_cycles = (2 * N_HOMING_LOCATE_CYCLE) + 1;
    homing_data.is_approaching = true;  // Start with limit approach phase
    homing_data.homing_rate = settings.homing_seek_rate;  // Initial rapid rate

    // Initialize step pin masks and max travel for active axes
    for (uint8_t idx = 0; idx < N_AXIS; idx++) {
#ifdef STM32
        homing_data.target_pos[idx] = 0.0f;
        // Get step pin mask for current axis
        homing_data.axis_lock |= (bit_istrue(cycle_mask, bit(idx))) ? step_pin_mask[idx] : 0;
#elif ATMEGA328P
        homing_data.axis_lock |= (bit_istrue(cycle_mask, bit(idx))) ? get_step_pin_mask(idx) : 0;
#endif

        // Calculate max travel for limit search (ensure switch trigger)
        if (bit_istrue(cycle_mask, bit(idx))) {
            homing_data.active_axis_count++;
            float axis_max_travel = (-HOMING_AXIS_SEARCH_SCALAR) * settings.max_travel[idx];
            homing_data.max_travel = max(homing_data.max_travel, axis_max_travel);
        }
    }

    // Configure motion plan for system homing (no feed override)
    memset(&homing_data.plan_data, 0, sizeof(plan_line_data_t));
    homing_data.plan_data.condition = (PL_COND_FLAG_SYSTEM_MOTION | PL_COND_FLAG_NO_FEED_OVERRIDE);
#ifdef USE_LINE_NUMBERS
    homing_data.plan_data.line_number = HOMING_CYCLE_LINE_NUMBER;
#endif

    // Lock non-homing axes during motion
    sys.homing_axis_lock = homing_data.axis_lock;
}

// Check if homing is in progress
bool limits_homing_in_progress() {
    return (homing_data.state != HOMING_STATE_IDLE && 
            homing_data.state != HOMING_STATE_COMPLETE && 
            homing_data.state != HOMING_STATE_ERROR);
}

// Start non-blocking homing cycle
void limits_go_home(uint8_t cycle_mask) {
    // Abort if homing already active or system in invalid state
    if (limits_homing_in_progress() || sys.abort) return;

    // Disable hard limits during homing (prevent false triggers)
    limits_disable();

    // Initialize homing parameters and start state machine
    homing_init(cycle_mask);
}

// Non-blocking homing state machine processor
void limits_homing_process() {
    // Skip if homing not active
    if (!limits_homing_in_progress()) return;

    // Handle system abort (emergency stop)
    if (sys.abort) {
        homing_data.state = HOMING_STATE_ERROR;
        st_reset();  // Stop all motion
        sys.state = STATE_IDLE;
        limits_enable();  // Re-enable limits
        return;
    }

    switch (homing_data.state) {
        case HOMING_STATE_INIT: {
            // Convert current step position to mm for target calculation
            system_convert_array_steps_to_mpos(homing_data.target_pos, sys_position);

            // Set target position for current phase (approach/retract)
            for (homing_data.current_axis_idx = 0; homing_data.current_axis_idx < N_AXIS; homing_data.current_axis_idx++) {
                if (bit_istrue(homing_data.cycle_mask, bit(homing_data.current_axis_idx))) {
                    // Set direction based on homing direction mask
                    if (bit_istrue(settings.homing_dir_mask, bit(homing_data.current_axis_idx))) {
                        homing_data.target_pos[homing_data.current_axis_idx] = 
                            homing_data.is_approaching ? -homing_data.max_travel : homing_data.max_travel;
                    } else {
                        homing_data.target_pos[homing_data.current_axis_idx] = 
                            homing_data.is_approaching ? homing_data.max_travel : -homing_data.max_travel;
                    }
                }
            }

            // Adjust homing rate for number of active axes (prevent over-speed)
            if (homing_data.active_axis_count > 1) {
                homing_data.homing_rate *= sqrtf((float)homing_data.active_axis_count);
            }
            homing_data.plan_data.feed_rate = homing_data.homing_rate;

            // Plan and start homing motion
            if (plan_buffer_line(homing_data.target_pos, &homing_data.plan_data) == PLAN_OK) {
                sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION;
                st_prep_buffer();  // Load step segments
                st_wake_up();      // Start stepper interrupt
                homing_data.motion_in_progress = true;
                homing_data.state = HOMING_STATE_APPROACH;
            } else {
                homing_data.state = HOMING_STATE_ERROR;
            }
            break;
        }

        case HOMING_STATE_APPROACH: {
            // Skip if motion not active
            if (!homing_data.motion_in_progress) break;

            // Check if limit switch triggered (for active axes)
            if (homing_data.is_approaching) {
                uint8_t limit_state = limits_get_state();
                for (uint8_t idx = 0; idx < N_AXIS; idx++) {
                    if (bit_istrue(homing_data.cycle_mask, bit(idx)) && (limit_state & (1 << idx))) {
                        // Lock axis once limit is triggered
#ifdef STM32
                        homing_data.axis_lock &= ~step_pin_mask[idx];
#elif ATMEGA328P
                        homing_data.axis_lock &= ~get_step_pin_mask(idx);
#endif
                        sys.homing_axis_lock = homing_data.axis_lock;
                    }
                }
            }

            // Check if current motion is complete
            st_prep_buffer();  // Refresh step segment buffer
            if ((homing_data.axis_lock & STEP_MASK) == 0) {
                // Motion complete: stop stepper and reset
                homing_data.motion_in_progress = false;
                st_reset();
                delay_ms(settings.homing_debounce_delay);  // Debounce limit switch

                // Prepare for next phase
                homing_data.is_approaching = !homing_data.is_approaching;
                homing_data.remaining_cycles--;

                // Update parameters for next cycle
                if (homing_data.is_approaching) {
                    // Switch to fine locate phase (slow rate)
                    homing_data.max_travel = settings.homing_pulloff * HOMING_AXIS_LOCATE_SCALAR;
                    homing_data.homing_rate = settings.homing_feed_rate;
                } else {
                    // Switch to retract phase (fast rate)
                    homing_data.max_travel = settings.homing_pulloff;
                    homing_data.homing_rate = settings.homing_seek_rate;
                }

                // Transition to next state
                if (homing_data.remaining_cycles > 0) {
                    homing_data.state = HOMING_STATE_INIT;  // Restart with new parameters
                } else {
                    homing_data.state = HOMING_STATE_COMPLETE;  // All cycles done
                }
            }

            // Handle homing errors (safety door, reset, etc.)
            if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP)) {
                uint8_t rt_exec = sys_rt_exec_state;
                // Set appropriate alarm code
                if (rt_exec & EXEC_RESET) {
                    system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET);
                } else if (rt_exec & EXEC_SAFETY_DOOR) {
                    system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DOOR);
                } else if (!homing_data.is_approaching && (limits_get_state() & homing_data.cycle_mask)) {
                    system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_PULLOFF);
                } else if (homing_data.is_approaching && (rt_exec & EXEC_CYCLE_STOP)) {
                    system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_APPROACH);
                }

                // Trigger system reset on error
                if (sys_rt_exec_alarm) {
                    mc_reset();
                    protocol_execute_realtime();
                    homing_data.state = HOMING_STATE_ERROR;
                    sys.state = STATE_ALARM;
                } else {
                    system_clear_exec_state_flag(EXEC_CYCLE_STOP);
                    homing_data.state = HOMING_STATE_COMPLETE;
                }
            }
            break;
        }

        case HOMING_STATE_COMPLETE: {
            // Calculate final machine position (include pulloff distance)
            int32_t final_axis_pos = 0;
            for (uint8_t idx = 0; idx < N_AXIS; idx++) {
                if (bit_istrue(homing_data.cycle_mask, bit(idx))) {
#ifdef HOMING_FORCE_SET_ORIGIN
                    final_axis_pos = 0;  // Force origin at limit switch
#else
                    // Calculate position with pulloff offset
                    if (bit_istrue(settings.homing_dir_mask, bit(idx))) {
                        final_axis_pos = lround(
                            (settings.max_travel[idx] + settings.homing_pulloff) * settings.steps_per_mm[idx]
                        );
                    } else {
                        final_axis_pos = lround(
                            -settings.homing_pulloff * settings.steps_per_mm[idx]
                        );
                    }
#endif

                    // Update system position for homed axis
                    sys_position[idx] = final_axis_pos;
                }
            }

            // Reset system state and re-enable limits
            sys.step_control = STEP_CONTROL_NORMAL_OP;
            sys.state = STATE_IDLE;
            st_go_idle();  // Disable steppers (per idle settings)
            limits_enable();  // Re-enable hard limits

            // Reset homing state
            homing_data.state = HOMING_STATE_IDLE;
            break;
        }

        case HOMING_STATE_ERROR: {
            // Clean up on error
            st_reset();
            sys.state = STATE_ALARM;
            limits_enable();
            homing_data.state = HOMING_STATE_IDLE;
            break;
        }

        default:
            // Invalid state: reset
            homing_data.state = HOMING_STATE_IDLE;
            break;
    }
}

// Check for soft limit violations before executing motion.
// Called from mc_line() to ensure target position is within soft limits.
void limits_soft_check(float *target) {
    // Skip check if soft limits are disabled
    if (bit_isfalse(settings.flags, BITFLAG_SOFT_LIMIT_ENABLE)) {
        return;
    }

    // Check each axis against its soft limits
    for (uint8_t idx = 0; idx < N_AXIS; idx++) {
        // Skip axes not configured for soft limits
        if (settings.max_travel[idx] <= 0.0f) {
            continue;
        }

        // Check if target exceeds positive or negative soft limit
        if (target[idx] < 0.0f) {
            // Violation: negative limit (below 0)
            system_set_exec_alarm(EXEC_ALARM_SOFT_LIMIT);
            sys.soft_limit = true;  // Flag soft limit violation
            return;
        } else if (target[idx] > settings.max_travel[idx]) {
            // Violation: positive limit (above max travel)
            system_set_exec_alarm(EXEC_ALARM_SOFT_LIMIT);
            sys.soft_limit = true;  // Flag soft limit violation
            return;
        }
    }
}

