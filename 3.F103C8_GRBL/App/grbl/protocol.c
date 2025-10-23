/*
  protocol.c - controls Grbl execution protocol and procedures
  Part of Grbl
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
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

#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)

static char line[LINE_BUFFER_SIZE];
static void protocol_exec_rt_suspend();

// Main Grbl loop (modified to run non-blocking homing)
void protocol_main_loop() {
    // Initial machine checks (original logic preserved)
#ifdef CHECK_LIMITS_AT_INIT
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
        if (limits_get_state()) {
            sys.state = STATE_ALARM;
            report_feedback_message(MESSAGE_CHECK_LIMITS);
        }
    }
#endif

    // Initialize system state after reset/power-up
    if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
        report_feedback_message(MESSAGE_ALARM_LOCK);
        sys.state = STATE_ALARM;
    } else {
        sys.state = STATE_IDLE;
        // Check safety door state
        if (system_check_safety_door_ajar()) {
            bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
            protocol_execute_realtime();
        }
        // Execute startup script
        system_execute_startup(line);
    }

    // Primary main loop
    uint8_t line_flags = 0;
    uint8_t char_counter = 0;
    uint8_t c;
    for (;;) {
        // --------------------------
        // Non-blocking Homing Update
        // --------------------------
        if (sys.state == STATE_HOMING) {
            limits_homing_process();
        }

        // --------------------------
        // Serial Command Processing
        // --------------------------
        while ((c = serial_read()) != SERIAL_NO_DATA) {
            if ((c == '\n') || (c == '\r')) {
                // End of line: process command
                protocol_execute_realtime();
                if (sys.abort) {
                    return;
                }

                line[char_counter] = 0;  // Null-terminate string
#ifdef REPORT_ECHO_LINE_RECEIVED
                report_echo_line_received(line);
#endif

                // Execute command based on type
                if (line_flags & LINE_FLAG_OVERFLOW) {
                    report_status_message(STATUS_OVERFLOW);
                } else if (line[0] == 0) {
                    report_status_message(STATUS_OK);
                } else if (line[0] == '$') {
                    // Process system commands ($H, $X, etc.)
                    report_status_message(system_execute_line(line));
                } else if (sys.state & (STATE_ALARM | STATE_JOG | STATE_HOMING)) {
                    // Block G-code during alarm/jog/homing
                    report_status_message(STATUS_SYSTEM_GC_LOCK);
                } else {
                    // Process G-code
                    report_status_message(gc_execute_line(line));
                }

                // Reset for next line
                line_flags = 0;
                char_counter = 0;
            } else {
                // Process character (filter comments/whitespace)
                if (line_flags) {
                    if (c == ')') {
                        if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) {
                            line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES);
                        }
                    }
                } else {
                    if (c <= ' ') {
                        // Ignore whitespace/control chars
                    } else if (c == '/') {
                        // Ignore block delete (not supported)
                    } else if (c == '(') {
                        line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
                    } else if (c == ';') {
                        line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
                    } else if (char_counter >= (LINE_BUFFER_SIZE - 1)) {
                        line_flags |= LINE_FLAG_OVERFLOW;
                    } else if (c >= 'a' && c <= 'z') {
                        // Convert lowercase to uppercase
                        line[char_counter++] = c - 'a' + 'A';
                    } else {
                        line[char_counter++] = c;
                    }
                }
            }
        }

        // Auto-start cycle if planner has blocks
        protocol_auto_cycle_start();

        // Check realtime commands (emergency stop, feed hold, etc.)
        protocol_execute_realtime();
        if (sys.abort) {
            return;
        }
    }
}

// Block until planner buffer is empty (original logic preserved)
void protocol_buffer_synchronize() {
    protocol_auto_cycle_start();
    do {
        protocol_execute_realtime();
        if (sys.abort) {
            return;
        }
    } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}

// Auto-start cycle if planner has blocks (original logic preserved)
void protocol_auto_cycle_start() {
    if (plan_get_current_block() != NULL) {
        system_set_exec_state_flag(EXEC_CYCLE_START);
    }
}

// Execute realtime commands (original logic preserved)
void protocol_execute_realtime() {
    protocol_exec_rt_system();
    if (sys.suspend) {
        protocol_exec_rt_suspend();
    }
}

// Execute system realtime commands (original logic preserved)
void protocol_exec_rt_system() {
    uint8_t rt_exec = sys_rt_exec_alarm;
    if (rt_exec) {
        // Handle system alarms
        sys.state = STATE_ALARM;
        report_alarm_message(rt_exec);

        // Critical alarm: block until reset
        if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
            report_feedback_message(MESSAGE_CRITICAL_EVENT);
            system_clear_exec_state_flag(EXEC_RESET);
            do {
                if (sys_rt_exec_state & EXEC_RESET) {
                    break;
                }
            } while (1);
        }
        system_clear_exec_alarm();
    }

    // Process realtime state flags
    rt_exec = sys_rt_exec_state;
    if (rt_exec) {
        // Handle system reset
        if (rt_exec & EXEC_RESET) {
            sys.abort = true;
            return;
        }

        // Handle status report request
        if (rt_exec & EXEC_STATUS_REPORT) {
            report_realtime_status();
            system_clear_exec_state_flag(EXEC_STATUS_REPORT);
        }

        // Handle hold/stop commands
        if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP)) {
            if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {
                if (sys.state & (STATE_CYCLE | STATE_JOG)) {
                    if (!(sys.suspend & (SUSPEND_MOTION_CANCEL | SUSPEND_JOG_CANCEL))) {
                        st_update_plan_block_parameters();
                        sys.step_control = STEP_CONTROL_EXECUTE_HOLD;
                        if (sys.state == STATE_JOG && !(rt_exec & EXEC_SLEEP)) {
                            sys.suspend |= SUSPEND_JOG_CANCEL;
                        }
                    }
                }

                if (sys.state == STATE_IDLE) {
                    sys.suspend = SUSPEND_HOLD_COMPLETE;
                }

                if (rt_exec & EXEC_MOTION_CANCEL && !(sys.state & STATE_JOG)) {
                    sys.suspend |= SUSPEND_MOTION_CANCEL;
                }

                if (rt_exec & EXEC_FEED_HOLD && !(sys.state & (STATE_SAFETY_DOOR | STATE_JOG | STATE_SLEEP))) {
                    sys.state = STATE_HOLD;
                }

                if (rt_exec & EXEC_SAFETY_DOOR && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
                    report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR);
                    if (sys.state == STATE_SAFETY_DOOR && (sys.suspend & SUSPEND_INITIATE_RESTORE)) {
#ifdef PARKING_ENABLE
                        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
                            st_update_plan_block_parameters();
                            sys.step_control = (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION);
                            sys.suspend &= ~(SUSPEND_HOLD_COMPLETE);
                        }
#endif
                        sys.suspend &= ~(SUSPEND_RETRACT_COMPLETE | SUSPEND_INITIATE_RESTORE | SUSPEND_RESTORE_COMPLETE);
                        sys.suspend |= SUSPEND_RESTART_RETRACT;
                    }
                    if (sys.state != STATE_SLEEP) {
                        sys.state = STATE_SAFETY_DOOR;
                    }
                    sys.suspend |= SUSPEND_SAFETY_DOOR_AJAR;
                }

                if (rt_exec & EXEC_SLEEP) {
                    if (sys.state == STATE_ALARM) {
                        sys.suspend |= (SUSPEND_RETRACT_COMPLETE | SUSPEND_HOLD_COMPLETE);
                    }
                    sys.state = STATE_SLEEP;
                }
            }
            system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP));
        }

        // Handle cycle start
        if (rt_exec & EXEC_CYCLE_START) {
            if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) {
                if ((sys.state == STATE_SAFETY_DOOR) && !(sys.suspend & SUSPEND_SAFETY_DOOR_AJAR)) {
                    if (sys.suspend & SUSPEND_RESTORE_COMPLETE) {
                        sys.state = STATE_IDLE;
                    } else if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
                        sys.suspend |= SUSPEND_INITIATE_RESTORE;
                    }
                }

                if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
                    if (sys.state == STATE_HOLD && sys.spindle_stop_ovr) {
                        sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE_CYCLE;
                    } else {
                        sys.step_control = STEP_CONTROL_NORMAL_OP;
                        if (plan_get_current_block() && bit_isfalse(sys.suspend, SUSPEND_MOTION_CANCEL)) {
                            sys.suspend = SUSPEND_DISABLE;
                            sys.state = STATE_CYCLE;
                            st_prep_buffer();
                            st_wake_up();
                        } else {
                            sys.suspend = SUSPEND_DISABLE;
                            sys.state = STATE_IDLE;
                        }
                    }
                }
            }
            system_clear_exec_state_flag(EXEC_CYCLE_START);
        }

        // Handle cycle stop (from stepper completion)
        if (rt_exec & EXEC_CYCLE_STOP) {
            if ((sys.state & (STATE_HOLD | STATE_SAFETY_DOOR | STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
                plan_cycle_reinitialize();
                if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
                    sys.suspend |= SUSPEND_HOLD_COMPLETE;
                }
                bit_false(sys.step_control, (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION));
            } else {
                if (sys.suspend & SUSPEND_JOG_CANCEL) {
                    sys.step_control = STEP_CONTROL_NORMAL_OP;
                    plan_reset();
                    st_reset();
                    gc_sync_position();
                    plan_sync_position();
                }

                if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) {
                    sys.suspend &= ~(SUSPEND_JOG_CANCEL);
                    sys.suspend |= SUSPEND_HOLD_COMPLETE;
                    sys.state = STATE_SAFETY_DOOR;
                } else {
                    sys.suspend = SUSPEND_DISABLE;
                    sys.state = STATE_IDLE;
                }
            }
            system_clear_exec_state_flag(EXEC_CYCLE_STOP);
        }
    }

    // Process motion overrides (feed/rapid)
    rt_exec = sys_rt_exec_motion_override;
    if (rt_exec) {
        system_clear_exec_motion_overrides();
        uint8_t new_f_override = sys.f_override;
        uint8_t new_r_override = sys.r_override;

        // Update feed override
        if (rt_exec & EXEC_FEED_OVR_RESET) new_f_override = DEFAULT_FEED_OVERRIDE;
        if (rt_exec & EXEC_FEED_OVR_COARSE_PLUS) new_f_override += FEED_OVERRIDE_COARSE_INCREMENT;
        if (rt_exec & EXEC_FEED_OVR_COARSE_MINUS) new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT;
        if (rt_exec & EXEC_FEED_OVR_FINE_PLUS) new_f_override += FEED_OVERRIDE_FINE_INCREMENT;
        if (rt_exec & EXEC_FEED_OVR_FINE_MINUS) new_f_override -= FEED_OVERRIDE_FINE_INCREMENT;
        new_f_override = min(new_f_override, MAX_FEED_RATE_OVERRIDE);
        new_f_override = max(new_f_override, MIN_FEED_RATE_OVERRIDE);

        // Update rapid override
        if (rt_exec & EXEC_RAPID_OVR_RESET) new_r_override = DEFAULT_RAPID_OVERRIDE;
        if (rt_exec & EXEC_RAPID_OVR_MEDIUM) new_r_override = RAPID_OVERRIDE_MEDIUM;
        if (rt_exec & EXEC_RAPID_OVR_LOW) new_r_override = RAPID_OVERRIDE_LOW;

        // Apply override changes
        if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
            sys.f_override = new_f_override;
            sys.r_override = new_r_override;
            sys.report_ovr_counter = 0;
            plan_update_velocity_profile_parameters();
            plan_cycle_reinitialize();
        }
    }

    // Process accessory overrides (spindle/coolant)
    rt_exec = sys_rt_exec_accessory_override;
    if (rt_exec) {
        system_clear_exec_accessory_overrides();
        uint8_t last_s_override = sys.spindle_speed_ovr;

        // Update spindle override
        if (rt_exec & EXEC_SPINDLE_OVR_RESET) last_s_override = DEFAULT_SPINDLE_SPEED_OVERRIDE;
        if (rt_exec & EXEC_SPINDLE_OVR_COARSE_PLUS) last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT;
        if (rt_exec & EXEC_SPINDLE_OVR_COARSE_MINUS) last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT;
        if (rt_exec & EXEC_SPINDLE_OVR_FINE_PLUS) last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT;
        if (rt_exec & EXEC_SPINDLE_OVR_FINE_MINUS) last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT;
        last_s_override = min(last_s_override, MAX_SPINDLE_SPEED_OVERRIDE);
        last_s_override = max(last_s_override, MIN_SPINDLE_SPEED_OVERRIDE);

        if (last_s_override != sys.spindle_speed_ovr) {
            bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
            sys.spindle_speed_ovr = last_s_override;
            sys.report_ovr_counter = 0;
        }

        // Handle spindle stop override
        if (rt_exec & EXEC_SPINDLE_OVR_STOP && sys.state == STATE_HOLD) {
            if (!(sys.spindle_stop_ovr)) {
                sys.spindle_stop_ovr = SPINDLE_STOP_OVR_INITIATE;
            } else if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_ENABLED) {
                sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE;
            }
        }

        // Handle coolant toggle
        if (rt_exec & (EXEC_COOLANT_FLOOD_OVR_TOGGLE | EXEC_COOLANT_MIST_OVR_TOGGLE)) {
            if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD))) {
                uint8_t coolant_state = gc_state.modal.coolant;
#ifdef ENABLE_M7
                if (rt_exec & EXEC_COOLANT_MIST_OVR_TOGGLE) {
                    coolant_state ^= COOLANT_MIST_ENABLE;
                }
                if (rt_exec & EXEC_COOLANT_FLOOD_OVR_TOGGLE) {
                    coolant_state ^= COOLANT_FLOOD_ENABLE;
                }
#else
                coolant_state ^= COOLANT_FLOOD_ENABLE;
#endif
                coolant_set_state(coolant_state);
                gc_state.modal.coolant = coolant_state;
            }
        }
    }

#ifdef DEBUG
    if (sys_rt_exec_debug) {
        report_realtime_debug();
        sys_rt_exec_debug = 0;
    }
#endif

    // Refresh step segment buffer
    if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP | STATE_JOG)) {
        st_prep_buffer();
    }
}

// System suspend handler (original logic preserved)
static void protocol_exec_rt_suspend() {
#ifdef PARKING_ENABLE
    float restore_target[N_AXIS];
    float parking_target[N_AXIS];
    float retract_waypoint = PARKING_PULLOUT_INCREMENT;
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    memset(pl_data, 0, sizeof(plan_line_data_t));
    pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION | PL_COND_FLAG_NO_FEED_OVERRIDE);
#ifdef USE_LINE_NUMBERS
    pl_data->line_number = PARKING_MOTION_LINE_NUMBER;
#endif
#endif

    plan_block_t *block = plan_get_current_block();
    uint8_t restore_condition;
#ifdef VARIABLE_SPINDLE
    float restore_spindle_speed;
    if (block == NULL) {
        restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant);
        restore_spindle_speed = gc_state.spindle_speed;
    } else {
        restore_condition = block->condition;
        restore_spindle_speed = block->spindle_speed;
    }
#ifdef DISABLE_LASER_DURING_HOLD
    if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
        system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP);
    }
#endif
#else
    restore_condition = (block == NULL) ? (gc_state.modal.spindle | gc_state.modal.coolant) : block->condition;
#endif

    while (sys.suspend) {
        if (sys.abort) {
            return;
        }

        if (sys.suspend & SUSPEND_HOLD_COMPLETE) {
            if (sys.state & (STATE_SAFETY_DOOR | STATE_SLEEP)) {
                if (bit_isfalse(sys.suspend, SUSPEND_RETRACT_COMPLETE)) {
                    sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED;
#ifndef PARKING_ENABLE
                    spindle_set_state(SPINDLE_DISABLE, 0.0);
                    coolant_set_state(COOLANT_DISABLE);
#else
                    system_convert_array_steps_to_mpos(parking_target, sys_position);
                    if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                        memcpy(restore_target, parking_target, sizeof(parking_target));
                        retract_waypoint += restore_target[PARKING_AXIS];
                        retract_waypoint = min(retract_waypoint, PARKING_TARGET);
                    }

#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
                    if ((bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) &&
                        (parking_target[PARKING_AXIS] < PARKING_TARGET) &&
                        bit_false(settings.flags, BITFLAG_LASER_MODE) &&
                        (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
#else
                    if ((bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) &&
                        (parking_target[PARKING_AXIS] < PARKING_TARGET) &&
                        bit_false(settings.flags, BITFLAG_LASER_MODE)) {
#endif
                        if (parking_target[PARKING_AXIS] < retract_waypoint) {
                            parking_target[PARKING_AXIS] = retract_waypoint;
                            pl_data->feed_rate = PARKING_PULLOUT_RATE;
                            pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK);
                            pl_data->spindle_speed = restore_spindle_speed;
                            mc_parking_motion(parking_target, pl_data);
                        }

                        pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION | PL_COND_FLAG_NO_FEED_OVERRIDE);
                        pl_data->spindle_speed = 0.0;
                        spindle_set_state(SPINDLE_DISABLE, 0.0);
                        coolant_set_state(COOLANT_DISABLE);

                        if (parking_target[PARKING_AXIS] < PARKING_TARGET) {
                            parking_target[PARKING_AXIS] = PARKING_TARGET;
                            pl_data->feed_rate = PARKING_RATE;
                            mc_parking_motion(parking_target, pl_data);
                        }
                    } else {
                        spindle_set_state(SPINDLE_DISABLE, 0.0);
                        coolant_set_state(COOLANT_DISABLE);
                    }
#endif
                    sys.suspend &= ~(SUSPEND_RESTART_RETRACT);
                    sys.suspend |= SUSPEND_RETRACT_COMPLETE;
                } else {
                    if (sys.state == STATE_SLEEP) {
                        report_feedback_message(MESSAGE_SLEEP_MODE);
                        spindle_set_state(SPINDLE_DISABLE, 0.0);
                        coolant_set_state(COOLANT_DISABLE);
                        st_go_idle();
                        while (!(sys.abort)) {
                            protocol_exec_rt_system();
                        }
                        return;
                    }

                    if (sys.state == STATE_SAFETY_DOOR && !(system_check_safety_door_ajar())) {
                        sys.suspend &= ~(SUSPEND_SAFETY_DOOR_AJAR);
                    }

                    if (sys.suspend & SUSPEND_INITIATE_RESTORE) {
#ifdef PARKING_ENABLE
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
                        if (((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
                            (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
#else
                        if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
#endif
                            if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                                parking_target[PARKING_AXIS] = retract_waypoint;
                                pl_data->feed_rate = PARKING_RATE;
                                mc_parking_motion(parking_target, pl_data);
                            }
                        }
#endif

                        if (gc_state.modal.spindle != SPINDLE_DISABLE) {
                            if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                                if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
                                    bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
                                } else {
                                    spindle_set_state(
                                        (restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)),
                                        restore_spindle_speed
                                    );
                                    delay_sec(SAFETY_DOOR_SPINDLE_DELAY, DELAY_MODE_SYS_SUSPEND);
                                }
                            }
                        }

                        if (gc_state.modal.coolant != COOLANT_DISABLE) {
                            if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                                coolant_set_state((restore_condition &
                                                   (PL_COND_FLAG_COOLANT_FLOOD | PL_COND_FLAG_COOLANT_MIST)));
                                delay_sec(SAFETY_DOOR_COOLANT_DELAY, DELAY_MODE_SYS_SUSPEND);
                            }
                        }

#ifdef PARKING_ENABLE
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
                        if (((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
                            (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
#else
                        if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
#endif
                            if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                                pl_data->feed_rate = PARKING_PULLOUT_RATE;
                                pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK);
                                pl_data->spindle_speed = restore_spindle_speed;
                                mc_parking_motion(restore_target, pl_data);
                            }
                        }
#endif

                        if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                            sys.suspend |= SUSPEND_RESTORE_COMPLETE;
                            system_set_exec_state_flag(EXEC_CYCLE_START);
                        }
                    }
                }
            } else {
                if (sys.spindle_stop_ovr) {
                    if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_INITIATE) {
                        if (gc_state.modal.spindle != SPINDLE_DISABLE) {
                            spindle_set_state(SPINDLE_DISABLE, 0.0);
                            sys.spindle_stop_ovr = SPINDLE_STOP_OVR_ENABLED;
                        } else {
                            sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED;
                        }
                    } else if (sys.spindle_stop_ovr & (SPINDLE_STOP_OVR_RESTORE | SPINDLE_STOP_OVR_RESTORE_CYCLE)) {
                        if (gc_state.modal.spindle != SPINDLE_DISABLE) {
                            report_feedback_message(MESSAGE_SPINDLE_RESTORE);
                            if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
                                bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
                            } else {
                                spindle_set_state(
                                    (restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)),
                                    restore_spindle_speed
                                );
                            }
                        }
                        if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_RESTORE_CYCLE) {
                            system_set_exec_state_flag(EXEC_CYCLE_START);
                        }
                        sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED;
                    }
                } else {
                    if (bit_istrue(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
                        spindle_set_state(
                            (restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)),
                            restore_spindle_speed
                        );
                        bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
                    }
                }
            }
        }
        protocol_exec_rt_system();
    }
}
