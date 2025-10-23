/*
  limits.h - code pertaining to limit-switches and performing the homing cycle
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
#ifndef limits_h
#define limits_h

#include "g32core.h"

// Homing state machine states (non-blocking core)
typedef enum {
    HOMING_STATE_IDLE,        // Homing not active
    HOMING_STATE_INIT,        // Initialize homing parameters
    HOMING_STATE_APPROACH,    // Rapid approach to limit switch
    HOMING_STATE_RETRACT,     // Retract from limit switch
    HOMING_STATE_LOCATE,      // Fine locate limit switch edge
    HOMING_STATE_COMPLETE,    // Homing successful
    HOMING_STATE_ERROR        // Homing failed
} homing_state_t;

// Homing data structure to track non-blocking progress
typedef struct {
    homing_state_t state;          // Current homing state
    uint8_t cycle_mask;             // Axes to home (bitmask: X=bit0, Y=bit1, etc.)
    uint8_t remaining_cycles;      // Remaining homing cycles (approach+retract+locate)
    bool is_approaching;           // True=approaching limit, False=retracting
    float max_travel;              // Max travel distance for current phase
    float homing_rate;             // Feedrate for current phase (mm/min)
    uint16_t axis_lock;            // Lock non-homing axes (step pin mask)
    uint8_t active_axis_count;     // Number of axes being homed
    plan_line_data_t plan_data;    // Motion plan parameters for homing
    float target_pos[N_AXIS];      // Target position for current phase
    uint8_t current_axis_idx;      // Current axis being processed
    bool motion_in_progress;       // Flag: current homing motion active
} homing_data_t;

// Initialize the limits module
void limits_init();

// Disables hard limits interrupt
void limits_disable();

// Enables hard limits interrupt
void limits_enable();

// Returns limit state as a bit-wise uint8 variable (1=triggered)
uint8_t limits_get_state();

// Start non-blocking homing cycle for specified axes
void limits_go_home(uint8_t cycle_mask);

// Process homing state machine (call periodically in main loop)
void limits_homing_process();

// Check if homing is in progress
bool limits_homing_in_progress();

// Check for soft limit violations
void limits_soft_check(float *target);

#ifdef STM32
void HandleLimitIT(void);
#endif

#endif
