/**
 * @file homing.cpp
 * @brief Implementation of the non-blocking hardware homing sequencer with
 *        soft-start acceleration ramps, precision latching, and CAD home alignment.
 */

#include "homing.h"
#include "step_engine.h"
#include <stdlib.h>
#include <math.h>

static HomingState current_stage = HOMING_IDLE;
static JointSubState substate = SUBSTATE_INIT;

static uint8_t active_mask = 0;
static uint8_t homed_mask = 0;

static uint32_t homing_step_interval = 0;
static uint32_t next_step_clock = 0;
static int32_t  steps_moved = 0;
static int32_t  steps_target = 0;
static int32_t  decel_steps = 0;
static uint8_t  debounce_count = 0;
static float    current_speed = 5.0f;

// Configurations per joint from printer.cfg
struct JointHomeCfg {
    uint8_t joint_idx;
    uint8_t seek_dir;     // 0 = negative, 1 = positive
    float   start_speed;  // deg/s (soft start)
    float   seek_speed;   // deg/s (seek cruise)
    float   accel;        // deg/s^2 (linear ramp)
    float   max_travel;   // deg
    float   retreat_pos;  // safe position after homing (deg)
    float   retreat_speed;// deg/s
    float   cad_home_deg; // CAD reference angle
};

static const JointHomeCfg HOME_CFGS[NUM_JOINTS] = {
    // J1: MOVE=-360 SPEED=20 ACCEL=40 -> RETREAT=+174 SPEED=80 -> CAD=0
    {0, 0, 5.0f, 20.0f, 40.0f, 360.0f,  174.0f, 80.0f,   0.0f},
    // J2: MOVE=-180 SPEED=20 ACCEL=40 -> RETREAT=+50 SPEED=40 -> CAD=-90
    {1, 0, 5.0f, 20.0f, 40.0f, 180.0f,   50.0f, 40.0f, -90.0f},
    // J3: MOVE=-240 SPEED=20 ACCEL=40 -> RETREAT=+70 SPEED=30 -> CAD=180
    {2, 0, 5.0f, 20.0f, 40.0f, 240.0f,   70.0f, 30.0f, 180.0f},
    // J4: MOVE=+270 SPEED=60 ACCEL=80 -> RETREAT=-260 SPEED=80 -> CAD=0
    {3, 1, 5.0f, 60.0f, 80.0f, 270.0f, -260.0f, 80.0f,   0.0f},
    // J5: MOVE=-360 SPEED=60 ACCEL=80 -> RETREAT=+120 SPEED=80 -> CAD=0 (with J6 moves)
    {4, 0, 5.0f, 60.0f, 80.0f, 360.0f,  120.0f, 80.0f,   0.0f},
    // J6: MOVE=+360 SPEED=60 ACCEL=80 -> RETREAT=-4 SPEED=80 -> CAD=180
    {5, 1, 5.0f, 60.0f, 80.0f, 360.0f,   -4.0f, 80.0f, 180.0f}
};

void homing_init(void) {
    current_stage = HOMING_IDLE;
    substate = SUBSTATE_INIT;
    active_mask = 0;
    homed_mask = 0;
}

void homing_start(uint8_t mask) {
    if (mask == 0) return;
    active_mask = mask;
    enable_all_coils();

    // Sequence per printer.cfg: J2 -> J3 -> J1 -> J4 -> J6 -> J5
    if (active_mask & (1 << 1)) {
        current_stage = HOMING_STAGE_J2;
    } else if (active_mask & (1 << 2)) {
        current_stage = HOMING_STAGE_J3;
    } else if (active_mask & (1 << 0)) {
        current_stage = HOMING_STAGE_J1;
    } else if (active_mask & (1 << 3)) {
        current_stage = HOMING_STAGE_J4;
    } else if (active_mask & (1 << 5)) {
        current_stage = HOMING_STAGE_J6;
    } else if (active_mask & (1 << 4)) {
        current_stage = HOMING_STAGE_J5;
    } else {
        current_stage = HOMING_IDLE;
        return;
    }
    substate = SUBSTATE_INIT;
}

void homing_abort(void) {
    current_stage = HOMING_IDLE;
    substate = SUBSTATE_INIT;
    active_mask = 0;
}

bool homing_is_active(void) {
    return (current_stage != HOMING_IDLE && current_stage != HOMING_COMPLETE && current_stage != HOMING_ERROR);
}

HomingState homing_get_state(void) {
    return current_stage;
}

uint8_t homing_get_homed_mask(void) {
    return homed_mask;
}

static void advance_to_next_stage(void) {
    substate = SUBSTATE_INIT;
    switch (current_stage) {
        case HOMING_STAGE_J2:
            homed_mask |= (1 << 1);
            if (active_mask & (1 << 2)) current_stage = HOMING_STAGE_J3;
            else if (active_mask & (1 << 0)) current_stage = HOMING_STAGE_J1;
            else if (active_mask & (1 << 3)) current_stage = HOMING_STAGE_J4;
            else if (active_mask & (1 << 5)) current_stage = HOMING_STAGE_J6;
            else if (active_mask & (1 << 4)) current_stage = HOMING_STAGE_J5;
            else current_stage = HOMING_APPLY_CAD_HOME;
            break;

        case HOMING_STAGE_J3:
            homed_mask |= (1 << 2);
            if (active_mask & (1 << 0)) current_stage = HOMING_STAGE_J1;
            else if (active_mask & (1 << 3)) current_stage = HOMING_STAGE_J4;
            else if (active_mask & (1 << 5)) current_stage = HOMING_STAGE_J6;
            else if (active_mask & (1 << 4)) current_stage = HOMING_STAGE_J5;
            else current_stage = HOMING_APPLY_CAD_HOME;
            break;

        case HOMING_STAGE_J1:
            homed_mask |= (1 << 0);
            if (active_mask & (1 << 3)) current_stage = HOMING_STAGE_J4;
            else if (active_mask & (1 << 5)) current_stage = HOMING_STAGE_J6;
            else if (active_mask & (1 << 4)) current_stage = HOMING_STAGE_J5;
            else current_stage = HOMING_APPLY_CAD_HOME;
            break;

        case HOMING_STAGE_J4:
            homed_mask |= (1 << 3);
            if (active_mask & (1 << 5)) current_stage = HOMING_STAGE_J6;
            else if (active_mask & (1 << 4)) current_stage = HOMING_STAGE_J5;
            else current_stage = HOMING_APPLY_CAD_HOME;
            break;

        case HOMING_STAGE_J6:
            homed_mask |= (1 << 5);
            if (active_mask & (1 << 4)) current_stage = HOMING_STAGE_J5;
            else current_stage = HOMING_APPLY_CAD_HOME;
            break;

        case HOMING_STAGE_J5:
            homed_mask |= (1 << 4);
            current_stage = HOMING_APPLY_CAD_HOME;
            break;

        default:
            current_stage = HOMING_COMPLETE;
            break;
    }
}

// Process single joint homing state machine
static void process_joint_homing(uint8_t cfg_idx) {
    const JointHomeCfg* cfg = &HOME_CFGS[cfg_idx];
    uint8_t j = cfg->joint_idx;
    uint32_t now = get_master_clock();

    switch (substate) {
        case SUBSTATE_INIT:
            // Initialize soft-start speed
            current_speed = cfg->start_speed;
            homing_step_interval = (uint32_t)(16000000.0f / (current_speed * STEPS_PER_DEG[j]));
            next_step_clock = now + homing_step_interval;
            steps_moved = 0;
            steps_target = (int32_t)(cfg->max_travel * STEPS_PER_DEG[j]);
            debounce_count = 0;

            // Handle special pre-moves (e.g. J5 requires J6 move -10 deg)
            if (j == 4 && (active_mask == 0xFF || (active_mask & (1 << 5)))) {
                substate = SUBSTATE_PRE_MOVE;
                set_joint_dir(5, 0); // J6 dir negative
                steps_target = (int32_t)(10.0f * STEPS_PER_DEG[5]);
                homing_step_interval = (uint32_t)(16000000.0f / (80.0f * STEPS_PER_DEG[5]));
            } else {
                set_joint_dir(j, cfg->seek_dir);
                substate = SUBSTATE_SEEK_ENDSTOP;
            }
            break;

        case SUBSTATE_PRE_MOVE:
            if ((int32_t)(now - next_step_clock) >= 0) {
                next_step_clock = now + homing_step_interval;
                pulse_single_step(5); // Pre-move J6
                steps_moved++;
                step_engine_set_position(5, step_engine_get_position(5) - 1);
                if (steps_moved >= steps_target) {
                    // Pre-move finished, begin J5 seek with soft-start
                    steps_moved = 0;
                    steps_target = (int32_t)(cfg->max_travel * STEPS_PER_DEG[j]);
                    current_speed = cfg->start_speed;
                    homing_step_interval = (uint32_t)(16000000.0f / (current_speed * STEPS_PER_DEG[j]));
                    set_joint_dir(j, cfg->seek_dir);
                    substate = SUBSTATE_SEEK_ENDSTOP;
                }
            }
            break;

        case SUBSTATE_SEEK_ENDSTOP:
            if ((int32_t)(now - next_step_clock) >= 0) {
                next_step_clock = now + homing_step_interval;

                if (read_endstop_triggered(j)) {
                    // Switch hit during seek -> prepare for switch release backoff
                    steps_moved = 0;
                    steps_target = (int32_t)(3.0f * STEPS_PER_DEG[j]); // 3 degrees release backoff
                    set_joint_dir(j, cfg->seek_dir ? 0 : 1);           // reverse direction
                    homing_step_interval = (uint32_t)(16000000.0f / (8.0f * STEPS_PER_DEG[j])); // gentle release speed
                    substate = SUBSTATE_LATCH_RELEASE;
                } else {
                    pulse_single_step(j);
                    steps_moved++;
                    step_engine_set_position(j, step_engine_get_position(j) + (cfg->seek_dir ? 1 : -1));

                    // Soft-start acceleration ramp up to seek cruise speed
                    if (current_speed < cfg->seek_speed) {
                        float dt = 1.0f / (current_speed * STEPS_PER_DEG[j]);
                        current_speed += cfg->accel * dt;
                        if (current_speed > cfg->seek_speed) {
                            current_speed = cfg->seek_speed;
                        }
                        homing_step_interval = (uint32_t)(16000000.0f / (current_speed * STEPS_PER_DEG[j]));
                    }

                    if (steps_moved >= steps_target) {
                        // Limit reached without hitting switch -> homing fault
                        current_stage = HOMING_ERROR;
                    }
                }
            }
            break;

        case SUBSTATE_LATCH_RELEASE:
            if ((int32_t)(now - next_step_clock) >= 0) {
                next_step_clock = now + homing_step_interval;
                pulse_single_step(j);
                steps_moved++;
                step_engine_set_position(j, step_engine_get_position(j) + (cfg->seek_dir ? -1 : 1));

                // Wait until switch is released plus min steps
                if (!read_endstop_triggered(j) && steps_moved >= steps_target) {
                    // Switch released! Begin slow precision latch approach
                    set_joint_dir(j, cfg->seek_dir);
                    steps_moved = 0;
                    steps_target = (int32_t)(6.0f * STEPS_PER_DEG[j]);
                    current_speed = 3.0f; // Ultra-slow precision latch speed (3 deg/s)
                    homing_step_interval = (uint32_t)(16000000.0f / (current_speed * STEPS_PER_DEG[j]));
                    debounce_count = 0;
                    substate = SUBSTATE_LATCH_PRECISION;
                }
            }
            break;

        case SUBSTATE_LATCH_PRECISION:
            if ((int32_t)(now - next_step_clock) >= 0) {
                next_step_clock = now + homing_step_interval;

                if (read_endstop_triggered(j)) {
                    debounce_count++;
                    if (debounce_count >= 5) {
                        // Switch reliably precision-latched!
                        // Lock current physical position as 0
                        step_engine_set_position(j, 0);

                        // Configure retreat move with smooth trapezoidal acceleration/deceleration
                        float retreat_deg = cfg->retreat_pos;
                        uint8_t retreat_dir = (retreat_deg >= 0.0f) ? 1 : 0;
                        set_joint_dir(j, retreat_dir);

                        steps_target = (int32_t)(fabs(retreat_deg) * STEPS_PER_DEG[j]);
                        steps_moved = 0;

                        // Calculate decel distance: s_ramp = (v_max^2 - v_start^2) / (2 * a)
                        float s_ramp = (cfg->retreat_speed * cfg->retreat_speed - cfg->start_speed * cfg->start_speed) / (2.0f * cfg->accel);
                        decel_steps = (int32_t)(s_ramp * STEPS_PER_DEG[j]);
                        if (decel_steps > steps_target / 2) {
                            decel_steps = steps_target / 2;
                        }

                        current_speed = cfg->start_speed;
                        homing_step_interval = (uint32_t)(16000000.0f / (current_speed * STEPS_PER_DEG[j]));
                        next_step_clock = now + homing_step_interval;
                        substate = SUBSTATE_RETREAT;
                    }
                } else {
                    debounce_count = 0;
                    pulse_single_step(j);
                    steps_moved++;
                    step_engine_set_position(j, step_engine_get_position(j) + (cfg->seek_dir ? 1 : -1));

                    if (steps_moved >= steps_target) {
                        // Latch precision distance exhausted without switch contact
                        current_stage = HOMING_ERROR;
                    }
                }
            }
            break;

        case SUBSTATE_RETREAT:
            if ((int32_t)(now - next_step_clock) >= 0) {
                next_step_clock = now + homing_step_interval;
                pulse_single_step(j);
                steps_moved++;
                step_engine_set_position(j, step_engine_get_position(j) + (cfg->retreat_pos >= 0 ? 1 : -1));

                int32_t steps_remaining = steps_target - steps_moved;
                float dt = 1.0f / (current_speed * STEPS_PER_DEG[j]);

                // Trapezoidal acceleration & deceleration profile
                if (steps_remaining <= decel_steps) {
                    // Decelerate smoothly into retreat destination
                    current_speed -= cfg->accel * dt;
                    if (current_speed < cfg->start_speed) {
                        current_speed = cfg->start_speed;
                    }
                } else if (current_speed < cfg->retreat_speed) {
                    // Accelerate softly up to retreat cruise speed
                    current_speed += cfg->accel * dt;
                    if (current_speed > cfg->retreat_speed) {
                        current_speed = cfg->retreat_speed;
                    }
                }
                homing_step_interval = (uint32_t)(16000000.0f / (current_speed * STEPS_PER_DEG[j]));

                if (steps_moved >= steps_target) {
                    if (j == 4 && (active_mask == 0xFF || (active_mask & (1 << 5)))) {
                        // J5 post-move: J6 MOVE=-5 deg
                        substate = SUBSTATE_POST_MOVE;
                        set_joint_dir(5, 0); // J6 dir negative
                        steps_target = (int32_t)(5.0f * STEPS_PER_DEG[5]);
                        steps_moved = 0;
                        homing_step_interval = (uint32_t)(16000000.0f / (80.0f * STEPS_PER_DEG[5]));
                    } else {
                        advance_to_next_stage();
                    }
                }
            }
            break;

        case SUBSTATE_POST_MOVE:
            if ((int32_t)(now - next_step_clock) >= 0) {
                next_step_clock = now + homing_step_interval;
                pulse_single_step(5);
                steps_moved++;
                step_engine_set_position(5, step_engine_get_position(5) - 1);
                if (steps_moved >= steps_target) {
                    advance_to_next_stage();
                }
            }
            break;

        default:
            advance_to_next_stage();
            break;
    }
}

void homing_poll(void) {
    if (!homing_is_active()) return;

    switch (current_stage) {
        case HOMING_STAGE_J2:
            process_joint_homing(1); // J2 is index 1
            break;
        case HOMING_STAGE_J3:
            process_joint_homing(2); // J3 is index 2
            break;
        case HOMING_STAGE_J1:
            process_joint_homing(0); // J1 is index 0
            break;
        case HOMING_STAGE_J4:
            process_joint_homing(3); // J4 is index 3
            break;
        case HOMING_STAGE_J6:
            process_joint_homing(5); // J6 is index 5
            break;
        case HOMING_STAGE_J5:
            process_joint_homing(4); // J5 is index 4
            break;

        case HOMING_APPLY_CAD_HOME:
            // All requested joints homed successfully!
            // Apply CAD reference step positions (from GO_CAD_HOME in printer.cfg)
            if (homed_mask == 0x3F) {
                step_engine_set_position(0, (int32_t)(  0.0f * STEPS_PER_DEG[0]));
                step_engine_set_position(1, (int32_t)(-90.0f * STEPS_PER_DEG[1]));
                step_engine_set_position(2, (int32_t)(180.0f * STEPS_PER_DEG[2]));
                step_engine_set_position(3, (int32_t)(  0.0f * STEPS_PER_DEG[3]));
                step_engine_set_position(4, (int32_t)(  0.0f * STEPS_PER_DEG[4]));
                step_engine_set_position(5, (int32_t)(180.0f * STEPS_PER_DEG[5]));
            }
            current_stage = HOMING_COMPLETE;
            break;

        default:
            break;
    }
}
