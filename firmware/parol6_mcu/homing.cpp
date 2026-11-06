/**
 * @file homing.cpp
 * @brief Implementation of the non-blocking hardware homing sequencer with
 *        simultaneous [J1+J2+J3] stepping, precision latching, double J6 pass,
 *        and CAD zero reference alignment.
 */

#include "homing.h"
#include "step_engine.h"
#include <stdlib.h>
#include <math.h>

static HomingState current_stage = HOMING_IDLE;
static uint8_t active_mask = 0;
static uint8_t homed_mask = 0;
static uint8_t single_joint_idx = 0;

// Configurations per joint from current printer.cfg
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

struct JointHomeContext {
    JointSubState substate;
    uint32_t step_interval;
    uint32_t next_step_clock;
    int32_t  steps_moved;
    int32_t  steps_target;
    int32_t  decel_steps;
    uint8_t  debounce_count;
    float    current_speed;
    bool     done;
};

static JointHomeContext joint_ctx[NUM_JOINTS];

static void init_joint_context(uint8_t j, JointHomeContext* ctx, uint32_t now) {
    const JointHomeCfg* cfg = &HOME_CFGS[j];
    ctx->done = false;
    ctx->current_speed = cfg->start_speed;
    ctx->step_interval = (uint32_t)(16000000.0f / (ctx->current_speed * STEPS_PER_DEG[j]));
    ctx->next_step_clock = now + ctx->step_interval;
    ctx->steps_moved = 0;
    ctx->steps_target = (int32_t)(cfg->max_travel * STEPS_PER_DEG[j]);
    ctx->debounce_count = 0;
    ctx->decel_steps = 0;

    // Handle special pre-moves (J5 requires J6 move -10 deg)
    if (j == 4 && (active_mask == 0xFF || (active_mask & (1 << 5)))) {
        ctx->substate = SUBSTATE_PRE_MOVE;
        set_joint_dir(5, 0); // J6 dir negative
        ctx->steps_target = (int32_t)(10.0f * STEPS_PER_DEG[5]);
        ctx->step_interval = (uint32_t)(16000000.0f / (80.0f * STEPS_PER_DEG[5]));
        ctx->next_step_clock = now + ctx->step_interval;
    } else {
        set_joint_dir(j, cfg->seek_dir);
        ctx->substate = SUBSTATE_SEEK_ENDSTOP;
    }
}

static void step_joint_homing(uint8_t j, JointHomeContext* ctx, uint32_t now) {
    if (ctx->done) return;
    if ((int32_t)(now - ctx->next_step_clock) < 0) return;

    const JointHomeCfg* cfg = &HOME_CFGS[j];

    switch (ctx->substate) {
        case SUBSTATE_PRE_MOVE:
            ctx->next_step_clock = now + ctx->step_interval;
            pulse_single_step(5); // Pre-move J6
            ctx->steps_moved++;
            step_engine_set_position(5, step_engine_get_position(5) - 1);
            if (ctx->steps_moved >= ctx->steps_target) {
                // Pre-move finished, begin J5 seek with soft-start
                ctx->steps_moved = 0;
                ctx->steps_target = (int32_t)(cfg->max_travel * STEPS_PER_DEG[j]);
                ctx->current_speed = cfg->start_speed;
                ctx->step_interval = (uint32_t)(16000000.0f / (ctx->current_speed * STEPS_PER_DEG[j]));
                ctx->next_step_clock = now + ctx->step_interval;
                set_joint_dir(j, cfg->seek_dir);
                ctx->substate = SUBSTATE_SEEK_ENDSTOP;
            }
            break;

        case SUBSTATE_SEEK_ENDSTOP:
            ctx->next_step_clock = now + ctx->step_interval;
            if (read_endstop_triggered(j)) {
                // Switch hit during seek -> prepare for switch release backoff
                ctx->steps_moved = 0;
                ctx->steps_target = (int32_t)(3.0f * STEPS_PER_DEG[j]); // 3 degrees release backoff
                set_joint_dir(j, cfg->seek_dir ? 0 : 1);           // reverse direction
                ctx->step_interval = (uint32_t)(16000000.0f / (8.0f * STEPS_PER_DEG[j])); // gentle release speed
                ctx->substate = SUBSTATE_LATCH_RELEASE;
            } else {
                pulse_single_step(j);
                ctx->steps_moved++;
                step_engine_set_position(j, step_engine_get_position(j) + (cfg->seek_dir ? 1 : -1));

                // Soft-start acceleration ramp up to seek cruise speed
                if (ctx->current_speed < cfg->seek_speed) {
                    float dt = 1.0f / (ctx->current_speed * STEPS_PER_DEG[j]);
                    ctx->current_speed += cfg->accel * dt;
                    if (ctx->current_speed > cfg->seek_speed) {
                        ctx->current_speed = cfg->seek_speed;
                    }
                    ctx->step_interval = (uint32_t)(16000000.0f / (ctx->current_speed * STEPS_PER_DEG[j]));
                }

                if (ctx->steps_moved >= ctx->steps_target) {
                    // Limit reached without hitting switch -> homing fault
                    current_stage = HOMING_ERROR;
                }
            }
            break;

        case SUBSTATE_LATCH_RELEASE:
            ctx->next_step_clock = now + ctx->step_interval;
            pulse_single_step(j);
            ctx->steps_moved++;
            step_engine_set_position(j, step_engine_get_position(j) + (cfg->seek_dir ? -1 : 1));

            // Wait until switch is released plus min steps
            if (!read_endstop_triggered(j) && ctx->steps_moved >= ctx->steps_target) {
                // Switch released! Begin slow precision latch approach
                set_joint_dir(j, cfg->seek_dir);
                ctx->steps_moved = 0;
                ctx->steps_target = (int32_t)(6.0f * STEPS_PER_DEG[j]);
                ctx->current_speed = 3.0f; // Ultra-slow precision latch speed (3 deg/s)
                ctx->step_interval = (uint32_t)(16000000.0f / (ctx->current_speed * STEPS_PER_DEG[j]));
                ctx->debounce_count = 0;
                ctx->substate = SUBSTATE_LATCH_PRECISION;
            }
            break;

        case SUBSTATE_LATCH_PRECISION:
            ctx->next_step_clock = now + ctx->step_interval;
            if (read_endstop_triggered(j)) {
                ctx->debounce_count++;
                if (ctx->debounce_count >= 5) {
                    // Switch reliably precision-latched!
                    // Lock current physical position as 0
                    step_engine_set_position(j, 0);

                    // Configure retreat move with smooth trapezoidal acceleration/deceleration
                    float retreat_deg = cfg->retreat_pos;
                    uint8_t retreat_dir = (retreat_deg >= 0.0f) ? 1 : 0;
                    set_joint_dir(j, retreat_dir);

                    ctx->steps_target = (int32_t)(fabs(retreat_deg) * STEPS_PER_DEG[j]);
                    ctx->steps_moved = 0;

                    // Calculate decel distance: s_ramp = (v_max^2 - v_start^2) / (2 * a)
                    float s_ramp = (cfg->retreat_speed * cfg->retreat_speed - cfg->start_speed * cfg->start_speed) / (2.0f * cfg->accel);
                    ctx->decel_steps = (int32_t)(s_ramp * STEPS_PER_DEG[j]);
                    if (ctx->decel_steps > ctx->steps_target / 2) {
                        ctx->decel_steps = ctx->steps_target / 2;
                    }

                    ctx->current_speed = cfg->start_speed;
                    ctx->step_interval = (uint32_t)(16000000.0f / (ctx->current_speed * STEPS_PER_DEG[j]));
                    ctx->next_step_clock = now + ctx->step_interval;
                    ctx->substate = SUBSTATE_RETREAT;
                }
            } else {
                ctx->debounce_count = 0;
                pulse_single_step(j);
                ctx->steps_moved++;
                step_engine_set_position(j, step_engine_get_position(j) + (cfg->seek_dir ? 1 : -1));

                if (ctx->steps_moved >= ctx->steps_target) {
                    current_stage = HOMING_ERROR;
                }
            }
            break;

        case SUBSTATE_RETREAT:
            ctx->next_step_clock = now + ctx->step_interval;
            pulse_single_step(j);
            ctx->steps_moved++;
            step_engine_set_position(j, step_engine_get_position(j) + (cfg->retreat_pos >= 0 ? 1 : -1));

            {
                int32_t steps_remaining = ctx->steps_target - ctx->steps_moved;
                float dt = 1.0f / (ctx->current_speed * STEPS_PER_DEG[j]);

                // Trapezoidal acceleration & deceleration profile
                if (steps_remaining <= ctx->decel_steps) {
                    ctx->current_speed -= cfg->accel * dt;
                    if (ctx->current_speed < cfg->start_speed) {
                        ctx->current_speed = cfg->start_speed;
                    }
                } else if (ctx->current_speed < cfg->retreat_speed) {
                    ctx->current_speed += cfg->accel * dt;
                    if (ctx->current_speed > cfg->retreat_speed) {
                        ctx->current_speed = cfg->retreat_speed;
                    }
                }
                ctx->step_interval = (uint32_t)(16000000.0f / (ctx->current_speed * STEPS_PER_DEG[j]));
            }

            if (ctx->steps_moved >= ctx->steps_target) {
                if (j == 4 && (active_mask == 0xFF || (active_mask & (1 << 5)))) {
                    // J5 post-move: J6 MOVE=-5 deg
                    ctx->substate = SUBSTATE_POST_MOVE;
                    set_joint_dir(5, 0); // J6 dir negative
                    ctx->steps_target = (int32_t)(5.0f * STEPS_PER_DEG[5]);
                    ctx->steps_moved = 0;
                    ctx->step_interval = (uint32_t)(16000000.0f / (80.0f * STEPS_PER_DEG[5]));
                    ctx->next_step_clock = now + ctx->step_interval;
                } else {
                    ctx->substate = SUBSTATE_DONE;
                    ctx->done = true;
                }
            }
            break;

        case SUBSTATE_POST_MOVE:
            ctx->next_step_clock = now + ctx->step_interval;
            pulse_single_step(5);
            ctx->steps_moved++;
            step_engine_set_position(5, step_engine_get_position(5) - 1);
            if (ctx->steps_moved >= ctx->steps_target) {
                ctx->substate = SUBSTATE_DONE;
                ctx->done = true;
            }
            break;

        default:
            ctx->substate = SUBSTATE_DONE;
            ctx->done = true;
            break;
    }
}

void homing_init(void) {
    current_stage = HOMING_IDLE;
    active_mask = 0;
    homed_mask = 0;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        joint_ctx[i].done = false;
        joint_ctx[i].substate = SUBSTATE_INIT;
    }
}

void homing_start(uint8_t mask) {
    if (mask == 0) return;
    active_mask = mask;
    enable_all_coils();
    uint32_t now = get_master_clock();

    // Reset contexts
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        joint_ctx[i].done = false;
        joint_ctx[i].substate = SUBSTATE_INIT;
    }

    // Sequence per architecture: [J1+J2+J3] simultaneous -> [J4] -> [J6 pass 1] -> [J5] -> [J6 pass 2] -> [Apply CAD Home]
    if ((active_mask & 0x07) == 0x07) {
        init_joint_context(0, &joint_ctx[0], now);
        init_joint_context(1, &joint_ctx[1], now);
        init_joint_context(2, &joint_ctx[2], now);
        current_stage = HOMING_STAGE_J123_PARALLEL;
    } else if (active_mask & (1 << 3)) {
        init_joint_context(3, &joint_ctx[3], now);
        current_stage = HOMING_STAGE_J4;
    } else if (active_mask & (1 << 5)) {
        init_joint_context(5, &joint_ctx[5], now);
        current_stage = HOMING_STAGE_J6_PASS1;
    } else if (active_mask & (1 << 4)) {
        init_joint_context(4, &joint_ctx[4], now);
        current_stage = HOMING_STAGE_J5;
    } else {
        // Individual single joint fallback (e.g. only J1 or only J2 or only J3)
        for (uint8_t j = 0; j < NUM_JOINTS; j++) {
            if (active_mask & (1 << j)) {
                single_joint_idx = j;
                init_joint_context(j, &joint_ctx[j], now);
                current_stage = HOMING_STAGE_SINGLE_JOINT;
                return;
            }
        }
        current_stage = HOMING_IDLE;
    }
}

void homing_abort(void) {
    current_stage = HOMING_IDLE;
    active_mask = 0;
    homed_mask = 0; // Reset homed mask: position is lost on abort/estop
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        joint_ctx[i].done = true;
    }
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

void homing_poll(void) {
    if (!homing_is_active()) return;

    uint32_t now = get_master_clock();

    switch (current_stage) {
        case HOMING_STAGE_J123_PARALLEL:
            step_joint_homing(0, &joint_ctx[0], now);
            step_joint_homing(1, &joint_ctx[1], now);
            step_joint_homing(2, &joint_ctx[2], now);

            if (joint_ctx[0].done && joint_ctx[1].done && joint_ctx[2].done) {
                homed_mask |= 0x07; // J1, J2, J3 homed!
                if (active_mask & (1 << 3)) {
                    init_joint_context(3, &joint_ctx[3], now);
                    current_stage = HOMING_STAGE_J4;
                } else if (active_mask & (1 << 5)) {
                    init_joint_context(5, &joint_ctx[5], now);
                    current_stage = HOMING_STAGE_J6_PASS1;
                } else if (active_mask & (1 << 4)) {
                    init_joint_context(4, &joint_ctx[4], now);
                    current_stage = HOMING_STAGE_J5;
                } else {
                    current_stage = HOMING_APPLY_CAD_HOME;
                }
            }
            break;

        case HOMING_STAGE_J4:
            step_joint_homing(3, &joint_ctx[3], now);
            if (joint_ctx[3].done) {
                homed_mask |= (1 << 3);
                if (active_mask & (1 << 5)) {
                    init_joint_context(5, &joint_ctx[5], now);
                    current_stage = HOMING_STAGE_J6_PASS1;
                } else if (active_mask & (1 << 4)) {
                    init_joint_context(4, &joint_ctx[4], now);
                    current_stage = HOMING_STAGE_J5;
                } else {
                    current_stage = HOMING_APPLY_CAD_HOME;
                }
            }
            break;

        case HOMING_STAGE_J6_PASS1:
            step_joint_homing(5, &joint_ctx[5], now);
            if (joint_ctx[5].done) {
                homed_mask |= (1 << 5);
                if (active_mask & (1 << 4)) {
                    init_joint_context(4, &joint_ctx[4], now);
                    current_stage = HOMING_STAGE_J5;
                } else {
                    current_stage = HOMING_APPLY_CAD_HOME;
                }
            }
            break;

        case HOMING_STAGE_J5:
            step_joint_homing(4, &joint_ctx[4], now);
            if (joint_ctx[4].done) {
                homed_mask |= (1 << 4);
                // J5 moved J6 by -10° pre-move and -5° post-move!
                // Home J6 second pass to restore true zero before applying CAD home!
                if (active_mask & (1 << 5)) {
                    init_joint_context(5, &joint_ctx[5], now);
                    current_stage = HOMING_STAGE_J6_PASS2;
                } else {
                    current_stage = HOMING_APPLY_CAD_HOME;
                }
            }
            break;

        case HOMING_STAGE_J6_PASS2:
            step_joint_homing(5, &joint_ctx[5], now);
            if (joint_ctx[5].done) {
                homed_mask |= (1 << 5);
                current_stage = HOMING_APPLY_CAD_HOME;
            }
            break;

        case HOMING_STAGE_SINGLE_JOINT:
            step_joint_homing(single_joint_idx, &joint_ctx[single_joint_idx], now);
            if (joint_ctx[single_joint_idx].done) {
                homed_mask |= (1 << single_joint_idx);
                current_stage = HOMING_COMPLETE;
            }
            break;

        case HOMING_APPLY_CAD_HOME:
            // Apply CAD reference zero offsets per printer.cfg:
            // J1: 0°, J2: -90°, J3: 180°, J4: 0°, J5: 0°, J6: 180°
            if ((homed_mask & 0x3F) == 0x3F) {
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
