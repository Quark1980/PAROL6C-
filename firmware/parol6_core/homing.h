/**
 * @file homing.h
 * @brief Non-blocking hardware homing sequencer for PAROL6 6-DOF arm.
 *        Implements simultaneous [J1+J2+J3] homing and sequential wrist homing
 *        with double J6 pass to achieve exact CAD reference pose.
 */

#ifndef HOMING_H
#define HOMING_H

#include <stdint.h>
#include <stdbool.h>
#include "ramps_pins.h"

enum HomingState {
    HOMING_IDLE = 0,
    HOMING_STAGE_J123_PARALLEL, // [J1+J2+J3] simultaneously
    HOMING_STAGE_J4,            // [J4]
    HOMING_STAGE_J6_PASS1,      // [J6] first pass
    HOMING_STAGE_J5,            // [J5] (with J6 -10° pre-move and -5° post-move)
    HOMING_STAGE_J6_PASS2,      // [J6] second pass (to restore true zero after J5 moves)
    HOMING_APPLY_CAD_HOME,      // Apply CAD reference zero offsets
    HOMING_STAGE_SINGLE_JOINT,  // Fallback for standalone single-joint homing
    HOMING_COMPLETE,
    HOMING_ERROR
};

enum JointSubState {
    SUBSTATE_INIT = 0,
    SUBSTATE_PRE_MOVE,
    SUBSTATE_SEEK_ENDSTOP,
    SUBSTATE_LATCH_RELEASE,
    SUBSTATE_LATCH_PRECISION,
    SUBSTATE_RETREAT,
    SUBSTATE_POST_MOVE,
    SUBSTATE_DONE
};

void homing_init(void);
void homing_start(uint8_t mask);
void homing_abort(void);
bool homing_is_active(void);
HomingState homing_get_state(void);
uint8_t homing_get_homed_mask(void);
void homing_poll(void);

#endif // HOMING_H
