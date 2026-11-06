/**
 * @file homing.h
 * @brief Non-blocking hardware homing sequencer for PAROL6 6-DOF arm.
 *        Implements the exact endstop retreat and latch behavior from printer.cfg.
 */

#ifndef HOMING_H
#define HOMING_H

#include <stdint.h>
#include <stdbool.h>
#include "ramps_pins.h"

enum HomingState {
    HOMING_IDLE = 0,
    HOMING_STAGE_J2,
    HOMING_STAGE_J3,
    HOMING_STAGE_J1,
    HOMING_STAGE_J4,
    HOMING_STAGE_J6,
    HOMING_STAGE_J5,
    HOMING_APPLY_CAD_HOME,
    HOMING_COMPLETE,
    HOMING_ERROR
};

enum JointSubState {
    SUBSTATE_INIT = 0,
    SUBSTATE_PRE_MOVE,
    SUBSTATE_SEEK_ENDSTOP,
    SUBSTATE_LATCH_RELEASE,
    SUBSTATE_LATCH_PRECISION,
    SUBSTATE_LATCH_DEBOUNCE,
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
