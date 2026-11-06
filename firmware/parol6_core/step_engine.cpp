/**
 * @file step_engine.cpp
 * @brief Implementation of the absolute hardware-clock step execution engine.
 */

#include "step_engine.h"
#include <avr/interrupt.h>
#include <util/atomic.h>

// Master 16 MHz Timer 1 overflow high-word counter
static volatile uint16_t timer1_overflow_count = 0;

// Joint state ring buffers and active motion profiles
JointQueue joints[NUM_JOINTS];

// ============================================================================
// MASTER HARDWARE CLOCK ACCESS (16 MHz, 62.5 ns per tick)
// ============================================================================

uint32_t get_master_clock(void) {
    uint16_t h;
    uint16_t l;
    uint8_t sreg = SREG;
    cli();
    h = timer1_overflow_count;
    l = TCNT1;
    // Glitch guard: check if timer overflow flag is pending while low-word wrapped
    if ((TIFR1 & _BV(TOV1)) && (l < 32768)) {
        h++;
    }
    SREG = sreg;
    return ((uint32_t)h << 16) | l;
}

// ============================================================================
// DYNAMIC COMPARE MATCH RESCHEDULER
// ============================================================================

void step_engine_reschedule(void) {
    uint8_t sreg = SREG;
    cli();

    uint32_t now = get_master_clock();

    // 1. Promote pending chunks for any inactive joints
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        if (!joints[j].active && joints[j].count > 0) {
            StepChunk* c = &joints[j].queue[joints[j].tail];
            joints[j].active = true;
            // Overdue pace clamp: only intervene if the chunk arrived late by more than an entire step interval
            if ((int32_t)(now - c->first_step_clock) > (int32_t)c->interval) {
                joints[j].next_step_clock = now + c->interval;
            } else {
                joints[j].next_step_clock = c->first_step_clock;
            }
            joints[j].steps_left = c->step_count;
            joints[j].interval = c->interval;
            joints[j].add = c->add;
            joints[j].dir = c->dir;
            set_joint_dir(j, c->dir);
            joints[j].tail = (joints[j].tail + 1) % STEP_QUEUE_SIZE;
            joints[j].count--;
        }
    }

    // 2. Find the earliest next_step_clock among all active joints
    bool any_active = false;
    uint32_t earliest_clock = 0;
    int32_t min_diff = 0x7FFFFFFF;

    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        if (joints[j].active) {
            any_active = true;
            int32_t d = (int32_t)(joints[j].next_step_clock - now);
            if (d < min_diff) {
                min_diff = d;
                earliest_clock = joints[j].next_step_clock;
            }
        }
    }

    if (!any_active) {
        OCR1A = 0xFFFF;
        SREG = sreg;
        return;
    }

    // 3. Set Compare Match OCR1A relative to Timer 1 counter
    // Minimum 160 ticks = 10 us (100 kHz safety ceiling) to prevent ISR choking foreground serial loop
    uint16_t cur_tcnt = TCNT1;
    uint16_t target = (uint16_t)earliest_clock;
    int32_t ticks_ahead = (int32_t)(earliest_clock - now) - (int32_t)((uint16_t)(cur_tcnt - (uint16_t)now));

    if (ticks_ahead <= 160) {
        OCR1A = cur_tcnt + 160;
    } else if (ticks_ahead < 50000) {
        OCR1A = target;
        // CRITICAL RACE GUARD for 8-bit MCU: If TCNT1 advanced past target while setting OCR1A,
        // force immediate compare match pace to prevent full 65,536 tick wrap (4.1 ms stall)
        uint16_t after = TCNT1;
        if ((uint16_t)(target - after) > 50000 || (uint16_t)(target - after) <= 16) {
            OCR1A = after + 160;
        }
    } else {
        OCR1A = cur_tcnt + 45000;
    }

    SREG = sreg;
}

// ============================================================================
// TIMER 1 INTERRUPT SERVICE ROUTINES
// ============================================================================

// Overflow vector: increments 32-bit software master clock expansion
ISR(TIMER1_OVF_vect) {
    timer1_overflow_count++;
    step_engine_reschedule();
}

// Compare Match A vector: fires at scheduled absolute step timestamps
ISR(TIMER1_COMPA_vect) {
    uint32_t now = get_master_clock();
    uint8_t step_mask = 0;

    // 1. Identify which joints must fire at this exact timestamp
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        if (joints[j].active && ((int32_t)(now - joints[j].next_step_clock) >= 0)) {
            step_mask |= (1 << j);
        }
    }

    // 2. Direct register STEP pulse HIGH for all eligible joints
    if (step_mask & (1 << 0)) J1_STEP_PORT |= _BV(J1_STEP_BIT);
    if (step_mask & (1 << 1)) J2_STEP_PORT |= _BV(J2_STEP_BIT);
    if (step_mask & (1 << 2)) J3_STEP_PORT |= _BV(J3_STEP_BIT);
    if (step_mask & (1 << 3)) J4_STEP_PORT |= _BV(J4_STEP_BIT);
    if (step_mask & (1 << 4)) J5_STEP_PORT |= _BV(J5_STEP_BIT);
    if (step_mask & (1 << 5)) J6_STEP_PORT |= _BV(J6_STEP_BIT);

    // 3. Cycle-accurate step pulse duration (>= 2.5 us for TI DRV8825 t_WH(STEP): 42 cycles @ 16 MHz = 2.625 us)
    __builtin_avr_delay_cycles(42);

    // 4. Direct register STEP pulse LOW
    if (step_mask & (1 << 0)) J1_STEP_PORT &= ~_BV(J1_STEP_BIT);
    if (step_mask & (1 << 1)) J2_STEP_PORT &= ~_BV(J2_STEP_BIT);
    if (step_mask & (1 << 2)) J3_STEP_PORT &= ~_BV(J3_STEP_BIT);
    if (step_mask & (1 << 3)) J4_STEP_PORT &= ~_BV(J4_STEP_BIT);
    if (step_mask & (1 << 4)) J5_STEP_PORT &= ~_BV(J5_STEP_BIT);
    if (step_mask & (1 << 5)) J6_STEP_PORT &= ~_BV(J6_STEP_BIT);

    // 5. Update joint positions and chunk step counters
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        if (step_mask & (1 << j)) {
            joints[j].step_position += (joints[j].dir ? 1 : -1);
            joints[j].steps_left--;

            if (joints[j].steps_left > 0) {
                // Phase-locked microstep step scheduling: preserve exact interval timing
                // Only clamp if severely overdue by more than an interval (e.g. queue starvation / comms stall)
                if ((int32_t)(now - joints[j].next_step_clock) > (int32_t)joints[j].interval) {
                    joints[j].next_step_clock = now + joints[j].interval;
                } else {
                    joints[j].next_step_clock += joints[j].interval;
                }
                int32_t next_iv = (int32_t)joints[j].interval + joints[j].add;
                if (next_iv < MIN_STEP_INTERVAL) {
                    next_iv = MIN_STEP_INTERVAL;
                }
                joints[j].interval = (uint32_t)next_iv;
            } else {
                // Chunk finished: pop next if queued
                // Seamless C1 boundary: first step fires at c->first_step_clock without an extra interval delay
                if (joints[j].count > 0) {
                    StepChunk* c = &joints[j].queue[joints[j].tail];
                    if ((int32_t)(now - c->first_step_clock) > (int32_t)c->interval) {
                        joints[j].next_step_clock = now + c->interval;
                    } else {
                        joints[j].next_step_clock = c->first_step_clock;
                    }
                    joints[j].steps_left = c->step_count;
                    joints[j].interval = c->interval;
                    joints[j].add = c->add;
                    joints[j].dir = c->dir;
                    set_joint_dir(j, c->dir);
                    joints[j].tail = (joints[j].tail + 1) % STEP_QUEUE_SIZE;
                    joints[j].count--;
                } else {
                    joints[j].active = false;
                }
            }
        }
    }

    // 6. Reschedule Compare Match for earliest upcoming event
    step_engine_reschedule();
}

// ============================================================================
// INITIALIZATION AND PUBLIC QUEUE INTERFACES
// ============================================================================

void step_engine_init(void) {
    // Initialize hardware pins
    pins_init();

    // Zero out joint queues and active states
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        joints[j].head = 0;
        joints[j].tail = 0;
        joints[j].count = 0;
        joints[j].active = false;
        joints[j].next_step_clock = 0;
        joints[j].steps_left = 0;
        joints[j].interval = 0;
        joints[j].add = 0;
        joints[j].dir = 0;
        joints[j].step_position = 0;
    }

    // Configure Timer 1: 16 MHz Normal Mode (WGM1 = 0, CS1 = 001)
    TCCR1A = 0;
    TCCR1B = _BV(CS10); // Prescaler 1
    TCCR1C = 0;
    TCNT1 = 0;
    timer1_overflow_count = 0;
    OCR1A = 0xFFFF;

    // Clear any pending interrupt flags and enable OVF + COMPA interrupts
    TIFR1 = _BV(TOV1) | _BV(OCF1A);
    TIMSK1 = _BV(TOIE1) | _BV(OCIE1A);
}

bool step_engine_push_chunk(uint8_t joint, const StepChunk* chunk) {
    if (joint >= NUM_JOINTS) return false;

    uint8_t sreg = SREG;
    cli();

    if (joints[joint].count >= STEP_QUEUE_SIZE) {
        SREG = sreg;
        return false; // Queue full
    }

    uint8_t h = joints[joint].head;
    joints[joint].queue[h] = *chunk;
    joints[joint].head = (h + 1) % STEP_QUEUE_SIZE;
    joints[joint].count++;

    SREG = sreg;

    // Reschedule compare match to consider this chunk
    step_engine_reschedule();
    return true;
}

uint8_t step_engine_free_slots(uint8_t joint) {
    if (joint >= NUM_JOINTS) return 0;
    uint8_t sreg = SREG;
    cli();
    uint8_t free_slots = STEP_QUEUE_SIZE - joints[joint].count;
    SREG = sreg;
    return free_slots;
}

void step_engine_clear(void) {
    uint8_t sreg = SREG;
    cli();
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        joints[j].head = 0;
        joints[j].tail = 0;
        joints[j].count = 0;
        joints[j].active = false;
    }
    OCR1A = 0xFFFF;
    SREG = sreg;
}

bool step_engine_is_busy(void) {
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        if (joints[j].active || joints[j].count > 0) return true;
    }
    return false;
}

int32_t step_engine_get_position(uint8_t joint) {
    if (joint >= NUM_JOINTS) return 0;
    int32_t pos;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pos = joints[joint].step_position;
    }
    return pos;
}

void step_engine_set_position(uint8_t joint, int32_t pos) {
    if (joint >= NUM_JOINTS) return;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        joints[joint].step_position = pos;
    }
}
