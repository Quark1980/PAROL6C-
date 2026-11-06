/**
 * @file step_engine.h
 * @brief Absolute hardware-clock event-driven step execution engine
 *        for PAROL6 on ATmega2560 (architecturally identical to Klipper).
 */

#ifndef STEP_ENGINE_H
#define STEP_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include "ramps_pins.h"

#define STEP_QUEUE_SIZE 16
#define MIN_STEP_INTERVAL 160 // 160 ticks = 10 microseconds (max 100 kHz per axis)

/**
 * @brief Compressed quadratic step chunk packet matching Klipper stepcompress.c
 */
struct StepChunk {
    uint32_t first_step_clock; // 16 MHz master clock timestamp of the first step
    uint16_t step_count;       // Total steps in this chunk
    uint16_t interval;         // Initial interval in 16 MHz ticks between step 0 and 1
    int16_t  add;              // Clock ticks added to interval per step (acceleration slope)
    uint8_t  dir;              // Direction (0 or 1)
};

/**
 * @brief Joint active state and chunk ring buffer
 */
struct JointQueue {
    StepChunk queue[STEP_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;

    // Active executing chunk
    volatile bool     active;
    volatile uint32_t next_step_clock;
    volatile uint16_t steps_left;
    volatile uint32_t interval;
    volatile int16_t  add;
    volatile uint8_t  dir;

    // Running absolute step count
    volatile int32_t  step_position;
};

// Global joints array
extern JointQueue joints[NUM_JOINTS];

// Public Engine Functions
void     step_engine_init(void);
uint32_t get_master_clock(void);
bool     step_engine_push_chunk(uint8_t joint, const StepChunk* chunk);
uint8_t  step_engine_free_slots(uint8_t joint);
void     step_engine_clear(void);
bool     step_engine_is_busy(void);
int32_t  step_engine_get_position(uint8_t joint);
void     step_engine_set_position(uint8_t joint, int32_t pos);
void     step_engine_reschedule(void);

#endif // STEP_ENGINE_H
