/**
 * @file ramps_pins.h
 * @brief Cycle-accurate direct register mappings for RAMPS 1.4 on ATmega2560.
 *        Extracted directly from printer.cfg for PAROL6 6-DOF Robot Arm.
 */

#ifndef RAMPS_PINS_H
#define RAMPS_PINS_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

#define NUM_JOINTS 6

// ============================================================================
// DIRECT PORT PIN MASKS
// ============================================================================

// --- Axis 1: Joint 1 (Base Yaw) ---
#define J1_STEP_PORT   PORTF
#define J1_STEP_DDR    DDRF
#define J1_STEP_BIT    0

#define J1_DIR_PORT    PORTF
#define J1_DIR_DDR     DDRF
#define J1_DIR_BIT     1

#define J1_EN_PORT     PORTD
#define J1_EN_DDR      DDRD
#define J1_EN_BIT      7

#define J1_STOP_PINR   PINE
#define J1_STOP_PORT   PORTE
#define J1_STOP_DDR    DDRE
#define J1_STOP_BIT    5

// --- Axis 2: Joint 2 (Shoulder Pitch) ---
#define J2_STEP_PORT   PORTF
#define J2_STEP_DDR    DDRF
#define J2_STEP_BIT    6

#define J2_DIR_PORT    PORTF
#define J2_DIR_DDR     DDRF
#define J2_DIR_BIT     7

#define J2_EN_PORT     PORTF
#define J2_EN_DDR      DDRF
#define J2_EN_BIT      2

#define J2_STOP_PINR   PINE
#define J2_STOP_PORT   PORTE
#define J2_STOP_DDR    DDRE
#define J2_STOP_BIT    4

// --- Axis 3: Joint 3 (Elbow Pitch) ---
#define J3_STEP_PORT   PORTL
#define J3_STEP_DDR    DDRL
#define J3_STEP_BIT    3

#define J3_DIR_PORT    PORTL
#define J3_DIR_DDR     DDRL
#define J3_DIR_BIT     1

#define J3_EN_PORT     PORTK
#define J3_EN_DDR      DDRK
#define J3_EN_BIT      0

#define J3_STOP_PINR   PINJ
#define J3_STOP_PORT   PORTJ
#define J3_STOP_DDR    DDRJ
#define J3_STOP_BIT    1

// --- Axis 4: Joint 4 (Forearm Roll) ---
#define J4_STEP_PORT   PORTA
#define J4_STEP_DDR    DDRA
#define J4_STEP_BIT    4

#define J4_DIR_PORT    PORTA
#define J4_DIR_DDR     DDRA
#define J4_DIR_BIT     6

#define J4_EN_PORT     PORTA
#define J4_EN_DDR      DDRA
#define J4_EN_BIT      2

#define J4_STOP_PINR   PINJ
#define J4_STOP_PORT   PORTJ
#define J4_STOP_DDR    DDRJ
#define J4_STOP_BIT    0

// --- Axis 5: Joint 5 (Wrist Pitch) ---
#define J5_STEP_PORT   PORTC
#define J5_STEP_DDR    DDRC
#define J5_STEP_BIT    1

#define J5_DIR_PORT    PORTC
#define J5_DIR_DDR     DDRC
#define J5_DIR_BIT     3

#define J5_EN_PORT     PORTC
#define J5_EN_DDR      DDRC
#define J5_EN_BIT      7

#define J5_STOP_PINR   PIND
#define J5_STOP_PORT   PORTD
#define J5_STOP_DDR    DDRD
#define J5_STOP_BIT    3

// --- Axis 6: Joint 6 (Wrist Roll) ---
#define J6_STEP_PORT   PORTG
#define J6_STEP_DDR    DDRG
#define J6_STEP_BIT    1

#define J6_DIR_PORT    PORTG
#define J6_DIR_DDR     DDRG
#define J6_DIR_BIT     0

#define J6_EN_PORT     PORTK
#define J6_EN_DDR      DDRK
#define J6_EN_BIT      1

#define J6_STOP_PINR   PIND
#define J6_STOP_PORT   PORTD
#define J6_STOP_DDR    DDRD
#define J6_STOP_BIT    2

// --- Gripper RC Servo (Pin D11 = PB5, Pin D4 = PG5) ---
#define SERVO_D11_PORT PORTB
#define SERVO_D11_DDR  DDRB
#define SERVO_D11_BIT  5

#define SERVO_D4_PORT  PORTG
#define SERVO_D4_DDR   DDRG
#define SERVO_D4_BIT   5

// Steps per degree:
static const float STEPS_PER_DEG[NUM_JOINTS] = {
    88.88888889f,
    444.44444444f,
    444.44444444f,
    39.75308642f,
    39.14373089f,
    8.88888889f
};

// Dynamic direction and endstop inversion masks (settable via serial 'D' and 'I')
extern volatile uint8_t dir_invert_mask;
extern volatile uint8_t endstop_invert_mask;

// ============================================================================
// FAST HARDWARE CONTROL INLINES
// ============================================================================

static inline void pins_init(void) {
    // 1. STEP pins as OUTPUT, initial state LOW
    J1_STEP_DDR |= _BV(J1_STEP_BIT); J1_STEP_PORT &= ~_BV(J1_STEP_BIT);
    J2_STEP_DDR |= _BV(J2_STEP_BIT); J2_STEP_PORT &= ~_BV(J2_STEP_BIT);
    J3_STEP_DDR |= _BV(J3_STEP_BIT); J3_STEP_PORT &= ~_BV(J3_STEP_BIT);
    J4_STEP_DDR |= _BV(J4_STEP_BIT); J4_STEP_PORT &= ~_BV(J4_STEP_BIT);
    J5_STEP_DDR |= _BV(J5_STEP_BIT); J5_STEP_PORT &= ~_BV(J5_STEP_BIT);
    J6_STEP_DDR |= _BV(J6_STEP_BIT); J6_STEP_PORT &= ~_BV(J6_STEP_BIT);

    // 2. DIR pins as OUTPUT, initial state LOW
    J1_DIR_DDR |= _BV(J1_DIR_BIT); J1_DIR_PORT &= ~_BV(J1_DIR_BIT);
    J2_DIR_DDR |= _BV(J2_DIR_BIT); J2_DIR_PORT &= ~_BV(J2_DIR_BIT);
    J3_DIR_DDR |= _BV(J3_DIR_BIT); J3_DIR_PORT &= ~_BV(J3_DIR_BIT);
    J4_DIR_DDR |= _BV(J4_DIR_BIT); J4_DIR_PORT &= ~_BV(J4_DIR_BIT);
    J5_DIR_DDR |= _BV(J5_DIR_BIT); J5_DIR_PORT &= ~_BV(J5_DIR_BIT);
    J6_DIR_DDR |= _BV(J6_DIR_BIT); J6_DIR_PORT &= ~_BV(J6_DIR_BIT);

    // 3. ENABLE pins as OUTPUT, initial state LOW (Active / Energized)
    J1_EN_DDR |= _BV(J1_EN_BIT); J1_EN_PORT &= ~_BV(J1_EN_BIT);
    J2_EN_DDR |= _BV(J2_EN_BIT); J2_EN_PORT &= ~_BV(J2_EN_BIT);
    J3_EN_DDR |= _BV(J3_EN_BIT); J3_EN_PORT &= ~_BV(J3_EN_BIT);
    J4_EN_DDR |= _BV(J4_EN_BIT); J4_EN_PORT &= ~_BV(J4_EN_BIT);
    J5_EN_DDR |= _BV(J5_EN_BIT); J5_EN_PORT &= ~_BV(J5_EN_BIT);
    J6_EN_DDR |= _BV(J6_EN_BIT); J6_EN_PORT &= ~_BV(J6_EN_BIT);

    // 4. ENDSTOP pins as INPUT with internal PULLUP enabled
    J1_STOP_DDR &= ~_BV(J1_STOP_BIT); J1_STOP_PORT |= _BV(J1_STOP_BIT);
    J2_STOP_DDR &= ~_BV(J2_STOP_BIT); J2_STOP_PORT |= _BV(J2_STOP_BIT);
    J3_STOP_DDR &= ~_BV(J3_STOP_BIT); J3_STOP_PORT |= _BV(J3_STOP_BIT);
    J4_STOP_DDR &= ~_BV(J4_STOP_BIT); J4_STOP_PORT |= _BV(J4_STOP_BIT);
    J5_STOP_DDR &= ~_BV(J5_STOP_BIT); J5_STOP_PORT |= _BV(J5_STOP_BIT);
    J6_STOP_DDR &= ~_BV(J6_STOP_BIT); J6_STOP_PORT |= _BV(J6_STOP_BIT);

    // 5. Gripper Servo pins as OUTPUT, initial state LOW
    SERVO_D11_DDR |= _BV(SERVO_D11_BIT); SERVO_D11_PORT &= ~_BV(SERVO_D11_BIT);
    SERVO_D4_DDR  |= _BV(SERVO_D4_BIT);  SERVO_D4_PORT  &= ~_BV(SERVO_D4_BIT);
}

// Emergency stop: Immediately de-energize all stepper coils (Active LOW drivers -> set HIGH)
static inline void emergency_stop_coils(void) {
    J1_EN_PORT |= _BV(J1_EN_BIT);
    J2_EN_PORT |= _BV(J2_EN_BIT);
    J3_EN_PORT |= _BV(J3_EN_BIT);
    J4_EN_PORT |= _BV(J4_EN_BIT);
    J5_EN_PORT |= _BV(J5_EN_BIT);
    J6_EN_PORT |= _BV(J6_EN_BIT);
}

// Enable all stepper drivers (Active LOW drivers -> set LOW)
static inline void enable_all_coils(void) {
    J1_EN_PORT &= ~_BV(J1_EN_BIT);
    J2_EN_PORT &= ~_BV(J2_EN_BIT);
    J3_EN_PORT &= ~_BV(J3_EN_BIT);
    J4_EN_PORT &= ~_BV(J4_EN_BIT);
    J5_EN_PORT &= ~_BV(J5_EN_BIT);
    J6_EN_PORT &= ~_BV(J6_EN_BIT);
}

// Set Direction for a single joint respecting dynamic dir_invert_mask
static inline void set_joint_dir(uint8_t joint, uint8_t dir) {
    uint8_t physical_dir = (dir ? 1 : 0) ^ ((dir_invert_mask >> joint) & 1);
    switch (joint) {
        case 0:
            if (physical_dir) J1_DIR_PORT |= _BV(J1_DIR_BIT);
            else J1_DIR_PORT &= ~_BV(J1_DIR_BIT);
            break;
        case 1:
            if (physical_dir) J2_DIR_PORT |= _BV(J2_DIR_BIT);
            else J2_DIR_PORT &= ~_BV(J2_DIR_BIT);
            break;
        case 2:
            if (physical_dir) J3_DIR_PORT |= _BV(J3_DIR_BIT);
            else J3_DIR_PORT &= ~_BV(J3_DIR_BIT);
            break;
        case 3:
            if (physical_dir) J4_DIR_PORT |= _BV(J4_DIR_BIT);
            else J4_DIR_PORT &= ~_BV(J4_DIR_BIT);
            break;
        case 4:
            if (physical_dir) J5_DIR_PORT |= _BV(J5_DIR_BIT);
            else J5_DIR_PORT &= ~_BV(J5_DIR_BIT);
            break;
        case 5:
            if (physical_dir) J6_DIR_PORT |= _BV(J6_DIR_BIT);
            else J6_DIR_PORT &= ~_BV(J6_DIR_BIT);
            break;
        default:
            break;
    }
}

// Single step pulse generator (>= 2.5 us width for DRV8825 t_WH(STEP))
static inline void pulse_single_step(uint8_t joint) {
    switch (joint) {
        case 0: J1_STEP_PORT |= _BV(J1_STEP_BIT); break;
        case 1: J2_STEP_PORT |= _BV(J2_STEP_BIT); break;
        case 2: J3_STEP_PORT |= _BV(J3_STEP_BIT); break;
        case 3: J4_STEP_PORT |= _BV(J4_STEP_BIT); break;
        case 4: J5_STEP_PORT |= _BV(J5_STEP_BIT); break;
        case 5: J6_STEP_PORT |= _BV(J6_STEP_BIT); break;
        default: return;
    }
    // Hold pulse HIGH for at least 2.5 us (42 cycles @ 16 MHz = 2.625 us)
    __builtin_avr_delay_cycles(42);
    switch (joint) {
        case 0: J1_STEP_PORT &= ~_BV(J1_STEP_BIT); break;
        case 1: J2_STEP_PORT &= ~_BV(J2_STEP_BIT); break;
        case 2: J3_STEP_PORT &= ~_BV(J3_STEP_BIT); break;
        case 3: J4_STEP_PORT &= ~_BV(J4_STEP_BIT); break;
        case 4: J5_STEP_PORT &= ~_BV(J5_STEP_BIT); break;
        case 5: J6_STEP_PORT &= ~_BV(J6_STEP_BIT); break;
        default: return;
    }
}

// Read Endstop state for a joint (respects endstop_invert_mask)
// NC switches on RAMPS 1.4:
// When unpressed: switch connects pin to GND -> (PIN & BIT) is 0 -> OPEN (false)
// When pressed: switch opens -> internal pullup pulls pin HIGH -> (PIN & BIT) is 1 -> TRIGGERED (true)
static inline bool read_endstop_triggered(uint8_t joint) {
    bool raw = false;
    switch (joint) {
        case 0: raw = (J1_STOP_PINR & _BV(J1_STOP_BIT)) != 0; break;
        case 1: raw = (J2_STOP_PINR & _BV(J2_STOP_BIT)) != 0; break;
        case 2: raw = (J3_STOP_PINR & _BV(J3_STOP_BIT)) != 0; break;
        case 3: raw = (J4_STOP_PINR & _BV(J4_STOP_BIT)) != 0; break;
        case 4: raw = (J5_STOP_PINR & _BV(J5_STOP_BIT)) != 0; break;
        case 5: raw = (J6_STOP_PINR & _BV(J6_STOP_BIT)) != 0; break;
        default: return false;
    }
    return raw ^ ((endstop_invert_mask >> joint) & 1);
}

// Read all 6 endstop states as a single bitmask (bit 0 = J1, ..., bit 5 = J6; 1 = triggered)
static inline uint8_t read_all_endstops_mask(void) {
    uint8_t mask = 0;
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        if (read_endstop_triggered(j)) {
            mask |= (1 << j);
        }
    }
    return mask;
}

#endif // RAMPS_PINS_H
