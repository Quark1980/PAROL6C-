/**
 * @file parol6_core.ino
 * @brief High-frequency, jitter-free scheduled pulse generator & AVR firmware
 *        for PAROL6 6-DOF Robot Arm running on ATmega2560 + RAMPS 1.4.
 * 
 * Hardware-Clock Event-Driven Stepping Engine (Klipper architecture):
 * - Timer 1: 16 MHz master clock queue with dynamic OCR1A compare match.
 * - Timer 3: 50 Hz jitter-free hardware pulse generator for Gripper RC Servo (D11 / D4).
 * - Direct register manipulation (PORTx, DDRx, PINx) across all 6 axes.
 * - Direct manual pulse generator for rapid motor testing & diagnostics ('M').
 * - Non-blocking homing sequencer matching printer.cfg exactly.
 * - Binary serial protocol over USB-UART at 250,000 baud.
 */

#include "ramps_pins.h"
#include "step_engine.h"
#include "homing.h"
#include <Arduino.h>

#define SERIAL_BAUD 250000

// Status flags
#define STATUS_FLAG_ESTOP   (1 << 0)
#define STATUS_FLAG_HOMING  (1 << 1)
#define STATUS_FLAG_BUSY    (1 << 2)
#define STATUS_FLAG_HOMED   (1 << 3)

volatile uint8_t dir_invert_mask = 0x03;     // Invert J1 (bit 0) and J2 (bit 1) per printer.cfg (!PF1, !PF7)
volatile uint8_t endstop_invert_mask = 0x00; // Default: unpressed=OPEN (0), pressed=TRIGGERED (1)
static volatile bool estop_active = false;
static volatile uint16_t servo_pulse_ticks = 3000; // Default 1500 us (90 deg) = 3000 ticks at 2 MHz
static volatile bool servo_enabled = true;

// ============================================================================
// TIMER 3: 50 HZ JITTER-FREE HARDWARE RC SERVO GENERATOR (Pin D11 & D4)
// ============================================================================
static void servo_init(void) {
    TCCR3A = 0;
    TCCR3B = _BV(CS31); // Prescaler 8 -> 2 MHz
    TCCR3C = 0;
    TCNT3 = 0;
    OCR3A = 40000;      // 20 ms period (50 Hz)
    OCR3B = servo_pulse_ticks;

    TIFR3 = _BV(TOV3) | _BV(OCF3A) | _BV(OCF3B);
    TIMSK3 = _BV(TOIE3) | _BV(OCIE3B);
}

ISR(TIMER3_OVF_vect) {
    if (servo_enabled) {
        SERVO_D11_PORT |= _BV(SERVO_D11_BIT);
        SERVO_D4_PORT  |= _BV(SERVO_D4_BIT);
    }
}

ISR(TIMER3_COMPB_vect) {
    SERVO_D11_PORT &= ~_BV(SERVO_D11_BIT);
    SERVO_D4_PORT  &= ~_BV(SERVO_D4_BIT);
}

static inline void set_gripper_pulse_us(uint16_t pulse_us) {
    if (pulse_us < 500)  pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    servo_pulse_ticks = pulse_us * 2;
    OCR3B = servo_pulse_ticks;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    cli();
    pins_init();
    step_engine_init();
    homing_init();
    servo_init();
    sei();

    Serial.begin(SERIAL_BAUD);

    // Initial state: Motor coils energized and ready for motion / homing
    enable_all_coils();
    estop_active = false;
}

// ============================================================================
// PROTOCOL DISPATCHER
// ============================================================================

static uint8_t rx_buffer[32];
static uint8_t rx_idx = 0;
static uint32_t last_rx_time_ms = 0;

static inline void handle_estop(void) {
    estop_active = true;
    emergency_stop_coils();
    step_engine_clear();
    homing_abort();
    Serial.write('E');
    Serial.write(0x01);
    Serial.write('\n');
}

static inline void handle_reset(void) {
    estop_active = false;
    step_engine_clear();
    homing_abort();
    enable_all_coils();
    Serial.write('R');
    Serial.write(0x01);
    Serial.write('\n');
}

static inline void handle_ping(void) {
    uint32_t clock_now = get_master_clock();
    Serial.write('P');
    Serial.write((const uint8_t*)&clock_now, 4);
    Serial.write('\n');
}

static inline void handle_query_status(void) {
    uint32_t clock_now = get_master_clock();
    uint8_t flags = 0;
    if (estop_active) flags |= STATUS_FLAG_ESTOP;
    if (homing_is_active()) flags |= STATUS_FLAG_HOMING;
    if (step_engine_is_busy()) flags |= STATUS_FLAG_BUSY;
    if (homing_get_homed_mask() == 0x3F) flags |= STATUS_FLAG_HOMED;

    int32_t positions[NUM_JOINTS];
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        positions[j] = step_engine_get_position(j);
    }
    uint8_t free_slots[NUM_JOINTS];
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        free_slots[j] = step_engine_free_slots(j);
    }
    uint8_t endstops = read_all_endstops_mask();

    Serial.write('!');
    Serial.write(flags);
    Serial.write((const uint8_t*)positions, sizeof(positions)); // 24 bytes
    Serial.write((const uint8_t*)free_slots, sizeof(free_slots)); // 6 bytes
    Serial.write(endstops);
    Serial.write((const uint8_t*)&clock_now, 4);
    Serial.write('\n');
}

// Fast Binary Step Chunk ('Q' packet, 12-byte payload, returns 0x06 ACK or 0x15 NAK)
// Payload: [axis: u8][first_clock: u32_le][count: u16_le][interval: u16_le][add: i16_le][dir: u8]
static inline void handle_q_step_chunk(const uint8_t* payload) {
    uint8_t joint = payload[0];
    if (joint >= NUM_JOINTS || estop_active || (homing_get_homed_mask() != 0x3F)) {
        Serial.write((uint8_t)0x15); // NAK (rejected: invalid, estop, or unhomed)
        return;
    }

    StepChunk chunk;
    memcpy(&chunk.first_step_clock, &payload[1], 4);
    memcpy(&chunk.step_count,       &payload[5], 2);
    memcpy(&chunk.interval,         &payload[7], 2);
    memcpy(&chunk.add,              &payload[9], 2);
    chunk.dir = payload[11] ? 1 : 0;

    bool ok = step_engine_push_chunk(joint, &chunk);
    if (ok) {
        Serial.write((uint8_t)0x06); // ACK
    } else {
        Serial.write((uint8_t)0x15); // NAK (Buffer full)
    }
}

// ============================================================================
// HARDWARE WATCHDOG & TIMING
// ============================================================================
static uint32_t last_comm_time_ms = 0;

// 11-byte Compact Binary Step Chunk (Header: 0x55 sync byte)
// Layout: [axis_dir: u8][start_clock: u32_le][step_count: u16_le][interval: u16_le][add_scaled: i8]
// axis_dir: Bits 0-2: Axis (0-5), Bit 3: Direction (0/1)
static inline void handle_compact_step_chunk(const uint8_t* payload) {
    uint8_t axis_dir = payload[0];
    uint8_t joint = axis_dir & 0x07;
    uint8_t dir = (axis_dir >> 3) & 0x01;

    if (joint >= NUM_JOINTS) {
        Serial.write('N');
        Serial.write(joint);
        Serial.write(0x03); // Invalid joint
        Serial.write('\n');
        return;
    }

    if (estop_active) {
        Serial.write('N');
        Serial.write(joint);
        Serial.write(0x04); // Rejected: ESTOP
        Serial.write('\n');
        return;
    }

    if (homing_get_homed_mask() != 0x3F) {
        Serial.write('N');
        Serial.write(joint);
        Serial.write(0x05); // Rejected: Must home first
        Serial.write('\n');
        return;
    }

    StepChunk chunk;
    memcpy(&chunk.first_step_clock, &payload[1], 4);
    memcpy(&chunk.step_count,       &payload[5], 2);
    memcpy(&chunk.interval,         &payload[7], 2);
    int8_t add_scaled = (int8_t)payload[9];
    chunk.add = (int16_t)add_scaled;
    chunk.dir = dir;

    bool ok = step_engine_push_chunk(joint, &chunk);
    if (ok) {
        uint8_t free_slots = step_engine_free_slots(joint);
        Serial.write('K');
        Serial.write(joint);
        Serial.write(free_slots);
        Serial.write('\n');
    } else {
        Serial.write('N');
        Serial.write(joint);
        Serial.write(0x01); // Buffer full
        Serial.write('\n');
    }
}

// Full 14-byte Step Chunk ('S' packet)
static inline void handle_step_chunk(const uint8_t* payload) {
    uint8_t joint = payload[0];
    if (joint >= NUM_JOINTS) {
        Serial.write('N');
        Serial.write(joint);
        Serial.write(0x03); // Invalid joint
        Serial.write('\n');
        return;
    }

    uint8_t crc = 'S';
    for (uint8_t i = 0; i < 12; i++) {
        crc ^= payload[i];
    }
    if (crc != payload[12]) {
        Serial.write('N');
        Serial.write(joint);
        Serial.write(0x02); // Checksum error
        Serial.write('\n');
        return;
    }

    if (estop_active) {
        Serial.write('N');
        Serial.write(joint);
        Serial.write(0x04); // Rejected due to ESTOP
        Serial.write('\n');
        return;
    }

    // Safety Interlock: The MCU must not accept joint step chunks unless the robot has been homed
    if (homing_get_homed_mask() != 0x3F) {
        Serial.write('N');
        Serial.write(joint);
        Serial.write(0x05); // Rejected: Robot must be homed first
        Serial.write('\n');
        return;
    }

    StepChunk chunk;
    memcpy(&chunk.first_step_clock, &payload[1], 4);
    memcpy(&chunk.step_count,       &payload[5], 2);
    memcpy(&chunk.interval,         &payload[7], 2);
    memcpy(&chunk.add,              &payload[9], 2);
    chunk.dir = payload[11] ? 1 : 0;

    bool ok = step_engine_push_chunk(joint, &chunk);
    if (ok) {
        uint8_t free_slots = step_engine_free_slots(joint);
        Serial.write('K');
        Serial.write(joint);
        Serial.write(free_slots);
        Serial.write('\n');
    } else {
        Serial.write('N');
        Serial.write(joint);
        Serial.write(0x01); // Buffer full
        Serial.write('\n');
    }
}

// Direct Manual Step for Instant Diagnostics & Jog ('M' packet)
// Layout: payload[0] = joint (0..5), payload[1] = dir (0/1), payload[2..3] = steps (uint16_le), payload[4..5] = delay_us (uint16_le)
static inline void handle_manual_step(const uint8_t* payload) {
    uint8_t joint = payload[0];
    uint8_t dir = payload[1] ? 1 : 0;
    uint16_t steps;
    uint16_t delay_us;
    memcpy(&steps, &payload[2], 2);
    memcpy(&delay_us, &payload[4], 2);

    if (joint >= NUM_JOINTS) return;
    if (delay_us < 20) delay_us = 20;

    set_joint_dir(joint, dir);
    enable_all_coils();
    estop_active = false;

    for (uint16_t s = 0; s < steps; s++) {
        pulse_single_step(joint);
        delayMicroseconds(delay_us);
    }

    int32_t current_pos = step_engine_get_position(joint);
    step_engine_set_position(joint, current_pos + (dir ? steps : -steps));

    Serial.write('M');
    Serial.write(joint);
    Serial.write(0x01);
    Serial.write('\n');
}

static inline void handle_set_invert(uint8_t mask) {
    endstop_invert_mask = mask;
    Serial.write('I');
    Serial.write(mask);
    Serial.write('\n');
}

static inline void handle_set_dir_invert(uint8_t mask) {
    dir_invert_mask = mask;
    Serial.write('D');
    Serial.write(mask);
    Serial.write('\n');
}

static inline void handle_gripper(const uint8_t* payload) {
    uint16_t pulse_us;
    memcpy(&pulse_us, payload, 2);
    set_gripper_pulse_us(pulse_us);
    Serial.write('G');
    Serial.write(0x01);
    Serial.write('\n');
}

static inline void handle_homing_cmd(uint8_t mask) {
    if (estop_active) {
        Serial.write('H');
        Serial.write(0xFE); // Cannot home during E-Stop
        Serial.write('\n');
        return;
    }
    homing_start(mask);
    Serial.write('H');
    Serial.write(0x01);
    Serial.write('\n');
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // 1. Service non-blocking homing state machine
    homing_poll();

    // 2. Tier 2 Hardware Watchdog: de-energize coils if serial communication ceases for >250 ms during active motion
    if (!estop_active && !homing_is_active()) {
        if (last_comm_time_ms > 0 && (millis() - last_comm_time_ms > 250)) {
            if (step_engine_is_busy()) {
                handle_estop();
            }
        }
    }

    // Reset buffer index if framed packet reception stalled (> 20 ms)
    if (rx_idx > 0 && (millis() - last_rx_time_ms > 20)) {
        rx_idx = 0;
    }

    // 3. Process incoming serial commands with robust framing & sliding-window resync
    while (Serial.available() > 0) {
        int b = Serial.read();
        if (b < 0) break;
        last_rx_time_ms = millis();
        last_comm_time_ms = last_rx_time_ms;

        // Store into receive buffer
        if (rx_idx < sizeof(rx_buffer)) {
            rx_buffer[rx_idx++] = (uint8_t)b;
        } else {
            // Buffer overflow guard: shift left by 1 byte to attempt resync
            memmove((void*)&rx_buffer[0], (void*)&rx_buffer[1], sizeof(rx_buffer) - 1);
            rx_buffer[sizeof(rx_buffer) - 1] = (uint8_t)b;
        }

        // Process recognized packet framing
        bool packet_processed = false;

        // 1. Delimited single-byte ASCII maintenance commands (Command + '\n')
        if (rx_buffer[0] == 'E') {
            if (rx_idx >= 2 && rx_buffer[1] == '\n') {
                handle_estop();
                rx_idx = 0;
                packet_processed = true;
            } else if (rx_idx >= 2) {
                // Not followed by '\n': shift left to resync
                memmove((void*)&rx_buffer[0], (void*)&rx_buffer[1], --rx_idx);
            }
        }
        else if (rx_buffer[0] == 'R') {
            if (rx_idx >= 2 && rx_buffer[1] == '\n') {
                handle_reset();
                rx_idx = 0;
                packet_processed = true;
            } else if (rx_idx >= 2) {
                memmove((void*)&rx_buffer[0], (void*)&rx_buffer[1], --rx_idx);
            }
        }
        else if (rx_buffer[0] == 'P') {
            if (rx_idx >= 2 && rx_buffer[1] == '\n') {
                handle_ping();
                rx_idx = 0;
                packet_processed = true;
            } else if (rx_idx >= 2) {
                memmove((void*)&rx_buffer[0], (void*)&rx_buffer[1], --rx_idx);
            }
        }
        else if (rx_buffer[0] == '?') {
            if (rx_idx >= 2 && rx_buffer[1] == '\n') {
                handle_query_status();
                rx_idx = 0;
                packet_processed = true;
            } else if (rx_idx >= 2) {
                memmove((void*)&rx_buffer[0], (void*)&rx_buffer[1], --rx_idx);
            }
        }

        // 2. 0x55 or 'C': 12-byte CRC-checked CompactStepChunk (or 11-byte legacy)
        else if (rx_buffer[0] == 0x55 || rx_buffer[0] == 'C') {
            if (rx_idx == 12) {
                // Verify XOR CRC8 of bytes 0..10 against byte 11
                uint8_t crc = 0;
                for (uint8_t i = 0; i < 11; i++) crc ^= rx_buffer[i];
                if (crc == rx_buffer[11]) {
                    handle_compact_step_chunk(&rx_buffer[1]);
                    rx_idx = 0;
                    packet_processed = true;
                } else {
                    // Checksum error or frame misalignment: slide left by 1 byte to find next sync
                    uint8_t next_s = 0;
                    for (uint8_t i = 1; i < rx_idx; i++) {
                        if (rx_buffer[i] == 0x55 || rx_buffer[i] == 'C' || rx_buffer[i] == 'E' || rx_buffer[i] == 'R' || rx_buffer[i] == 'P' || rx_buffer[i] == '?' || rx_buffer[i] == 'H' || rx_buffer[i] == 'G' || rx_buffer[i] == 'M') {
                            next_s = i;
                            break;
                        }
                    }
                    if (next_s > 0) {
                        memmove((void*)&rx_buffer[0], (void*)&rx_buffer[next_s], rx_idx - next_s);
                        rx_idx -= next_s;
                    } else {
                        rx_idx = 0;
                    }
                }
            }
        }

        // 3. 'Q' packet: Header 'Q' + 12 bytes payload = 13 bytes total
        else if (rx_buffer[0] == 'Q') {
            if (rx_idx == 13) {
                handle_q_step_chunk(&rx_buffer[1]);
                rx_idx = 0;
                packet_processed = true;
            }
        }

        // 4. 'S' packet: Header 'S' + 13 bytes payload = 14 bytes total
        else if (rx_buffer[0] == 'S') {
            if (rx_idx == 14) {
                handle_step_chunk(&rx_buffer[1]);
                rx_idx = 0;
                packet_processed = true;
            }
        }

        // 5. 'M' packet: Header 'M' + 6 bytes payload + '\n' = 8 bytes total
        else if (rx_buffer[0] == 'M') {
            if (rx_idx == 8) {
                if (rx_buffer[7] == '\n') {
                    handle_manual_step(&rx_buffer[1]);
                }
                rx_idx = 0;
                packet_processed = true;
            }
        }

        // 6. 'I' packet: Header 'I' + 1 byte mask + '\n' = 3 bytes total
        else if (rx_buffer[0] == 'I') {
            if (rx_idx == 3) {
                if (rx_buffer[2] == '\n') {
                    handle_set_invert(rx_buffer[1]);
                }
                rx_idx = 0;
                packet_processed = true;
            }
        }

        // 7. 'D' packet: Header 'D' + 1 byte mask + '\n' = 3 bytes total
        else if (rx_buffer[0] == 'D') {
            if (rx_idx == 3) {
                if (rx_buffer[2] == '\n') {
                    handle_set_dir_invert(rx_buffer[1]);
                }
                rx_idx = 0;
                packet_processed = true;
            }
        }

        // 8. 'G' packet: Header 'G' + 2 bytes pulse_us + '\n' = 4 bytes total
        else if (rx_buffer[0] == 'G') {
            if (rx_idx == 4) {
                if (rx_buffer[3] == '\n') {
                    handle_gripper(&rx_buffer[1]);
                }
                rx_idx = 0;
                packet_processed = true;
            }
        }

        // 9. 'H' packet: Header 'H' + 1 byte mask + '\n' = 3 bytes total
        else if (rx_buffer[0] == 'H') {
            if (rx_idx == 3) {
                if (rx_buffer[2] == '\n') {
                    handle_homing_cmd(rx_buffer[1]);
                }
                rx_idx = 0;
                packet_processed = true;
            }
        }

        // 10. Unrecognized header: slide buffer by 1 byte until valid start byte is found
        else if (rx_idx > 0 && !packet_processed) {
            uint8_t next_s = 0;
            for (uint8_t i = 1; i < rx_idx; i++) {
                if (rx_buffer[i] == 0x55 || rx_buffer[i] == 'C' || rx_buffer[i] == 'E' || rx_buffer[i] == 'R' || rx_buffer[i] == 'P' || rx_buffer[i] == '?' || rx_buffer[i] == 'H' || rx_buffer[i] == 'G' || rx_buffer[i] == 'M') {
                    next_s = i;
                    break;
                }
            }
            if (next_s > 0) {
                memmove((void*)&rx_buffer[0], (void*)&rx_buffer[next_s], rx_idx - next_s);
                rx_idx -= next_s;
            } else {
                rx_idx = 0;
            }
        }
    }
}
