# AGENT OPERATIONAL DIRECTIVE \& ARCHITECTURAL IMMUTABILITY RULES

The rules below are strict design boundaries for this project. They supersede any standard code generator defaults, library assumptions, or conversational shortcuts.

\---

### RULE 1: STRICT TRILATERAL BOUNDARY ISOLATION

Never collapse, blend, or shift compute responsibilities across these boundaries\[cite: 1]:

* **Windows Control Suite (Laptop):** User interface, manual/stream jogging, trajectory generation, and 3D preview\[cite: 1].
* **Raspberry Pi 5 (Host Daemon):** Network routing, WebSocket handling, kinematics solving (LM-damped IK), trajectory velocity blending, persistent calibration (`offsets.json`), and local Touchscreen Kiosk UI\[cite: 1].
* **Arduino Mega 2560 + RAMPS 1.4 (Firmware):** Pure deterministic real-time pulse execution, hardware limit switch polling, homing state execution, and hardware E-Stop disable\[cite: 1].

\---

### RULE 2: ZERO-ASSUMPTION HARDWARE INHERITANCE

Never guess, invent, or hardcode pin mappings, step inversions, pulley ratios, or homing directions:

* **The Ground Truth:** The physical hardware is already operational and homing successfully using `C:\\Users\\mseij\\Desktop\\PAROLC\_Firmware`.
* **The Reference Base:** All physical transmission ratios, step-per-degree metrics, and link geometries must be extracted directly from `C:\\Users\\mseij\\Desktop\\PAROLC\_Firmware\\current printer.cfg` and its associated URDF in `C:\\Users\\mseij\\Desktop\\PAROL6-Klipper-Studio-v1.0.0-baseline`.
* If a setting works in `PAROLC\_Firmware`, retain it without refactoring the physical logic if possible.

\---

### RULE 3: communication CODE

* Communication between the Windows Suite and Raspberry Pi must be JSON/binary state packets (Target Joint Angles or Cartesian Targets) over WebSockets\[cite: 1].
the fastest method should be chosen, since latency must be as low as possible. 
* Communication between the Raspberry Pi and Arduino Mega must be low-overhead binary step-chunk frames (`'Q'`) over USB-UART at 250,000 baud\[cite: 1].

\---

### RULE 4: 8-BIT CORE \& EVENT-DRIVEN STEPPING DISCIPLINE

* **Architecture:** The ATmega2560 is an 8-bit RISC MCU; do not write code assuming 32-bit hardware ALU performance.
* **Master Clock Virtualization:** Maintain a 32-bit virtual hardware clock at 16 MHz ($62.5\\text{ ns}$ ticks) by pairing 16-bit Timer 1 with an overflow interrupt (`TIMER1\_OVF\_vect`).
* **Timer Execution:** Stepping must use dynamically scheduled `OCR1A` interrupts (`TIMER1\_COMPA\_vect`). Fixed-rate Bresenham polling loops (e.g., permanent 20 kHz ISR polling) are prohibited. When the robot is idle, the interrupt rate must be 0 Hz.
* **Direct Register Control:** Standard Arduino abstractions (`digitalWrite()`, `pinMode()`) inside step generation or limit polling loops are forbidden. Use direct register bit-manipulation (`PORTx`, `DDRx`, `PINx`).
* **Pulse Duration:** Direct register step pulses must maintain a physical HIGH hold duration $\\ge 2.5\\ \\mu\\text{s}$ to respect DRV8825 timing parameters.

\---

### RULE 5: HOMING, TRIMS, AND CAD ZERO DISCIPLINE

* **Switch Type:** Switches are wired Common (C) to Normally Open (NO) with pull-ups (untriggered = HIGH, contact = LOW)\[cite: 2].
* **Soft-Start Homing:** Homing moves must use acceleration ramps (soft-start) instead of sudden high-speed starts.
* **Persistent Offsets:** Base homing offsets live in configuration; fine-tuning trims ($\\pm^\\circ$) must be stored in `offsets.json` on the Raspberry Pi.
* **CAD Zero Convergence:** Upon receiving `HOMING\_COMPLETE` from the MCU, the Pi must automatically coordinate a blended motion to:
$$\\text{Target Position}\[i] = \\text{Base Offset}\[i] + \\text{Homing Trim}\[i]$$
* Trims must be adjustable at runtime without re-flashing the microcontroller.

\---

### RULE 6: EMERGENCY STOP IMMUTABILITY

* The system uses a software-mediated E-Stop pipeline triggered via Touchscreen, Desktop, or Network\[cite: 1].
* An `ESTOP` command must immediately trigger:

  1. Instant de-energizing of stepper coils via hardware Enable registers (`PORTx` writes).
  2. Total purge/clearing of all queue buffers on both the Pi and Arduino Mega.
  3. Immediate state lockout until an explicit `RESET` command is issued.
* Never route E-stop through a blocking trajectory queue; it must be executed as an out-of-band, top-priority interrupt command\[cite: 1].

\---

### RULE 7: CODE COMPLETENESS GUARANTEE

* The agent must never emit placeholders, truncated logic blocks (`// ... rest of code`), mock kinematics, or pseudo-code.
* Every file generated (`parol6\_mcu.ino`, `robot\_engine.py`, `touchscreen.html`, deploy scripts) must be syntactically valid, production-ready, and directly executable on the target physical environment.

