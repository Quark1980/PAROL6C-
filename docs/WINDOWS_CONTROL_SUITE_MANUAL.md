# PAROL6C Windows 3D Control Suite: Operator & Developer Manual

> **Reference Guide for the Native Windows C# / .NET 8 WPF 3D Digital Twin Control Suite**  
> Supports Simulate Only, Send On Release, and Live Follow Modes with Strict Flange World Orientation Lock

---

## 1. Overview & Architecture

The **PAROL6C Windows Control Suite** (`Windows_ControlSuite` / `PAROLC_ControlSuite_Test`) is a real-time 3D digital twin and trajectory planner for the custom 6-axis Klipper PAROL6 robotic arm.

```text
┌────────────────────────────────────────────────────────────────┐
│          Windows Control Suite (.NET 8 WPF / HelixToolkit)     │
│  - 3D URDF Digital Twin (Live Opaque + Staged Cyan Ghost)      │
│  - 3 Modes: SIMULATE ONLY | ON-RELEASE | LIVE FOLLOW (50 Hz)   │
│  - 2 IK Modes: KEEP TOOL STEADY (Locked R0) | FREE WRIST       │
│  - Analytical Spherical Wrist + DLS IK & 3.0° Delta Gate       │
└──────────────┬──────────────────────────────────┬──────────────┘
               │ TCP Port 8765 (JSON RPC)         │ UDP Port 8766 (39B Binary)
               ▼                                  ▼
┌────────────────────────────────────────────────────────────────┐
│            Raspberry Pi 5 Host Daemon (robot_engine.py)        │
│  - 100 Hz Circular Jitter Buffer & Synced 6D 1-Euro Filter     │
│  - Parol6Kinematics + validate_and_lock_branch() Safety Gate   │
│  - Non-Blocking PTY Bridge (/tmp/printer) -> 6-Axis Klipper    │
└────────────────────────────────────────────────────────────────┘
```

---

## 2. Three Interactive Operating Modes

Select the active mode from the **OPERATING MODE** selector in the left sidebar:

1. **`SIMULATE ONLY` (Purely Virtual)**:
   - Dragging the 3D TCP gizmo, clicking Cartesian jog buttons, or moving joint sliders updates the cyan ghost robot locally using the C# kinematics engine.
   - **Zero motion packets** are transmitted over TCP or UDP; the physical robot remains completely stationary.
   - When switching out of `SIMULATE ONLY` into `ON-RELEASE`, click **COMMIT STAGED POSE** to execute the simulated pose on the physical arm.

2. **`ON-RELEASE` (Staged Dispatch)**:
   - While dragging the 3D gizmo or a joint slider, the planned trajectory path line is rendered in the 3D viewport from the actual tooltip to the target tooltip.
   - Upon releasing the mouse button (`MouseUp`), the suite interpolates and executes a smooth velocity-blended trajectory to the staged target pose.

3. **`LIVE FOLLOW` (Real-Time 50 Hz Teleoperation)**:
   - Continuously streams 50 Hz setpoints as you drag the 3D gizmo or sliders, smoothed by a client-side sectional B-spline evaluator and a host-side synchronized 6-axis 1-Euro filter.
   - Renders a live 3D trajectory trail that automatically prunes and fades out once the physical robot reaches the final pose.

---

## 3. Two Inverse Kinematics Orientation Modes

Located in the **Cartesian** tab under **TOOL ORIENTATION MODE (CARTESIAN IK)**:

1. **`KEEP TOOL STEADY (Locked 3D Orientation)`**:
   - Locks the $3 \times 3$ flange world rotation matrix $R_0$ across **every intermediate interpolated step** during $\pm X, \pm Y, \pm Z$ jogs, `ON-RELEASE` moves, and `LIVE FOLLOW` moves:
     $$T_k = \begin{bmatrix} R_0 & \mathbf{p}_k \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix}$$
   - Synchronized 6-axis velocity scaling ensures all 6 joints scale proportionally when any axis approaches its speed limit, so the tool never tilts, rolls, or yaws mid-move.
   - Includes quick presets (**Tool Vertical Down**, **Tool Horizontal Fwd**, **Lock Current Angle**) and world-frame rotation jog buttons ($\pm R_x, \pm R_y, \pm R_z$).

2. **`FREE WRIST (Probe / Tip Mode)`**:
   - Keeps the tooltip on the target $(X, Y, Z)$ coordinate while allowing the wrist orientation to relax naturally for maximum reach.

---

## 4. Multi-Layer Kinematic Safety & Cable Protection

Both [`RobotKinematics.cs`](../Windows_ControlSuite/Models/RobotKinematics.cs) on Windows and [`kinematics.py`](../daemon/kinematics.py) on the Raspberry Pi enforce:

- **Deterministic Analytical Wrist Branch Locking**: Evaluates both spherical wrist branches ($[J_4, +J_5, J_6]$ and $[J_4 \pm 180^\circ, -J_5, J_6 \pm 180^\circ]$) using weighted Euclidean modular distance against $\mathbf{q}_{\text{current}}$ and locks onto the closest branch.
- **Singularity Crossing Guard ($|J_5| < 1.0^\circ$)**: Locks $J_4 = q_{\text{current}, 4}$ and resolves projected roll onto $J_6$ when $J_4$ and $J_6$ become collinear.
- **Hardware Cable Limits (Relative to CAD Home $[0, -90, 180, 0, 0, 180]$)**:
  - $J_1 \in [-170^\circ, +170^\circ]$
  - $J_2 \in [-150^\circ, +50^\circ]$
  - $J_3 \in [+65^\circ, +290^\circ]$
  - $J_4 \in [-160^\circ, +160^\circ]$
  - $J_5 \in [-110^\circ, +110^\circ]$
  - $J_6 \in [0^\circ, +360^\circ]$ ($180^\circ \pm 180^\circ$)
- **Delta Spike Clamp (`ValidateAndLockBranch`)**: Rejects any frame where $|\Delta J_i| > 3.0^\circ$ per $10\text{ ms}$ window, logs a warning in the communication log, and holds the previous valid state.
