# PAROL6C Windows 3D Digital Twin Control Suite (.NET 8 WPF)

> **High-Performance C# / .NET 8 WPF Desktop Control Suite for the Custom PAROL6 6-DOF Klipper Robotic Arm**  
> Hybrid TCP WebSocket (`:8765`) + Raw UDP (`:8766`) Streaming | HelixToolkit 3D Digital Twin | Interactive TCP Gizmo | Strict Flange Orientation Lock & Cable Protection

---

## ⚠️ EXPERIMENTAL & "VIBE-CODED" DISCLAIMER

> [!CAUTION]
> This application is **experimental and vibe-coded** for controlling a custom 6-DOF robotic arm. Always keep a physical **Emergency Stop (E-Stop)** within reach when operating the physical robot, verify trajectories in **Simulate Mode** first, and use at your own risk.

---

## 1. Key Features

### 1. Three Explicit Interaction Modes
- **Simulate Mode (`SIMULATE ONLY`)**:
  - Purely virtual 3D kinematics sandbox. Drag the 3D TCP gizmo, Cartesian directional pads, or joint sliders to move the translucent cyan ghost robot locally with **zero network motion packets** sent to the physical arm.
- **Send On Release Mode (`ON-RELEASE`)**:
  - Drag the 3D gizmo or sliders to stage a target pose and visualize the planned trajectory line in the 3D viewport.
  - On mouse release (or when clicking **COMMIT STAGED POSE**), the suite interpolates and executes a velocity-blended move from the current pose to the target pose.
- **Live Follow Mode (`LIVE FOLLOW`)**:
  - Continuous 50 Hz real-time teleoperation streaming packed 39-byte binary UDP frames (`0x01` / `0x02`) with sectional B-spline smoothing and a live decaying 3D trajectory trail.

### 2. Two Inverse Kinematics Orientation Modes
- **Keep Tool Steady (Locked 3D Orientation)**:
  - Preserves the exact $3 \times 3$ flange world rotation matrix $R_0$ across **every single intermediate step** during linear translations ($\pm X, \pm Y, \pm Z$), `OnRelease` moves, and `Follow` teleoperation.
  - Includes quick orientation presets (**Tool Vertical Down**, **Tool Horizontal Fwd**, **Lock Current Angle**) and $R_x / R_y / R_z$ world rotation jog buttons.
- **Free Wrist (Probe / Tooltip Mode)**:
  - Keeps the tooltip on the target $(X, Y, Z)$ coordinate while allowing the wrist angles to relax naturally.

### 3. Multi-Layer Kinematic Safety & Wrist Branch Guards
- **Deterministic Spherical Wrist Branch Locking**: Evaluates both $[J_4, +J_5, J_6]$ and $[J_4 \pm 180^\circ, -J_5, J_6 \pm 180^\circ]$ using weighted Euclidean distance against $\mathbf{q}_{\text{current}}$ and prevents branch flipping.
- **Singularity Crossing Guard ($|J_5| < 1.0^\circ$)**: Locks $J_4 = q_{\text{current}, 4}$ and resolves roll onto $J_6$ when $J_4$ and $J_6$ are collinear.
- **Mechanical Cable Protection Limits**:
  - $J_1 \in [-170^\circ, +170^\circ]$
  - $J_2 \in [-150^\circ, +50^\circ]$
  - $J_3 \in [+65^\circ, +290^\circ]$
  - $J_4 \in [-160^\circ, +160^\circ]$
  - $J_5 \in [-110^\circ, +110^\circ]$
  - $J_6 \in [0^\circ, +360^\circ]$ ($180^\circ \pm 180^\circ$ from CAD Home)
- **Delta Spike Clamp (`ValidateAndLockBranch`)**: Rejects any step exceeding $3.0^\circ$ per tick and holds the previous valid state.

### 4. Dual-Channel Live Communication Logs (`COMM LOGS` Tab)
- Real-time inspection of both communication links:
  - **`SUITE <-> RPI` (WebSocket `:8765` & UDP `:8766`)**
  - **`RPI <-> KLIPPER MCU` (`/tmp/printer` G-code & serial status)**

---

## 2. Building & Launching

### Option A: One-Click Launch
Double-click `Launch-ControlSuite.bat`.

### Option B: Build from CLI
```powershell
dotnet build PAROLC_ControlSuite.csproj -c Release
dotnet run --project PAROLC_ControlSuite.csproj -c Release
```
