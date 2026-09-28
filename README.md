# Autonomous Precision Landing on a Dynamic Platform

A simulation-based flight controller for a quadrotor that detects, tracks, follows, and lands on a moving rover in PyBullet. The rover carries an ArUco marker, which the drone observes with a downward-facing camera.

The project was developed for the **Intra IIT Tech Meet 1.0** problem, **"Autonomous Precision Landing on Dynamic Platforms."**

## GitHub Page

[View the GitHub Page](https://dhyani-lakshay.github.io/Autonomous-Precision-Landing-on-Dynamic-Platforms/)

## Features

- **Marker detection:** OpenCV ArUco detection finds the rover's marker. It works with any 5×5 marker ID, detected automatically.
- **Low-altitude tracking:** template matching takes over when the full marker no longer fits in the camera view.
- **Rover state estimation:** a six-state Kalman filter estimates the rover's position, velocity, and acceleration.
- **Measurement validation:** a Mahalanobis-distance test rejects visual measurements that are inconsistent with the estimate.
- **Flight-state machine:** SEARCH → TRACK → DESCEND → FINAL → CUTOFF.
- **Horizontal tracking:** rover-acceleration feed-forward, position/velocity feedback, and disturbance compensation.
- **Vertical control:** a scheduled, smooth descent with limited descent speed.
- **Wind handling:** external disturbance (wind) is estimated and compensated.
- **Force computation:** the desired acceleration is converted into attitude, thrust, and four motor forces.
- **Touchdown:** the motors are cut off once the landing conditions are satisfied.

## Repository Structure

| File | Description |
|---|---|
| `controller.py` | Main flight controller (the submitted solution). |
| `local_arena.py` | Supplied PyBullet arena without wind. |
| `local_areana_wind.py` | Supplied PyBullet arena with dynamic wind disturbance. |
| `test_all_markers.py` | Utility for testing the controller with different marker IDs. |
| `Autonomous_Precision_Landing_Technical_Report.pdf` | Technical report: approach, algorithms, control system, and results. |
| `demo_video.mp4` | Demonstration video of the autonomous landing. |
| `README.md` | Setup, usage, and solution overview. |

> The filename `local_areana_wind.py` is intentional and matches the supplied project file.

## Requirements

| Package | Required version | Version tested |
|---|---|---|
| Python | 3.8 or newer | 3.12.3 |
| PyBullet | any recent version | 3.2.7 |
| NumPy | 1.20 or newer | 2.4.4 |
| OpenCV (contrib build) | **4.7.0 or newer** | 4.13.0.92 |

**Why OpenCV 4.7 or newer:** the arenas call `cv2.aruco.generateImageMarker()`, which was added in OpenCV 4.7. On older versions the arena itself fails before the controller runs.

**Why the contrib build:** some builds of the plain `opencv-python` package do not include the `cv2.aruco` module, so use `opencv-contrib-python`. Install only one OpenCV package. If several are installed they conflict, so remove the others first.

## Setup

1. **Clone the repository** (or extract the zip) so that `controller.py`, `local_arena.py` and `local_areana_wind.py` are in the same folder.

2. **Optional: create a virtual environment.**

   ```bash
   python -m venv venv
   source venv/bin/activate        # Windows: venv\Scripts\activate
   ```

3. **Remove any existing OpenCV packages**, so they can't conflict:

   ```bash
   pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python
   ```

4. **Install the dependencies:**

   ```bash
   pip install pybullet numpy "opencv-contrib-python>=4.7"
   ```

5. **Check the installation:**

   ```bash
   python -c "import cv2, pybullet; print(cv2.__version__, hasattr(cv2.aruco, 'generateImageMarker'))"
   ```

   It should print a version of 4.7 or higher, followed by `True`.

## Running the Controller

**Standard arena:**

```bash
python local_arena.py
```

**Wind-enabled arena:**

```bash
python local_areana_wind.py
```

A PyBullet window and a camera-view window open. The camera view shows the image the controller receives, with an overlay of:

- the current flight state,
- the measurement source (`aruco`, `model` or `-`),
- the estimated rover position,
- the marker ID,
- the wind level, when wind is detected.

The same `controller.py` is used for both arenas. Wind is detected automatically, so nothing needs to be changed.

### Testing different marker IDs (optional)

`test_all_markers.py` runs the arena headless and only changes the marker ID. The arena files are not edited.

```bash
python test_all_markers.py 5 42 99        # selected IDs, no wind
python test_all_markers.py --wind 5 42    # selected IDs, with wind
python test_all_markers.py --both         # IDs 0-99, with and without wind (slow)
python test_all_markers.py --gui 42       # watch one ID in the GUI
```

### Troubleshooting

| Problem | Fix |
|---|---|
| `module 'cv2' has no attribute 'aruco'` | Install `opencv-contrib-python` (see Setup, steps 3–4). |
| `cv2.aruco` has no `generateImageMarker` | OpenCV is older than 4.7. Upgrade it. |
| No window opens | The GUI arenas need a display. On a headless machine, use `test_all_markers.py`. |

## Solution Overview

The controller runs at the simulation rate of **240 Hz**. Each step, it goes through this pipeline:

```text
PyBullet Environment
        ↓
Drone + Downward Camera
        ↓
ArUco / Template Detection
        ↓
Camera-to-Deck Coordinate Transformation
        ↓
Mahalanobis Measurement Validation
        ↓
Kalman Rover State Estimation
        ↓
Relative Position / Velocity
        ↓
Flight-State Machine
        ↓
Horizontal + Vertical Control
        ↓
Desired Acceleration
        ↓
Attitude + Thrust
        ↓
Four-Motor Force Allocation
        ↓
Drone Motion
        ↺
Camera Feedback
```

### 1. Visual Detection

**High altitude.** OpenCV ArUco detection (`DICT_5X5_1000`, which contains every 5×5 dictionary) finds the complete marker. The marker's ID and dictionary are locked on the first detection.

**Low altitude.** Below roughly 1 m the marker extends beyond the camera image, so ArUco can no longer find it. The controller then:
1. regenerates the exact marker pattern from the detected ID,
2. draws the expected camera view around the predicted rover position,
3. finds the true offset with normalised template matching.

**Pixel to deck.** Each image point is mapped to the rover deck by intersecting its camera ray with the deck plane:

```math
\mathbf{r}_{b} = \begin{bmatrix} -\dfrac{v-c_y}{f} & -\dfrac{u-c_x}{f} & -1 \end{bmatrix}^{T},
\qquad
\mathbf{r} = R\,\mathbf{r}_{b},
\qquad
\mathbf{P} = \mathbf{C} + \frac{z_{deck} - C_z}{r_z}\,\mathbf{r}
```

where:
- (u, v) is the pixel,
- (c_x, c_y) is the image centre,
- **C** is the camera position,
- R is the drone's rotation matrix,
- f is the focal length, which comes from the **vertical** 60° field of view:

```math
f = \frac{240/2}{\tan(30^\circ)} \approx 207.8\ \text{px}
```

### 2. Rover State Estimation

The rover's horizontal state is estimated by a six-state, constant-acceleration Kalman filter:

```math
\mathbf{x} = \begin{bmatrix} x & y & v_x & v_y & a_x & a_y \end{bmatrix}^{T}
```

Each axis is predicted with:

```math
\begin{bmatrix} p \\ v \\ a \end{bmatrix}_{k+1}
=
\begin{bmatrix} 1 & \Delta t & \tfrac{1}{2}\Delta t^{2} \\ 0 & 1 & \Delta t \\ 0 & 0 & 1 \end{bmatrix}
\begin{bmatrix} p \\ v \\ a \end{bmatrix}_{k}
```

**Validation.** Before a measurement **z** (with covariance R_m) is used to correct the prediction, it must pass a Mahalanobis-distance test:

```math
d^{2} = (\mathbf{z} - H\mathbf{x})^{T}\left(HPH^{T} + R_m\right)^{-1}(\mathbf{z} - H\mathbf{x})
```

The measurement is accepted if d² < 40 (ArUco) or d² < 25 (template matching).

**Blind prediction.** During the last part of the descent the camera is too close to the deck to see anything, and the filter's prediction is used on its own.

### 3. Flight-State Machine

```text
SEARCH → TRACK → DESCEND → FINAL → CUTOFF
```

| State | Function |
|---|---|
| `SEARCH` | Acquire the rover and establish a valid estimate. |
| `TRACK` | Follow the rover while holding altitude, until within 0.12 m and 0.25 m/s of it for 0.6 s. |
| `DESCEND` | Descend while position and relative-velocity limits are met. The sink rate is 0.9 → 0.6 → 0.4 m/s depending on height. |
| `FINAL` | Controlled low-altitude final descent at 0.35 m/s, using the Kalman prediction. |
| `CUTOFF` | Set all motor forces to zero once touchdown is detected. |

If alignment or visual tracking becomes poor during descent, the controller slows or stops the descent. It can also climb back and return to `TRACK`.

### 4. Horizontal Control

The horizontal acceleration command is:

```math
\mathbf{a}_{xy} = \mathbf{a}_{rover} + K_p\,\mathbf{e}_p + K_d\,\mathbf{e}_v - \frac{\mathbf{F}_{wind,xy}}{m}
```

where:
- **e**_p is the position error (rover minus drone, capped at 1.5 m),
- **e**_v is the velocity error.

The command is limited to g·tan(θ_max).

| Setting | Kp | Kd | Max tilt |
|---|---:|---:|---:|
| Normal tracking | 4.0 | 4.2 | 22° |
| FINAL / below 1 m | 7.0 | 5.0 | 15° |
| Wind detected | same | same | +8° |

### 5. Vertical Control

The vertical acceleration command is:

```math
a_z = 16\,(z_{ref} - z) + 8\,(v_{z,ref} - v_z) - \frac{F_{wind,z}}{m}
```

It is clipped to −6 to +8 m/s².

The altitude reference z_ref is smooth: the rate of change of v_z,ref is limited to 1.2 m/s², so there are no abrupt descent commands. The final descent reference is **−0.35 m/s**, well inside the −0.65 m/s limit.

### 6. Attitude and Motor Forces

**Desired force and thrust direction:**

```math
\mathbf{F}_{des} = m\left(\mathbf{a}_{des} + g\,\hat{\mathbf{z}}\right),
\qquad
\mathbf{u}_3 = \frac{\mathbf{F}_{des}}{\lVert \mathbf{F}_{des} \rVert}
```

**Desired orientation.** The current yaw heading is kept:

```math
\mathbf{h} = \begin{bmatrix} \cos\psi & \sin\psi & 0 \end{bmatrix}^{T},
\quad
\mathbf{u}_2 = \frac{\mathbf{u}_3 \times \mathbf{h}}{\lVert \mathbf{u}_3 \times \mathbf{h} \rVert},
\quad
\mathbf{u}_1 = \mathbf{u}_2 \times \mathbf{u}_3,
\quad
R_{des} = \begin{bmatrix} \mathbf{u}_1 & \mathbf{u}_2 & \mathbf{u}_3 \end{bmatrix}
```

Here **u**_3 is the desired thrust direction, **u**_2 the desired left direction, and **u**_1 the desired forward direction.

**Attitude error:**

```math
\mathbf{e}_R = \frac{1}{2}\,\mathrm{vee}\!\left(R_{des}^{T}R - R^{T}R_{des}\right)
```

**Angular velocity**, estimated from the change in rotation between steps:

```math
\boldsymbol{\omega}_k = \frac{1}{\Delta t}\,\mathrm{vee}\!\left[\frac{1}{2}\left(R_{k-1}^{T}R_k - R_k^{T}R_{k-1}\right)\right]
```

**Attitude torque** (a PD law), with I = 0.018 kg·m², ω_n = 14 rad/s and ζ = 0.9:

```math
\boldsymbol{\tau} = -I\,\omega_n^{2}\,\mathbf{e}_R - 2\,\zeta\,\omega_n\,I\,\boldsymbol{\omega}
```

Each torque component is clipped to ±1.2 N·m. Yaw is not actively controlled, because the arena's motors only produce vertical forces.

**Total thrust**, the part of the desired force along the drone's current up-axis:

```math
T = \max\!\left(0,\ \mathbf{F}_{des} \cdot R\,\hat{\mathbf{z}}\right)
```

**Motor allocation.** The motors sit at f1 (+L, +L), f2 (−L, +L), f3 (+L, −L) and f4 (−L, −L), with arm length L = 0.25 m:

```math
\begin{aligned}
f_1 &= \tfrac{T}{4} + \tfrac{\tau_x}{4L} - \tfrac{\tau_y}{4L}, &
f_2 &= \tfrac{T}{4} + \tfrac{\tau_x}{4L} + \tfrac{\tau_y}{4L}, \\
f_3 &= \tfrac{T}{4} - \tfrac{\tau_x}{4L} - \tfrac{\tau_y}{4L}, &
f_4 &= \tfrac{T}{4} - \tfrac{\tau_x}{4L} + \tfrac{\tau_y}{4L}
\end{aligned}
```

The motor forces are constrained to be non-negative.

## Wind / Disturbance Handling

**Estimate.** PyBullet integrates velocity exactly, so the external force follows from the measured change in velocity, the previously applied thrust, and gravity:

```math
\mathbf{F}_{ext} = m\,\frac{\Delta \mathbf{v}}{\Delta t} - \mathbf{F}_{thrust} - \mathbf{F}_{gravity},
\qquad
\mathbf{F}_{gravity} = \begin{bmatrix} 0 & 0 & -mg \end{bmatrix}^{T}
```

**Use.** The filtered estimate is subtracted in both the horizontal and the vertical control. In the no-wind arena it stays close to zero, so the same controller works in both arenas.

| Parameter | Value |
|---|---:|
| Wind ON threshold | 0.30 N average horizontal force |
| Wind OFF threshold | 0.12 N |
| Sample clipping | ±6 N per axis |
| Filtering | about 50 ms |
| Wind averaging | about 1 s |
| Additional tilt authority | +8° |

Wind estimation is disabled below z = 0.20 m, so that contact forces from the rover are not mistaken for wind.

## Key Simulation Parameters

| Parameter | Value |
|---|---:|
| Controller / physics rate | 240 Hz |
| Physics timestep | 1/240 s |
| Gravity | 9.81 m/s² |
| Drone mass | 1.20 kg |
| Motor arm | 0.25 m |
| Initial drone altitude | 3.0 m |
| Search altitude | 3.6 m |
| Rover size | 1.0 × 1.0 × 0.1 m |
| Marker size | 0.8 m |
| Camera resolution | 320 × 240 px |
| Vertical FOV | 60° |
| Camera offset | 0.10 m below drone centre |
| Rover deck height | 0.102 m |

## Landing Constraints

The supplied scorer uses the following conditions:

| Condition | Threshold |
|---|---:|
| Ground collision | z < 0.08 m |
| Maximum touchdown vertical velocity | −0.65 m/s |
| Maximum touchdown tilt | 0.52 rad (about 30°) |
| Touchdown zone | z ≤ 0.16 m (rover surface 0.15 m + 0.01 m) |
| Horizontal touchdown region | < 0.5 m from rover centre |
| Settled vertical velocity | \|v_z\| < 0.05 m/s |
| Minimum scorer time | > 2 s |

The controller enters `CUTOFF` when both of these hold:
- **Contact:** z < 0.152 m, or z < 0.17 m with v_z > −0.05 m/s.
- **Alignment:** estimated horizontal distance < 0.35 m.

## Recorded Simulation Results

These values are from the recorded no-wind and wind-enabled runs in the technical report. They are individual simulation runs, not a statistically representative multi-trial evaluation.

| Parameter | No Wind | Wind Enabled |
|---|---:|---:|
| Landing time | 7.32 s | 7.64 s |
| Touchdown height | 0.151 m | 0.151 m |
| Touchdown vertical velocity | −0.18 m/s | −0.35 m/s |
| Horizontal touchdown error | 0.008 m | 0.011 m |
| Touchdown tilt | 4.9° | 3.4° |
| Motor cut-off | Confirmed | Confirmed |

## Limitations

- The system is evaluated in simulation, not on physical UAV hardware.
- Visual tracking can become less reliable under severe marker occlusion or complete visual loss.
- During extended visual loss, the Kalman prediction accumulates position error.
- The simulated actuator and wind models do not fully represent real quadrotor aerodynamics and actuator dynamics.

## Future Improvements

- Validate the perception and control pipeline on a physical UAV.
- Fuse additional sensors to improve robustness during visual loss.
- Use a more representative rover-motion model for prediction.
- Run larger multi-trial evaluations across different trajectories, initial conditions, and disturbance levels.