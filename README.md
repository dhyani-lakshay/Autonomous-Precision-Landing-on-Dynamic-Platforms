# Autonomous Precision Landing on a Dynamic Platform

A simulation-based flight controller for a quadrotor that detects, tracks, follows, and lands on a moving rover in PyBullet. The rover carries a visual marker observed by a downward-facing camera.

The project was developed for the **Intra IIT Tech Meet 1.0** problem, **“Autonomous Precision Landing on Dynamic Platforms.”**

## Features

- Vision-based rover detection using OpenCV ArUco detection.
- Low-altitude tracking using model/template matching when the full marker is no longer visible.
- Six-state Kalman filtering for rover position, velocity, and acceleration estimation.
- Mahalanobis-distance validation to reject inconsistent visual measurements.
- SEARCH → TRACK → DESCEND → FINAL → CUTOFF flight-state machine.
- Horizontal tracking using rover-acceleration feed-forward, position/velocity feedback, and disturbance compensation.
- Scheduled vertical descent with controlled descent speed.
- External disturbance/wind estimation and compensation.
- Conversion of desired acceleration into attitude, thrust, and four motor forces.
- Motor cut-off after the landing conditions are satisfied.

## Repository Structure

The repository contains the controller, supplied simulation arenas, marker-testing utility, technical report, and demonstration video.

| File | Description |
|---|---|
| `controller.py` | Main flight controller and primary project implementation. |
| `local_arena.py` | Supplied PyBullet arena without wind. |
| `local_areana_wind.py` | Supplied PyBullet arena with dynamic wind disturbance. |
| `test_all_markers.py` | Utility for testing the controller with different marker IDs. |
| `Autonomous_Precision_Landing_Technical_Report.pdf` | Technical report describing the approach, algorithms, control system, and results. |
| `demo_video.mp4` | Demonstration video of the autonomous landing system. |
| `README.md` | Project setup, usage, and solution overview. |

> The filename `local_areana_wind.py` is intentional and matches the supplied project file.

## Requirements

- Python 3.8 or newer
- PyBullet
- NumPy
- OpenCV with the ArUco module (`opencv-contrib-python`)

Install the required Python packages:

```bash
pip install pybullet numpy opencv-contrib-python
```

## Setup

1. Place `controller.py`, `local_arena.py`, and `local_areana_wind.py` in the same directory.
2. Install the dependencies listed above.
3. Use the supplied arena files to run the controller.

## Running the Controller

### Standard arena

```bash
python local_arena.py
```

### Wind-enabled arena

```bash
python local_areana_wind.py
```

The simulation opens a PyBullet window and a camera-view window. The camera view shows the visual input available to the controller.

## Solution Overview

The controller executes at the simulation rate of **240 Hz**.

```text
PyBullet Environment
        ↓
Drone + Downward Camera
        ↓
ArUco / Template Detection
        ↓
Camera-to-Deck Coordinate Transformation
        ↓
Kalman Rover State Estimation
        ↓
Mahalanobis Measurement Validation
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

At higher altitude, the complete marker is detected using OpenCV ArUco detection. At low altitude, the marker may extend beyond the camera image, so the controller uses a generated marker/platform model and template matching around the predicted rover position.

### 2. Rover State Estimation

The detected horizontal rover position is processed using a six-state Kalman filter:

$$
\mathbf{x} =
\begin{bmatrix}
x & y & v_x & v_y & a_x & a_y
\end{bmatrix}^{T}
$$

The filter predicts rover motion between visual measurements and corrects the prediction when an accepted measurement is available.

A Mahalanobis-distance test is applied before correction so that measurements inconsistent with the predicted state can be rejected.

### 3. Flight-State Machine

```text
SEARCH → TRACK → DESCEND → FINAL → CUTOFF
```

| State | Function |
|---|---|
| `SEARCH` | Acquire the rover and establish a valid estimate. |
| `TRACK` | Follow the rover while maintaining altitude. |
| `DESCEND` | Descend while maintaining position and relative-velocity constraints. |
| `FINAL` | Perform the controlled low-altitude final descent. |
| `CUTOFF` | Set all motor forces to zero after touchdown conditions are satisfied. |

If alignment or visual-tracking conditions become unsuitable during descent, the controller reduces or stops descent and can return to tracking.

### 4. Horizontal Control

The horizontal acceleration command is:

$$
\mathbf{a}_{xy}
=
\mathbf{a}_{rover}
+
K_p\mathbf{e}_p
+
K_d\mathbf{e}_v
-
\frac{\mathbf{F}_{wind,xy}}{m}
$$

Normal tracking uses:

- $K_p = 4.0$
- $K_d = 4.2$
- Maximum tilt = $22^\circ$

FINAL / low-altitude control uses:

- $K_p = 7.0$
- $K_d = 5.0$
- Maximum tilt = $15^\circ$

Additional tilt authority of $8^\circ$ is available when wind is detected.

### 5. Vertical Control

The vertical acceleration command is:

$$
a_z =
16(z_{ref}-z)
+
8(v_{z,ref}-v_z)
-
\frac{F_{wind,z}}{m}
$$

The controller uses a smooth altitude reference and limits the reference-velocity change to avoid abrupt descent commands. The final descent reference is approximately **$-0.35\ \mathrm{m/s}$**.

### 6. Attitude and Motor Forces

The desired physical force is:

$$
\mathbf{F}_{des}
=
m(\mathbf{a}_{des}+\mathbf{g})
$$

The normalized force direction defines the desired thrust direction:

$$
\mathbf{u}_3
=
\frac{\mathbf{F}_{des}}
{\left\|\mathbf{F}_{des}\right\|}
$$

The desired attitude is constructed while preserving the current yaw heading, followed by rotational control and four-motor force allocation.

The attitude error is defined as:

$$
\mathbf{e}_R
=
\frac{1}{2}
\operatorname{vee}
\left(
R_{des}^{T}R
-
R^{T}R_{des}
\right)
$$

The body angular velocity is estimated from the change in rotation matrix:

$$
\boldsymbol{\omega}_k
=
\frac{
\operatorname{vee}
\left[
\frac{1}{2}
\left(
R_{k-1}^{T}R_k
-
R_k^{T}R_{k-1}
\right)
\right]
}
{\Delta t}
$$

The implemented attitude torque equation is:

$$
\boldsymbol{\tau}
=
-I\omega_n^2\mathbf{e}_R
-
2\zeta\omega_n I\boldsymbol{\omega}
$$

The effective rotational inertia used by the controller is:

$$
I = 0.018
$$

The desired orientation is constructed from three unit vectors in the order:

$$
\mathbf{F}_{des}
\rightarrow
\mathbf{u}_3
\rightarrow
\mathbf{u}_2
\rightarrow
\mathbf{u}_1
\rightarrow
R_{des}
$$

where $\mathbf{u}_3$ is the desired thrust direction, $\mathbf{u}_2$ is the desired left direction, and $\mathbf{u}_1$ is the desired forward direction.

The motor arm length is:

$$
L = 0.25\ \mathrm{m}
$$

The motor forces are constrained to be non-negative.

## Wind / Disturbance Handling

The controller estimates external force from the drone's measured change in velocity, previously applied thrust, and gravity:

$$
\mathbf{F}_{ext}
=
m\frac{\Delta\mathbf{v}}{\Delta t}
-
\mathbf{F}_{thrust}
-
\mathbf{F}_{gravity}
$$

The disturbance estimate is filtered before wind-state detection.

| Parameter | Value |
|---|---:|
| Wind ON threshold | $0.30$ N average horizontal force |
| Wind OFF threshold | $0.12$ N |
| Sample clipping | $\pm 6$ N per axis |
| Filtering | approximately 50 ms |
| Wind averaging | approximately 1 s |
| Additional tilt authority | $+8^\circ$ |

Wind estimation is disabled near the landing deck so that contact forces are not interpreted as wind.

## Key Simulation Parameters

| Parameter | Value |
|---|---:|
| Controller / physics rate | 240 Hz |
| Physics timestep | $1/240$ s |
| Gravity | $9.81\ \mathrm{m/s^2}$ |
| Drone mass | $1.20$ kg |
| Motor arm | $0.25$ m |
| Initial drone altitude | $3.0$ m |
| Search altitude | $3.6$ m |
| Rover size | $1.0 \times 1.0 \times 0.1$ m |
| Marker size | $0.8$ m |
| Camera resolution | 320 × 240 px |
| Vertical FOV | $60^\circ$ |
| Camera offset | $0.10$ m below drone centre |
| Rover deck height | $0.102$ m |

## Landing Constraints

The supplied scorer uses the following conditions:

| Condition | Threshold |
|---|---:|
| Ground collision | $z < 0.08$ m |
| Maximum touchdown vertical velocity | $-0.65$ m/s |
| Maximum touchdown tilt | approximately $30^\circ$ |
| Rover surface | $z = 0.15$ m |
| Horizontal touchdown region | $< 0.5$ m |
| Settled vertical velocity | $|v_z| < 0.05$ m/s |
| Minimum scorer time | $> 2$ s |

The controller uses an internal cut-off height of approximately **0.152 m** and requires horizontal distance below **0.35 m** before entering `CUTOFF`.

## Recorded Simulation Results

The following values correspond to the recorded no-wind and wind-enabled runs reported in the technical report. They are individual simulation runs, not a statistically representative multi-trial evaluation.

| Parameter | No Wind | Wind Enabled |
|---|---:|---:|
| Landing time | 7.32 s | 7.64 s |
| Touchdown height | 0.151 m | 0.151 m |
| Touchdown vertical velocity | −0.18 m/s | −0.35 m/s |
| Horizontal touchdown error | 0.008 m | 0.011 m |
| Touchdown tilt | 4.9° | 3.4° |
| Motor cut-off | Confirmed | Confirmed |

## Limitations

- The current system is evaluated in simulation rather than on physical UAV hardware.
- Visual tracking can become less reliable under severe marker occlusion or complete visual loss.
- During extended visual loss, Kalman prediction can accumulate position error.
- The simulated actuator and wind models do not fully represent real quadrotor aerodynamics and actuator dynamics.

## Future Improvements

- Validate the perception and control pipeline on a physical UAV.
- Fuse additional sensing sources to improve robustness during visual loss.
- Use a more representative rover-motion model for prediction.
- Perform larger multi-trial evaluations across different trajectories, initial conditions, and disturbance levels.
