# Autonomous Precision Landing on a Dynamic Platform

A simulation-based flight controller for a quadrotor that detects,
tracks, follows, and lands on a moving rover in PyBullet. The rover
carries a visual marker observed by a downward-facing camera.

The project was developed for the **Intra IIT Tech Meet 1.0** problem,
**"Autonomous Precision Landing on Dynamic Platforms."**

## Features

-   Vision-based rover detection using OpenCV ArUco detection.
-   Low-altitude tracking using model/template matching when the full
    marker is no longer visible.
-   Six-state Kalman filtering for rover position, velocity, and
    acceleration estimation.
-   Mahalanobis-distance validation to reject inconsistent visual
    measurements.
-   SEARCH → TRACK → DESCEND → FINAL → CUTOFF flight-state machine.
-   Horizontal tracking using rover-acceleration feed-forward,
    position/velocity feedback, and disturbance compensation.
-   Scheduled vertical descent with controlled descent speed.
-   External disturbance/wind estimation and compensation.
-   Conversion of desired acceleration into attitude, thrust, and four
    motor forces.
-   Motor cut-off after the landing conditions are satisfied.

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

-   Python 3.8 or newer
-   PyBullet
-   NumPy
-   OpenCV with the ArUco module (`opencv-contrib-python`)

Install the required Python packages:

``` bash
pip install pybullet numpy opencv-contrib-python
```

## Setup

1.  Place `controller.py`, `local_arena.py`, and `local_areana_wind.py`
    in the same directory.
2.  Install the dependencies listed above.
3.  Use the supplied arena files to run the controller.

## Running the Controller

### Standard arena

``` bash
python local_arena.py
```

### Wind-enabled arena

``` bash
python local_areana_wind.py
```

The simulation opens a PyBullet window and a camera-view window. The
camera view shows the visual input available to the controller.

## Solution Overview

The controller executes at the simulation rate of **240 Hz**.

``` text
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

At higher altitude, the complete marker is detected using OpenCV ArUco
detection. At low altitude, the marker may extend beyond the camera
image, so the controller uses a generated marker/platform model and
template matching around the predicted rover position.

### 2. Rover State Estimation

The detected horizontal rover position is processed using a six-state
Kalman filter:

``` text
[x, y, vx, vy, ax, ay]
```

The filter predicts rover motion between visual measurements and
corrects the prediction when an accepted measurement is available.

A Mahalanobis-distance test is applied before correction so that
measurements inconsistent with the predicted state can be rejected.

### 3. Flight-State Machine

``` text
SEARCH → TRACK → DESCEND → FINAL → CUTOFF
```

  -----------------------------------------------------------------------
  State                               Function
  ----------------------------------- -----------------------------------
  `SEARCH`                            Acquire the rover and establish a
                                      valid estimate.

  `TRACK`                             Follow the rover while maintaining
                                      altitude.

  `DESCEND`                           Descend while maintaining position
                                      and relative-velocity constraints.

  `FINAL`                             Perform the controlled low-altitude
                                      final descent.

  `CUTOFF`                            Set all motor forces to zero after
                                      touchdown conditions are satisfied.
  -----------------------------------------------------------------------

If alignment or visual-tracking conditions become unsuitable during
descent, the controller reduces or stops descent and can return to
tracking.

### 4. Horizontal Control

The horizontal acceleration command is:

\[ `\mathbf{a}`{=tex}*{xy} = `\mathbf{a}`{=tex}*{rover} +
K_p`\mathbf{e}`{=tex}\_p + K_d`\mathbf{e}`{=tex}\_v -
`\frac{\mathbf{F}_{wind,xy}}{m}`{=tex} \]

Normal tracking uses:

-   (K_p = 4.0)
-   (K_d = 4.2)
-   Maximum tilt = (22\^`\circ`{=tex})

FINAL / low-altitude control uses:

-   (K_p = 7.0)
-   (K_d = 5.0)
-   Maximum tilt = (15\^`\circ`{=tex})

Additional tilt authority of (8\^`\circ`{=tex}) is available when wind
is detected.

### 5. Vertical Control

The vertical acceleration command is:

\[ a_z = 16(z\_{ref}-z) + 8(v\_{z,ref}-v_z) -
`\frac{F_{wind,z}}{m}`{=tex} \]

The controller uses a smooth altitude reference and limits the
reference-velocity change to avoid abrupt descent commands. The final
descent reference is approximately **−0.35 m/s**.

### 6. Attitude and Motor Forces

The desired physical force is:

\[ `\mathbf{F}`{=tex}*{des} =
m(`\mathbf{a}`{=tex}*{des}+`\mathbf{g}`{=tex}) \]

Its normalized direction defines the desired thrust direction. The
desired attitude is constructed while preserving the current yaw
heading, followed by rotational control and four-motor force allocation.

The motor arm length is:

\[ L=0.25`\text{ m}`{=tex} \]

The motor forces are constrained to be non-negative.

## Wind / Disturbance Handling

The controller estimates external force from the drone's measured change
in velocity, previously applied thrust, and gravity:

\[ `\mathbf{F}`{=tex}*{ext} =
m`\frac{\Delta\mathbf{v}}{\Delta t}`{=tex} -
`\mathbf{F}`{=tex}*{thrust} - `\mathbf{F}`{=tex}\_{gravity} \]

The disturbance estimate is filtered before wind-state detection.

  Parameter                                               Value
  --------------------------- ---------------------------------
  Wind ON threshold             0.30 N average horizontal force
  Wind OFF threshold                                     0.12 N
  Sample clipping                                 ±6 N per axis
  Filtering                                 approximately 50 ms
  Wind averaging                              approximately 1 s
  Additional tilt authority                                 +8°

Wind estimation is disabled near the landing deck so that contact forces
are not interpreted as wind.

## Key Simulation Parameters

  Parameter                                         Value
  --------------------------- ---------------------------
  Controller / physics rate                        240 Hz
  Physics timestep                                1/240 s
  Gravity                                       9.81 m/s²
  Drone mass                                      1.20 kg
  Motor arm                                        0.25 m
  Initial drone altitude                            3.0 m
  Search altitude                                   3.6 m
  Rover size                        1.0 m × 1.0 m × 0.1 m
  Marker size                                       0.8 m
  Camera resolution                          320 × 240 px
  Vertical FOV                                        60°
  Camera offset                 0.10 m below drone centre
  Rover deck height                               0.102 m

## Landing Constraints

The supplied scorer uses the following conditions:

  Condition                                                       Threshold
  ------------------------------------- -----------------------------------
  Ground collision                                            (z \< 0.08) m
  Maximum touchdown vertical velocity                           (-0.65) m/s
  Maximum touchdown tilt                  approximately (30\^`\circ`{=tex})
  Rover surface                                                (z = 0.15) m
  Horizontal touchdown region                                    (\< 0.5) m
  Settled vertical velocity                                               (
  Minimum scorer time                                              (\> 2) s

The controller uses an internal cut-off height of approximately **0.152
m** and requires horizontal distance below **0.35 m** before entering
`CUTOFF`.

## Recorded Simulation Results

The following values correspond to the recorded no-wind and wind-enabled
runs reported in the technical report. They are individual simulation
runs, not a statistically representative multi-trial evaluation.

  Parameter                         No Wind   Wind Enabled
  ----------------------------- ----------- --------------
  Landing time                       7.32 s         7.64 s
  Touchdown height                  0.151 m        0.151 m
  Touchdown vertical velocity     −0.18 m/s      −0.35 m/s
  Horizontal touchdown error        0.008 m        0.011 m
  Touchdown tilt                       4.9°           3.4°
  Motor cut-off                   Confirmed      Confirmed

## Limitations

-   The current system is evaluated in simulation rather than on
    physical UAV hardware.
-   Visual tracking can become less reliable under severe marker
    occlusion or complete visual loss.
-   During extended visual loss, Kalman prediction can accumulate
    position error.
-   The simulated actuator and wind models do not fully represent real
    quadrotor aerodynamics and actuator dynamics.

## Future Improvements

-   Validate the perception and control pipeline on a physical UAV.
-   Fuse additional sensing sources to improve robustness during visual
    loss.
-   Use a more representative rover-motion model for prediction.
-   Perform larger multi-trial evaluations across different
    trajectories, initial conditions, and disturbance levels.
