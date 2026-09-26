# Autonomous Drone Landing on Dynamic Platforms

A flight controller that makes a quadcopter find a moving rover, follow it, and land on it. The rover carries an ArUco marker, and the drone tracks that marker with its downward camera.

Built for the **Intra IIT Tech Meet 1.0** problem *"Autonomous Precision Landing on Dynamic Platforms"* (PyBullet + OpenCV).

## Features

- **Works with any ArUco marker ID.** It reads the ID on its own, so the marker can be changed without touching the code.
- **Keeps tracking close to the ground.** When the marker is too big for the camera to see whole, it keeps tracking by template matching.
- **Handles wind.** It detects wind by itself and pushes back against it. The same file works in both the normal and the windy arena.
- **Lands safely.** Touchdown is at about -0.35 m/s, well inside the -0.65 m/s limit, and the motors cut off on contact.

## Files

| File | What it is |
|---|---|
| `controller.py` | The solution: the flight controller the arena runs |
| `local_arena.py` | Test arena without wind (provided, not edited) |
| `local_areana_wind.py` | Test arena with random wind gusts (provided, not edited) |
| `test_all_markers.py` | Optional: tests many marker IDs automatically, without a window |

## Setup

You need Python 3.8 or newer.

```bash
pip install pybullet numpy opencv-contrib-python
```

> Use `opencv-contrib-python` (version 4.7 or newer). The plain `opencv-python` package may not include the `cv2.aruco` module.

## How to run

Put all the files in the same folder, then run one of these:

```bash
# Normal arena
python local_arena.py

# Arena with wind
python local_areana_wind.py
```

A PyBullet window opens along with a camera window. The camera window shows what the drone sees and which phase it is in.

### Testing other marker IDs

To try a different marker, change the `0` in this line of the arena file:

```python
marker_img = cv2.aruco.generateImageMarker(aruco_dict, 0, 7)
```

You can also use the test script, which does this for you without editing any files:

```bash
python test_all_markers.py 5 42 99        # these IDs, no wind
python test_all_markers.py --wind 5 42    # these IDs, with wind
python test_all_markers.py --both         # IDs 0-99, with and without wind (slow)
python test_all_markers.py --gui 42       # watch one ID in the window
```

## How it works

Every simulation step, the controller goes through five stages:

1. **Wind check.** It compares how the drone actually moved with how its motors should have moved it. Any difference is an outside force, which it treats as wind.
2. **Find the marker.**
   - From high up, OpenCV's ArUco detector finds the marker.
   - Closer than about 1 m, the marker no longer fits in the camera. The code knows the marker's pattern, so it draws what the camera should see and matches that drawing against the real image.
3. **Track the rover.** A Kalman filter estimates the rover's position, speed and acceleration. It keeps predicting the rover's position for the last few centimetres, when the camera is too close to see anything.
4. **Pick a phase.**

   ```
   SEARCH → TRACK → DESCEND → FINAL → CUTOFF
   ```

   | Phase | What happens |
   |---|---|
   | SEARCH | Look for the marker |
   | TRACK | Fly above the rover and match its speed |
   | DESCEND | Go down, but only while lined up with the rover |
   | FINAL | Last bit of descent, with the camera too close to see |
   | CUTOFF | Turn the motors off on touchdown |

   If the drone drifts off or loses the marker, it climbs back up and tries again.
5. **Control the motors.** It works out the tilt and thrust needed to follow the rover, then splits the thrust between the four motors.

## Results

Tested in simulation using the arena's own physics and scorer:

- All tested IDs landed successfully, both with and without wind.
- The drone lands about 7–8 seconds after the start.
- It usually lands within about 1–8 cm of the rover's centre. The allowed limit is 50 cm.
- Low-altitude marker tracking was checked on all 100 IDs (0–99).

## Settings you can change

All the main numbers are at the top of `controller.py` under `SETTINGS`, for example:

| Setting | Meaning |
|---|---|
| `CRUISE_Z` | Height while chasing the rover (default 3.0 m) |
| `WIND_ON_N` / `WIND_OFF_N` | Force levels that turn wind mode on and off |

To hide the camera overlay or the terminal messages, set `debug_view` or `verbose` to `False` in `FlightController.__init__`.
