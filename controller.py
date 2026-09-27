"""
controller.py - lands the quadcopter on the moving rover.

This one file works in both arenas:
    local_arena.py          (no wind)
    local_areana_wind.py    (random wind gusts)
It finds out by itself whether there is wind and adjusts.


HOW IT WORKS, IN SHORT
======================

Every physics step (240 times a second) the arena calls calculate_forces().
Each call goes through the same steps:

  1. Wind check      Compare how the drone actually accelerated with how our
                     motors should have made it accelerate. The difference is
                     an outside force, which we call wind.

  2. Find the rover  a) ArUco: while the whole marker fits in the camera
                        (above about 1 m) OpenCV finds it directly.
                     b) Template matching: lower down, the marker is too big
                        for the camera, so ArUco fails. We already know the
                        marker's ID, so we draw what the camera SHOULD see and
                        slide that drawing over the real image until they match.
                        This works for every marker ID.

  3. Rover tracker   A Kalman filter smooths the rover's position and works
                     out its velocity and acceleration. It also keeps guessing
                     where the rover is during the last ~25 cm, where the
                     camera is too close to the deck to see anything.

  4. Decide phase    SEARCH  -> TRACK  -> DESCEND -> FINAL -> CUTOFF
                     (find it)  (match   (go down   (last    (motors off
                                speed)   while      bit,     on touchdown)
                                         aligned)   blind)

  5. Fly             Work out the acceleration we want, turn it into a tilt
                     and a total thrust, then split that thrust between the
                     four motors.


Words used in the comments
--------------------------
  deck        the top surface of the rover (where the marker is)
  world       PyBullet's fixed x, y, z axes
  body        the drone's own axes: x forward, y left, z up
  R           rotation matrix that turns body directions into world directions
"""

import math

import cv2
import numpy as np


# =============================================================================
# SETTINGS
# =============================================================================

# ---- Timing and the drone itself ----
DT = 1.0 / 240.0                  # one physics step [s]
GRAVITY = 9.81                    # [m/s^2]
MASS = 1.0 + 4 * 0.05             # chassis 1.0 kg + four 50 g rotors [kg]
ARM = 0.25                        # each motor sits at (+/-ARM, +/-ARM) [m]

# ---- Camera (must match get_drone_camera_image() in the arena) ----
IMG_W, IMG_H = 320, 240
FOV_VERTICAL_DEG = 60.0           # PyBullet's "fov" is the VERTICAL angle
FOCAL = (IMG_H / 2.0) / math.tan(math.radians(FOV_VERTICAL_DEG / 2.0))  # ~208 px
CX, CY = (IMG_W - 1) / 2.0, (IMG_H - 1) / 2.0     # image centre [px]
CAM_BELOW_DRONE = 0.10            # camera is 10 cm under the drone's centre [m]

# ---- Rover and marker geometry ----
DECK_Z = 0.102                    # height of the marker surface [m]
TOUCH_Z = 0.14                    # drone height when it is sitting on the deck [m]
MARKER_SIZE = 0.8                 # marker width, including its black border [m]
PAD_SIZE = 1.0                    # white top of the rover [m]

# ---- Flight plan ----
CRUISE_Z = 3.0                    # height used while chasing the rover [m]
SEARCH_Z = 3.6                    # climb here if we have never seen the marker [m]
VISION_FLOOR_Z = DECK_Z + CAM_BELOW_DRONE + 0.18  # below this the camera is useless
CUT_Z = TOUCH_Z + 0.012           # drone this low -> we are on the deck

# ---- Wind detection ----
WIND_ON_N = 0.30                  # average side force above this -> wind mode [N]
WIND_OFF_N = 0.12                 # ...and below this -> back to normal [N]


# =============================================================================
# SMALL MATH HELPERS
# =============================================================================

def quaternion_to_matrix(q):
    """PyBullet quaternion (x, y, z, w) -> 3x3 rotation matrix (body -> world)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def vee(M):
    """Take the 3-vector out of a skew-symmetric matrix (inverse of the cross-product matrix)."""
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


# =============================================================================
# 1. CAMERA: convert between image pixels and points on the deck
# =============================================================================

class DownwardCamera:
    """Pin-hole model of the arena's camera for one drone pose.

    The arena points the camera straight down (body -z) with the top of the
    image towards body +x, so the right side of the image is body -y.
    Checked against real PyBullet renders: error is about 1 pixel.
    """

    # Camera matrix with the axis swap described above built in
    K = np.array([[0.0, FOCAL, CX],
                  [FOCAL, 0.0, CY],
                  [0.0, 0.0, 1.0]])

    def __init__(self, drone_pos, R):
        self.R = R
        self.position = np.asarray(drone_pos, float) + R @ np.array([0.0, 0.0, -CAM_BELOW_DRONE])

        # Distance from the camera to the deck, measured along the camera's view line
        height_above_deck = self.position[2] - DECK_Z
        cos_tilt = max(0.2, R[2, 2])
        self.distance = height_above_deck / cos_tilt

        # Homography: deck point (X, Y, 1) -> image point (u*w, v*w, w)
        to_camera = np.array([[1.0, 0.0, -self.position[0]],
                              [0.0, 1.0, -self.position[1]],
                              [0.0, 0.0, DECK_Z - self.position[2]]])
        self.deck_to_image = self.K @ R.T @ to_camera

    def pixel_to_deck(self, u, v):
        """Where on the deck (world x, y) does this pixel look? None if nowhere."""
        ray_in_body = np.array([-(v - CY) / FOCAL, -(u - CX) / FOCAL, -1.0])
        ray = self.R @ ray_in_body
        if ray[2] > -1e-6:                 # ray points up or sideways
            return None
        steps = (DECK_Z - self.position[2]) / ray[2]
        if steps <= 0:
            return None
        return (self.position + steps * ray)[:2]

    def deck_to_pixel(self, xy):
        """Where in the image does this deck point (world x, y) appear? None if behind us."""
        h = self.deck_to_image @ np.array([xy[0], xy[1], 1.0])
        if h[2] >= -1e-9:
            return None
        return h[:2] / h[2]


# =============================================================================
# 2. ROVER TRACKER: Kalman filter for the rover's motion
# =============================================================================

class RoverTracker:
    """Keeps track of the rover's position, velocity and acceleration (world x, y).

    State vector: [x, y, vx, vy, ax, ay].
    Model: acceleration stays roughly constant from one step to the next,
    with some random "jerk" allowed so it can follow the rover's curves.
    """

    JERK_NOISE = 3.0              # how quickly we let the acceleration change

    def __init__(self):
        self.state = None
        self.P = None             # uncertainty (covariance) of the state
        self.time_since_seen = 1e9

        # One axis: position += v*dt + a*dt^2/2,  velocity += a*dt
        step = np.array([[1, DT, 0.5 * DT * DT],
                         [0, 1, DT],
                         [0, 0, 1]])
        # Uncertainty added per step by random jerk
        noise = np.array([[DT ** 5 / 20, DT ** 4 / 8, DT ** 3 / 6],
                          [DT ** 4 / 8, DT ** 3 / 3, DT ** 2 / 2],
                          [DT ** 3 / 6, DT ** 2 / 2, DT]]) * self.JERK_NOISE

        # Use the same model for x and for y
        self.F = np.zeros((6, 6))
        self.Q = np.zeros((6, 6))
        for axis in range(2):
            idx = [axis, axis + 2, axis + 4]
            self.F[np.ix_(idx, idx)] = step
            self.Q[np.ix_(idx, idx)] = noise

        # A measurement tells us the position only
        self.H = np.zeros((2, 6))
        self.H[0, 0] = self.H[1, 1] = 1.0

    @property
    def has_estimate(self):
        return self.state is not None

    @property
    def position(self):
        return self.state[0:2]

    @property
    def velocity(self):
        return self.state[2:4]

    @property
    def acceleration(self):
        return self.state[4:6]

    def start_at(self, pos):
        """Start (or restart) tracking from a first sighting."""
        self.state = np.array([pos[0], pos[1], 0, 0, 0, 0], float)
        self.P = np.diag([0.05 ** 2] * 2 + [1.5 ** 2] * 2 + [1.5 ** 2] * 2)
        self.time_since_seen = 0.0

    def predict(self):
        """Move the estimate forward by one time step."""
        if self.state is None:
            return
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.time_since_seen += DT

        # If we have been blind for a while, slowly forget the acceleration
        # so an old value is not pushed forward for seconds.
        if self.time_since_seen > 0.6:
            self.state[4:6] *= 0.99

        # The rover never accelerates harder than this
        accel = np.linalg.norm(self.state[4:6])
        if accel > 2.5:
            self.state[4:6] *= 2.5 / accel

    def surprise(self, measured_pos, meas_cov):
        """How unlikely a measurement is given our estimate (squared Mahalanobis distance).
        Small = consistent; large = probably a bad measurement."""
        S = self.H @ self.P @ self.H.T + meas_cov
        diff = measured_pos - self.state[:2]
        return float(diff @ np.linalg.solve(S, diff))

    def correct(self, measured_pos, meas_cov):
        """Blend a position measurement into the estimate (standard Kalman update)."""
        S = self.H @ self.P @ self.H.T + meas_cov
        gain = self.P @ self.H.T @ np.linalg.inv(S)
        self.state = self.state + gain @ (measured_pos - self.state[:2])
        self.P = (np.eye(6) - gain @ self.H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)          # keep it symmetric
        self.time_since_seen = 0.0

    def position_uncertainty(self):
        """1-sigma position uncertainty in the worst direction [m]."""
        return math.sqrt(max(np.linalg.eigvalsh(self.P[:2, :2])))


# =============================================================================
# 3a. MARKER FINDER: plain ArUco detection (works while the whole marker is visible)
# =============================================================================

class ArucoFinder:
    """Finds the landing marker with OpenCV ArUco, whatever its ID is.

    All 5x5 dictionaries (50/100/250/1000) are the start of DICT_5X5_1000, so
    searching DICT_5X5_1000 accepts every ID the arena can make. Other marker
    families are only tried if nothing has been found for a while.
    After the first detection we stick to that dictionary and that ID.
    """

    DICTIONARY_NAMES = ["DICT_5X5_1000", "DICT_4X4_1000", "DICT_6X6_1000",
                        "DICT_7X7_1000", "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_36h11"]

    def __init__(self):
        self.dictionaries = []      # list of (name, dictionary, detect_function)
        for name in self.DICTIONARY_NAMES:
            if hasattr(cv2.aruco, name):
                dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
                self.dictionaries.append((name, dictionary, self._make_detector(dictionary)))

        self.chosen = None          # the (name, dictionary, detect_function) that worked
        self.marker_id = None       # the ID we locked on to
        self.next_fallback = 1
        self.frames_without_marker = 0

    @staticmethod
    def _make_detector(dictionary):
        """Build a detect(gray) function that works on old and new OpenCV versions."""
        try:
            params = cv2.aruco.DetectorParameters()
        except AttributeError:
            params = cv2.aruco.DetectorParameters_create()

        settings = {
            "cornerRefinementMethod": getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", 1),
            "minMarkerPerimeterRate": 0.02,
            "maxMarkerPerimeterRate": 4.0,
            "adaptiveThreshWinSizeMax": 33,
            "minDistanceToBorder": 1,
        }
        for key, value in settings.items():
            if hasattr(params, key):
                try:
                    setattr(params, key, value)
                except Exception:
                    pass

        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            return lambda gray: detector.detectMarkers(gray)
        return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    def find(self, gray):
        """Return (corners as 4x2 array, marker id), or None if not found."""
        if self.chosen is not None:
            to_try = [self.chosen]
        else:
            to_try = [self.dictionaries[0]]
            # Nothing found for a while: also try one of the other families
            if self.frames_without_marker > 30 and len(self.dictionaries) > 1:
                to_try.append(self.dictionaries[self.next_fallback])
                self.next_fallback = 1 + self.next_fallback % (len(self.dictionaries) - 1)

        for entry in to_try:
            _, _, detect = entry
            try:
                all_corners, ids, _ = detect(gray)
            except Exception:
                continue
            if ids is None or len(ids) == 0:
                continue

            # Pick the best marker: our locked ID first, otherwise the biggest one
            best = None
            for corners, marker_id in zip(all_corners, np.asarray(ids).reshape(-1)):
                corners = np.asarray(corners, np.float32).reshape(4, 2)
                area = abs(cv2.contourArea(corners))
                if area < 30:
                    continue
                is_ours = self.marker_id is not None and marker_id == self.marker_id
                score = area + (1e7 if is_ours else 0)
                if best is None or score > best[0]:
                    best = (score, corners, int(marker_id))

            if best is None:
                continue
            if self.marker_id is not None and best[2] != self.marker_id:
                continue                    # some other marker, not ours

            if self.chosen is None:
                self.chosen = entry
            self.frames_without_marker = 0
            return best[1], best[2]

        self.frames_without_marker += 1
        return None


# =============================================================================
# 3b. TEMPLATE MATCHER: tracks the marker when it is too close for ArUco
# =============================================================================

class TemplateMatcher:
    """Draws the expected camera view of the marker and matches it to the real image.

    Once ArUco has read the marker, we know its exact black/white pattern.
    We also know exactly where the drone is and how it is tilted, so for any
    guess of the rover's position we can draw the picture the camera should
    see. Sliding that drawing over the real image and finding the best match
    tells us how wrong the guess was.
    """

    PX_PER_CELL = 24              # resolution of the drawn marker
    GROUND_BORDER_CELLS = 3.0     # ground drawn around the rover, in marker cells

    def __init__(self):
        self.image = None             # drawn marker + white pad + ground (grayscale)
        self.image_to_cells = None    # 3x3: drawing pixel -> marker cell (column, row)
        self.cells = 7                # marker width in cells (5 data bits + 2 border)
        self.cell_size = MARKER_SIZE / 7.0
        self.heading = None           # marker rotation on the ground, as a unit complex number
        self.mirror = -1.0            # -1 because image rows point the opposite way to world y
        self.ground_gray = 150.0      # how bright the floor looks

    @property
    def ready(self):
        return self.image is not None

    # ---- set-up (done once, from the first good ArUco detection) ----

    def build(self, dictionary, marker_id, gray, corners):
        """Draw the marker (with pad and floor) using brightness levels from the real image."""
        self.cells = int(getattr(dictionary, "markerSize", 5)) + 2
        self.cell_size = MARKER_SIZE / self.cells
        try:
            bits = cv2.aruco.generateImageMarker(dictionary, marker_id, self.cells)
        except AttributeError:
            bits = cv2.aruco.drawMarker(dictionary, marker_id, self.cells)

        k = self.PX_PER_CELL
        white_border = (PAD_SIZE - MARKER_SIZE) / 2.0 / self.cell_size   # in cells
        marker_start = self.GROUND_BORDER_CELLS + white_border           # in cells
        size = int(round((self.cells + 2 * marker_start) * k))

        # Floor brightness: look at the image well outside the marker
        outside = np.full(gray.shape, 255, np.uint8)
        centre = corners.mean(0)
        cv2.fillConvexPoly(outside, ((corners - centre) * 1.4 + centre).astype(np.int32), 0)
        if np.count_nonzero(outside) > 500:
            self.ground_gray = float(np.median(gray[outside > 0]))

        # Black and white levels: look inside the marker
        inside = np.zeros(gray.shape, np.uint8)
        cv2.fillConvexPoly(inside, corners.astype(np.int32), 255)
        values = gray[inside > 0]
        if len(values) > 50:
            dark, light = np.percentile(values, 10), np.percentile(values, 90)
        else:
            dark, light = 25, 255
        if light - dark < 40:           # not enough contrast to trust
            dark, light = 25.0, 255.0

        # Paint: floor, then white pad, then the marker pattern
        drawing = np.full((size, size), self.ground_gray, np.float32)
        pad_start = int(round(self.GROUND_BORDER_CELLS * k))
        pad_end = size - pad_start
        drawing[pad_start:pad_end, pad_start:pad_end] = light

        pattern = cv2.resize(bits, (self.cells * k, self.cells * k), interpolation=cv2.INTER_NEAREST)
        m0 = int(round(marker_start * k))
        drawing[m0:m0 + self.cells * k, m0:m0 + self.cells * k] = \
            np.where(pattern > 127, light, dark).astype(np.float32)

        self.image = cv2.GaussianBlur(drawing, (0, 0), 0.6).astype(np.uint8)
        self.image_to_cells = np.array([[1.0 / k, 0, -marker_start + 0.5 / k],
                                        [0, 1.0 / k, -marker_start + 0.5 / k],
                                        [0, 0, 1]])

    def update_heading(self, deck_corners):
        """Learn how the marker is rotated on the ground from its 4 corners (world x, y)."""
        n = self.cells
        cell_corners = np.array([[0, 0], [n, 0], [n, n], [0, n]], float) - n / 2.0
        world_corners = deck_corners - deck_corners.mean(0)

        # Best-fit 2x2 matrix: marker cells -> world
        A = (world_corners.T @ cell_corners) @ np.linalg.inv(cell_corners.T @ cell_corners)
        mirror = -1.0 if np.linalg.det(A) < 0 else 1.0
        B = A @ np.diag([1.0, mirror])
        angle = math.atan2(B[1, 0] - B[0, 1], B[0, 0] + B[1, 1])

        # Smooth the angle over detections (averaging unit complex numbers
        # avoids problems at +/-180 degrees)
        new = complex(math.cos(angle), math.sin(angle))
        self.heading = new if self.heading is None else 0.8 * self.heading + 0.2 * new
        self.mirror = mirror

    def cells_to_world(self, marker_centre):
        """3x3 matrix: marker cell (column, row) -> world (x, y), for a given marker centre."""
        angle = math.atan2(self.heading.imag, self.heading.real)
        c, s = math.cos(angle), math.sin(angle)
        A = self.cell_size * np.array([[c, -s], [s, c]]) @ np.diag([1.0, self.mirror])
        offset = np.asarray(marker_centre) - A @ np.array([self.cells / 2.0, self.cells / 2.0])
        return np.array([[A[0, 0], A[0, 1], offset[0]],
                         [A[1, 0], A[1, 1], offset[1]],
                         [0, 0, 1]])

    # ---- the actual matching (every frame at low altitude) ----

    def match(self, gray, camera, guess, search_radius):
        """Find the marker centre near `guess` (world x, y).

        Returns (measured centre, 2x2 covariance, match score) or None.
        search_radius: how far from the guess we look [m].
        """
        if self.image is None or camera.distance < 0.17:
            return None

        # Shrink the images so the search is never more than ~10 pixels each way
        radius_px = FOCAL * search_radius / camera.distance
        scale = float(np.clip(10.0 / max(radius_px, 1e-3), 0.15, 1.0))
        small_w, small_h = int(round(IMG_W * scale)), int(round(IMG_H * scale))
        margin = int(math.ceil(radius_px * scale)) + 2

        # Drawing pixel -> marker cell -> world -> camera pixel -> shrunk pixel -> padded pixel
        shrink = np.array([[scale, 0, 0.5 * scale - 0.5],
                           [0, scale, 0.5 * scale - 0.5],
                           [0, 0, 1]])
        pad = np.array([[1, 0, margin], [0, 1, margin], [0, 0, 1]], float)
        H = pad @ shrink @ camera.deck_to_image @ self.cells_to_world(guess) @ self.image_to_cells
        if abs(H[2, 2]) < 1e-12:
            return None

        # What the camera should see if the guess were right (plus a margin on every side)
        expected = cv2.warpPerspective(self.image, H / H[2, 2],
                                       (small_w + 2 * margin, small_h + 2 * margin),
                                       flags=cv2.INTER_LINEAR,
                                       borderMode=cv2.BORDER_CONSTANT,
                                       borderValue=int(self.ground_gray))
        actual = cv2.resize(gray, (small_w, small_h), interpolation=cv2.INTER_AREA)
        if actual.std() < 6.0 or expected.std() < 6.0:
            return None                   # plain white or plain black view: nothing to match

        # Slide the real image over the expected one
        scores = cv2.matchTemplate(expected, actual, cv2.TM_CCOEFF_NORMED)
        _, best_score, _, (bx, by) = cv2.minMaxLoc(scores)
        on_edge = bx < 1 or by < 1 or bx > scores.shape[1] - 2 or by > scores.shape[0] - 2
        if best_score < 0.55 or on_edge:
            return None

        # Refine the peak to a fraction of a pixel by fitting a bowl to the
        # 3x3 scores around it. The bowl's curvature also tells us how sharp
        # the match is in each direction.
        s3 = scores[by - 1:by + 2, bx - 1:bx + 2].astype(float)
        dxx = s3[1, 2] - 2 * s3[1, 1] + s3[1, 0]
        dyy = s3[2, 1] - 2 * s3[1, 1] + s3[0, 1]
        dxy = 0.25 * (s3[2, 2] - s3[2, 0] - s3[0, 2] + s3[0, 0])
        curvature = -np.array([[dxx, dxy], [dxy, dyy]])
        slope = np.array([0.5 * (s3[1, 2] - s3[1, 0]), 0.5 * (s3[2, 1] - s3[0, 1])])
        try:
            fine = np.linalg.solve(curvature + 1e-6 * np.eye(2), slope)
        except np.linalg.LinAlgError:
            fine = np.zeros(2)
        fine = np.clip(fine, -1.0, 1.0)

        # How far the real marker is from where we drew it, in full-size pixels
        shift_u = (bx + fine[0] - margin) / scale
        shift_v = (by + fine[1] - margin) / scale

        # Turn the pixel shift into a shift on the deck
        at_centre = camera.pixel_to_deck(CX, CY)
        at_shift = camera.pixel_to_deck(CX + shift_u, CY + shift_v)
        one_px_right = camera.pixel_to_deck(CX + shift_u + 1.0 / scale, CY + shift_v)
        one_px_down = camera.pixel_to_deck(CX + shift_u, CY + shift_v + 1.0 / scale)
        if at_centre is None or at_shift is None or one_px_right is None or one_px_down is None:
            return None
        measured = np.asarray(guess) + (at_centre - at_shift)

        # How much to trust it: about 0.35 px in the sharpest direction. In a
        # direction where the match is flat (e.g. the view only shows one
        # straight edge) we trust it much less.
        strengths, directions = np.linalg.eigh(curvature)
        sharpest = max(strengths[1], 1e-4)
        sigmas_px = []
        for strength in strengths:
            ratio = sharpest / max(strength, sharpest * 1e-4)
            sigmas_px.append(min(0.35 * math.sqrt(ratio), 40.0))
        cov_px = directions @ np.diag(np.square(sigmas_px)) @ directions.T
        px_to_m = np.column_stack([one_px_right - at_shift, one_px_down - at_shift])
        cov = px_to_m @ cov_px @ px_to_m.T
        cov += np.eye(2) * (0.003 ** 2 + ((1.0 - best_score) * 0.05) ** 2)
        return measured, cov, best_score


# =============================================================================
# 4. WIND DETECTOR
# =============================================================================

class WindDetector:
    """Measures any outside force on the drone and decides if there is wind.

    PyBullet updates velocity as  v_new = v_old + dt * (total force) / m.
    We know our own thrust and gravity, so whatever is left over is wind:

        wind = m * (v_new - v_old) / dt  -  (our thrust)  -  (gravity)

    With no wind this stays at about zero, so the same code works in both arenas.
    """

    def __init__(self):
        self.force = np.zeros(3)         # smoothed wind force, world frame [N]
        self.level = 0.0                 # slow average of the sideways part [N]
        self.detected = False
        self.ever_detected = False
        self.prev_velocity = None
        self.prev_thrust = None          # thrust vector we applied last step [N]

    def update(self, velocity, z, log):
        if self.prev_velocity is not None and self.prev_thrust is not None:
            weight = np.array([0.0, 0.0, -MASS * GRAVITY])
            sample = MASS * (velocity - self.prev_velocity) / DT - self.prev_thrust - weight

            # Near the deck the rover pushes on us - that is not wind, so skip it
            if z > TOUCH_Z + 0.06:
                sample = np.clip(sample, -6.0, 6.0)
                self.force += (DT / 0.05) * (sample - self.force)     # ~50 ms smoothing
                sideways = np.linalg.norm(self.force[:2])
                self.level += (DT / 1.0) * (sideways - self.level)   # ~1 s average
        self.prev_velocity = velocity.copy()

        # Switch modes with a gap between the on and off levels, so it doesn't flicker
        if not self.detected and self.level > WIND_ON_N:
            self.detected = True
            self.ever_detected = True
            log(f"WIND DETECTED (~{self.level:.2f} N) -> disturbance "
                f"compensation + extra tilt authority")
        elif self.detected and self.level < WIND_OFF_N:
            self.detected = False
            log("Wind has died down -> normal mode")


# =============================================================================
# 5. THE FLIGHT CONTROLLER (the class the arena uses)
# =============================================================================

class FlightController:

    def __init__(self):
        self.aruco = ArucoFinder()
        self.template = TemplateMatcher()
        self.rover = RoverTracker()
        self.wind = WindDetector()

        self.mode = "SEARCH"
        self.height_goal = CRUISE_Z   # height to hold while in TRACK
        self.aligned_time = 0.0       # how long tracking has been good enough to descend

        # Smooth height reference that the altitude controller follows
        self.z_ref = None
        self.vz_ref = 0.0

        self.prev_R = None            # attitude last step (to get rotation speed)
        self.time = 0.0
        self.seen_by = "-"            # what found the rover this step: "aruco", "model" or "-"
        self.last_corners = None      # ArUco corners, for drawing

        self.debug_view = True        # draw the overlay in the camera window
        self.verbose = True           # print status to the terminal
        self._print_timer = 0.0

    # These two are read by the test script
    @property
    def wind_detected(self):
        return self.wind.detected

    @property
    def wind_ever_detected(self):
        return self.wind.ever_detected

    def log(self, message):
        if self.verbose:
            print(f"[{self.time:6.2f}s] {message}")

    # -------------------------------------------------------------------------
    # Called by the arena every step
    # -------------------------------------------------------------------------
    def calculate_forces(self, camera_frame, drone_pos, drone_ori, drone_vel):
        self.time += DT
        pos = np.asarray(drone_pos, float)
        vel = np.asarray(drone_vel, float)
        R = quaternion_to_matrix(drone_ori)
        z = pos[2]
        if self.z_ref is None:
            self.z_ref = z

        if self.mode == "CUTOFF":
            return 0.0, 0.0, 0.0, 0.0

        spin = self._rotation_speed(R)

        # Steps 1-3: wind, where is the rover, update the tracker
        self.wind.update(vel, z, self.log)
        self.rover.predict()
        camera = DownwardCamera(pos, R)
        if camera_frame is not None and z > VISION_FLOOR_Z - 0.02:
            self._look_for_rover(camera_frame, camera)

        # How far off are we?
        have_rover = self.rover.has_estimate and self.rover.time_since_seen < 2.5
        if have_rover:
            pos_error = self.rover.position - pos[:2]      # rover minus drone
            vel_error = self.rover.velocity - vel[:2]
            rover_accel = self.rover.acceleration
        else:
            pos_error = vel_error = np.zeros(2)
            rover_accel = np.zeros(2)
        distance = float(np.linalg.norm(pos_error))
        speed_diff = float(np.linalg.norm(vel_error))
        tilt = math.acos(np.clip(R[2, 2], -1, 1))

        # Step 4: phase and height plan
        self._update_mode(z, have_rover, distance, speed_diff)
        self._update_height_reference(z, distance, speed_diff)

        # Touchdown?
        if self.mode == "FINAL":
            on_deck = z < CUT_Z or (z < TOUCH_Z + 0.03 and vel[2] > -0.05)
            if on_deck and distance < 0.35:
                self.mode = "CUTOFF"
                self.log(f"TOUCHDOWN -> MOTOR CUT-OFF  z={z:.3f} vz={vel[2]:.2f} "
                         f"err={distance:.3f} tilt={math.degrees(tilt):.1f}deg")
                return 0.0, 0.0, 0.0, 0.0

        # Step 5: fly
        wanted_accel = self._wanted_acceleration(z, vel, have_rover, pos_error, vel_error, rover_accel)
        forces = self._acceleration_to_motor_forces(wanted_accel, R, spin)
        self.wind.prev_thrust = R[:, 2] * sum(forces)

        self._print_status(z, distance, speed_diff, tilt)
        if self.debug_view and camera_frame is not None:
            self._draw_overlay(camera_frame, camera, distance)
        return forces

    # -------------------------------------------------------------------------
    # Step 2-3: vision -> rover tracker
    # -------------------------------------------------------------------------
    def _look_for_rover(self, frame, camera):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        found_by = None

        # a) ArUco (whole marker visible)
        result = self.aruco.find(gray)
        if result is not None:
            corners, marker_id = result
            deck_corners = [camera.pixel_to_deck(u, v) for u, v in corners]
            if all(c is not None for c in deck_corners):
                deck_corners = np.array(deck_corners)
                centre = deck_corners.mean(0)
                cut_off_by_edge = (corners.min() < 2 or corners[:, 0].max() > IMG_W - 3
                                   or corners[:, 1].max() > IMG_H - 3)

                # Only learn from markers that are fully inside the image
                if not cut_off_by_edge:
                    self.template.update_heading(deck_corners)
                    if not self.template.ready:
                        self.template.build(self.aruco.chosen[1], marker_id, gray, corners)
                        self.aruco.marker_id = marker_id     # from now on, only this marker
                        self.log(f"Marker acquired: {self.aruco.chosen[0]} id={marker_id}")

                # ArUco gets less precise the further away we are
                sigma = 0.004 + 0.006 * camera.distance
                meas_cov = np.eye(2) * sigma ** 2
                if not self.rover.has_estimate or self.rover.time_since_seen > 1.5:
                    self.rover.start_at(centre)
                    found_by = "aruco"
                elif self.rover.surprise(centre, meas_cov) < 40.0:
                    self.rover.correct(centre, meas_cov)
                    found_by = "aruco"
                self.last_corners = corners

        # b) Template matching (marker too close / partly outside the image)
        if found_by is None and self.rover.has_estimate and self.template.ready:
            # Search a bit wider than our uncertainty, but not so wide that we
            # could match the wrong square of the pattern
            max_radius = 0.05 if camera.distance < 0.8 else 0.20
            radius = float(np.clip(3.0 * self.rover.position_uncertainty(), 0.025, max_radius))
            match = self.template.match(gray, camera, self.rover.position.copy(), radius)
            if match is not None:
                measured, meas_cov, _ = match
                if self.rover.surprise(measured, meas_cov) < 25.0:
                    self.rover.correct(measured, meas_cov)
                    found_by = "model"
            if found_by is None:
                self.last_corners = None

        self.seen_by = found_by or "-"

    # -------------------------------------------------------------------------
    # Step 4: phases
    # -------------------------------------------------------------------------
    def _update_mode(self, z, have_rover, distance, speed_diff):
        old_mode = self.mode
        blind_for = self.rover.time_since_seen

        if self.mode == "SEARCH":
            if have_rover and blind_for < 0.1:
                self.mode = "TRACK"
                self.height_goal = float(np.clip(z, 1.2, CRUISE_Z))
                self.aligned_time = 0.0

        elif self.mode == "TRACK":
            if not have_rover:
                self.mode = "SEARCH"
            # Descend once we have been right above the rover, at its speed, for 0.6 s
            aligned = distance < 0.12 and speed_diff < 0.25 and blind_for < 0.1
            if aligned:
                self.aligned_time += DT
            else:
                self.aligned_time = max(0.0, self.aligned_time - 2 * DT)
            if self.aligned_time > 0.6:
                self.mode = "DESCEND"

        elif self.mode == "DESCEND":
            if z < VISION_FLOOR_Z + 0.03:
                # Camera is about to become useless: commit or retry
                if distance < 0.15 and speed_diff < 0.35:
                    self.mode = "FINAL"
                elif distance > 0.25:
                    self.mode = "TRACK"              # badly off: climb back and retry
                    self.height_goal = 1.0
            elif blind_for > 1.2:
                self.mode = "TRACK"                  # lost the marker for too long
                self.height_goal = max(z + 0.5, 1.5)

        elif self.mode == "FINAL":
            if distance > 0.32:                      # drifted off the pad: abort
                self.mode = "TRACK"
                self.height_goal = 1.0

        if self.mode != old_mode:
            self.log(f"{old_mode} -> {self.mode}  z={z:.2f} err={distance:.3f} relv={speed_diff:.2f}")

    def _update_height_reference(self, z, distance, speed_diff):
        """Move the smooth height reference (z_ref, vz_ref) according to the phase."""
        blind_for = self.rover.time_since_seen

        if self.mode == "SEARCH":
            wanted_vz = float(np.clip(1.2 * (SEARCH_Z - self.z_ref), -0.5, 0.5))

        elif self.mode == "TRACK":
            wanted_vz = float(np.clip(1.2 * (self.height_goal - self.z_ref), -0.4, 0.6))

        elif self.mode == "DESCEND":
            height = z - TOUCH_Z
            if height > 1.6:
                sink = 0.9
            elif height > 0.8:
                sink = 0.6
            else:
                sink = 0.4
            # Only go down while we are well lined up; slow down if not
            if distance > 0.25 or speed_diff > 0.6 or blind_for > 0.4:
                sink = 0.0
            elif distance > 0.12 or speed_diff > 0.35 or blind_for > 0.15:
                sink *= 0.4
            wanted_vz = -sink

        else:   # FINAL: gentle, constant sink until we touch
            wanted_vz = -0.35

        # Change vertical speed gradually (max 1.2 m/s^2) for a smooth profile
        max_change = 1.2 * DT
        self.vz_ref += float(np.clip(wanted_vz - self.vz_ref, -max_change, max_change))
        self.z_ref += self.vz_ref * DT

        if self.mode in ("DESCEND", "FINAL"):
            self.z_ref = max(self.z_ref, TOUCH_Z - 0.15)
            # Don't let the reference run far away from where the drone really is
            self.z_ref = float(np.clip(self.z_ref, z - 0.25, z + 0.25))

    # -------------------------------------------------------------------------
    # Step 5: control
    # -------------------------------------------------------------------------
    def _wanted_acceleration(self, z, vel, have_rover, pos_error, vel_error, rover_accel):
        """The acceleration (world x, y, z) we want the drone to have."""

        # --- Sideways ---
        if have_rover and self.mode != "SEARCH":
            # Follow the rover: copy its acceleration, plus corrections for
            # position and speed differences. Stiffer when low and close.
            if self.mode == "FINAL" or z < 1.0:
                kp, kd, max_tilt = 7.0, 5.0, math.radians(15)
            else:
                kp, kd, max_tilt = 4.0, 4.2, math.radians(22)
            if self.wind.detected:
                max_tilt += math.radians(8)     # room to lean into ~2 N gusts

            # Don't overreact to a far-away rover
            capped_error = pos_error.copy()
            if np.linalg.norm(capped_error) > 1.5:
                capped_error *= 1.5 / np.linalg.norm(capped_error)
            accel_xy = rover_accel + kp * capped_error + kd * vel_error
        else:
            # Nothing to follow: just stop moving
            max_tilt = math.radians(20) if self.wind.detected else math.radians(10)
            accel_xy = -1.5 * vel[:2]

        # Push back against the wind (this is zero when there is no wind)
        accel_xy = accel_xy - self.wind.force[:2] / MASS

        # Limit how far we lean
        max_accel = GRAVITY * math.tan(max_tilt)
        if np.linalg.norm(accel_xy) > max_accel:
            accel_xy *= max_accel / np.linalg.norm(accel_xy)

        # --- Up/down: follow the height reference, cancel vertical wind ---
        accel_z = (16.0 * (self.z_ref - z) + 8.0 * (self.vz_ref - vel[2])
                   - self.wind.force[2] / MASS)
        accel_z = float(np.clip(accel_z, -6.0, 8.0))

        return np.array([accel_xy[0], accel_xy[1], accel_z])

    def _acceleration_to_motor_forces(self, accel, R, spin):
        """Tilt the drone so its thrust points the right way, then split thrust over 4 motors."""
        wanted_force = MASS * (accel + np.array([0.0, 0.0, GRAVITY]))

        # Wanted attitude: thrust axis along wanted_force, keep the current heading (yaw)
        up = wanted_force / np.linalg.norm(wanted_force)
        yaw = math.atan2(R[1, 0], R[0, 0])
        heading = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        left = np.cross(up, heading)
        left /= np.linalg.norm(left)
        forward = np.cross(left, up)
        R_wanted = np.column_stack([forward, left, up])

        # Attitude error and a PD torque (like a spring + damper, 14 rad/s)
        angle_error = 0.5 * vee(R_wanted.T @ R - R.T @ R_wanted)
        inertia = 0.018
        natural_freq = 14.0
        torque = (-inertia * natural_freq ** 2 * angle_error
                  - inertia * 2.0 * 0.9 * natural_freq * spin)
        torque_x = float(np.clip(torque[0], -1.2, 1.2))
        torque_y = float(np.clip(torque[1], -1.2, 1.2))
        # (no yaw torque: the arena's motors can only push straight up)

        # Only the part of the wanted force along our current up-axis is useful
        total_thrust = max(0.0, float(wanted_force @ R[:, 2]))
        return self._split_between_motors(total_thrust, torque_x, torque_y)

    @staticmethod
    def _split_between_motors(total, torque_x, torque_y):
        """Motor positions:  f1 (+L,+L)   f2 (-L,+L)   f3 (+L,-L)   f4 (-L,-L)

        Roll torque  = L * (f1 + f2 - f3 - f4)
        Pitch torque = L * (-f1 + f2 - f3 + f4)
        """
        base = total / 4.0
        roll = torque_x / (4 * ARM)
        pitch = torque_y / (4 * ARM)
        forces = [base + roll - pitch,
                  base + roll + pitch,
                  base - roll - pitch,
                  base - roll + pitch]
        return tuple(max(0.0, f) for f in forces)      # motors can't pull

    def _rotation_speed(self, R):
        """Body rotation rates [rad/s] from this step's and last step's attitude."""
        if self.prev_R is None:
            spin = np.zeros(3)
        else:
            change = self.prev_R.T @ R
            spin = vee(0.5 * (change - change.T)) / DT
        self.prev_R = R
        return spin

    # -------------------------------------------------------------------------
    # Status output
    # -------------------------------------------------------------------------
    def _print_status(self, z, distance, speed_diff, tilt):
        if not self.verbose:
            return
        self._print_timer += DT
        if self._print_timer < 0.5:
            return
        self._print_timer = 0.0
        w = self.wind.force
        print(f"[{self.time:6.2f}s] {self.mode:7s} z={z:5.2f} err={distance:.3f} "
              f"relv={speed_diff:.2f} src={self.seen_by:5s} "
              f"stale={min(self.rover.time_since_seen, 9):.2f} "
              f"tilt={math.degrees(tilt):4.1f}")

    def _draw_overlay(self, frame, camera, distance):
        try:
            img = frame.copy()
            cv2.drawMarker(img, (int(CX), int(CY)), (255, 0, 0), cv2.MARKER_CROSS, 12, 1)

            if self.last_corners is not None and self.seen_by == "aruco":
                cv2.polylines(img, [self.last_corners.astype(np.int32)], True, (0, 255, 0), 2)

            # Where we think the rover is: red = seen this frame, orange = predicted
            if self.rover.has_estimate:
                p = camera.deck_to_pixel(self.rover.position)
                if p is not None and abs(p[0]) < 5000 and abs(p[1]) < 5000:
                    colour = (0, 0, 255) if self.seen_by != "-" else (0, 165, 255)
                    cv2.circle(img, (int(p[0]), int(p[1])), 6, colour, 2)

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, f"{self.mode} src={self.seen_by} err={distance:.2f}",
                        (4, 14), font, 0.42, (0, 255, 255), 1)
            if self.wind.detected:
                cv2.putText(img, f"WIND {np.linalg.norm(self.wind.force[:2]):.1f}N",
                            (220, 30), font, 0.42, (0, 0, 255), 1)
            if self.aruco.marker_id is not None:
                cv2.putText(img, f"id={self.aruco.marker_id}", (4, 30), font, 0.42, (0, 255, 255), 1)

            cv2.imshow("Drone Downward Camera", img)
            cv2.waitKey(1)
        except Exception:
            self.debug_view = False