"""
test_all_markers.py -- headless batch test of controller.py for many ArUco IDs.

local_arena.py is NOT modified: this script imports its functions and only
(1) switches PyBullet to DIRECT mode (no window), and
(2) swaps the marker ID that spawn_rover() passes to generateImageMarker().
The physics loop below is a copy of local_arena.main() with a time limit.

Usage:
    python test_all_markers.py              # IDs 0..99
    python test_all_markers.py 0 5 42 99    # selected IDs
    python test_all_markers.py --gui 17     # watch one ID in the GUI
"""
import sys, math, time, importlib
import cv2
import pybullet as p

import local_arena as arena

TIME_LIMIT = 60.0      # simulated seconds
VERBOSE = False


def run(marker_id, gui=False, time_offset=0.0):
    orig_gen = cv2.aruco.generateImageMarker
    cv2.aruco.generateImageMarker = lambda d, _id, size, *a, **k: orig_gen(d, marker_id, size, *a, **k)
    try:
        if gui:
            arena.setup_environment()
        else:
            p.connect(p.DIRECT)
            import pybullet_data
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            p.loadURDF("plane.urdf")
        rover_id = arena.spawn_rover()
    finally:
        cv2.aruco.generateImageMarker = orig_gen
    drone_id = arena.spawn_quadcopter()

    import controller
    importlib.reload(controller)
    brain = controller.FlightController()
    brain.debug_view = gui
    brain.verbose = gui or VERBOSE

    scorer = arena.CompetitionScorer()
    result = {"id": marker_id, "ok": False, "msg": "TIMEOUT", "t": TIME_LIMIT}
    orig_fail, orig_succ = scorer._trigger_failure, scorer._trigger_success

    def fail(reason):
        result.update(ok=False, msg=reason, t=scorer.sim_time); orig_fail(reason)

    def succ(err):
        result.update(ok=True, msg=f"err={err:.3f} m", t=scorer.sim_time); orig_succ(err)

    scorer._trigger_failure, scorer._trigger_success = fail, succ

    dt, t = 1.0 / 240.0, time_offset
    while scorer.sim_time < TIME_LIMIT:
        rover_vx = 1.2 * math.cos(t * 0.4) + 0.5 * math.cos(t * 1.1)
        rover_vy = 1.2 * math.sin(t * 0.3) + 0.5 * math.sin(t * 0.9)
        p.resetBaseVelocity(rover_id, linearVelocity=[rover_vx, rover_vy, 0])
        drone_pos, drone_ori = p.getBasePositionAndOrientation(drone_id)
        drone_vel, _ = p.getBaseVelocity(drone_id)
        rover_pos, _ = p.getBasePositionAndOrientation(rover_id)
        frame = arena.get_drone_camera_image(drone_id)
        scorer.update(drone_pos, drone_vel, drone_ori, rover_pos, dt)
        if scorer.run_complete:
            break
        f1, f2, f3, f4 = brain.calculate_forces(frame, drone_pos, drone_ori, drone_vel)
        L = 0.25
        p.applyExternalForce(drone_id, -1, [0, 0, f1], [L, L, 0], p.LINK_FRAME)
        p.applyExternalForce(drone_id, -1, [0, 0, f2], [-L, L, 0], p.LINK_FRAME)
        p.applyExternalForce(drone_id, -1, [0, 0, f3], [L, -L, 0], p.LINK_FRAME)
        p.applyExternalForce(drone_id, -1, [0, 0, f4], [-L, -L, 0], p.LINK_FRAME)
        p.stepSimulation()
        if gui:
            time.sleep(dt)
        t += dt
    p.disconnect()
    return result


if __name__ == "__main__":
    args = sys.argv[1:]
    gui = "--gui" in args
    args = [a for a in args if a != "--gui"]
    if not gui:
        cv2.imshow = lambda *a, **k: None
        cv2.waitKey = lambda *a, **k: -1
        arena.p.connect = p.connect
    ids = [int(a) for a in args] if args else list(range(100))
    results = []
    for mid in ids:
        r = run(mid, gui=gui)
        results.append(r)
        print(f"ID {mid:3d}: {'PASS' if r['ok'] else 'FAIL'}  t={r['t']:5.1f}s  {r['msg']}", flush=True)
    n = sum(r["ok"] for r in results)
    print(f"\n{n}/{len(results)} markers landed successfully")
