import math
import time


def play_in_simulator(settings, report):
    if report.get("status") != "success" or report.get("valid_ratio") != 1.0:
        raise RuntimeError("Simulator playback requires successful path validation")
    from .connection import connect_existing
    from .json_program import OWNER, OWNER_PARAM
    from robodk.robolink import ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT, PROGRAM_RUN_ON_SIMULATOR, RUNMODE_SIMULATE, COLLISION_ON
    from robodk.robomath import invH, Mat
    rdk = connect_existing()
    original_speed = None
    try:
        from .station import check_station
        check_station(rdk, settings)
        program = rdk.Item(report["program"], ITEM_TYPE_PROGRAM)
        if not program.Valid() or program.getParam(OWNER_PARAM) != OWNER:
            raise RuntimeError("Owned simulator program is missing")
        if program.getParam("DrawBotGeneration") != report["generation_id"].encode():
            raise RuntimeError("Playback validation belongs to another generation")
        rdk.setRunMode(RUNMODE_SIMULATE)
        program.setRunType(PROGRAM_RUN_ON_SIMULATOR)
        valid, estimate, distance, ratio, message = program.Update(
            COLLISION_ON, timeout_sec=120, mm_step=1., deg_step=1.)
        if ratio != 1 or valid < program.InstructionCount():
            raise RuntimeError(f"Current-start playback path check failed: {message}")
        original_speed = rdk.SimulationSpeed()
        rdk.setSimulationSpeed(10)
        started = time.monotonic()
        program.RunCode()
        time.sleep(.2)
        while program.Busy():
            if time.monotonic()-started > max(60, estimate*2):
                program.Stop()
                raise RuntimeError("Simulator playback timed out")
            time.sleep(.1)
        robot = rdk.Item(settings["robot_name"], ITEM_TYPE_ROBOT)
        calibration = report["station_calibration"]
        actual = (invH(Mat(calibration["reference_frame_pose_robot"]))
                  * robot.SolveFK(robot.Joints()) * Mat(calibration["pen_tcp_pose"]))
        expected = report["motion_manifest"][-1]["xyz_mm"]
        endpoint_error = math.dist(actual.Pos(), expected)
        if endpoint_error > .01:
            raise RuntimeError(f"Simulator did not reach final Retract: {endpoint_error} mm")
        return {"success": True, "program": report["program"], "simulation_speed": 10,
                "playback_wall_time_s": time.monotonic()-started,
                "current_start_valid_ratio": ratio, "current_start_estimate_s": estimate,
                "current_start_distance_mm": distance, "message": message,
                "final_target": report["motion_manifest"][-1]["target"],
                "final_xyz_mm": actual.Pos(), "endpoint_error_mm": endpoint_error,
                "physical_robot_used": False}
    finally:
        if original_speed is not None:
            rdk.setSimulationSpeed(original_speed)
        rdk.Disconnect()
