import os
import sys
import traci

# Verify SUMO environment path pointers
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("CRITICAL ERROR: Please configure your SUMO_HOME environment pointer.")


def run_simulation():
    print("\n>>> ACTIVE FAULT-TOLERANT CONTROLLER INTERFACE RUNNING...")
    print("----------------------------------------------------------------")

    sensor_ids = ["crest_sensor_lane_0", "crest_sensor_lane_1"]
    # Physical distance from crest peak to light (meters)
    distance_to_light = 40.0
    # Safe time window needed for a sedan to complete its left turn (seconds)
    clearance_window_threshold = 4.5

    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # Loop through both lane sensors at the top of the hill crest
        for sensor in sensor_ids:
            # Query the sensor: Count how many vehicles are currently passing over it
            vehicle_count = traci.inductionloop.getLastStepVehicleNumber(
                sensor)

            if vehicle_count > 0:
                # Get the specific list of vehicle IDs passing over the sensor loop
                veh_ids = traci.inductionloop.getLastStepVehicleIDs(sensor)
                for veh_id in veh_ids:
                    # 1. Read the raw metric speed from the vehicle (meters per second)
                    raw_speed_ms = traci.vehicle.getSpeed(veh_id)

                    # Change your existing radar reporting conditional checks to filter zero-values:
                    if raw_speed_ms > 0.1:  # Only track moving vehicles
                        tti = distance_to_light / raw_speed_ms
                        speed_mph = raw_speed_ms * 2.23694

                        # Only report vehicles actively moving down the hill
                        print(
                            f"[RADAR HIT] Vehicle ID: '{veh_id}' | Speed: {speed_mph:.1f} MPH | Time-To-Intersection: {tti:.2f}s")

                        if tti <= clearance_window_threshold:
                            print(
                                f" [!!!] SAFETY OVERRIDE TRIGGERED against '{veh_id}'!")
                            traci.trafficlight.setRedYellowGreenState(
                                "center", "rrrrrrrrrrrrrrrrrrrr")
                    else:
                        # Optional console log for queue monitoring
                        if step % 200 == 0:
                            print(
                                f"[QUEUE MONITOR] Vehicle '{veh_id}' is stationary in the summit queue spillback zone.")

        step += 1

    traci.close()
    print("\n>>> SIMULATION COMPLETE. TELEMETRY BUFFER DISPATCHED.")


if __name__ == "__main__":
    # Standard backend binary launcher configuration
    sumo_binary = os.path.join(os.environ['SUMO_HOME'], 'bin', 'sumo')
    sumo_cmd = [sumo_binary, "-c", "simulation.sumocfg", "--no-warnings"]

    traci.start(sumo_cmd)
    run_simulation()
