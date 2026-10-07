import os
import sys
import traci

# 1. Establish absolute verification pointers for your SUMO_HOME path environment
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("CRITICAL ERROR: Please configure your SUMO_HOME environment pointer.")


def run_simulation():
    """
    Main loop interface commanding the step-by-step physics of your intersection.
    """
    print("\n>>> SIMULATION INTERFACE ACTIVE: CONNECTING TO NETWORK TIMING MATRIX...")

    step = 0
    # Run the traffic physics loop step-by-step (one tick = 0.1 seconds of real life)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # Every 100 steps (10 seconds), let's sample the network telemetry data
        if step % 100 == 0:
            active_vehicles = traci.vehicle.getIDList()
            print(
                f"Simulation Step: {step} | Live Active Vehicles on Grid: {len(active_vehicles)}")

            # Look at what state your main intersection signal is sitting in
            # 'center' is the ID we assigned your traffic light node in your nodes XML file
            signal_state = traci.trafficlight.getRedYellowGreenState("center")
            print(
                f" -> Current Intersection Node Signal Matrix: '{signal_state}'")

        step += 1

    traci.close()
    print(">>> SIMULATION MATRIX TERMINATED: TELEMETRY DATA BUFFERED SUCCESSFULLY.")


if __name__ == "__main__":
    # Change 'sumo-gui' to 'sumo' to run the fast, rock-solid text-based backend
    sumo_binary = os.path.join(os.environ['SUMO_HOME'], 'bin', 'sumo')

    # Structure your master project execution pipeline arguments
    # We add "--no-warnings" to keep your terminal perfectly clean
    sumo_cmd = [sumo_binary, "-c", "simulation.sumocfg", "--no-warnings"]

    # Initialize connection to the traffic software engine
    traci.start(sumo_cmd)

    # Fire up your operational model loop
    run_simulation()
