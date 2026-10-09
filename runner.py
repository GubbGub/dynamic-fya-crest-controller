import os
import sys
import traci
import csv

# Verify SUMO environment path pointers
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("CRITICAL ERROR: Please configure your SUMO_HOME environment pointer.")

def run_simulation():
    print("\n>>> ACTIVE CONTROLLER RUNNING & DATA LOGGING LOGGING ACTIVE...")
    print("----------------------------------------------------------------")
    
    sensor_ids = ["crest_sensor_lane_0", "crest_sensor_lane_1"]
    distance_to_light = 40.0        
    clearance_window_threshold = 4.5 
    
    # Initialize a list to hold all our statistical rows
    telemetry_data = []

    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        for sensor in sensor_ids:
            vehicle_count = traci.inductionloop.getLastStepVehicleNumber(sensor)
            
            if vehicle_count > 0:
                veh_ids = traci.inductionloop.getLastStepVehicleIDs(sensor)
                for veh_id in veh_ids:
                    raw_speed_ms = traci.vehicle.getSpeed(veh_id)
                    speed_mph = raw_speed_ms * 2.23694
                    
                    # Filter out stationary traffic jams to keep statistics clean
                    if raw_speed_ms > 0.1:
                        tti = distance_to_light / raw_speed_ms
                        
                        # Determine if this specific car would trigger an override
                        override_active = 1 if tti <= clearance_window_threshold else 0
                        
                        # Store data row: [Simulation_Step, Vehicle_ID, Speed_MPH, TTI, Override_Status]
                        telemetry_data.append([step, veh_id, round(speed_mph, 1), round(tti, 2), override_active])
                        
                        if override_active == 1:
                            # Force the signal state to red to shield the turn lane
                            traci.trafficlight.setRedYellowGreenState("center", "rrrrrrrrrrrrrrrrrrrr")
        step += 1

    traci.close()
    
    # Write everything cleanly into a CSV spreadsheet file
    csv_path = "data_outputs/simulation_telemetry.csv"
    with open(csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Step", "Vehicle_ID", "Speed_MPH", "TTI", "Override_Triggered"])
        writer.writerows(telemetry_data)
        
    print(f"\n>>> SUCCESS: {len(telemetry_data)} SPATIAL DATA POINTS EXPORTED TO {csv_path}")

if __name__ == "__main__":
    sumo_binary = os.path.join(os.environ['SUMO_HOME'], 'bin', 'sumo')
    sumo_cmd = [sumo_binary, "-c", "simulation.sumocfg", "--no-warnings"]
    traci.start(sumo_cmd)
    run_simulation()
