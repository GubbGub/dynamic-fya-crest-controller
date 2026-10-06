import math

class BlindCrestController:
    def __init__(self):
        # 1. Establish your real Google Earth & AASHTO Geometric Constants
        self.posted_speed_limit_mph = 45.0
        self.perception_reaction_time_sec = 2.5  # AASHTO Standard
        self.deceleration_rate_ms2 = 3.4         # AASHTO Standard (11.2 ft/s²)
        
        # Distance from the physical blind crest peak to the intersection impact site (meters)
        # Based on your Google Earth delta: ~40 meters from peak to light
        self.distance_crest_to_intersection = 40.0 
        
        # Left-turn clearance threshold (seconds): Time it takes a slow sedan (5mph) to clear the lane
        self.safety_clearance_buffer = 4.5 

    def calculate_aashto_ssd(self, current_speed_mph, grade_percentage):
        """
        Calculates the required Stopping Sight Distance (SSD) in meters 
        accounting for downhill grades based on AASHTO physics equations.
        """
        # Convert mph to meters per second
        v_ms = current_speed_mph * 0.44704
        g = 9.81 # Gravity m/s²
        G = grade_percentage / 100.0 # Convert whole number to decimal
        
        # AASHTO Formula: Brake Distance = v^2 / [2 * g * ((a/g) +/- G)]
        # Simplified engineering version below:
        perception_distance = v_ms * self.perception_reaction_time_sec
        braking_distance = (v_ms ** 2) / (2 * g * ((self.deceleration_rate_ms2 / g) + G))
        
        return perception_distance + braking_distance

    def process_sensor_input(self, detected_speed_mph):
        """
        Executes the Boolean logic to determine if the Flashing Yellow Arrow (FYA)
        must be overridden and forced to a Protected Red Arrow.
        """
        # Convert vehicle speed to meters per second
        v_ms = detected_speed_mph * 0.44704
        
        # Calculate Time-To-Intersection (TTI) from the peak of the crest
        if v_ms > 0:
            tti = self.distance_crest_to_intersection / v_ms
        else:
            tti = 999.0 # Car is stopped
            
        # CORE ALGORITHM EQUATION:
        # If the oncoming car's arrival window is smaller than our safe left-turn window, 
        # the static flashing yellow arrow is structurally deadly. Force an override.
        if tti <= self.safety_clearance_buffer:
            return "OVERRIDE: Force Protected RED Arrow (Danger Imminent)"
        else:
            return "PERMISSIVE: Allow Flashing Yellow Arrow (Safe Gap)"

# --- SIMULATION EXPERIMENTAL TEST RUNS ---
if __name__ == "__main__":
    controller = BlindCrestController()
    
    # Let's test three distinct oncoming vehicle profiles over your hill (-7% downhill grade)
    downhill_grade = -7.0 
    test_scenarios = [
        {"name": "Compliant Sedan", "speed": 45.0},
        {"name": "Your Crash Scenario (Speeding Sierra)", "speed": 60.0},
        {"name": "Ultra-Reckless Speeding", "speed": 70.0}
    ]
    
    print("=== DYNAMIC TRAFFIC SIGNAL CONTROLLER TESTING CRADLE ===")
    for vehicle in test_scenarios:
        ssd_required = controller.calculate_aashto_ssd(vehicle["speed"], downhill_grade)
        signal_status = controller.process_sensor_input(vehicle["speed"])
        
        print(f"\nVehicle Type: {vehicle['name']}")
        print(f" -> Approaching Speed: {vehicle['speed']} MPH")
        print(f" -> Required AASHTO Stopping Distance: {ssd_required:.2f} meters")
        print(f" -> Available Visual Sight Distance over Crest: {controller.distance_crest_to_intersection} meters")
        
        if ssd_required > controller.distance_crest_to_intersection:
            print(" [!] HAZARD DIAGNOSIS: Vehicle cannot physically stop if a car turns. It is blindly trapped.")
            
        print(f" -> CONTROLLER DECISION: {signal_status}")
