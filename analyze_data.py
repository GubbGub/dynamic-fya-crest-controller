import pandas as pd
import matplotlib.pyplot as plt
import os

def generate_safety_charts():
    csv_path = "data_outputs/simulation_telemetry.csv"
    
    if not os.path.exists(csv_path):
        print(f"Error: Cannot find {csv_path}. Run runner.py first to generate the data!")
        return
        
    # Read the spreadsheet using pandas
    df = pd.read_csv(csv_path)
    
    print("=== EXECUTING MATHEMATICAL ANALYSIS ON SIMULATION DATA ===")
    total_samples = len(df)
    overrides = df['Override_Triggered'].sum()
    max_speed = df['Speed_MPH'].max()
    avg_speed = df['Speed_MPH'].mean()
    
    print(f"Total Vehicles Tracked over Crest: {total_samples}")
    print(f"High-Risk Structural Conflict Interventions: {overrides}")
    print(f"Average Approach Speed: {avg_speed:.1f} MPH")
    print(f"Maximum Speeding Hazard Captured: {max_speed:.1f} MPH")
    
    # --- CHART GENERATION ---
    plt.figure(figsize=(10, 6))
    
    # Split data into normal vehicles and high-risk overrides
    normal_cars = df[df['Override_Triggered'] == 0]
    override_cars = df[df['Override_Triggered'] == 1]
    
    # Scatter plot mapping Speed vs Time-To-Intersection
    plt.scatter(normal_cars['Speed_MPH'], normal_cars['TTI'], color='blue', alpha=0.5, label='Permissive Phase Intact (Safe)')
    plt.scatter(override_cars['Speed_MPH'], override_cars['TTI'], color='red', alpha=0.7, edgecolors='black', label='Signal Overridden to Red (Hazard Mitigated)')
    
    # Draw the AASHTO clearance threshold line
    plt.axhline(y=4.5, color='darkred', linestyle='--', linewidth=2, label='AASHTO Safety Boundary (4.5s)')
    
    # Label the axes professionally for an academic paper
    plt.title("Dynamic Intersection Controller: Speed vs. Time-To-Intersection (TTI) Matrix", fontsize=12, fontweight='bold')
    plt.xlabel("Detected Oncoming Approach Speed (MPH)", fontsize=11)
    plt.ylabel("Computed Time-To-Intersection (Seconds)", fontsize=11)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend(loc='upper right')
    
    # Save the chart as a high-resolution PNG for your slide deck and paper
    output_image_path = "data_outputs/safety_chart.png"
    plt.savefig(output_image_path, dpi=300)
    plt.close()
    
    print(f"\n>>> ACADEMIC GRAPH GENERATED SUCCESSFULLY: saved to {output_image_path}")

if __name__ == "__main__":
    generate_safety_charts()
