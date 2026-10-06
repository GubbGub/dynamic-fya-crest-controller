# Dynamic Permissive Traffic Signal Phasing Controller for Blind-Crest Intersections

[![GitHub License](https://shields.io)](LICENSE)
[![Simulation Platform](https://shields.io)](https://eclipse.dev)
[![Python Version](https://shields.io)](https://python.org)

An intelligent, sensor-driven traffic control framework built to dynamically mitigate vehicle conflict points and structural sight-distance deficits at vertical crest curves operating under permissive **Flashing Yellow Arrow (FYA)** control logic.

## 📌 Project Overview & Origin
Traditional Flashing Yellow Arrow (FYA) installations rely on the structural assumption that a left-turning driver possesses an unobstructed line of sight to accurately judge safe gaps in oncoming traffic. However, when paired with extreme vertical topography—such as a steep blind crest curve—this assumption fundamentally fails. Standard static signal timing loops cannot adapt to high-risk, non-compliant speeding vehicles ascending localized geometric obstructions.

This project introduces a localized, sensor-actuated controller that tracks real-time vehicle kinematics at the crest summit. By computing instantaneous **Time-to-Intersection (TTI)** boundaries against **AASHTO Stopping Sight Distance (SSD)** constraints, the script overrides dangerous permissive windows, shifting the system into a protected Red Arrow state to prevent high-speed collisions.

## 📐 Geometric Parameter Space (Real-World Case Study)
The framework is explicitly modeled and mathematically calibrated using empirical geospatial profile metrics extracted from a high-risk institutional corridor in Montverde, Florida (N Hancock Rd):
*   **Left-Turn Phase Elevation (\(Z_{turn}\)):** 42.92 m
*   **Summit Crest Peak Elevation (\(Z_{crest}\)):** 48.66 m (Δ h = 5.74 m profile delta)
*   **Impact Trajectory Coordinate (\(Z_{impact}\)):** 44.35 m
*   **Corridor Posted Velocity Baseline:** 45 MPH (20.1 m/s)
*   **Topographic Gradient Profile (G):** -7.0% Downhill Approach Grade

## 🔬 Scientific Hypotheses
*   **Primary Hypothesis (\(H_a\)):** \(\mu_{SAC} > \mu_{FYA}\)
    The implementation of a localized, sensor-driven dynamic traffic signal controller will yield a statistically significant increase (p < 0.05) in the mean Time-to-Collision (\(\mu_{SAC}\)) compared to standard static Flashing Yellow Arrow operations (\(\mu_{FYA}\)), expanding the physical safety margin between conflicting trajectories.
*   **Secondary Hypothesis (\(H_a\)):** The frequency of critical close-proximity conflicts (defined as TTC ≤ 1.5 s) will drop significantly under sensor-actuated loop parameters.

## 📁 Repository Structure
```text
dynamic-fya-crest-controller/
│
├── network/          # Spatial XML elements, junction properties, and elevation matrices (.net.xml)
├── data_outputs/     # Statistical telemetry outputs containing vehicle trajectory log CSVs
├── documentation/    # Comprehensive academic manuscript data and LaTeX source scripts
├── core_logic.py     # Pure algorithmic testbed simulating AASHTO braking models and TTI checks
└── README.md         # Academic landing documentation
```

## 🛠️ System Requirements & Technical Stack
*   **Operating System:** macOS (Optimized for Apple Silicon ARM architecture)
*   **Simulation Suite:** Eclipse SUMO (Simulation of Urban MObility)
*   **Control Interface:** TraCI (Traffic Control Interface) Python API Core Library
*   **Mathematical Tooling:** `numpy`, `scipy.stats`, `matplotlib`
