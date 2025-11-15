# Minimal-camera-network
MATLAB implementation of the ILP-based minimal camera network optimization framework  proposed in "Autonomous BIM-Aware UAV Path Planning for Construction Inspection". Includes visibility simulation, penalty matrices, ILP solver  (intlinprog), and example datasets for UAV inspection scenarios.
Minimal Camera Network Optimization

MATLAB implementation accompanying the paper:
“Autonomous BIM-Aware UAV Path Planning for Construction Inspection” (B. Alsadik, 2025)

🔍 Overview

This repository provides a full implementation of the ILP-based camera network optimization framework proposed in the paper. The method takes a dense set of candidate viewpoints, computes visibility and photogrammetric penalty matrices (GSD, B/H, accuracy), and solves for the minimal set of cameras that still satisfies quality and coverage constraints.

📁 Repository Contents

-ilp_camera_optimization_with_accuracy.m
-run_minimal_ILP_demo.m
-visibilitytesting_plotting.m
-visibilitytesting.m
-read_wpk.m


🚀 Usage
run('main_ILP_demo.m')


This script:

Loads a mesh / IFC model
Generates candidate viewpoints
Computes the visibility & penalty matrices
Solves the ILP
Outputs plots and statistics matching the paper

🧩 Dependencies
MATLAB R2024a or later
Optimization Toolbox (intlinprog)
Computer Vision Toolbox (optional, for IFC operations)

📚 Citation
If you use this code, please cite:

Abdulateef, N. A.; Jasim, Z. N.; Hasan, H. A.; Alsadik, B.; Khalaf, Y. H. Autonomous BIM-Aware UAV Path Planning for Construction Inspection. Preprints 2025, 2025102437. https://doi.org/10.20944/preprints202510.2437.v1


BibTeX will be added once the paper is published.
