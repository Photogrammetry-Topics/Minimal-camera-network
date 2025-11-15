# Minimal-camera-network
MATLAB implementation of the ILP-based minimal camera network optimization framework  proposed in "Autonomous BIM-Aware UAV Path Planning for Construction Inspection". Includes visibility simulation, penalty matrices, ILP solver  (intlinprog), and example datasets for UAV inspection scenarios.
Minimal Camera Network Optimization

MATLAB implementation accompanying the paper:
“Autonomous BIM-Aware UAV Path Planning for Construction Inspection” (B. Alsadik, 2025)

🔍 Overview

This repository provides the full MATLAB implementation of the ILP-based minimal camera network optimization framework described in the paper.
The method takes a dense set of candidate viewpoints around a structure, computes visibility and photogrammetric penalty matrices (GSD, B/H, triangulation accuracy), and solves an Integer Linear Program to select the smallest subset of cameras that still satisfies coverage and quality constraints.

The framework is designed for UAV-based inspection and photogrammetry, enabling significant reductions in:

number of images

mission duration

battery consumption

redundancy in coverage

…while preserving reconstruction accuracy.

📁 Repository Contents

-ilp_camera_optimization_with_accuracy.m

-run_minimal_ILP_demo.m

-visibilitytesting_plotting.m

-visibilitytesting.m

-read_wpk.m


🚀 Usage
run('main_ILP_demo.m')

Minimal-camera-network/
│
├─ src/
│   ├─ ilp_camera_optimization_with_accuracy.m
│   ├─ visibilitytesting.m
│   ├─ visibilitytesting_plotting.m
│   ├─ read_wpk.m
│
├─ examples/
│   ├─ run_minimal_ILP_demo.m
│   ├─ example_wpk.mat
│   ├─ example_points.mat
│
├─ README.md
├─ LICENSE
└─ .gitignore


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
