# ML Models & Python Scripts — Vision & Precision Pipeline

> ⚠️ **Work in progress.** This folder is a placeholder. Python tooling for the vision/perception pipeline (color/object detection, camera calibration, coordinate-mapping validation) and any future ML-based precision or defect-detection models are still in development. Code will be committed here incrementally as each script or model is tested against the simulation and, later, real camera data.

## Planned contents

- `vision/` — Python/OpenCV scripts for HSV-based object detection, prototyping ahead of the ROS 2 node port.
- `calibration/` — Camera intrinsic/extrinsic calibration scripts, monocular-projection accuracy validation.
- `sim_tools/` — Scripts for generating synthetic training/test data from the Unity3D simulation.
- `models/` — Any trained model weights and training scripts added as the precision/defect-detection roadmap (see root README's Future Work section) is implemented.

No functional code is guaranteed to run yet. If you're cloning this repo to reference the *design*, see the root [`README.md`](../../README.md) and the POC document instead — this folder will be updated as modules pass testing.
