# ROS 2 — Perception & Motion Interface

> ⚠️ **Work in progress.** This folder is a placeholder. The ROS 2 nodes described in the project POC (Unity3D digital-twin bridge, `/camera/image_raw` publisher, OpenCV-based HSV segmentation and bounding-box detection, monocular coordinate-projection node) are still being built and tested against the simulation environment. Code will be committed here incrementally as each node is validated — expect this folder to be empty or partially populated until then.

## Planned contents

- `perception/` — OpenCV pipeline node: HSV segmentation, morphological filtering, contour extraction, bounding-box → world-coordinate projection.
- `sim_bridge/` — Unity3D ↔ ROS 2 topic bridge (camera feed, scene object ground-truth).
- `motion_interface/` — ROS 2 ⇄ CAN-FD bridge node forwarding trajectory commands to the Vesper main controller.
- `launch/` — launch files tying the above together for the digital-twin proof of concept.

No functional code is guaranteed to run yet. If you're cloning this repo to reference the *design*, see the root [`README.md`](../../README.md) and the POC document instead — this folder will be updated as modules pass testing.
