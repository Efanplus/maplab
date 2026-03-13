# maplab — CLAUDE.md

## Project Overview

**maplab 2.0** is an open-source, research-oriented visual-inertial mapping and localization framework written in C++, developed at ETH Zurich's Autonomous Systems Lab (ASL). It supports multi-session and multi-robot mapping, offline map optimization, and real-time online operation via ROS.

Papers:
- [maplab (2018)](https://arxiv.org/abs/1711.10250)
- [maplab 2.0 (2022)](https://arxiv.org/abs/2212.00654)

---

## Repository Layout

```
maplab/
├── algorithms/          # Core SLAM algorithms (optimization, loop closure, feature tracking, etc.)
├── applications/        # Executable nodes (rovioli, maplab-node, console, server)
├── aslam_cv2/           # Computer vision primitives (cameras, frames, tracker, geometric vision)
├── backend/             # Map storage (map-manager, map-resources)
├── common/              # Shared utilities (vio-common, message-flow, maplab-common)
├── console-plugins/     # Interactive console commands for offline map operations
├── docs/                # Sphinx documentation and tutorials
├── interfaces/          # External format exporters (pmvs-interface)
├── map-structure/       # Core map data structures (vi-map, posegraph, sensors)
├── maplab/              # Top-level metapackage
├── maplab-launch/       # ROS launch files
├── test/                # Integration test applications
├── tools/               # Utility tools (csv-export, resource-importer)
└── visualization/       # RViz visualization helpers
```

---

## Build System

- **ROS Catkin workspace** — build with `catkin build maplab`
- **CMake** — each package has a `CMakeLists.txt` and `package.xml`
- Dependencies managed through `dependencies/` (external submodules/cmake files)
- Target platforms: Ubuntu 18.04 / ROS Melodic, Ubuntu 20.04 / ROS Noetic

---

## Key Design Principles

1. **ROS-agnostic core** — Internal algorithms are independent of ROS. ROS interfaces are thin wrappers at the application level.
2. **Message-flow pub/sub** — `common/message-flow` provides an asynchronous publish/subscribe system for decoupling pipeline stages.
3. **VI-Map as central data structure** — The `vi-map` package defines all map data; algorithms operate on it through a clean API.
4. **Plugin architecture** — `maplab-console` loads plugins at runtime; each plugin registers commands.
5. **JPL quaternion convention** — `[x, y, z, w]` throughout. Parameterizations enforce this in Ceres.

---

## Core Workflow

### Online (Robot)
```
Sensors → DataSource → ImuCameraSynchronizer → FeatureTracking → VIO estimator (ROVIO)
                                                      ↓
                                              onlineMapBuilder → VIMap
                                                      ↓
                                              (optional) Localizer
```

### Offline (Console)
```
Load VIMap → loop_closure → optimize_vi → lc_merge_landmarks →
             retriangulate_landmarks → save_map
```

---

## Quaternion Convention

All code uses **JPL quaternion** convention: `q = [x, y, z, w]` where `w` is the scalar component. This matches Eigen's `coeffs()` memory layout. Do **not** use Hamilton convention (`[w, x, y, z]`) without explicit conversion.

---

## Important Data Types

| Type | Location | Description |
|------|----------|-------------|
| `vi_map::VIMap` | `map-structure/vi-map` | Top-level map container |
| `vi_map::Vertex` | `map-structure/vi-map` | Pose-graph node (keyframe) |
| `vi_map::Edge` (subtypes) | `map-structure/vi-map` | IMU, loop-closure, odometry edges |
| `vi_map::Landmark` | `map-structure/vi-map` | 3D map point |
| `vi_map::MissionId` | `map-structure/vi-map` | Identifies one mapping session |
| `aslam::NCamera` | `aslam_cv2/aslam_cv_cameras` | Multi-camera rig |
| `aslam::VisualNFrame` | `aslam_cv2/aslam_cv_frames` | Multi-camera synchronized frame |
| `vio::SynchronizedNFrameImu` | `common/vio-common` | Frame bundle + IMU data |
| `vio::ViNodeState` | `common/vio-common` | Full VIO state (pose, velocity, biases) |

---

## Coordinate Frames

| Symbol | Meaning |
|--------|---------|
| `G` | Global / world frame |
| `M` | Mission baseframe (relative to G via `T_G_M`) |
| `I` | IMU body frame |
| `C` | Camera frame |
| `B` | Landmark base vertex frame |

Transforms are named `T_A_B` meaning "transform FROM frame B TO frame A" (i.e., takes a point in B and gives it in A).

---

## Testing

```bash
catkin build maplab --no-deps
catkin run_tests maplab_package_name
```

Integration tests live under `test/` and `vi-mapping-test-app`.

---

## Documentation

Full documentation: https://maplab.asl.ethz.ch/index.html

Local docs can be built:
```bash
cd docs && pip install -r requirements.txt && make html
```

---

## Architecture Documentation

A comprehensive architecture document covering all VSLAM submodules is available at:

**[`docs/pages/ARCHITECTURE.md`](docs/pages/ARCHITECTURE.md)**

It covers:
- System overview and module directory mapping
- VI-Map data structure (Mission / Vertex / Edge / Landmark hierarchy)
- Visual front-end: aslam_cv2 (camera models, feature tracking pipeline)
- Online mapping front-end: ROVIOLI (ROVIO iEKF + async map building)
- General mapping node: maplab-node
- IMU preintegration (RK4, covariance propagation)
- Feature tracking and loop closure (ORB/BRISK/FREAK, IMI/HNSW, PnP RANSAC)
- Landmark triangulation (DLT N-view, LiDAR averaging)
- Bundle adjustment (all Ceres residual terms, observability analysis, outlier rejection)
- Multi-robot mapping: maplab-server (submap merge workflow)
- Offline console plugin system (all plugin commands)
- Dense reconstruction (SGBM stereo, TSDF depth integration, PMVS export)
- Map sparsification (keyframe pruning, ILP landmark compression)
- Message-flow pub/sub system
- Coordinate frame conventions
- Typical workflows (single-session, multi-session, multi-robot, localization mode)
