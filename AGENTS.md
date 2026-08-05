# AGENTS.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project overview

Visual monocular SLAM using a 1-Point RANSAC EKF. Implemented as a ROS 2 Humble package (`ekf_mono_slam`) with C++20, Eigen, and OpenCV. Environment and packaging are managed with [pixi](https://pixi.sh) and the `pixi-build-ros` backend.

Domain logic lives in a ROS-free library target `ekf_mono_slam_core`. ROS nodes are thin wrappers.

## Setup

```bash
pixi install
# Optional datasets (gitignored under datasets/):
# AGZ Subset: https://pub-db0cd070a4f94dabb9b58161850d4868.r2.dev/AGZ_subset.zip
# Desk Translation: https://pub-db0cd070a4f94dabb9b58161850d4868.r2.dev/desk_translation.zip
```

Platforms: `linux-64`, `osx-arm64`. Channels: `robostack-humble`, `conda-forge`. Preview: `pixi-build`.

`pixi install` builds the path package `ros-humble-ekf-mono-slam` from `ekf-mono-slam/` via `pixi-build-ros` and installs it into `.pixi/envs/default`.

## Common commands

```bash
pixi install            # resolve + build path package into the env
pixi run test           # run installed slam_test (cwd = share/ekf_mono_slam)
pixi run app            # ros2 launch ekf_mono_slam vslam.launch.py
pixi build              # from ekf-mono-slam/: produce a conda package artifact
```

There is no colcon workspace. Do not recreate root `build/`, `install/`, or `log/` for this project.

### Tests

- Binary: installed as `lib/ekf_mono_slam/slam_test` (via `ros2 run ekf_mono_slam slam_test`).
- Sources listed explicitly in `CMakeLists.txt` (`test/utest_*.cpp`, `test/itest_*.cpp`).
- Fixtures: `test/resources/desk_translation/` (also installed under `share/ekf_mono_slam/test/resources`).
- Paths in tests are relative to the package root / installed share prefix.
- CI: `pixi run test` on `main` push/PR.

```bash
pixi run test
# or with a gtest filter:
pixi run test -- --gtest_filter=ExtendedKalmanFilter.PredictState
```

| File | Suites |
|------|--------|
| `test/utest_filter.cpp` | `ExtendedKalmanFilter` |
| `test/utest_feature.cpp` | `FeatureDetectors`, `ImageFeatureMeasurement`, `Zones` |
| `test/utest_math.cpp` | `JacobianDirectionalVector`, `QuaternionDerivatives`, `RotationMatrix`, `FeatureDistortion` |
| `test/utest_image.cpp` | `FileSequenceImageProvider`, `FileSequenceProvider` |
| `test/itest_slam.cpp` | `SLAMIntegration` |

### Formatting / IDE

- `.clang-format`: Google-based, 80 cols, `NamespaceIndentation: All`.
- `.clangd`: looks for `compile_commands.json` under `ekf-mono-slam/` (copy/symlink from the latest `.pixi/bld/ros-humble-ekf-mono-slam/*/work/build/` if needed).
- No dedicated pixi lint task; format touched C++ with clang-format.

### Run the system

Launch starts two nodes in namespace `slam`:

1. `file_sequence_image` — disk image playback (`image_dir`, default `./datasets/desk_translation/`)
2. `ekf` — filter node with **in-process** feature detection

```bash
pixi run app
# or:
ros2 launch ekf_mono_slam vslam.launch.py \
  image_dir:=./datasets/desk_translation/ \
  config:=$(ros2 pkg prefix ekf_mono_slam)/share/ekf_mono_slam/config/ekf.yaml
```

`file_sequence_image` params: `image_dir`, `start_image_index` (1), `end_image_index` (350), ~25 Hz.

EKF runtime config: installed `config/ekf.yaml` (camera, kinematics, image_feature, `delta_t`). Loaded as ROS parameters on the `ekf` node and passed into `EKF(SlamConfig)`.

## Repository layout

```
pixi.toml                 # workspace: path dep + tasks
pixi.lock
ekf-mono-slam/            # single ament_cmake package + package pixi.toml
  pixi.toml               # pixi-build-ros backend, distro=humble
  CMakeLists.txt          # explicit source lists; ekf_mono_slam_core lib
  package.xml
  config/ekf.yaml
  launch/vslam.launch.py
  msg/                    # State, CovarianceMatrix, image feature msgs
  include/                # public headers (domain + node headers)
  src/                    # core + *_node.cpp wrappers
  test/
datasets/                 # gitignored
```

Package name `ekf_mono_slam`; directory `ekf-mono-slam`. Conda package name `ros-humble-ekf-mono-slam`.

## Architecture

### Runtime data flow

```
file_sequence_image                         ekf
  (disk images)                          filter node
        |                                    |
        | camera/image                       v
        +-------------------------->  process_frame(cv::Mat)
                                      (FeatureDetector in-process)
                                             |
                                             v
                                      filter/state
                                      filter/covariance
```

- **Init:** first frame runs AKAZE (configurable) detect/describe and `add_features`.
- **Tracking (partial):** later frames `predict()` then `match_predicted_features` (match/update still TODO inside `filter/`).
- No feature-detect ROS service. No Rerun dependency.

### Build targets

| Target | Role |
|--------|------|
| `ekf_mono_slam_core` | Domain library: configuration, filter, feature, math, image, visual |
| `file_sequence_image` | Image playback node |
| `ekf` | ROS wrapper around `EKF::process_frame` |
| `slam_test` | GTest binary linking the core lib |

Core links Eigen + OpenCV only (no rclcpp).

### Domain modules

- **`filter/`** — `EKF` owns `State`, `CovarianceMatrix`, and lazy `FeatureDetector`. Prefer extending `process_frame` here, not in the node.
- **`feature/`** — detectors, zones/ellipses, measurements/predictions, inverse-depth/cartesian map features.
- **`math/`** — `EkfMath` Jacobians, distortion, quaternion derivatives.
- **`image/`** — `ImageProvider` / `FileSequenceImageProvider`.
- **`configuration/`** — `SlamConfig` POD (`CameraConfig`, `KinematicsConfig`, `ImageFeatureConfig`) with defaults matching `config/ekf.yaml`; core objects store and pass config (no process-wide parameter headers).
- **`visual/`** — OpenCV drawing helpers used by the detector mask path.

### ROS interfaces

Msgs only (no srvs): `State`, `CovarianceMatrix`, `ImagePoint`, `ImageFeatureMeasurement`, `ImageFeatureMeasurementArray`, `ImageFeaturePrediction`.

### Implementation status

- Post-init path: predict + stub match; 1-Point RANSAC update still TODO in `filter/`.
- Detector defaults to AKAZE; selectable via `image_feature.detector_type` / `descriptor_type` in YAML.

## Dependencies

ROS (via package.xml → RoboStack): humble `rclcpp`, msgs, `cv_bridge`, `image_transport`, `rosidl_*`.

Libs: Eigen, OpenCV, GTest/GMock (test), mapped by `pixi-build-ros`.

C++20. Warnings: `-Wall -Wextra -Wpedantic`.

## Commits

Use [Conventional Commits](https://www.conventionalcommits.org/):

```text
<type>(optional scope): <short summary>

<body>
```

Common types: `feat`, `fix`, `refactor`, `test`, `docs`, `build`, `ci`, `chore`, `perf`.

Every commit must include a body. Keep it short: at most two paragraphs, each at most five lines. Prefer bullets when listing concrete changes.

```text
refactor(filter): thread SlamConfig through core objects

Remove process-wide parameter headers and pass config from ekf.yaml
into State, CovarianceMatrix, and FeatureDetector.

- Drop CameraParameters / KinematicsParameters globals
- Keep defaults aligned with config/ekf.yaml
```
