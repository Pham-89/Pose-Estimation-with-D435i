# Robotics Vision & Grasp Pose Estimation

## Table of Contents

* [Overview](#overview)
* [Pipeline](#pipeline)
* [Repository Structure](#repository-structure)
* [Requirements](#requirements)
* [Installation & Build](#installation--build)
* [Running](#running)
* [Module Documentation](#module-documentation)

  * [1. Perception Input](#1-perception-input)
  * [2. Preprocessing](#2-preprocessing)
  * [3. Clustering](#3-clustering)
  * [4. Oriented Bounding Box (OBB) Estimation](#4-oriented-bounding-box-obb-estimation)
  * [5. Classification](#5-classification)
  * [6. Visualization](#6-visualization)
  * [7. Grasp Pose Service](#7-grasp-pose-service)
* [Calibration Guide](#calibration-guide)
* [Tuning Parameters](#tuning-parameters)
* [Known Limitations](#known-limitations)
* [License](#license)

---

## Overview

This project estimates the grasping pose of a specific target Lego block (e.g. "give me the pose of the red rectangular block") from a single RGB-D camera stream.

The scenario consists of **8 Lego blocks** — 4 rectangular and 4 square blocks, with 4 different colors — arranged in two rows on a table.

The system continuously detects, classifies, and tracks every visible block, and provides grasp poses on demand through a ROS 2 service. A separate manipulation/robot-arm node can call the service with a target label and receive a ready-to-use `PoseStamped`.

### Design Priorities

* **Stability over cleverness** — Every stage is designed to minimize frame-to-frame jitter in orientation and classification. Downstream grasp planning requires a consistent pose, not only an accurate average pose.

* **No ML, fully classical CV/geometry** — The complete pipeline uses deterministic geometric and statistical methods, including RANSAC, DBSCAN, convex hull + rotating calipers, and HSV hue matching. No training data or neural networks are required.

* **Decoupled, swappable stages** — Each pipeline stage is implemented as its own class with minimal dependency on ROS 2 message types where possible, allowing individual stages to be unit-tested or reused outside the ROS 2 node.

---

## Pipeline

<img width="1213" height="125" alt="Pipeline overview" src="https://github.com/user-attachments/assets/cf3a8e3b-f7a5-4d2b-86ef-f207381f2994" />

<img width="698" height="874" alt="Pipeline visualization" src="https://github.com/user-attachments/assets/74f11b64-5353-417b-b228-d8ca60edec03" />

<img width="685" height="647" alt="Pipeline visualization" src="https://github.com/user-attachments/assets/a8a1be09-352b-4c7d-b05d-27cfed2e4589" />

The perception pipeline follows these main stages:

```text
RGB-D Point Cloud
       │
       ▼
   Crop / ROI
       │
       ▼
    VoxelGrid
       │
       ▼
  RANSAC Plane Removal
       │
       ▼
    DBSCAN Clustering
       │
       ▼
   OBB Estimation
       │
       ├──────────────┐
       ▼              ▼
 Shape Classification  Color Classification
       │              │
       └───────┬──────┘
               ▼
        Block Classification
               │
               ▼
         Block Registry
               │
               ▼
       Grasp Pose Service
```

---

## Repository Structure

```text
ros2_ws/src/
├── robocup_vision_core/           # Main perception pipeline
│   ├── include/
│   ├── src/
│   │   ├── main.cpp                        # Node entry point
│   │   ├── camera_subscriber.hpp/.cpp      # ROS 2 ↔ PCL bridge
│   │   ├── crop_space.hpp/.cpp              # CropBox ROI filter
│   │   ├── voxelization.hpp/.cpp            # VoxelGrid downsampling
│   │   ├── ransac_plane.hpp/.cpp            # Table plane segmentation/removal
│   │   ├── dbscan_clustering.hpp/.cpp       # Custom DBSCAN
│   │   ├── obb_estimator.hpp/.cpp           # OBB struct + legacy PCA estimator
│   │   ├── plana_obb_estimator.hpp/.cpp     # Stable hull + rotating-calipers OBB
│   │   ├── obb_visualizer.hpp/.cpp          # OBB → RViz CUBE marker
│   │   ├── color_utils.hpp/.cpp             # RGB → HSV, hue statistics
│   │   ├── classifier.hpp/.cpp              # Shape + color classification
│   │   ├── classification_visualizer.hpp/.cpp
│   │   └── block_registry.hpp/.cpp          # Label → latest pose store
│   ├── CMakeLists.txt
│   └── package.xml
│
└── robocup_vision_interfaces/     # Custom ROS 2 interface package
    ├── srv/
    │   └── GraspPose.srv          # target_label → success, pose, width_m
    ├── CMakeLists.txt
    └── package.xml
```

> **Note:** The `OBB` struct is defined in `obb_estimator.hpp` and reused by `plana_obb_estimator.hpp`. Both files are therefore required even though the PCA-based estimator in `obb_estimator.cpp` is no longer called from `main.cpp`.

---

## Requirements

* Ubuntu 22.04
* ROS 2 Humble
* Intel RealSense SDK
* [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

  * `realsense2_camera`
  * `realsense2_camera_msgs`
* PCL (Point Cloud Library)
* `pcl_conversions`
* Eigen3
* Intel RealSense D435i or compatible RGB-D camera publishing `sensor_msgs/PointCloud2`

---

## Installation & Build

### 1. Clone the repository

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <this-repo-url>
```

### 2. Install dependencies

For the required ROS 2 PCL packages:

```bash
sudo apt install ros-humble-pcl-conversions ros-humble-pcl-ros
```

Make sure the Intel RealSense ROS package is also installed and configured.

### 3. Build the interfaces package

The custom interface package should be built first because the main perception package depends on it.

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --packages-select robocup_vision_interfaces
```

### 4. Source and build the main package

```bash
source install/setup.bash

colcon build --packages-select robocup_vision_core --symlink-install
```

---

## Running

### Terminal 1 — Start the camera

```bash
source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=true
```

### Terminal 2 — Run the perception node

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run robocup_vision_core camera_subscriber
```

### Terminal 3 — Visualize in RViz2

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

rviz2
```

Set the RViz2 **Fixed Frame** to:

```text
camera_link
```

Add the following topics:

| Topic             | Type        | Description                         |
| ----------------- | ----------- | ----------------------------------- |
| `/cloud_roi`      | PointCloud2 | Cloud after cropping                |
| `/cloud_objects`  | PointCloud2 | Cloud after table removal           |
| `/cloud_clusters` | PointCloud2 | Detected object clusters            |
| `/obb_markers`    | MarkerArray | OBB boxes and classification labels |

### Terminal 4 — Request a grasp pose

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 service call /get_grasp_pose robocup_vision_interfaces/srv/GraspPose \
  "{target_label: 'RS'}"
```

> **Important:** Source the ROS 2 and workspace environments in every terminal before running ROS 2 commands.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

# Module Documentation

## 1. Perception Input

`CameraSubscriber` subscribes to:

```text
/camera/camera/depth/color/points
```

using `SensorDataQoS()`.

Incoming `sensor_msgs::PointCloud2` messages are converted to:

```text
pcl::PointCloud<PointXYZRGB>
```

The cloud is protected by a mutex.

A new cloud object is allocated for every callback rather than mutating a shared buffer. This prevents the processing loop from reading a cloud while it is being concurrently overwritten, at the cost of one additional allocation per frame.

The camera runs at approximately 30 FPS while the processing loop operates at approximately 10 Hz.

---

## 2. Preprocessing

### CropSpace

`CropSpace` uses a `pcl::CropBox` filter to restrict the point cloud to a configurable 3D region.

Default range:

```text
(-0.3, -0.5, 0.3) → (0.3, 0.5, 5.0) meters
```

This removes irrelevant areas such as the floor, walls, and background clutter before further processing.

### Voxelizer

`Voxelizer` uses `pcl::VoxelGrid` downsampling.

Default leaf size:

```text
3 mm
```

The voxel grid reduces point density for faster downstream processing and also reduces some sensor noise.

### RansacPlane

`RansacPlane` uses:

```text
pcl::SACSegmentation
SACMODEL_PLANE
```

with:

* Distance threshold: `5 mm`
* Maximum iterations: `100`

The dominant plane is interpreted as the table surface and removed from the scene.

The resulting plane coefficients:

```text
[a, b, c, d]
```

represent:

```text
ax + by + cz + d = 0
```

The plane normal is reused by `PlanarObbEstimator` to determine the "up" direction relative to the table independently of the camera mounting angle.

---

## 3. Clustering

`DBSCANClustering` is a custom implementation of DBSCAN rather than `pcl::EuclideanClusterExtraction`.

It uses:

```text
pcl::search::KdTree
```

for radius queries.

The implementation follows the standard DBSCAN algorithm using the following label scheme:

```text
-1  = unvisited
-2  = noise
-3  = enqueued but unassigned
≥0  = cluster ID
```

The algorithm returns one `pcl::PointIndices` set for each detected object.

Typical parameters:

```text
eps ≈ 0.7–1.0 cm
min_samples ≈ 30–80
```

Small clusters are removed using `min_cluster_size`.

These parameters may need to be tuned depending on the lighting and environment.

---

## 4. Oriented Bounding Box (OBB) Estimation

Two OBB estimators exist in the codebase. Only one is used in production.

### Legacy PCA-based estimator

`ObbEstimator::computeOBBFromMoments()` in:

```text
obb_estimator.cpp
```

uses:

```text
pcl::MomentOfInertiaEstimation
```

This is a PCA/moment-of-inertia based approach.

It is **not used by `main.cpp`**.

The estimator is retained as a reference implementation and as the home of the shared `OBB` structure.

A major problem with PCA occurs when an object's footprint is close to square. Two covariance eigenvalues become nearly equal, making the corresponding principal axes numerically unstable.

This caused square blocks to occasionally rotate by approximately 45° between frames, resulting in unstable classification and visualization.

### Planar OBB estimator

The production estimator is:

```text
PlanarObbEstimator::computeOBB()
```

implemented in:

```text
plana_obb_estimator.cpp
```

Instead of using PCA for the in-plane orientation, it performs:

1. Projection of the cluster's 3D points onto the table plane.
2. Construction of a stable 2D `(u, v)` basis.
3. Computation of the 2D convex hull using Andrew's monotone chain algorithm.
4. Rotating-calipers search for the minimum-area bounding rectangle.
5. Combination of the planar rectangle with the extent along the plane normal to obtain the full 3D OBB.

Because the orientation is anchored to physical hull edges instead of an eigen-decomposition, this avoids the PCA degeneracy that caused the previous 45° orientation flips.

A smaller 90° ambiguity can still occur for nearly perfect square objects. See [Known Limitations](#known-limitations).

### OBB Structure

Both estimators use the same `OBB` structure defined in:

```text
obb_estimator.hpp
```

It contains:

* World-space position
* Orientation quaternion
* Extents along local X/Y/Z
* `corners()` helper
* `volume()` helper

### OBB Visualization

`ObbVisualizer::createBoxMarker()` converts an OBB into an RViz `CUBE` marker.

The markers use a cycling 15-color palette based on cluster ID for visual verification.

---

## 5. Classification

### Color Processing

`color_utils` converts cluster RGB data into a representative Hue value.

The main components are:

* `rgbToHsv()` — manual RGB → HSV conversion without an OpenCV dependency.
* `computeHueStats()` — filters unreliable pixels and computes the circular median hue.
* `circularMedianHue()` — handles hue statistics correctly across the 0°/360° boundary.
* `circularHueDistance()` — computes angular distance on the hue circle.

Pixels are filtered when:

```text
saturation < sat_min
```

or when the value falls outside:

```text
[val_min, val_max]
```

This rejects unreliable measurements caused by:

* Near-gray pixels
* Near-black pixels
* Overexposed pixels
* Specular highlights

Hue is used instead of raw RGB because it is more robust to lighting changes and camera auto white-balance adjustments.

### Shape Classification

The classifier determines the block shape using the ratio:

```text
long_side / short_side
```

A ratio close to `1.0` indicates:

```text
SQUARE
```

while a ratio closer to the rectangular block dimensions indicates:

```text
RECTANGULAR
```

The default threshold is:

```text
1.4
```

Using a ratio instead of absolute dimensions makes the classification less sensitive to systematic scale errors caused by voxelization or partial occlusion.

### Color Classification

Color classification uses nearest-centroid matching on the hue circle.

Each color has a calibrated reference hue:

```text
ColorCentroid
```

The measured median hue is compared with each centroid and the closest color is selected.

If the angular distance exceeds:

```text
max_color_match_distance_deg
```

the result is rejected as:

```text
ColorLabel::UNKNOWN
```

The default maximum matching distance is:

```text
18°
```

---

## 6. Visualization

`ClassificationVisualizer` publishes a short two-character label above each detected block.

Examples:

```text
RS = Red Square
BR = Blue Rectangular
```

An unknown or partially classified object can be displayed as:

```text
?
```

The label position is calculated relative to the block's own OBB.

A local offset:

```text
(0, 0, height/2 + offset)
```

is rotated using the block's OBB orientation quaternion.

This is preferable to simply adding to world Z because the block's local "up" direction follows the table plane and is not necessarily aligned with the camera frame's Z axis.

---

## 7. Grasp Pose Service

`BlockRegistry` maintains a thread-safe mapping:

```text
label → {OBB, timestamp}
```

The registry is cleared and repopulated during each perception frame.

This separates the continuous perception loop from the on-demand grasp pose service.

The service callback looks up the most recent observation for the requested label.

If the block has never been detected, or if the observation is older than:

```text
max_age_sec = 0.5 s
```

the service returns:

```text
success = false
```

This prevents stale poses from being returned for blocks that have disappeared, become occluded, or are no longer classified under the same label.

### Service Interface

`GraspPose.srv`:

```text
# Request
string target_label      # e.g. "RS", "BR"

---

# Response
bool success
string message
geometry_msgs/PoseStamped pose
float32 width_m          # in-plane short-side extent
```

Example request:

```bash
ros2 service call /get_grasp_pose robocup_vision_interfaces/srv/GraspPose \
  "{target_label: 'RS'}"
```

> **Important:** The service currently returns the object's own pose, not a fully transformed gripper-specific grasp pose. Approach-axis offsets, gripper-frame alignment, and other manipulation-specific transformations are expected to be handled by the manipulation-side client.

---

## Calibration Guide

Color classification depends on:

```text
Classifier::setColorCentroids()
```

The color centroids should be calibrated for the specific camera and lighting conditions before relying on the classification results.

Textbook HSV values such as:

```text
Red    = 0°
Yellow = 60°
Green  = 120°
Blue   = 240°
```

do not necessarily match the measured hue values of real Lego blocks.

### Recalibration Procedure

1. Place one block of a known color in front of the camera.

2. Run the perception node.

3. Read the logged `median_hue_deg` value for its cluster.

4. Repeat for all four colors.

5. Update the centroids in `main.cpp`:

```cpp
classifier.setColorCentroids({
    { robocup_vision_core::ColorLabel::RED,    <measured_hue> },
    { robocup_vision_core::ColorLabel::YELLOW, <measured_hue> },
    { robocup_vision_core::ColorLabel::GREEN,  <measured_hue> },
    { robocup_vision_core::ColorLabel::BLUE,   <measured_hue> },
});
```

6. Rebuild and test again.

Check the angular distance between the closest pair of color centroids and ensure:

```text
max_color_match_distance_deg
```

remains safely below approximately half of that gap.

### Troubleshooting Color Measurements

If:

```text
used_pts = 0
```

for a cluster, all pixels may have been rejected by the HSV reliability filter.

Possible causes include:

* Severe overexposure → consider increasing `val_max`
* Severe desaturation → consider lowering `sat_min`

The relevant parameters are implemented in:

```text
ColorUtils::computeHueStats()
```

<img width="689" height="930" alt="Calibration example" src="https://github.com/user-attachments/assets/47141d72-b3f8-458a-8859-2cd18a3cd60c" />

---

## Tuning Parameters

The main parameters that may require adjustment depending on the camera setup and environment include:

| Parameter             | Purpose                  | Typical / Default                     |
| --------------------- | ------------------------ | ------------------------------------- |
| CropBox bounds        | Define workspace ROI     | `(-0.3,-0.5,0.3)` → `(0.3,0.5,5.0)` m |
| Voxel leaf size       | Point-cloud downsampling | `3 mm`                                |
| RANSAC threshold      | Table-plane tolerance    | `5 mm`                                |
| RANSAC iterations     | Plane fitting iterations | `100`                                 |
| DBSCAN `eps`          | Neighborhood radius      | `0.7–1.0 cm`                          |
| DBSCAN `min_samples`  | Minimum core points      | `30–80`                               |
| Shape ratio threshold | Square vs. rectangular   | `1.4`                                 |
| Max color distance    | Hue matching tolerance   | `18°`                                 |
| `max_age_sec`         | Maximum valid pose age   | `0.5 s`                               |

These values may need to be tuned when changing:

* Camera position
* Camera angle
* Lighting
* Lego blocks
* Workspace dimensions
* Sensor noise characteristics

---

## Known Limitations

### 1. Lighting-dependent color calibration

`ColorCentroid` hue values and the HSV reliability filter (`sat_min`, `val_min`, `val_max`) are dependent on the lighting environment.

Changing between daylight, indoor lighting, and direct desk lighting can shift the measured hue values.

There is currently no automatic lighting adaptation. Recalibration using the [Calibration Guide](#calibration-guide) is therefore required when the lighting environment changes significantly.

### 2. 90° orientation ambiguity for near-perfect squares

The rotating-calipers OBB estimator eliminates the major 45° instability of the previous PCA-based estimator.

However, for an object whose two in-plane dimensions are extremely close, the algorithm can occasionally swap which hull edge is considered the width and depth axis.

This can produce a 90° orientation change between frames.

For a symmetric block and a parallel-jaw gripper, this is generally not critical because gripping along either equivalent axis produces the same physical result.

It can nevertheless matter for downstream grasp planners with directional constraints.

### 3. Shape ratio sensitivity to occlusion and viewing angle

Blocks near the edge of the camera's field of view can have part of their footprint clipped.

This can reduce the measured long-side extent and move the shape ratio toward the square/rectangular decision boundary.

Classification should therefore be visually verified in RViz for blocks close to the edge of the workspace crop.

### 4. No multi-frame temporal smoothing

`BlockRegistry` updates each label every frame without applying temporal filtering such as:

* Kalman filtering
* Exponential smoothing
* Multi-frame voting

A momentary misclassification or occlusion can therefore temporarily remove or change an entry in the registry.

The service's `max_age_sec` tolerance provides limited protection against short-term observations gaps.

### 5. Object pose instead of a final gripper pose

The `GraspPose` service currently returns the detected block's:

* Position
* Orientation

It does not yet apply:

* Gripper approach-axis offsets
* Finger-clearance constraints
* Gripper-frame alignment
* Manipulator-specific approach poses

The transformation from object pose to an executable robot grasp target is expected to be handled by the manipulation-side client.

### 6. Static-scene assumption

The pipeline assumes that blocks remain stationary after placement.

There is currently no persistent object tracking or re-identification across physical object movement.

If a block is moved, its registry entry is simply updated when it is detected again.

### 7. Legacy OBB implementation

`obb_estimator.cpp` contains the older PCA-based `ObbEstimator::computeOBBFromMoments()` implementation.

It is not used by `main.cpp`.

It is retained because it contains the shared `OBB` structure and provides a reference implementation for comparison with the production planar OBB estimator.

---

## License

MIT License.

See the repository's `LICENSE` file for the complete license text.
