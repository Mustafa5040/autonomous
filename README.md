# Autonomous Vehicle Stack: Perception, Kinematic Path Planning & MAVLink Control

[![Python Version](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An end-to-end autonomous navigation stack designed for unmanned surface/ground vehicles (USV/UGV). The project integrates a multi-threaded asynchronous perception pipeline, rigorous WGS84-to-Cartesian geodetic transformations, a kinematic-aware search planner (Hybrid $A^*$-inspired) with continuous obstacle distance fields, and MAVLink-based vehicle actuation.

---

## 📸 Demo & Visuals

<!-- Add your simulation or testPlanPath.py matplotlib output here -->
![Path Planning Demo](assets/path_planning_demo.png)
*Figure: Planned obstacle-free trajectory using kinematic discrete-heading expansion, continuous distance-transform cost field, and spline smoothing.*

---

## 🏛️ System Architecture

The software architecture is decoupled into three concurrent layers, mimicking a lightweight pub/sub robotics middleware:

```
[ Camera Stream / Video Feed ]
              │
              ▼
   [ frameDistributor ]  (Thread-Safe Multi-Consumer Queue Distributor)
        │            │
        │            ▼
        │    [ colorDetector ]  (HSV Thresholding & Contour Filtering)
        │            │
        │            ▼
        │   [ objectClassifier ] (Pinhole Math: Distance & Bearing Estimation)
        ▼            │
  [ rawSaver ]       ▼
           [ processedVideoSaver ] (Bounding Boxes & Visual Debugging)

─────────────────────────────────────────────────────────────────────────────
                             DECISION & CONTROL
─────────────────────────────────────────────────────────────────────────────

  [ GPS / IMU Sensor Data ] ──────┐
                                   ▼
[ Perceived Obstacles ] ──> [ PathPlanner Engine ]
                              ├─ Geodetic Projections (WGS84 ⇄ ECEF ⇄ ENU)
                              ├─ Discrete Kinematic Node Expansion (x, y, θ)
                              ├─ Euclidean Distance Transform (EDT) Margin Cost
                              ├─ Line-of-Sight Path Pruning & B-Spline Smoother
                              │
                              ▼
                     [ idaMavUtil / MAVLink ]
                              │
                              ▼
                  [ Flight Controller / Pixhawk ] (ArduPilot / PX4)
```

---

## 🧠 Key Modules & Technical Deep-Dive

### 1. Kinematic Path Planning (`planning/pathPlanning.py`)

Standard grid planners ($A^*$, Dijkstra) treat the agent as a point mass, generating sharp turns unachievable by non-holonomic or momentum-heavy vehicles. This engine implements search over continuous pose configuration $(x, y, \theta)$:

* **Geodetic Projections ($\text{WGS84} \leftrightarrow \text{ECEF} \leftrightarrow \text{ENU}$):**
  Translates global GNSS coordinates into a local topocentric East-North-Up (ENU) Cartesian frame using full ellipsoidal Earth constants ($a = 6378137.0\text{ m}$, $f = 1 / 298.257223563$), allowing metric-scale calculations without planar distortion.
* **Heading-Constrained Node Expansion:**
  Successor states are evaluated with fixed steering angle increments $\delta \in \{-30^\circ, 0^\circ, +30^\circ\}$ and step size $L$:
  $$x' = x + L \cdot \cos(\theta + \delta), \quad y' = y + L \cdot \sin(\theta + \delta), \quad \theta' = (\theta + \delta) \pmod{360^\circ}$$
* **Potential Field Cost with EDT (Euclidean Distance Transform):**
  Rather than binary collision checks alone, the cost function evaluates obstacle clearance using `scipy.ndimage.distance_transform_edt`:
  $$\text{Cost}(s_{\text{next}}) = g(s) + h(s, s_{\text{goal}}) + \frac{\alpha}{\max(d_{\text{obstacle}}, \epsilon)}$$
  This creates an artificial repulsion field, steering the agent toward the center of navigable waterways rather than grazing obstacle boundaries.
* **Rotated Polygon Collision Detection:**
  Evaluates bounding footprints for both vehicle and obstacles using Shapely's `prep()` vectorized STRtree structures for fast polygonal intersection tests under arbitrary orientations.
* **Path Simplification & B-Spline Smoothing:**
  * **Line-of-Sight (LOS) Shortcutter:** Iteratively tests collision-free raycasts between non-consecutive waypoints to eliminate redundant search artifacts.
  * **Parametric B-Spline Fitting:** Applies cubic spline interpolation (`scipy.interpolate.splprep`) to guarantee continuous curvature ($G^2$ continuity) for smooth steering actuation.

---

### 2. Thread-Safe Perception Pipeline (`camera/`, `classification/`)

A producer-consumer pattern designed to handle camera I/O and computer vision models without stalling system telemetry:

* **`frameDistributor`:** Captures raw camera buffers and non-blockingly broadcasts them into isolated consumer queues (`queue.Queue`) across independent threads.
* **Object Spatial Extraction (`Camera.py`):**
  Uses intrinsic sensor geometry (focal length, sensor dimensions, horizontal/vertical FOV) to calculate 3D relative range and horizontal bearing:
  $$Z = \frac{f \cdot H_{\text{real}}}{h_{\text{sensor\_pixel\_size}}}$$
  The calculated bearing and range are combined with vehicle IMU yaw to compute global geodetic positions for each detected barrier.

---

### 3. Autopilot Telemetry & Actuation (`navigation/`)

* **`idaMavUtil.py`:** Wrapper on top of `pymavlink` interfacing with Pixhawk / ArduPilot / PX4 flight controllers.
* **Actuation Primitives:** Direct programmatic control over arming, mode switching (`GUIDED`, `AUTO`), takeoff sequence, mission upload, and streaming global position targets (`SET_POSITION_TARGET_GLOBAL_INT`).

---

## 📁 Repository Structure

```text
├── camera/
│   ├── Camera.py                 # Intrinsic camera model & distance estimation
│   ├── frameDistributor.py       # Multi-threaded frame dispatcher
│   ├── processedVideoSaver.py    # Annotated video recorder thread
│   └── videoSaver.py             # Raw capture recorder thread
├── classification/
│   ├── colorDetector.py          # HSV mask segmentation & contour extraction
│   └── objectClassifier.py       # Detection-to-spatial projection worker
├── dto/
│   ├── DataTransferObject.py     # Thread communication data wrapper
│   └── Duba.py                   # Obstacle / buoy data model
├── navigation/
│   ├── idaMavDefinitions.py      # MAVLink mode & custom enum definitions
│   ├── idaMavUtil.py             # PyMAVLink vehicle command interface
│   └── idaOtonom.py              # Autonomous mission execution entry
├── planning/
│   ├── Duba.py                   # Spatial obstacle polygon definitions
│   ├── pathPlanning.py           # Core PathPlanner: WGS84, Hybrid A*, EDT, Spline
│   ├── testPathPlanning.py       # Test harness & Matplotlib visualizer
│   ├── Vehicle.py                # Kinematic vehicle state model
│   └── WGS84Def.py               # Ellipsoid datum constants
└── logic.py                      # Main entrypoint for multi-threaded vision
```

---

## 🚀 Quick Start

### 1. Installation

Clone the repository and install dependencies:

```bash
git clone https://github.com/your-username/autonomous-vehicle-stack.git
cd autonomous-vehicle-stack
pip install numpy scipy shapely matplotlib opencv-python pymavlink
```

### 2. Running Path Planning Simulation

To execute the path planner on a clustered obstacle field with full coordinate transforms and trajectory plots:

```bash
python planning/testPathPlanning.py
```

### 3. Running Perception Pipeline

To initialize the multi-threaded camera acquisition, color classification, and debug video recording:

```bash
python logic.py
```

---

## 🛠️ Engineering Highlights & Future Improvements

- [x] Full mathematical conversion pipeline: $\text{WGS84} \leftrightarrow \text{ECEF} \leftrightarrow \text{ENU} \leftrightarrow \text{Grid}$.
- [x] Heading-aware graph expansion considering vehicle footprint orientation.
- [x] Repulsion cost integration via Euclidean Distance Transform (EDT).
- [x] Asynchronous multi-queue video distribution architecture.
- [ ] Implement Reeds-Shepp or Dubins curves for analytical goal expansions.
- [ ] Migrate queue-based IPC to native ROS 2 nodes for distributed compute.
- [ ] Replace HSV segmentation with an edge-optimized YOLO inference pipeline (ONNX / TensorRT).