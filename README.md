# MDFly: High-Fidelity 6-DOF C++ Flight Dynamics Simulator & Autopilot

MDFly is a high-performance, physics-based 6-Degrees-of-Freedom (6-DOF) flight dynamics simulator written in C++. It features realistic aerodynamic modeling, electric motor-propeller coupling (with gyroscopic and torque reactions), spring-damper landing gear mechanics with ground friction, cascaded PID autopilots, and an advanced **Lead Pursuit** guidance system. 

It includes a real-time **3D WebGL Web Visualizer** with a nose-mounted FPV camera feed, horizon-stabilized chase view, and a simulated **YOLO bounding-box HUD overlay** for target tracking, alongside **MAVLink telemetry** streaming to ground control stations like QGroundControl.

---

## 🚀 Key Features

*   **6-DOF Rigid-Body Dynamics**: Quaternion-based attitude representation to prevent gimbal lock, integrating gravitational, aerodynamic, thrust, and ground reaction forces.
*   **Aerodynamic Table Lookups**: Interpolated lift, drag, and moment coefficient tables matching the JSBSim model profiles (e.g., Rascal 110).
*   **Propulsion Physics**: Captures motor torque reactions and propeller gyroscopic precession moments acting on the aircraft frame, configurable via JSON.
*   **Landing Gear & Ground Physics**:
    *   Spring-damper vertical suspension force calculations: $F_N = K_{spring} \cdot d_{comp} + C_{damper} \cdot v_{comp}$
    *   Rolling friction ($\mu_{roll} = 0.03$) and dynamic braking friction ($\mu_{brake} = 0.35$) models.
    *   Parasitic drag penalty integrated when landing gear is deployed.
*   **Lead Pursuit Guidance**: Follower predicts the target drone's future intercept coordinate by projecting its velocity vector forward in time ($t_{lead}$), steering toward the intercept point for fluid tracking.
*   **3D WebGL Dashboard Visualizer**:
    *   Three.js-based flight animation in real time.
    *   First-Person View (FPV) nose camera feed rendered via a secondary WebGL pipeline.
    *   2D HUD annotation canvas overlay representing artificial horizon and Pseudo-YOLO tracking bounding boxes.
    *   Horizon-stabilized follow camera locking to aircraft heading and position while avoiding roll/pitch tilt instability.
*   **MAVLink & WebSockets**: Standardized telemetry broadcasting to QGroundControl (MAVLink over UDP) and the WebSocket bridge for the web visualizer.

---

## 📐 Mathematical Framework

### 1. Translational & Rotational Equations of Motion (6-DOF)
The state of the aircraft is defined by its position $\begin{bmatrix} p_n & p_e & p_d \end{bmatrix}^T$ in the Inertial Frame (NED) and velocity $\begin{bmatrix} u & v & w \end{bmatrix}^T$, attitude quaternions $\mathbf{q} = \begin{bmatrix} q_0 & q_1 & q_2 & q_3 \end{bmatrix}^T$, and angular rates $\begin{bmatrix} p & q & r \end{bmatrix}^T$ in the Body Frame.

$$\dot{\mathbf{v}}_b = \frac{1}{m}\mathbf{F}_{total} - \boldsymbol{\omega}_b \times \mathbf{v}_b$$

$$\dot{\boldsymbol{\omega}}_b = \mathbf{J}^{-1} \left( \mathbf{M}_{total} - \boldsymbol{\omega}_b \times (\mathbf{J}\boldsymbol{\omega}_b) \right)$$

### 2. Motor & Propeller Reaction Forces
The motor creates thrust $F_{thrust}$ and counter-torque $Q_{motor}$ along the body $x$-axis. The spinning propeller behaves as a gyroscope; rotation rates pitch ($q$) and yaw ($r$) induce a precession moment:

$$\mathbf{M}_{gyro} = \begin{bmatrix} 0 \\ -J_p \Omega_{prop} r \\ J_p \Omega_{prop} q \end{bmatrix}$$

### 3. Lead Pursuit Intercept Extrapolation
Instead of pointing directly at the target's current position, the tracking autopilot computes an intercept point using the target's current velocity vector:

$$\mathbf{p}_{intercept} = \mathbf{p}_{target} + \mathbf{v}_{target} \cdot t_{lead}$$

where $t_{lead}$ is dynamically adjusted based on range. The autopilot steers the follower's attitude reference toward this future coordinate.

---

## 📂 Repository Structure

```
├── CMakeLists.txt              # CMake build configuration
├── README.md                   # Project documentation
├── mdfly_config.json           # Global physics and initial config
├── motor_prop_config.json      # Motor/Propeller specifications
├── rascal_param.json           # Aerodynamic parameters (Rascal 110)
├── run_scenario1.sh            # Run script for Formation Flight
├── run_scenario2.sh            # Run script for Active Pursuit Tracking
├── run_scenario3.sh            # Run script for Takeoff-Cruise-Landing
├── include/mdfly/              # Header files (Simulator, PID, MAVLink, Models)
├── src/                        # C++ source implementations
├── examples/                   # Scenario C++ entrypoints
└── visualizer/                 # HTML5/JS WebGL flight dashboard
```

---

## 🛠️ Build and Installation

### Prerequisites
*   CMake (>= 3.10)
*   C++17 Compiler (GCC/Clang)
*   Node.js (for running the WebSocket UDP bridge)

### Compilation
1. Create a build directory and compile:
   ```bash
   mkdir -p build && cd build
   cmake ..
   make -j$(nproc)
   ```

2. Start the WebSocket bridge dependencies in the visualizer folder:
   ```bash
   cd visualizer
   npm install ws  # If not already installed
   ```

---

## 🏃 Running Scenarios

Each scenario compiles into a separate executable and includes a shell script that starts the C++ simulation, launches the Python/Node WebSocket telemetry bridge, and prepares visualizer outputs.

### Scenario 1: Passive Formation Flight
Lider İHA flies a sinusoidal waving pattern while the follower tracks it passively.
```bash
./run_scenario1.sh
```

### Scenario 2: Active Lead Pursuit Tracking (YOLO HUD)
Follower İHA closes the distance to the leader and maintains a 60-meter separation using predictive lead-pursuit guidance laws. 
```bash
./run_scenario2.sh
```
*   **Visualizer View**: Open `visualizer/index.html` in your browser. You will see both İHA models in the 3D viewport, the nose camera FPV feed, and the active YOLO target tracking rectangle.

### Scenario 3: Takeoff, Cruise, and Landing Mission
A single UAV performs ground taxi, takeoff rotation, landing gear retraction, circuit flight, wheel deployment, touchdown flare, and rollout braking.
```bash
./run_scenario3.sh
```

---

## 📺 Flight Dashboard Visualizer

The web visualizer (`visualizer/index.html`) is built on **Three.js** and communicates via WebSockets on port `8765`. 

1.  **3D Flight Environment**: Follows the active UAV with a gimbal-stabilized camera. Rendered models animate surface control deflections (ailerons, elevators) and landing gear retracts.
2.  **FPV Camera Viewport**: Renders the view from a forward-looking camera mounted on the nose of the tracking UAV.
3.  **Pseudo-YOLO HUD**: Computes target bounds projection from 3D world coords to the nose camera's 2D image plane, rendering a bounding box target annotation on screen without high GPU overhead.
4.  **Glassmorphism HUD Telemetry**: Real-time glassmorphism gauges for Altitude, Heading, Airspeed, Energy (Potential/Kinetic), and Linear/Angular Momentum conservation.
