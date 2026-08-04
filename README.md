# Framework for inverse animation editing based on differentiable simulation
This is my Bachelor's thesis. The aim was to develop a tool for Unity which allowed the estimation of various simulation parameters within Unity.
It includes a differentiable physics engine built from scratch in C++, a Python module that does the optimizing using backpropagation to calculate the gradient of the loss function and a Unity project to display the results.

![My Image](Screenshots/4corners.gif)

The framework is able to generate a simulation with some parameters for the mass and stiffness of a cloth and use numerical optimization combined with differentiable simulation and the backpropagation algorithm to guess those parameters based on the position of that simulation over many frames.

# Building

## System dependencies

### Linux (Fedora)
```sh
sudo dnf install glslc python3-devel
```
Wayland development headers (`wayland-devel`) are typically preinstalled on desktop
systems.  If you prefer X11 add the X11 devel packages described in the Ubuntu
section below and set `GLFW_BUILD_WAYLAND=OFF GLFW_BUILD_X11=ON` in the viewer
CMakeLists.

### Linux (Ubuntu/Debian)
```sh
sudo apt install cmake g++ \
  libvulkan-dev glslang-tools glslc \
  libglfw3-dev libxinerama-dev libxrandr-dev libxcursor-dev libxi-dev \
  python3-dev
```

### macOS
```sh
brew install cmake glfw molten-vk glslang
```

### Windows
Install [Vulkan SDK](https://vulkan.lunarg.com/), [CMake](https://cmake.org/), and
Visual Studio 2022 with C++ workload.  The Vulkan SDK bundles `glslc`, headers,
and the loader.  FetchContent handles Eigen, nlohmann\_json, pybind11, GLFW,
and ImGui automatically.

## Build
```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Options (all default to `ON`):

| Flag | Effect |
|------|--------|
| `-DDPE_BUILD_PYTHON=OFF` | Skip Python module (no Python headers required) |
| `-DDPE_BUILD_UNITY_BRIDGE=OFF` | Skip Unity C API shared library |
| `-DDPE_BUILD_TESTS=OFF` | Skip unit tests |
| `-DDPE_BUILD_VIEWER=OFF` | Skip standalone Vulkan viewer |

## Artifacts

* `build/python/DifferentiablePhysicsEngine.*.so` — Python module.
  Add `build/python` to `PYTHONPATH` and `import DifferentiablePhysicsEngine`.
* `build/unity/libDifferentiablePhysicsEngine.so` — Unity native plugin
  (copied automatically into `UnityProject/Assets/`).
* `build/dpe_tests` — unit test executable.  Run with `ctest --test-dir build`.
* `build/dpe_viewer` — standalone Vulkan real-time viewer.

## Running the viewer
```sh
build/dpe_viewer UnityProject/Assets/Files/side.txt
```

Controls: **left-click drag** to orbit, **W/S** to zoom, **R** to reset camera, **Esc** to quit.

## Python dependencies
```sh
python -m pip install -r requirements.txt
```

# How it works

The framework has four components that work together:

```
                      ┌──────────────┐
                      │  Scene JSON  │  ← hand-authored or exported from Unity
                      └──────┬───────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
    ┌──────────────────┐ ┌──────────┐ ┌───────────────┐
    │ Python Optimizer │ │ dpe_core │ │ Vulkan Viewer │
    │  (scipy L-BFGS-B)│◄┤ engine   ├►│  (real-time)  │
    └────────┬─────────┘ │ (C++17)  │ └───────────────┘
             │           └────┬─────┘
             ▼                │
    ┌─────────────────┐       │
    │ Unity (optional) │◄─────┘  (via C API bridge)
    └─────────────────┘
```

**`dpe_core`** — the physics engine.  A mass-spring cloth simulator with implicit
Euler integration.  Crucially it is *differentiable*: every force has an
analytical Jacobian (dF/dx, dF/dv), and the whole integration step can be
backpropagated through.  This is what lets the optimizer compute gradients
without finite differences.

**Python optimizer** — wraps the engine via pybind11.  Given a scene JSON
with known parameters and a target animation, scipy's L‑BFGS‑B minimizes the
mean-squared position error by adjusting mass/stiffness values.  The gradient
is computed by backpropagating through every simulation step (adjoint method).

**Vulkan viewer** — standalone C++ application.  Loads a scene JSON, runs the
simulation on a background thread, and renders the cloth mesh in real time with
an orbit camera.  No Unity or Python required.

**Unity** (optional) — the original visualization front-end.  The engine is
loaded as a native plugin via P/Invoke and the cloth is rendered with Unity's
mesh system.

## Scene JSON format

Simulations are defined by JSON files (see `UnityProject/Assets/Files/` for
examples).  Key fields:

```jsonc
{
  "title":               "my scene",
  "delta":               0.02,        // time step (seconds)
  "integrationMethod":   2,           // 0=explicit  1=symplectic  2=implicit
  "tolerance":           0.001,       // CG solver tolerance
  "optimizationIterations": 100,      // number of simulation frames
  "forwardSubSteps":     1,           // substeps per frame
  "printTimes":          false,
  "windVel":             {"x":0, "y":0, "z":0},
  "objects": [{
    "vertPos":           [{"x":0,"y":1,"z":0}, …],
    "vertIsFixed":       [true, false, …],
    "vertMass":          [1.0, 1.0, …],
    "springs":           [0, 1, 1, 2, …],   // pairs of vertex indices
    "springStiffness":   [200.0, 200.0, …],
    "triangles":         [0, 1, 2, …],      // triples of vertex indices
    "dragCoefficient":   1.0,
    "damping":           0.5,
    "optimizationSettings": "nG"  // mass-mode + stiffness-mode:
  }],                             //   n=ignore  G=global  L=local
  "colliders": []
}
```

**`optimizationSettings`** controls which parameters the optimizer tunes.
A two-character string per object:

| Mode | Meaning |
|------|---------|
| `n` / `N` | Ignore this parameter (keep it fixed) |
| `G` / `g` | Treat as one *global* value for the object |
| `L` / `l` | Treat as a *local* value per vertex / spring |

Example: `"nL"` = keep mass fixed, optimize every spring stiffness individually.

## Running the optimizer

Build the Python module (requires `python3-devel`):

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Add the module to your Python path and install dependencies:

```sh
export PYTHONPATH="$PWD/build/python:$PYTHONPATH"   # Linux / macOS
pip install -r requirements.txt
```

Run one of the experiment scripts from the `Python/` directory:

```sh
cd Python
python Experiment1.py
```

`Experiment1.py` recovers the cloth parameters for each scene
(`side`, `corners`, `flag`) with four parameterizations (homogeneous /
heterogeneous mass, homogeneous / heterogeneous stiffness) and prints a
results table.  `Experiment2.py` repeats this with different numbers of
simulation steps to study how the error metric behaves (thesis Chapter 5).

The scripts load scene files from `UnityProject/Assets/Files/`, run the
L‑BFGS‑B optimization, print the results, and write optimized scene files into
`UnityProject/Assets/Files/Experiment1/` and `Experiment2/`.  You can then open
these in the viewer:

```sh
build/dpe_viewer UnityProject/Assets/Files/Experiment1/side_nL_optimized.txt
```

The green wireframe shows the simulation with the *recovered* parameters;
visually it should closely follow the target animation you generated.

# Results

The error metric is the root mean square position difference between the
optimized simulation and the target, per vertex per frame.  Reported values
below were produced on a Ryzen-class desktop; runtimes scale roughly with the
number of simulation steps and parameters.

## Experiment 1: parameter recovery

Each scene is optimized with four parameterizations — `Gn` (one global mass),
`Ln` (per-vertex masses), `nG` (one global stiffness), `nL` (per-spring
stiffnesses).

| Scene | Settings | Error | Time (s) |
|-------|----------|-------|----------|
| side (one edge fixed) | `Gn` | 3.04 | 16.0 |
|                       | `Ln` | 1.53 | 11.0 |
|                       | `nG` | 0.50 | 28.4 |
|                       | `nL` | 0.04 | 16.4 |
| corners (four corners fixed) | `Gn` | 0.82 | 15.5 |
|                              | `Ln` | 0.05 | 41.2 |
|                              | `nG` | 1.00 | 21.8 |
|                              | `nL` | 0.11 | 44.8 |
| flag (wind) | `Gn` | 32.69 (fails to converge) | 0.1 |
|             | `Ln` | 0.72 | 5.2 |
|             | `nG` | 0.21 | 1.1 |
|             | `nL` | 0.00 | 46.3 |

Observations:

- Local parameterizations (`nL`) recover the parameters almost perfectly
  (error ≈ 0.04 or less), confirming the analytic gradients through the
  implicit solver are correct.
- Global stiffness (`nG`) matches the target well even though the target was
  generated with per-spring stiffnesses — the optimizer finds the single value
  that best reproduces the overall behavior.
- The flag scene with a single global mass (`Gn`) cannot be fitted at all: the
  optimizer diverges immediately (error 32.69).  A cloth under wind needs
  spatial parameter variation.

## Experiment 2: effect of the number of optimization steps

`Experiment2.py` optimizes heterogeneous stiffness (`nL`).  Each row is a
separate optimization run, fitted over N frames (20, 40, ..., 200).  Two errors
are reported per run:

- **Error (N steps)** — how well the recovered parameters reproduce the target
  over the same N frames they were fitted to (the training error).
- **Error (500 steps)** — the *same* recovered parameters, re-simulated over
  500 frames and compared to the 500-frame target.  This is **not** a separate
  optimization: it validates whether the N-step solution generalizes beyond
  the horizon it was fitted to (thesis §5.4.1 — a solution can fit the first N
  frames perfectly and diverge later).

| Scene | Steps | Error (N steps) | Error (500 steps) | Time (s) |
|-------|-------|-----------------|-------------------|----------|
| side  | 20    | 0.00            | 0.00              | 12.6     |
|       | 40    | 0.00            | 0.00              | 29.1     |
|       | 60    | 0.02            | 0.00              | 13.0     |
|       | 80    | 0.03            | 0.00              | 15.7     |
|       | 100   | 0.04            | 0.00              | 15.4     |
|       | 120   | 0.04            | 0.00              | 18.7     |
|       | 140   | 0.04            | 0.00              | 30.4     |
|       | 160   | 0.04            | 0.00              | 31.3     |
|       | 180   | 0.04            | 0.00              | 25.1     |
|       | 200   | 0.03            | 0.00              | 57.8     |
| flag  | 20    | 0.00            | 0.00              | 44.2     |
|       | 40    | 0.05            | 0.00              | 7.4      |
|       | 60    | 0.08            | 0.00              | 6.0      |
|       | 80    | 0.48            | 0.00              | 9.8      |
|       | 100   | 0.61            | 0.00              | 14.6     |
|       | 120   | 1.14            | 0.00              | 18.5     |
|       | 140   | 1.02            | 0.00              | 11.6     |
|       | 160   | 0.67            | 0.00              | 129.0    |
|       | 180   | 57.89           | 0.00              | 47.1     |
|       | 200   | 71.00           | 0.00              | 65.3     |

Observations:

- The side scene is robust: the recovered parameters are essentially perfect
  at every step count (error ≤ 0.04, and 0.00 on the 500-frame evaluation).
- The flag scene shows the instability discussed in the thesis (§5.4.1):
  longer optimization horizons lead the optimizer into local minima, and the
  training error explodes at 180–200 steps.  The 500-frame validation stays at
  0.00 in these runs, meaning the solutions found there still happen to
  reproduce the 500-frame target simulation exactly — the failure is in the
  optimization itself, not in the recovered simulation.

## Using Unity (optional)

1. Build the Unity bridge:

   ```sh
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   ```

   The build automatically copies the native library into
   `UnityProject/Assets/`.

2. Open `UnityProject/` in **Unity 2020.3 LTS** or newer.

3. Open the `Simulation` scene in `Assets/Scenes/`.

4. Press **Play**. The cloth simulates in real time using the C++ engine
   running on a separate thread, with vertex positions streamed back to Unity
   via the C API bridge.

5. The `Optimizer` GameObject in the scene can invoke the Python optimizer
   at runtime (requires Python 3 installed and the engine module importable).
