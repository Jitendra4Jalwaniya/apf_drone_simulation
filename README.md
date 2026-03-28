# 🚁 APF Drone Simulation

A real-time, interactive **3D simulation** of a drone navigating through obstacles using the **Artificial Potential Field (APF)** path planning algorithm — rendered entirely in the browser with [Three.js](https://threejs.org/).

> **[▶ Live Demo](#)** — Open `apf_drone_sim.html` in any modern browser to run the simulation instantly. No build steps, no dependencies to install.

![Status](https://img.shields.io/badge/status-active-brightgreen) ![License](https://img.shields.io/badge/license-MIT-blue) ![Tech](https://img.shields.io/badge/built%20with-Three.js-black)

<p align="center">
  <img src="screenshot.png" alt="APF Drone Simulation Screenshot" width="800"/>
  <br/>
  <em>The drone navigating through obstacles toward the goal — cyberpunk HUD with real-time telemetry, force vectors, and APF trail.</em>
</p>

---

## 📖 Table of Contents

- [Overview](#overview)
- [The APF Algorithm](#the-apf-algorithm)
  - [How It Works](#how-it-works)
  - [Attractive Force](#1-attractive-force-f_att)
  - [Repulsive Force (Obstacles)](#2-repulsive-force-from-obstacles-f_rep)
  - [Repulsive Force (Walls)](#3-repulsive-force-from-walls)
  - [Net Force & Motion](#4-net-force--motion)
  - [Local Minima & Stuck Detection](#5-local-minima--stuck-detection)
- [Features](#features)
- [Getting Started](#getting-started)
- [Controls](#controls)
- [Simulation Parameters](#simulation-parameters)
- [Tech Stack](#tech-stack)
- [Project Structure](#project-structure)
- [Acknowledgements](#acknowledgements)

---

## Overview

This project visualizes how a drone can autonomously navigate from a **start point** to a **goal point** in a 3D environment filled with spherical obstacles — all without any pre-computed map or global path planner.

The drone operates inside a **10×10×10 unit bounding cube** and must fly from corner `(-4.2, -4.2, -4.2)` to the opposite corner `(4.2, 4.2, 4.2)`, avoiding **10 obstacles** placed along the diagonal path.

The entire simulation runs client-side in a single HTML file with no backend, no build process, and no external dependencies beyond the Three.js CDN.

---

## The APF Algorithm

### How It Works

The **Artificial Potential Field (APF)** method is a reactive path planning approach widely used in robotics. Instead of computing a full path upfront, the robot (drone) at each time step computes virtual **forces** acting on it and moves in the direction of the **net force**.

The environment is modeled as a potential field with two components:

- **Attractive Potential**: The goal creates a "valley" that pulls the drone toward it.
- **Repulsive Potential**: Each obstacle (and wall) creates a "hill" that pushes the drone away.

The drone simply "rolls downhill" in this combined potential field, naturally steering toward the goal while avoiding collisions.

```
F_total = F_attractive + F_repulsive
```

The drone moves in the direction of `F_total` at each time step.

---

### 1. Attractive Force (F_att)

The goal generates an attractive force that pulls the drone toward it. This simulation uses a **conic-parabolic hybrid**:

- **Close range** (distance < 2 units): Parabolic — force proportional to distance
  ```
  F_att = K_ATT × (goal - position)
  ```
- **Far range** (distance ≥ 2 units): Conic — constant magnitude in the goal direction
  ```
  F_att = K_ATT × 2.0 × normalize(goal - position)
  ```

The hybrid approach prevents excessively large forces when the drone is far away while ensuring smooth deceleration near the goal.

---

### 2. Repulsive Force from Obstacles (F_rep)

Each obstacle generates a repulsive force that pushes the drone away, but **only within an influence radius** `D0`:

```
If d < D0:
    F_rep = K_REP × (1/d - 1/D0) × (1/d²) × direction_away
```

Where:
- `d` = distance from drone to obstacle surface
- `D0` = influence radius (2.2 units in this simulation)
- `K_REP` = repulsive gain (3.5)

**Key properties:**
- The force grows rapidly as the drone approaches an obstacle (inversely proportional to d²)
- The force smoothly drops to zero at exactly `d = D0`
- Beyond `D0`, the obstacle has zero effect — keeping computation efficient

---

### 3. Repulsive Force from Walls

The six walls of the bounding cube act as additional repulsive surfaces, using the same formula as obstacles but with separate tuning parameters:

- `WALL_K_REP = 1.2` (weaker than obstacle repulsion)
- `WALL_D0 = 1.2` (smaller influence distance)

This keeps the drone contained within bounds without overpowering the obstacle avoidance.

---

### 4. Net Force & Motion

At each time step:

```
F_total = F_att + Σ F_rep(obstacles) + Σ F_rep(walls)
step = normalize(F_total) × BASE_SPEED × speed_multiplier
new_position = position + step
```

The drone moves at a constant step size in the direction of the net force, ensuring smooth and predictable motion regardless of force magnitudes.

---

### 5. Local Minima & Stuck Detection

A well-known limitation of APF is **local minima** — positions where attractive and repulsive forces cancel out, trapping the drone. This simulation handles it with a **stuck detector**:

- If the drone hasn't moved more than `0.01` units in **60 consecutive frames**, it's considered stuck
- A **random perturbation** force is injected to escape the local minimum:
  ```
  F_total += random_vector × 2.5
  ```
- The timer resets once the drone starts moving again

---

## Features

| Feature | Description |
|---|---|
| 🌐 **3D Visualization** | Full Three.js scene with perspective camera, fog, and starfield |
| 🎯 **Real-time APF** | Forces computed and applied every frame |
| 🏹 **Force Arrows** | Green (attractive), Red (repulsive), White (net) arrows on the drone |
| 📊 **HUD Telemetry** | Live readout of position, distance to goal, force magnitude |
| 📈 **Progress Bar** | Visual progress indicator toward the goal |
| 🌈 **Color Trail** | Cyan-to-blue fading trail showing the flight path |
| 🔄 **Orbit Camera** | Click-drag to rotate, scroll to zoom, touch supported |
| ⏯️ **Play/Pause/Reset** | Full playback control |
| 🏎️ **Speed Control** | Adjustable simulation speed (0.5× to 5×) |
| 🔴 **10 Obstacles** | Semi-transparent red spheres with wireframe overlays |
| 💥 **Stuck Detection** | Random perturbation to escape local minima |
| 📱 **Responsive** | Works on desktop & mobile (touch events supported) |
| 🎨 **Cyberpunk UI** | Scanline overlay, glassmorphism, Orbitron font, glowing accents |

---

## Getting Started

No installation required! Simply:

1. **Clone the repository:**
   ```bash
   git clone https://github.com/<your-username>/apf_drone_simulation.git
   cd apf_drone_simulation
   ```

2. **Open in browser:**
   ```bash
   open apf_drone_sim.html
   ```
   Or simply double-click the `apf_drone_sim.html` file.

3. **Watch the drone fly!** 🚁

> **Requirements:** Any modern browser (Chrome, Firefox, Safari, Edge) with WebGL support.

---

## Controls

| Input | Action |
|---|---|
| 🖱️ **Left-click + drag** | Orbit / rotate the camera |
| 🖱️ **Scroll wheel** | Zoom in / out |
| 📱 **Touch drag** | Orbit (mobile) |
| ⏸️ **Pause button** | Pause / resume the simulation |
| ↺ **Reset button** | Reset drone to start position |
| 🎚️ **Speed slider** | Adjust simulation speed (0.5× – 5×) |

---

## Simulation Parameters

| Parameter | Value | Description |
|---|---|---|
| `K_ATT` | 1.4 | Attractive force gain |
| `K_REP` | 3.5 | Repulsive force gain (obstacles) |
| `D0` | 2.2 | Obstacle influence radius |
| `WALL_K_REP` | 1.2 | Wall repulsive force gain |
| `WALL_D0` | 1.2 | Wall influence distance |
| `GOAL_THRESH` | 0.35 | Arrival detection threshold |
| `BASE_SPEED` | 0.038 | Base movement speed per frame |
| `CUBE_SIZE` | 10 | Bounding cube dimensions |

These can be modified directly in the `<script>` section of `apf_drone_sim.html` to experiment with different behaviors.

---

## Tech Stack

- **[Three.js](https://threejs.org/)** (r128) — 3D rendering via WebGL
- **Vanilla HTML/CSS/JavaScript** — No frameworks, no build tools
- **Google Fonts** — [Orbitron](https://fonts.google.com/specimen/Orbitron) & [Share Tech Mono](https://fonts.google.com/specimen/Share+Tech+Mono)

---

## Project Structure

```
apf_drone_simulation/
├── apf_drone_sim.html   # Complete self-contained simulation
└── README.md            # This file
```

---

## Acknowledgements

This project was created with the help of **[Claude](https://claude.ai)** by Anthropic — an AI assistant that helped design, develop, and document this simulation.

---

<p align="center">
  <sub>Built with ☕ and artificial potential fields</sub>
</p>