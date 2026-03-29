# Drone Path Planning Simulation

A collection of real-time, interactive **3D simulations** of drones navigating through obstacles using **Artificial Potential Fields (APF)** and **Harmonic Potential Fields (HPF)** — rendered entirely in the browser with [Three.js](https://threejs.org/).

![Status](https://img.shields.io/badge/status-active-brightgreen) ![License](https://img.shields.io/badge/license-MIT-blue) ![Tech](https://img.shields.io/badge/built%20with-Three.js-black)

<p align="center">
  <img src="screenshot.png" alt="APF Drone Simulation Screenshot" width="800"/>
</p>

> No build steps, no dependencies to install. Open `index.html` in a modern browser and run instantly.

---

## Simulations

Each simulation has an **APF / HPF toggle** in the controls bar so you can switch field types on the fly and compare trajectories side-by-side.

### 1. Static Obstacles
A single drone navigates from one corner of a 10×10×10 cube to the opposite corner, avoiding 10 fixed spherical obstacles.

### 2. Moving Obstacles
**15 obstacles move continuously**, bouncing off the cube walls. The drone reacts in real time to the changing environment. The force panel visualises attractive, repulsive, and net force magnitudes each frame.

### 3. Drone Swarm
**4 drones** launch simultaneously from 4 different corners and all navigate toward the same goal. Key behaviours:
- **Fixed arrival order** — Alpha first, then Beta, Gamma, Delta — enforced by decreasing attractive gain and max speed per drone.
- **Drone-drone collision avoidance** — drones repel each other, preventing mid-air collisions.
- **Goal capture zone** — repulsive forces suspended within 1.5 units of goal to prevent corner trapping.
- **Disappear on arrival** — each drone is removed from the scene on reaching the goal.
- **Per-drone HUD** — tracks Standby → Flying → #1–#4 Arrived, highlighted in gold / silver / bronze / purple.

---

## Path Planning Algorithms

Both field types are reactive — at each time step, virtual forces act on the drone and it moves in the direction of the net force. No pre-computed path is needed.

```
F_total = F_attractive + F_repulsive_obstacles + F_repulsive_walls [+ F_repulsive_drones]
```

---

### Artificial Potential Field (APF)

#### Attractive Force
Conic-parabolic hybrid:
- **Close range** (`d < D_STAR`): `F_att = K_ATT × (goal − pos)` — linear
- **Far range** (`d ≥ D_STAR`): `F_att = K_ATT × D_STAR × normalize(goal − pos)` — capped

#### Repulsive Force
Each obstacle repels within a hard influence radius `D0`:
```
F_rep = K_REP × (1/d − 1/D0) / d²    [only if d < D0, else 0]
```
Walls use the same formula with `K_WALL` and `D_WALL`.

#### Local Minima & Stuck Detection
If velocity drops below threshold for 40+ consecutive frames, a random perturbation is injected to escape the local minimum.

---

### Harmonic Potential Field (HPF)

HPF is based on the **fundamental solution to Laplace's equation in 3D** (the Newtonian / Coulomb potential, `φ = 1/r`), whose gradient gives a **1/r² force** — the same law as gravity and electrostatics.

Key properties:
- **No hard cutoff radius** — every obstacle always contributes, giving globally smooth, curl-free navigation.
- **Faster falloff (1/r²)** than APF's complex formula, so distant obstacles don't dominate.
- **No spurious local minima** by construction (superposition of harmonic functions is harmonic).

#### Attractive Force
Constant magnitude toward goal (gradient of the linear potential `K·r`), ensuring goal-ward force at every position:
```
F_att = K_ATT × normalize(goal − pos)
```

#### Repulsive Force
Coulomb 1/r² — globally aware, no cutoff:
```
F_rep = K_REP / d²    [for all d, where d = surface distance to obstacle]
```
Walls and drone-drone repulsion use the same formula.

#### APF vs HPF at a glance

| | APF | HPF |
|---|---|---|
| Repulsion formula | `K(1/d − 1/D₀)/d²` | `K/d²` (Coulomb) |
| Influence radius | Hard cutoff at `D0` | None — global |
| Force falloff | Complex, zero beyond `D0` | `1/d²`, always non-zero |
| Local minima | Possible | Fewer by construction |
| Trail colour | Cyan | Purple |

---

## Parameters

### APF

| Parameter | Moving Obstacles | Swarm | Description |
|---|---|---|---|
| `K_ATT` | 2.5 | 3.2 / 2.6 / 2.0 / 1.5 | Attractive gain (per drone in swarm) |
| `K_REP` | 40.0 | 40.0 | Obstacle repulsion gain |
| `D0` | 3.8 | 3.8 | Obstacle influence radius |
| `K_WALL` | 14.0 | 14.0 | Wall repulsion gain |
| `D_WALL` | 2.2 | 2.2 | Wall influence radius |
| `K_DRONE_REP` | — | 28.0 | Drone-drone repulsion gain |
| `D_DRONE` | — | 2.2 | Drone-drone influence radius |
| `VMAX` | 0.18 | 0.22 / 0.17 / 0.14 / 0.11 | Max speed (per drone in swarm) |

### HPF

| Parameter | Moving Obstacles | Swarm | Description |
|---|---|---|---|
| `K_ATT` | 6.0 | 3.2–6.4 (× 2.0 mult) | Constant attractive gain |
| `K_REP` | 3.0 | 3.0 | Coulomb repulsion gain (no cutoff) |
| `K_WALL` | 1.5 | 1.5 | Coulomb wall repulsion gain |
| `K_DRONE_REP` | — | 2.0 | Coulomb drone-drone repulsion gain |
| `D_MIN` | 0.3 | 0.3 | Min surface distance (singularity guard) |

---

## Controls

| Input | Action |
|---|---|
| Left-click + drag | Orbit / rotate camera |
| Scroll wheel | Zoom in / out |
| Touch drag | Orbit (mobile) |
| Launch / Pause button | Start or pause the simulation |
| Reset button | Reset all drones to start positions |

---

## Getting Started

```bash
git clone https://github.com/Jitendra4Jalwaniya/apf_drone_simulation.git
cd apf_drone_simulation
open index.html   # landing page with links to all three simulations
```

> Requires any modern browser with WebGL support (Chrome, Firefox, Safari, Edge).

---

## Tech Stack

- **[Three.js](https://threejs.org/)** (r128) — 3D rendering via WebGL
- **Vanilla HTML / CSS / JavaScript** — no frameworks, no build tools
- **Google Fonts** — Share Tech Mono, Exo 2

---

## TODO

- [x] Introduce a new version containing **harmonic potential fields** for smoother, curl-free navigation without local minima.
- [ ] Create bigger drone swarm (16-50-100 drones) with single target, all hitting it in random order.
- [ ] Turn the obstacles in interceptors.
- [ ] Scan a room with lot of objects lying around, convert it into a real 3D model of the room, then ask the drone to navigate from one corner to another.

---

## Acknowledgements

The idea originated from a conversation with **[Google Gemini](https://gemini.google.com)**, where the discussion on drone swarm algorithms introduced me to Artificial Potential Fields. You can read the [original conversation here](https://gemini.google.com/share/33bc55619bca).

The simulations were built with the help of **[Claude](https://claude.ai)**.

---

<p align="center"><sub>Built with APF and HPF — artificial and harmonic potential fields</sub></p>
