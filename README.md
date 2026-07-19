# SMA + Stigmergy 3D Drone Swarm

A multi-drone path-planning system that combines the Slime Mould Algorithm (SMA) with stigmergic communication (navigation and danger pheromones) to guide a swarm of simulated drones through an obstacle field toward a target, using only local, per-drone sensing — no drone ever has access to a global obstacle map.

Built and tested on PX4 SITL + Gazebo Harmonic, controlled via MAVSDK Python.

---

## Overview

Two (or more) drones spawn at a shared origin and must reach a target position while avoiding five static obstacles placed throughout the world. Each drone only knows what its own simulated proximity sensor detects — the optimization algorithm itself has zero knowledge of obstacle geometry. Coordination between drones emerges entirely through **stigmergy**: when one drone detects danger, it deposits a "danger pheromone" in a shared field that other drones read and avoid, even if they've never sensed that obstacle directly.

```
   drone position → LiDAR proximity sensor (GPS-based, local only)
                          │
                          ▼
              obstacle detected? ──yes──▶ reactive repulsion + deposit danger pheromone
                          │no
                          ▼
                 SMA exploration/exploitation step
                          │
                          ▼
              navigation pheromone gradient nudge
                          │
                          ▼
                greedy fitness-based selection
```

## Key Features

- SMA (Slime Mould Algorithm) core optimizer — biologically-inspired exploration/exploitation balance that adapts over the run
- Reactive obstacle avoidance layer** — takes priority over optimization when a drone is in immediate danger, with safety moves exempt from ordinary fitness-based rejection
- **Dual pheromone system**:
  - *Navigation pheromone* — reinforces promising regions of the search space
  - *Danger pheromone* — an emergent warning system; drones avoid zones flagged dangerous by teammates without ever sensing the obstacle themselves
- **Sequential waypoints** — the swarm can be required to visit a series of intermediate points before proceeding to the final target
- **Elite anti-stagnation** — a simulated-annealing-style mechanism prevents the current best solution from permanently freezing in place, letting it keep locally refining instead of stalling the whole run
- **Real PX4/Gazebo integration** — not a toy simulation; runs against actual PX4 SITL instances with MAVSDK offboard control, real GPS telemetry, and Gazebo physics

## Architecture

```
swarm_project/
├── main.py                      # orchestration: connect, arm, takeoff, run, land, log
├── algorithm/
│   ├── sma_core.py              # SMA + stigmergy + reactive avoidance step
│   ├── fitness.py               # fitness scoring (distance + obstacle + danger penalties)
│   └── pheromone.py             # PheromoneField3D (navigation) + DangerPheromoneField
├── sensors/
│   └── lidar.py                 # GPS-proximity-based simulated obstacle sensor
├── drone_control/
│   ├── connection.py            # MAVSDK connect / arm / offboard setup
│   ├── telemetry.py             # NED position readout
│   └── setpoint.py              # sends position setpoints to PX4
├── config/
│   └── settings.py              # all tunable constants
└── logs/                        # per-run JSON logs (auto-generated)
```

World definition (obstacles, markers, wind) lives in the PX4-Autopilot repo at `Tools/simulation/gz/worlds/windy.sdf`.

## Requirements

- PX4 Autopilot (SITL build)
- Gazebo Harmonic
- Python 3.10+, MAVSDK-Python, NumPy, SciPy
- WSL2 (if on Windows) or native Linux

## Running a Simulation

**1. Launch PX4 SITL + Gazebo:**
```bash
cd ~/PX4-Autopilot
PX4_GZ_MODEL_POSE="0,0,0.1,0,0,0" \
PX4_SIM_HOST_ADDR=127.0.0.1 \
make px4_sitl gz_x500_windy
```

**2. Launch the second drone instance** (in a separate terminal), then set:
```
param set EKF2_MAG_TYPE 0
```

**3. Run the algorithm:**
```bash
cd swarm_project
python3 main.py
```

Results are logged to `logs/run_<timestamp>.json`, including per-iteration fitness, positions, LiDAR readings, and pheromone state.

## Configuration

All tunables live in `config/settings.py`:

| Setting | Purpose |
|---|---|
| `N_AGENTS`, `MAX_ITER` | swarm size and run length |
| `TARGET`, `WAYPOINTS`, `WAYPOINT_RADIUS` | goal(s) the swarm must reach, in order |
| `WORLD_BOUNDS`, `JAMMED_ZONE` | spatial constraints |
| `Z_RANDOM`, `MAX_STEP` | SMA exploration parameters |
| `OBSTACLE_PENALTY`, `OBSTACLE_RADIUS` | fitness penalty for proximity to obstacles |
| `DANGER_PENALTY_WEIGHT`, `DANGER_DEPOSIT_AMOUNT`, `DANGER_EVAP_RATE` | danger pheromone dynamics |
| `LIDAR_DANGER_THRESHOLD`, `LIDAR_REPULSION_GAIN` | reactive avoidance sensitivity |

## How It Works

1. **SMA layer** — each drone probabilistically explores (random moves, weighted attraction toward better-performing peers) or exploits (moves toward the current best-known position), with the exploration/exploitation balance shifting over the course of the run.
2. **Stigmergy layer** — every accepted move deposits navigation pheromone proportional to how good that position is; the field diffuses and evaporates over time, forming gradients that pull the swarm toward historically good regions.
3. **Reactive layer** — if a drone's proximity sensor detects an obstacle within a safety threshold, it immediately computes a repulsion vector and deposits danger pheromone at that location — bypassing the normal optimization step entirely, since safety takes priority over search quality.
4. **Fitness** — every candidate position is scored as `distance_to_target + obstacle_penalty + danger_penalty`, and only accepted if it's an improvement (with a small controlled exception for the reactive layer and for the current-best individual, to prevent stagnation).

## Known Limitations

- Obstacle proximity near the final target constrains how close to the theoretical optimum the swarm can converge — this is a property of the world layout, not the algorithm.
- Drones can occasionally settle into an oscillation right at the obstacle-avoidance boundary distance rather than clearing it cleanly; a minimum-strength floor on the repulsion force is planned to address this.

## Roadmap

- [ ] Repulsion strength floor to eliminate boundary oscillation
- [ ] Hysteresis on the avoidance threshold (separate enter/exit distances)
- [ ] 3-condition benchmark (baseline / obstacles-only / obstacles+wind) for write-up
- [ ] Trajectory visualization / path-trace overlay in Gazebo
