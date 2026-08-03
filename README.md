# F1 Vehicle Dynamics Simulator

A standalone C++ simulation of a Formula One car's dynamics — a full nonlinear
bicycle model with tire grip limits, aerodynamic downforce, load transfer, and
a pure-pursuit path-following controller that can drive the car around a
closed-loop test circuit. Telemetry is logged to CSV and plotted with a small
Python/matplotlib utility.

**Author:** Nicholas Caesar
**Stack:** C++17, CMake, Python 3 (pandas, matplotlib)

> This project originally set out to build on top of the CARLA simulator. It
> has since pivoted to a self-contained physics simulation with no external
> simulator dependency — everything below describes what actually exists in
> the code today.

---

## Table of Contents

- [What's implemented](#whats-implemented)
- [Building and running](#building-and-running)
- [The physics model](#the-physics-model)
- [Scenarios and what their results mean](#scenarios-and-what-their-results-mean)
- [Plotting telemetry](#plotting-telemetry)
- [Project layout](#project-layout)
- [Known gaps / not yet implemented](#known-gaps--not-yet-implemented)

---

## What's implemented

- A **dynamic bicycle model** (`src/vehicle/`) with a linear tire model,
  aerodynamic downforce, longitudinal load transfer under acceleration/braking,
  and a **friction-circle grip limit** so cornering force is physically bounded
  instead of unlimited.
- A **path-following controller** (`src/track/`) — a synthetic closed-loop test
  track plus a pure-pursuit steering controller that drives the car around it
  autonomously.
- **Three demo scenarios** in `main.cpp` that exercise both of the above (see
  below).
- CSV telemetry output and a **Python plotting script** (`src/utils/plot_run.py`)
  that renders trajectory, speed, yaw rate, and lateral-g from any run.

## Building and running

```bash
cmake -S . -B build
cmake --build build
./build/f1sim
```

Run it from the repo root — it writes CSVs to `results/telemetry/` using a
relative path. Each run overwrites the three sample CSVs and prints a summary
line per scenario, e.g.:

```
Grip-limit demo (15 deg): final speed 3.6 km/h, wrote results/telemetry/run_grip_limit.csv
Normal corner (3 deg): final speed 317.147 km/h, wrote results/telemetry/run_normal.csv
Circuit lap (pure pursuit): completed lap 1 at t=19.917s
Circuit lap (pure pursuit): final speed 88.7437 km/h, 1 lap(s) completed, wrote results/telemetry/run_circuit.csv
```

## The physics model

### `CarParams` (src/vehicle/vehicle.h)

| Variable | Meaning | Value |
| --- | --- | --- |
| `mass` | Vehicle mass | 798.0 kg (FIA minimum) |
| `Iz` | Yaw moment of inertia | 1000.0 kg·m² |
| `a` | CG to front axle distance | 1.60 m |
| `b` | CG to rear axle distance | 2.00 m |
| `cgHeight` | CG height above ground | 0.30 m |
| `trackWidth` | Distance between left/right wheels | 1.60 m *(declared, not yet used — see [Known gaps](#known-gaps--not-yet-implemented))* |
| `Cf` / `Cr` | Front/rear tire cornering stiffness | 180,000 / 220,000 N/rad |
| `maxDriveF` | Max drive force | 12,000 N |
| `maxBrakeF` | Max brake force | 45,000 N |
| `dragCoeff` | Lumped drag term (½·ρ·Cd·A) | 1.10 |
| `liftCoeff` | Downforce coefficient (N per (m/s)²) | 3.20 |
| `muTire` | Peak tire friction coefficient | 1.60 |

### `State` (src/vehicle/vehicle.h)

`x`, `y` (m, world frame position) · `yaw` (rad, heading) · `vx`, `vy` (m/s,
body-frame longitudinal/lateral velocity) · `r` (rad/s, yaw rate).

### Per-step physics (`step()` in src/vehicle/vehicle.cpp)

**1. Slip angles** — the angle between where each axle's tires point and the
direction they're actually travelling:

```
alphaF = atan2(vy + a*r, vx) - steer
alphaR = atan2(vy - b*r, vx)
```

**2. Raw linear tire forces** — lateral force opposing slip angle, uncapped:

```
FyfRaw = -Cf * alphaF
FyrRaw = -Cr * alphaR
```

**3. Longitudinal force:**

```
drag = dragCoeff * vx^2
Fx   = throttle*maxDriveF - brake*maxBrakeF - drag
```

**4. Axle normal loads** (`axleLoads()`) — static weight distribution by CG
position, plus aero downforce (split 50/50 front/rear), plus longitudinal load
transfer under acceleration/braking:

```
staticFront = mass*g * b/(a+b)
staticRear  = mass*g * a/(a+b)
downforce   = liftCoeff * vx^2      (split 50/50 front/rear)
transfer    = mass * ax * cgHeight / (a+b)   (ax estimated as Fx/mass)

Fzf = staticFront + downforce/2 - transfer   (clamped >= 0)
Fzr = staticRear  + downforce/2 + transfer   (clamped >= 0)
```

Braking (`ax < 0`) loads the front axle; accelerating (`ax > 0`) loads the
rear — standard weight-transfer behavior.

**5. Friction-circle grip limit** — each axle has one shared grip budget for
combined lateral + longitudinal force, not two independent limits:

```
FyMax = sqrt(max(0, (muTire*Fz)^2 - FxDemand^2))
Fyf   = clamp(FyfRaw, -FyfMax, FyfMax)
Fyr   = clamp(FyrRaw, -FyrMax, FyrMax)
```

This is what caps the raw linear tire model at a physically plausible limit —
without it, the car could generate unlimited cornering force at any speed.

**6. Equations of motion** (standard dynamic bicycle model):

```
ax   = (Fx - Fyf*sin(steer)) / mass + vy*r
ay   = (Fyf*cos(steer) + Fyr) / mass - vx*r
rDot = (a*Fyf*cos(steer) - b*Fyr) / Iz
```

**7. Integration** — explicit Euler at `dt = 0.001s` (1ms):

```
vx += ax*dt;  vy += ay*dt;  r += rDot*dt;  yaw += r*dt
x  += (vx*cos(yaw) - vy*sin(yaw)) * dt
y  += (vx*sin(yaw) + vy*cos(yaw)) * dt
```

`vx` is floored at 1.0 m/s to avoid a divide-by-zero in the slip angle
calculation.

### Path following (`src/track/track.cpp`)

- **`Track::stadium()`** builds a closed-loop test circuit: two straights
  joined by two semicircles (120m straights, 40m-radius turns, ~491m lap
  length), sampled into waypoints roughly 1m apart.
- **`Track::lookaheadPoint()`** finds the nearest waypoint ahead of the car
  (searching a local window so it can't jump to the wrong side of the loop)
  and returns the point a fixed distance further along the track.
- **`purePursuitSteer()`** computes the front-wheel angle that arcs the car
  toward that lookahead point:

```
localX, localY = targetPoint transformed into the car's local frame
steer = atan2(2 * wheelbase * localY, localX^2 + localY^2)
```

- The circuit scenario also runs a simple proportional controller on
  throttle/brake to hold a constant target speed.

### Telemetry output

Every scenario logs the same CSV schema at 100 Hz (`time, x, y, yaw, vx, vy,
yaw_rate, lat_accel_g`). `lat_accel_g` is computed as `(vx * r) / 9.81` — a
quasi-steady-state approximation of lateral g that omits the `vy_dot` term, so
it's most accurate once the car has settled into a corner rather than during
the initial transient.

## Scenarios and what their results mean

`main.cpp` runs three scenarios per build, each demonstrating something
different about the model:

### 1. Grip-limit demo — `run_grip_limit.csv` / `run_grip_limit_plot.png`

Steer snaps to **15°** at `t=3s` while doing ~234 km/h. That's a huge,
instantaneous input for that speed — real F1 cars use only a couple degrees of
front wheel angle at high speed. The friction circle correctly saturates the
tires immediately: the car can't generate anywhere near the commanded turn
rate, slip angles spiral, and it spins (`vx` collapses to its 1.0 m/s floor,
`vy` and yaw rate run away). **This is the friction-circle limiter working as
intended**, not a bug — it's proof the model now enforces a real grip budget
instead of allowing unlimited cornering force.

Known limitation: the spin doesn't self-arrest — yaw rate grows roughly
linearly instead of settling into a stable flat spin, because the clamp always
supplies the maximum available grip opposing slip, rather than modeling how
real tires lose most of their force once well past peak slip angle.

### 2. Normal corner — `run_normal.csv` / `run_normal_plot.png`

Steer holds at **3°** — the same input used before grip limiting existed.
Numerically it produces the same result as that very first run (~4g at
317 km/h), but now that number is *validated* rather than an artifact: at
~88 m/s the model's downforce term adds enough grip that 4g is physically
supportable, which is also why real F1 cars can pull that much lateral g in
fast corners — aero downforce, not just mechanical tire grip.

### 3. Circuit lap — `run_circuit.csv` / `run_circuit_plot.png`

The car drives the synthetic stadium track autonomously via pure pursuit,
holding ~25 m/s (90 km/h) — modest on purpose, so the 40m-radius turns stay
comfortably within the tires' grip. It completes a lap in about 20 seconds,
with lateral g cycling cleanly between ~0 on the straights and ~1.5g in the
turns. This is the first scenario where the vehicle model and a path-following
controller work together, rather than following a hand-scripted steer input.

## Plotting telemetry

```bash
python3 -m venv venv
source venv/bin/activate
pip install pandas matplotlib

python src/utils/plot_run.py --csv results/telemetry/run_circuit.csv \
                              --out results/logs/run_circuit_plot.png
```

`--csv` and `--out` both default to the original single-run paths if omitted;
pass them explicitly to plot one of the three named scenarios.

## Project layout

```
src/
├── main.cpp              # Entry point — defines and runs the three scenarios
├── vehicle/
│   ├── vehicle.h/.cpp     # CarParams, State, axleLoads(), step() — the physics
├── track/
│   ├── track.h/.cpp       # Track waypoints/lookahead, pure-pursuit steering
├── telemetry/             # Not yet implemented (see below)
└── utils/
    └── plot_run.py        # Matplotlib telemetry plotting

results/
├── telemetry/             # CSV output per scenario
└── logs/                  # Plotted PNGs per scenario

tests/                     # Not yet implemented (see below)
config/                    # Not yet implemented (see below)
```

## Known gaps / not yet implemented

- **`src/telemetry/`** — currently just empty placeholder files. CSV writing
  still lives inline in `main.cpp`'s scenario functions rather than a reusable
  logger class.
- **`tests/test_vehicle.cpp`** — empty placeholder. No automated tests yet.
- **`config/simulation_config.json`** — empty placeholder. Simulation
  parameters are hardcoded in C++ rather than externalized.
- **No real circuit geometry** — `Track::stadium()` is a synthetic test loop,
  not a digitized real-world track.
- **No engine power curve** — `maxDriveF` is a constant regardless of speed,
  so top speed is currently bounded only by drag, not by realistic
  power-limited engine force.
- **`trackWidth`** is declared in `CarParams` but unused — there's no
  per-wheel (four-corner) load model yet, only front/rear axle loads.
- **No CARLA integration** — the project's original goal, not currently
  pursued in favor of the standalone simulation above.
