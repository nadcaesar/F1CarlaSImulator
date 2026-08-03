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
  and a **combined-slip friction circle** so lateral and longitudinal grip
  share one physically bounded budget per axle instead of being unlimited or
  independently capped.
- A **path-following stack** (`src/track/`) — a general track builder that
  lays out any sequence of straight+corner segments into a closed loop, a
  pure-pursuit steering controller with a speed-scaled lookahead, and a
  curvature-adaptive speed controller that brakes for corners before reaching
  them and targets a grip-safe speed through each one.
- **Four demo scenarios** in `main.cpp` that exercise the above (see below).
- CSV telemetry output and a **Python plotting script** (`src/utils/plot_run.py`)
  that renders trajectory, speed, yaw rate, and lateral-g from any run.

## Building and running

```bash
cmake -S . -B build
cmake --build build
./build/f1sim
```

Run it from the repo root — it writes CSVs to `results/telemetry/` using a
relative path. Each run overwrites the four sample CSVs and prints a summary
line per scenario, e.g.:

```
Grip-limit demo (15 deg): final speed 251.912 km/h, wrote results/telemetry/run_grip_limit.csv
Normal corner (3 deg): final speed 316.276 km/h, wrote results/telemetry/run_normal.csv
Oval circuit (pure pursuit): completed lap 1 at t=19.283s
Oval circuit (pure pursuit): final speed 104.631 km/h, 1 lap(s) completed, wrote results/telemetry/run_circuit.csv
Grand circuit lap (pure pursuit): completed lap 1 at t=41.45s
Grand circuit lap (pure pursuit): final speed 173.737 km/h, 1 lap(s) completed, wrote results/telemetry/run_grand_circuit.csv
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

**5. Combined-slip friction circle** — each axle has one shared grip budget for
lateral **and** longitudinal force together, not two independent limits. The
demanded `(Fx, Fy)` pair for an axle is scaled down as a vector — preserving
its direction — if its magnitude would exceed `muTire*Fz`:

```
mag   = sqrt(Fx^2 + Fy^2)
scale = mag > muTire*Fz ? (muTire*Fz) / mag : 1.0
Fx    = Fx * scale
Fy    = Fy * scale
```

This is what caps the raw linear tire model at a physically plausible limit —
without it, the car could generate unlimited cornering force at any speed. An
earlier version of this clamp granted `Fx` in full and only capped whatever
`Fy` budget was left over — which meant hard acceleration could silently zero
out an axle's cornering force even when the raw demand didn't need the whole
circle, turning any acceleration mid-corner (or right after exiting one) into
an unrecoverable spin. Scaling both components together is the textbook-correct
treatment: accelerating hard reduces available cornering force proportionally,
the way a real tire's combined grip actually works, rather than eliminating it
outright.

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

- **`Track::stadium()`** builds the simple oval test loop: two straights
  joined by two semicircles.
- **`Track::fromSegments()`** builds a closed loop from an arbitrary ordered
  list of `{straightBefore, direction, radius, sweepDegrees}` segments — this
  is what the 7-corner "grand circuit" is built from (see below). Every
  waypoint also stores its curvature (`0` on straights, `1/radius` in a
  corner), sampled roughly 1m apart along both straights and arcs.
- **`Track::lookaheadPoint()`** finds the nearest waypoint ahead of the car
  (searching a local window so it can't jump to the wrong side of the loop)
  and returns the point a given distance further along the track.
- **`Track::maxCurvatureAhead()`** returns the tightest curvature within a
  given distance ahead, used to decide how hard to brake before a corner.
- **`purePursuitSteer()`** computes the front-wheel angle that arcs the car
  toward a lookahead point:

```
localX, localY = targetPoint transformed into the car's local frame
steer = atan2(2 * wheelbase * localY, localX^2 + localY^2)
```

Both the steering lookahead and the braking lookahead **scale with speed**
(`distance = max(floor, timeHeadway * vx)`) rather than using a fixed
distance. A fixed lookahead is a shorter time horizon the faster the car
goes, which makes pure pursuit twitchy and unstable at speed — a small
uncorrected wobble can take several seconds to visibly grow into a full spin.
Scaling both lookaheads with speed keeps the controller's time horizon (and
therefore its stability margin) roughly constant across the whole speed
range.

### Speed control (`gripSpeedCap()` in `main.cpp`)

The circuit scenarios don't hold a constant target speed — they compute one
from the tightest curvature within the braking lookahead, using the same
friction-circle idea as the tire model itself. At the speed where cornering
demand `v^2/R` equals available grip `muTire*(g + liftCoeff*v^2/mass)`,
solving for `v` gives a corner's grip-limited speed cap; a straight (zero
curvature) instead targets a flat top-speed cap. A safety margin (0.8×) backs
off from that theoretical limit, since the car also needs some of that grip
budget for whatever steering correction it's making at the same time. A
throttle-only rate limiter (braking is left unlimited, matching how abruptly
real brakes can be applied) keeps the car from snapping straight to full power
right as it exits a corner, which is exactly when the tires are still
shedding lateral slip from the turn and least able to take a sudden
longitudinal hit.

### Telemetry output

Every scenario logs the same CSV schema at 100 Hz (`time, x, y, yaw, vx, vy,
yaw_rate, lat_accel_g`). `lat_accel_g` is computed as `(vx * r) / 9.81` — a
quasi-steady-state approximation of lateral g that omits the `vy_dot` term, so
it's most accurate once the car has settled into a corner rather than during
the initial transient.

## Scenarios and what their results mean

`main.cpp` runs four scenarios per build, each demonstrating something
different about the model:

### 1. Grip-limit demo — `run_grip_limit.csv` / `run_grip_limit_plot.png`

Steer snaps to **15°** at `t=3s` while doing ~234 km/h. That's a huge,
instantaneous input for that speed — real F1 cars use only a couple degrees of
front wheel angle at high speed. The friction circle correctly saturates the
tires immediately: the car can't generate anywhere near the commanded turn
rate and slip angles spiral. **This is the friction-circle limiter working as
intended**, not a bug — it's proof the model now enforces a real grip budget
instead of allowing unlimited cornering force.

### 2. Normal corner — `run_normal.csv` / `run_normal_plot.png`

Steer holds at **3°** — the same input used before grip limiting existed.
Numerically it lands close to that very first run (high-3g range at
~316 km/h), but now that number is *validated* rather than an artifact: at
speed the model's downforce term adds enough grip that several g of lateral
acceleration is physically supportable, which is also why real F1 cars can
pull that much lateral g in fast corners — aero downforce, not just
mechanical tire grip.

### 3. Oval circuit — `run_circuit.csv` / `run_circuit_plot.png`

The car drives the simple oval test loop (two 120m straights, two 40m-radius
turns) autonomously via pure pursuit and curvature-adaptive speed control. It
settles into a stable, repeating lap — about 19 seconds per lap, speed cycling
between ~55 km/h in the corners and ~175 km/h on the straights, lateral g
cycling cleanly with it. This is the first scenario where the vehicle model
and a path-following controller work together, rather than following a
hand-scripted steer input.

### 4. Grand circuit lap — `run_grand_circuit.csv` / `run_grand_circuit_plot.png`

A 7-corner circuit (3 left, 4 right) built with `Track::fromSegments()`, sized
around the friction-circle math above so each corner lands in a target speed
band:

| Corner type | Radius | Speed cap | Corners |
| --- | --- | --- | --- |
| Slow | 20 m | < 75 km/h | 2 |
| Medium | 45 m | < 130 km/h | 3 |
| Fast | 95 m | > 140 km/h (essentially flat out) | 2 (one per side) |

The car completes a full lap in ~41 seconds, ending back within about half a
meter of its starting position and heading.

Getting a stable lap out of this track surfaced a few non-obvious lessons,
worth knowing before extending it:

- **Turning-angle closure isn't position closure.** The 7 corner sweep angles
  were chosen so the signed turning sums to exactly 360°, which is required
  for *any* simple closed loop — but that only guarantees the path returns to
  its starting *heading*. It says nothing about returning to its starting
  *position*. The first version of this track had headings close perfectly
  while its endpoint landed ~275m from its start. The 7 straight-segment
  lengths were solved (via least-squares against the corner geometry, not
  guessed) to close the loop exactly, weighted to preserve real approach
  distance for the two fast corners.
- **A fixed lookahead distance is unstable across a wide speed range** — see
  [Path following](#path-following-srctracktrackcpp) above.
- **The friction circle has to be combined-slip, not lateral-only** — see the
  note on step 5 in [The physics model](#the-physics-model). This was the
  fix that took the car from spinning under any acceleration right after a
  corner to holding a stable line.
- **This particular tuning has a known edge case:** approaching the tightest
  corner (a 170°, 20m-radius hairpin) at the ~175 km/h this track reaches on
  its fastest straight, on a *second* lap where the car carries more
  momentum into the braking zone than on a standing first-lap start, is
  right at the edge of what the current braking lookahead and grip budget
  can arrest in time. The sample run stops right after the first clean lap
  rather than running into this.

## Plotting telemetry

```bash
python3 -m venv venv
source venv/bin/activate
pip install pandas matplotlib

python src/utils/plot_run.py --csv results/telemetry/run_circuit.csv \
                              --out results/logs/run_circuit_plot.png
```

`--csv` and `--out` both default to the original single-run paths if omitted;
pass them explicitly to plot one of the four named scenarios.

## Project layout

```
src/
├── main.cpp              # Entry point — defines and runs the four scenarios
├── vehicle/
│   ├── vehicle.h/.cpp     # CarParams, State, axleLoads(), step() — the physics
├── track/
│   ├── track.h/.cpp       # Track waypoints/curvature, lookahead, pure-pursuit steering
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
- **No real circuit geometry** — both tracks are synthetic (an oval and a
  hand-designed 7-corner loop), not digitized from a real-world circuit.
- **No engine power curve** — `maxDriveF` is a constant regardless of speed,
  so top speed is currently bounded only by drag, not by realistic
  power-limited engine force.
- **`trackWidth`** is declared in `CarParams` but unused — there's no
  per-wheel (four-corner) load model yet, only front/rear axle loads.
- **No CARLA integration** — the project's original goal, not currently
  pursued in favor of the standalone simulation above.
