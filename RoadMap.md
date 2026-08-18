# Roadmap / Future Ideas

Ideas that are genuinely good and worth pursuing, but **not now** — parked here
so they don't get lost or turn into mid-task detours. Nothing in this file
should be started until the current milestone (wishbone design load →
Fusion CAD → Simulation study) is finished.

---

## Tuning parameter optimization (no UI)

**Update (2026-08-17):** dropped the UI-page framing. This is not "build a
tuning screen." The actual idea: given a set of adjustable car-setup
parameters, find the parameter combination that produces the best simulated
performance — an optimization problem over the physics, not an input form.

**Original idea (kept for context):** was initially framed around exposing
car-setup parameters the way Assetto Corsa Competizione or Forza Horizon's
tuning screen does — camber, toe, ride height, spring rate, anti-roll bar
stiffness, brake bias, gear ratios.

**Why it's a real idea, not scope creep:** the physics already exists (axle
load split, aero downforce, lateral load transfer, friction-circle grip) —
this just makes their inputs adjustable instead of hardcoded.

**Why it's not next:** several of these parameters aren't represented in the
current model at all and would require real new physics, not just UI:

- **Camber / toe** — affect the tire's contact patch geometry and how slip
  angle translates to grip. Current tire model (linear stiffness + friction
  circle clamp) doesn't represent contact patch shape at all — this needs a
  more detailed tire model first.
- **Spring rate / ride height** — affect _how fast_ load transfers during a
  transient (turn-in, braking), not just the _steady-state_ amount. Current
  `wheelLoads()` calculates a static transfer for a given instant — no
  suspension dynamics (spring/damper response over time) exist yet.
- **Brake bias** — partially exists already (`maxBrakeF` is currently one
  number for the whole car, not split front/rear) — this one's the closest
  to "just wire it up," relatively low effort whenever this gets picked up.
- **Gear ratios** — would need an actual drivetrain/engine model; currently
  `maxDriveF` is a flat constant with no RPM/gear dependency at all.

**Prerequisite before starting this:** finish the current milestone (wishbone
design load → Fusion CAD → Simulation study). Also still needs the new
physics listed above (contact patch tire model, suspension dynamics) to
exist before "optimize the parameters" means anything real — optimizing over
a model that doesn't yet respond to those parameters would just find noise.

---

## ML / neural network methods in the simulator

**Added 2026-08-17.** Idea: incorporate machine learning (possibly a neural
network) into the simulator somehow.

**Why this is deliberately deprioritized, not just deferred:** every formula
currently in the sim (axle load split, aero downforce, friction circle,
lateral load transfer) was derived by hand and can be explained from first
principles — that's _why_ it's defensible in an interview. A neural network
is the opposite: a black box whose reasoning can't be fully explained, even
by the person who trained it. Replacing hand-derivable physics with a
learned approximation would make the project less defensible, not more
impressive, for a Design Engineer portfolio specifically.

**Where ML would genuinely fit, if this gets revisited:**

- Learning _optimal setup_ — i.e., actually implementing the tuning-parameter
  optimization idea above using ML as the search method. This makes ML a
  tool in service of an existing goal, not a bolt-on.
- Fitting a hard-to-derive piece (e.g. real tire behavior) from real test
  data, as a supplement to — not a replacement for — the hand-derived model.

**Sequencing:** this depends on the tuning-parameter optimization idea above,
which itself depends on new physics that doesn't exist yet. Do not start
this before both of those. Realistically: last item on this whole list.

---
