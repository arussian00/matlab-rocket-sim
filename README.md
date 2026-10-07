# Reusable Rocket Simulator (MATLAB)

Simulates a rocket that launches to space and lands back on Earth:

1. **Suborbital hop.** One stage climbs past the Kármán line (100 km), flips around, burns back toward the launch site, falls back through the atmosphere, steers with grid fins, and lands on its pad with a landing burn.
2. **Orbital mission.** A two-stage rocket puts a payload into a 250 km orbit. Its first stage does an entry burn and lands on a drone ship.

On top of that there is a **Monte Carlo** study (many flights with random errors) and a **slider app**.

Everything runs on **MATLAB Online Basic** (the free tier). It uses core MATLAB only: no Simulink and no toolboxes.

---

## Contents

1. [Quick start](#1-quick-start)
2. [Files in this project](#2-files-in-this-project)
3. [How the simulation works (big picture)](#3-how-the-simulation-works-big-picture)
4. [The physics model](#4-the-physics-model)
5. [The flight phases, step by step](#5-the-flight-phases-step-by-step)
6. [New additions in detail](#6-new-additions-in-detail)
   - 6.1 Boostback burn and impact predictor
   - 6.2 Engine limits (minimum throttle, gimbal limit, engine count)
   - 6.3 Attitude dynamics and control
   - 6.4 Grid-fin steering
   - 6.5 Landing-burn guidance (upgraded)
   - 6.6 Monte Carlo analysis
   - 6.7 Two-stage rocket to orbit + drone-ship landing
   - 6.8 Interactive app
7. [File-by-file walkthrough](#7-file-by-file-walkthrough)
8. [Parameter reference](#8-parameter-reference)
9. [Expected results](#9-expected-results)
10. [Experiments to try](#10-experiments-to-try)
11. [Assumptions and limitations](#11-assumptions-and-limitations)
12. [Troubleshooting on MATLAB Online](#12-troubleshooting-on-matlab-online)
13. [What changed from version 1](#13-what-changed-from-version-1)
14. [Glossary](#14-glossary)

---

## 1. Quick start

### Fastest: one click

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=arussian00/matlab-rocket-sim&file=rocket_hop_sim.m)

Click the badge, sign in to MATLAB Online (the free Basic account is fine) and accept the prompt to copy the repository. The project opens in your MATLAB Drive with `rocket_hop_sim.m` ready. Press **Run**.

### Or clone it inside MATLAB Online

In the MATLAB Online Command Window:

```matlab
gitclone("https://github.com/arussian00/matlab-rocket-sim.git");
cd matlab-rocket-sim
rocket_hop_sim
```

(`gitclone` needs R2023b or newer, and MATLAB Online is always newer. Alternatively, in the **Files** panel, right-click > **Source Control** > **Clone Repository** and paste the URL.)

### Or upload the files by hand

1. Go to **matlab.mathworks.com** and sign in. The free account is enough.
2. In the **Files** panel, make a folder (for example `rocket`). Upload **all** the `.m` files into it. They call each other, so they must sit in the same folder.
3. Double-click the folder so it becomes the **Current Folder**.
4. In the Command Window, type one of these:

| Command | What you get | Time |
|---|---|---|
| `rocket_hop_sim` | One hop flight: summary, 9-panel dashboard, animation | ~5 s plus the animation |
| `rocket_monte_carlo` | 50 flights with random errors, plus statistics plots | ~1–3 min |
| `rocket_two_stage_sim` | Two-stage launch to orbit and booster drone-ship landing | ~10 s plus the animation |
| `rocket_app` | Window with sliders. Change values and press **LAUNCH** | instant per run |

---

## 2. Files in this project

| File | Type | Purpose |
|---|---|---|
| `rocket_hop_sim.m` | script | **Start here.** Runs one hop mission end to end. |
| `rocket_params.m` | function | All tunable numbers (vehicle, engines, guidance, controller, success criteria). Has two presets: `'hop'` and `'booster'`. |
| `simulateBooster.m` | function | **The core engine.** Equations of motion, guidance, attitude control, grid fins, phase sequencing, impact predictor. |
| `earthModel.m` | function | Gravity and air density at a given altitude. |
| `flightMetrics.m` | function | Extracts the key numbers (apogee, max-g, touchdown speed, miss distance, pass/fail) and prints a summary. |
| `plotFlight.m` | function | 9-panel dashboard of one flight. |
| `animateFlight.m` | function | Two-panel animation: trajectory plus a chase camera, with optional GIF export. |
| `rocket_monte_carlo.m` | script | **New.** Many flights with random errors, then success rate and statistics. |
| `rocket_two_stage_sim.m` | script | **New.** Two-stage orbital launch; the booster lands on a drone ship. |
| `rocket_app.m` | function | **New.** Interactive slider app (uifigure). |

How they connect:

```
rocket_hop_sim ──┐
rocket_monte_carlo ──┤
rocket_app ──────────┼──> rocket_params ──> simulateBooster ──> earthModel
rocket_two_stage_sim ┘                          │
                                                ├──> flightMetrics
                                                ├──> plotFlight
                                                └──> animateFlight
```

---

## 3. How the simulation works (big picture)

A simulation like this always does the same three things:

1. **Describe the vehicle with a state vector.** This is the smallest set of numbers that captures everything about the vehicle at one instant: where it is, how fast it's moving, how heavy it is, and which way it points.
2. **Write the rate of change of every state.** These are the equations of motion: "given where I am now, how is each number changing?" This is Newton's second law (F = m·a) plus the rocket equation for mass.
3. **Let a solver march forward in time.** MATLAB's `ode45` takes many small steps, adapting the step size to keep errors small.

A real flight is a **sequence of phases**: ascent, flip, boostback, and so on. Each phase uses different rules: engine on or off, a different pointing target, different throttle logic. `ode45` has an **event** feature that stops integration exactly when a condition is met, for example "predicted apogee = 110 km". The code uses that to end one phase and start the next:

```
for each phase in the mission list:
    set up the phase (e.g. decide boostback direction)
    ode45( equations of motion for this phase,
           stop when this phase's event fires )
    append results; the final state becomes the next phase's start
```

Inside the equations of motion, every call goes through the same pipeline. These are the numbered steps in `boosterDynamics`:

```
GUIDANCE ─> desired direction + throttle
   │
ENGINE LIMITS ─> clamp throttle to [min, 100%], engine count, out-of-fuel
   │
ATTITUDE CONTROL ─> PD controller → gimbal angle + RCS torque
   │
AERODYNAMICS ─> drag (with wind), grid-fin side force
   │
NEWTON ─> accelerations (round-Earth terms included)
   │
ROCKET EQUATION ─> mass flow
   │
ROTATION ─> angular acceleration = torque / inertia
```

---

## 4. The physics model

### 4.1 State vector (7 numbers)

| Symbol | Meaning | Unit |
|---|---|---|
| `x` | downrange distance along Earth's surface | m |
| `h` | altitude above sea level | m |
| `vx` | horizontal velocity (+ = downrange) | m/s |
| `vh` | vertical velocity (+ = up) | m/s |
| `m` | total mass (structure + propellant) | kg |
| `θ` (theta) | pitch: angle of the nose from local vertical (+ = leaning downrange) | rad |
| `ω` (omega) | pitch rate | rad/s |

Version 1 had only the first five. `θ` and `ω` are **new**: the booster now has real rotation.

### 4.2 Gravity: inverse-square law (`earthModel.m`)

```
g(h) = g0 · (Re / (Re + h))²
```

At 110 km gravity is about 3% weaker than at the ground. At 250 km it is about 7% weaker.

### 4.3 Atmosphere: exponential model (`earthModel.m`)

```
ρ(h) = 1.225 · exp(−h / 8500)      [kg/m³]
```

Air density falls by a factor *e* (2.718) every 8.5 km. At 50 km there's about 0.3% of sea-level air. At 100 km there's essentially none.

### 4.4 Drag, now with wind

```
v_rel = [vx − wind(h);  vh]          velocity relative to the air
q     = ½ ρ |v_rel|²                  dynamic pressure
D     = q · Cd · A                    drag force, opposite to v_rel
```

- `Cd = 0.5` going up nose-first, and `Cd = 1.0` falling engines-first. The higher value comes from the blunt engine end and deployed drag surfaces.
- **Wind profile:** it grows linearly to full strength at 10 km (like a jet stream), then dies away above 12 km.

### 4.5 Engine: thrust and mass flow

```
F    = throttle × (number of engines lit) × T_eng
Isp  = Isp_vac − (Isp_vac − Isp_sl) · ρ/ρ0      engines work better in thin air
ṁ    = −F / (Isp · g0)                           the rocket equation, in rate form
```

### 4.6 Newton's second law on a round Earth

```
dx/dt  = vx · Re/(Re+h)
dh/dt  = vh
dvx/dt = a_x − vx·vh/(Re+h)
dvh/dt = a_h − g + vx²/(Re+h)
```

`a_x` and `a_h` are the thrust, drag and grid-fin accelerations. The two extra terms, `−vx·vh/r` and `+vx²/r`, exist because "horizontal" and "vertical" keep rotating as you travel around a curved planet. For the hop they are negligible. For the orbital stage they are everything: when `vx² / r = g`, you are in orbit, falling around Earth and never hitting it. **New in this version:** in version 1 Earth was flat.

### 4.7 Rotation (new)

```
I     = m · L² / 12              moment of inertia of a uniform rod
dθ/dt = ω − vx/(Re+h)            pitch relative to the turning local vertical
dω/dt = τ / I                    torque / inertia
```

Torque `τ` comes from the engine gimbal and the RCS thrusters (section 6.3).

---

## 5. The flight phases, step by step

The hop flies `{'ascent','flip','boostback','coast','landing'}`. The drone-ship booster flies `{'coast','entry','coast','landing'}`.

| # | Phase | Engine | Pointing target | Ends when (event) |
|---|---|---|---|---|
| 1 | **ascent** | full throttle | vertical, then leans `kickDeg` after `tKick` seconds | predicted apogee `h + vh²/2g` reaches `h_target` **or** propellant falls to `reserve` |
| 2 | **flip** | off | sideways, pointing back at the pad (±90°) | attitude error < 2° **and** rotation rate < 0.5°/s |
| 3 | **boostback** | full throttle | sideways, back toward the pad | predicted impact point crosses the pad **or** propellant falls to `reserveLand` |
| 4 | **coast** | off | engines into the direction of travel; grid fins steer below 40 km | landing-burn ignition condition **or** ground impact (crash) |
| 5 | **entry** *(two-stage only)* | 3 engines | exactly opposite the velocity | speed < `entryEndSpeed` |
| 6 | **landing** | throttled (min…100%) | landing guidance (section 6.5) | touchdown, h = 0 |

### Why "engine cut-off at predicted apogee"?

With the engine off and no air, a rising object reaches a maximum height of `h + vh²/(2g)`. Cutting the engine when that equals the target means we coast up to exactly the target altitude. Above about 40 km there's so little air that drag barely changes it. This is a simple form of *guidance*.

### Why "engines-first" when falling?

The engines are the toughest part of the booster, and the engine end is blunt, which gives high drag and slows the fall. Most importantly, the engine must face the direction of travel to brake during the landing burn.

---

## 6. New additions in detail

### 6.1 Boostback burn and impact predictor

**What it does.** During ascent the booster leans downrange (`kickDeg = 5°`), so without help it would fall about 11 km from the pad. After engine cut-off it **flips** sideways using its thrusters, then fires its engine **back toward the pad** until its predicted landing point is the pad.

**How the prediction works** (`predictImpact` in `simulateBooster.m`):

- It's a small simulation inside the simulation: a point mass, no thrust, 2-second steps (midpoint/RK2 method), with drag included.
- It uses **nominal** drag and **no wind**. That's what the onboard computer believes, so in the Monte Carlo it is deliberately slightly wrong. The grid fins and landing burn then clean up the error.
- It returns the downrange position where the altitude crosses zero.

**Choosing the direction.** At the start of the flip, `bbDir = −sign(predicted impact − pad)`. A value of −1 means "thrust uprange".

**When it stops.** The event is `(predicted impact − pad) × bbDir` crossing zero from below. In words: we keep pushing until the predicted landing point passes over the pad. There's also a safety stop if propellant gets down to `reserveLand`.

### 6.2 Engine limits

| Limit | Parameter | What it models |
|---|---|---|
| **Minimum throttle** | `thrMin` (hop 30%, booster 40%) | Real engines can't throttle to zero; below a certain level combustion becomes unstable. A lit engine always produces at least this much thrust. |
| **Gimbal limit** | `gimbalMaxDeg` (6°) | The nozzle can only swivel a few degrees, which limits how much turning torque the engine can make. |
| **Engine count per phase** | `engAscent`, `engBoost`, `engEntry`, `engLand` | The orbital booster lifts off on 9 engines, does its entry burn on 3 and lands on 1. One engine at minimum throttle is still too much to hover a light, nearly empty booster. |
| **Out of fuel** | automatic | When `m ≤ m_dry` the engine produces nothing. |

**Hoverslam.** If even minimum thrust is more than the booster's weight, it **cannot hover**. It must time the burn so its speed reaches zero exactly at the ground. The landing law (6.5) does exactly that. You can see it in the dashboard: the throttle stays well above the minimum all the way to touchdown.

### 6.3 Attitude dynamics and control

The booster is now a rigid body that has to be *turned*. It can't just point wherever guidance wants.

**Moment of inertia** `I = m L²/12` (a uniform rod). It drops as propellant burns, so the booster gets easier to turn.

**The controller** (`attitudeControl` in `simulateBooster.m`) is a **PD controller**, the classic proportional-derivative design:

```
error   = θ_cmd − θ                  (wrapped to ±180° so it turns the short way)
τ_cmd   = I · ( ωn² · error − 2·ζ·ωn · θ̇ )
```

- `ωn = 1 rad/s` sets how fast it responds.
- `ζ = 0.8` sets how damped it is (1 = no overshoot; lower = more overshoot).
- Multiplying by `I` means the response feels the same whatever the mass.

**Torque allocation.** There are two actuators:

1. **Engine gimbal** (only when the engine is on). Swiveling the nozzle by angle δ pushes the tail sideways, giving torque `F · (L/2) · sin δ`. The controller uses as much as the gimbal limit allows, then solves for δ: `δ = asin(τ / (F·L/2))`.
2. **RCS cold-gas thrusters** supply whatever torque is left over, up to `tauRCS`. With the engine off, as during the flip, the RCS does all the work. That's why the flip takes about 35 s.

**Thrust direction** now follows the actual body: `u = [sin(θ − δ); cos(θ − δ)]`. If the attitude lags the command, the thrust points the wrong way for a moment. That's realistic, and it's why the landing controller had to be tuned with the attitude loop in mind.

You can see all of this in the dashboard:
- Panel 7 shows the actual pitch against the commanded pitch.
- Panel 8 shows the gimbal angle with its ±6° limits, and the RCS torque.

### 6.4 Grid-fin steering

Real boosters steer during descent with lattice fins near the top. The model:

- They are active only in **coast**, while **falling**, **below 40 km** (they need air to work).
- Desired sideways speed: `vx_wanted = (pad − x) / time_to_fall`, where `time_to_fall ≈ h / |vh|`.
- Side force: `F_fin = m · (vx_wanted − vx) / tauFin`, **capped at** `q · finCLA`. More air and more speed give more authority.

This cancels most of the wind drift and drag-error drift before the engine relights.

### 6.5 Landing-burn guidance (upgraded)

**Vertical: the "suicide burn" law.** To slow from vertical speed `vh` to touchdown speed `v_td` over the remaining height `h` with constant deceleration:

```
a_up = (vh² − v_td²) / (2h) + g − (dragCredit × current drag)
```

This is basic kinematics (`v² = v0² + 2·a·d`) rearranged. It's re-evaluated continuously, so it corrects itself: if the booster brakes too hard, the next evaluation asks for less.
- **New: drag credit.** Near the ground, drag still provides a lot of braking. The guidance counts on 50% of the current drag. All of it would be risky, because drag fades as you slow down. None of it would light the engine too early and waste fuel.

**Horizontal: steering to the pad.**

```
tgo       = 2h / (|vh| + v_td)                         time-to-go for a constant-decel descent
vx_wanted = sign(Δx) · min( wnDivert·|Δx|, 2|Δx|/tgo ) · fade(h/50 m)
a_x       = (vx_wanted − vx) / τ  − (drag credit, horizontal)
```

- Far from the pad, `2|Δx|/tgo` limits the approach speed to one we can still stop from.
- Close to the pad, `wnDivert·|Δx|` gives a gentle proportional approach.
- **Below 50 m** the position term fades out, and the only job left is to stop sliding sideways.

**Tilt limit.** The thrust is allowed to lean at most `maxTiltDeg` (15°) from vertical. **Below 20 m this limit fades to 0**, so the booster touches down upright.

**Ignition.** The engine lights when the thrust the law above would need reaches `ignFrac` (75%) of the available thrust, but only below `ignAlt` (10 km). The 25% margin absorbs errors such as a slightly weak engine or more drag than expected.

### 6.6 Monte Carlo analysis (`rocket_monte_carlo.m`)

**Idea.** Fly the mission N times. Each time, draw random errors from a normal distribution (`randn`):

| Error source | 1-σ | Effect |
|---|---|---|
| Thrust | 2% | Engine slightly stronger or weaker than the guidance thinks |
| Isp | 1% | Fuel burns faster or slower |
| Drag coefficient | 10% | Booster slows more or less in the air |
| Dry mass | 2% | Booster heavier or lighter |
| Wind | 8 m/s | Pushes the booster sideways through the 10–12 km layer |

**Truth vs guidance.** The dispersions live in `P.thrustScale`, `P.CdScale` and so on. Only the **physics** uses them. Guidance, the impact predictor and the landing law all use the nominal values, just like a real flight computer. This is what makes the test meaningful.

**Outputs:**
- Success rate, using the criteria in `P.success`: miss ≤ 20 m, vertical ≤ 3 m/s, sideways ≤ 5 m/s, tilt ≤ 6°.
- Mean, standard deviation and worst case of each touchdown number.
- Six plots:
  - all trajectories overlaid
  - the touchdown footprint, with the allowed box drawn in
  - vertical speed against tilt
  - a histogram of miss distance
  - a histogram of propellant margin
  - a sensitivity plot of wind against miss, colored by drag error

`rng(42)` makes the random draws repeatable. Change the seed to get a different set.

### 6.7 Two-stage rocket to orbit plus drone-ship landing (`rocket_two_stage_sim.m`)

**The vehicle** is a fictional small reusable launcher, about 186 t at liftoff:

| | Stage 1 (booster) | Stage 2 |
|---|---|---|
| Dry mass | 14 t | 3 t |
| Propellant | 130 t | 38 t |
| Thrust | 2.6 MN (9 engines) | 300 kN |
| Isp | 282 s (SL) / 311 s (vac) | 348 s |
| Payload | – | 1.5 t |

**Step 1: stack ascent, the gravity turn.** The stack flies straight up for 10 s, leans 1.5° for 5 s, then points the thrust **along the velocity**. Gravity gradually bends the path toward horizontal without the rocket ever flying sideways through thick air, which would break it. This is how real launchers fly through the lower atmosphere.

**Step 2: MECO.** Stage 1 shuts down while it still has 14 t of propellant, enough for its own entry and landing burns.

**Step 3: separation.** The state vector is copied into two vehicles. Stage 2 gets the upper-stage and payload mass. The booster gets the rest, plus an initial pitch equal to its flight direction.

**Step 4: Stage 2 to orbit.** It coasts 3 s, then burns with a simple but effective guidance law:
- The **vertical** acceleration is a PD controller that drives altitude to 250 km and vertical speed to zero. It also cancels "net gravity", `g − vx²/r`. That term shrinks to zero as the stage approaches orbital speed.
- **All remaining thrust** goes into horizontal speed.
- **Cut-off (SECO)** comes when horizontal speed reaches circular-orbit speed `√(μ/r)`, about 7.75 km/s.

**Step 5: orbit check.** Classical two-body formulas:

```
energy = v²/2 − μ/r              (negative = captured by Earth)
a      = −μ / (2·energy)         semi-major axis
e      = √(1 − h²/(μ·a))         eccentricity (0 = circle)
perigee = a(1−e) − Re,   apogee = a(1+e) − Re,   period = 2π√(a³/μ)
```

Stage 2 then coasts one full orbit (about 90 min of simulated time) to prove it stays up.

**Step 6: booster return.** It reuses `simulateBooster` with the `'booster'` preset and the phases `coast > entry > coast > landing`.
- **Coast:** the RCS flips the booster engines-first.
- **Entry burn:** it starts falling through 55 km and ends below 900 m/s. Three engines fire retrograde to cut the speed before the dense atmosphere. Without this burn, aerodynamic heating and loads would destroy a real booster.
- **Drone-ship placement:** after the entry burn, the impact predictor decides where the ship waits (`P.xPad`, about 385 km downrange). Real drone ships are positioned before launch using the planned trajectory. Here we use the predicted point.
- **Grid fins, then a single-engine landing burn**, using the same guidance as the hop.

### 6.8 Interactive app (`rocket_app.m`)

The app is built with `uifigure`, `uigridlayout`, `uislider` and `uiaxes`. These all work in MATLAB Online, and no App Designer file is needed.

- **Sliders:** target apogee, pitch kick, engine thrust, propellant, minimum throttle, wind, drag error, thrust error.
- **LAUNCH** runs `simulateBooster` with those values and redraws four plots (trajectory, altitude, throttle and pitch, final approach) plus a text summary.
- **Animate last flight** and **Full dashboard** open the same windows as `rocket_hop_sim`.
- The callbacks are **nested functions**, so they share the variables of `rocket_app` (`sliders`, `lastOut`, ...) without globals.

Try making it fail. A 40-60% minimum throttle, a 25 m/s wind or a 10° kick each push the booster to its limits.

---

## 7. File-by-file walkthrough

### `rocket_hop_sim.m` (script)
- **STEP 1** `P = rocket_params('hop')`, with commented-out override examples.
- **STEP 2** `out = simulateBooster(P)` flies the mission and times it.
- **STEP 3** `flightMetrics(out)` prints the summary.
- **STEP 4** `plotFlight(out)` draws the dashboard.
- **STEP 5** `animateFlight(out, speedup, fps, gif)` plays the animation, with optional GIF export.

### `rocket_params.m` (function)
Ten numbered sections, from planet through vehicle, attitude, ascent, return, fins, landing, success criteria, dispersions and solver. The `switch vehicle` at the end swaps in the orbital booster's values. Derived values (area, angles in radians) are computed last. See section 8 for every field.

### `simulateBooster.m` (function): the core

| Sub-function | Role |
|---|---|
| `simulateBooster` (main) | **STEP 1** fills in defaults. **STEP 2** loops over phases: phase set-up (boostback direction), `ode45` with events, storing results, then reacting to which event fired (crash, drone-ship placement, notes). **STEP 3** recomputes extra signals for plotting. Packs everything into `out`. |
| `boosterDynamics` | The equations of motion, numbered 1–8: guidance, engine limits, attitude control, aerodynamics, grid fins, Newton, mass flow, rotation. With a second output it also returns the extra signals. |
| `guidanceLaw` | Desired pointing angle and throttle for each phase (a `switch` on the phase name). |
| `landingAccel` | Landing-burn law (6.5). |
| `attitudeControl` | PD controller, then the gimbal angle and RCS torque (6.3). |
| `phaseEvents` | The stop condition(s) for each phase (section 5 table). |
| `predictImpact`, `ballistic` | Onboard landing-point predictor (6.1). |
| `enginesLit` | Engine count per phase. |
| `windAt` | Wind profile. |
| `wrapAngle` | Keeps angles in ±π. MATLAB's `wrapToPi` needs the Mapping Toolbox, so the project has its own. |

The output struct `out` has these fields:

| Field | Contents |
|---|---|
| `t` | time [N×1] |
| `X` | state history [N×7] |
| `phase` | phase index [N×1] |
| `phaseList` | the phase names |
| `phaseStart` | start time of each phase |
| `aux` | `.throttle`, `.thrust`, `.gimbal`, `.gLoad`, `.q`, `.thetaCmd`, `.tauRCS`, `.finForce`, `.wind` (each N×1) |
| `P` | parameters, including `bbDir` and `xPad` decided in flight |
| `crashed`, `status`, `notes` | how the flight ended |

### `earthModel.m`
Two lines: inverse-square gravity and exponential density. It's shared by the booster model and the two-stage stack model.

### `flightMetrics.m`
- **STEP 1** Flight-wide numbers: apogee, time above 100 km (summed over the time steps), max q, max g (ignoring the first second), propellant used and remaining.
- **STEP 2** Phase start times and landing-burn ignition conditions.
- **STEP 3** Touchdown state and the pass/fail check against `P.success`.
- **STEP 4** Prints the summary, unless called as `flightMetrics(out, false)`.

### `plotFlight.m`
Nine tiles, each line colored by phase:
1. Trajectory
2. Altitude
3. Speed
4. g-load
5. Dynamic pressure
6. Mass and throttle (with the min-throttle line)
7. Pitch against command
8. Gimbal and RCS torque
9. Final-approach close-up

### `animateFlight.m`
- **STEP 1** Resamples the variable-step `ode45` output onto evenly spaced frames, using `interp1`. `unique(...,'last')` removes the duplicate time where phases join.
- **STEP 2** Builds the figure: a trajectory panel and a chase-camera panel (ground, launch pad, landing pad, rocket, flame, HUD).
- **STEP 3** Draws each frame:
  - The rocket outline is rotated from body coordinates to world coordinates using the **true pitch angle**.
  - The flame follows the **gimballed thrust line**.
  - The sky fades from blue to black with altitude.
  - The HUD shows time, phase, altitude, speed, throttle, pitch, gimbal and propellant.
  - Optionally, each frame is appended to a GIF with `exportgraphics(...,'Append',true)`.

### `rocket_monte_carlo.m`
- **STEP 1** Number of runs, random seed, 1-σ values.
- **STEP 2** The loop: draw errors, fly, record results.
- **STEP 3** Statistics printout.
- **STEP 4** Six plots.

### `rocket_two_stage_sim.m`

| Step | What it does |
|---|---|
| 1 | Parameters |
| 2 | Stack ascent with gravity turn, until MECO |
| 3 | Separation |
| 4 | Stage 2 burn to SECO |
| 5 | Orbit check and one orbit of coasting |
| 6 | Booster return via `simulateBooster` |
| 7 | Printout |
| 8 | Plots: Earth view, flat view, time histories, booster dashboard and animation |

Local functions: `stackDynamics` (point-mass model with three modes), `evMECO`, `evSECO`.

### `rocket_app.m`
- **STEP 1** Window and layout.
- **STEP 2** A slider table (`spec`), with each row giving label, field, min, max, default and unit scale.
- **STEP 3** Buttons.
- **Callbacks:** `paramsFromSliders`, `onLaunch`, `drawResults`, `onAnimate`, `onDashboard`, `onReset`.

---

## 8. Parameter reference

All fields are in `rocket_params.m`. These are the hop values; booster overrides are in brackets.

**Planet:** `g0` 9.80665 · `Re` 6371 km · `rho0` 1.225 · `Hscale` 8500 m

**Vehicle**

| Field | Value | Meaning |
|---|---|---|
| `m_dry` | 6 t [14 t] | empty mass |
| `m_prop` | 19 t [130 t] | propellant |
| `T_eng` | 400 kN [289 kN] | thrust per engine |
| `Isp_sl` / `Isp_vac` | 280 / 310 s [282 / 311] | engine efficiency |
| `thrMin` | 0.30 [0.40] | minimum throttle |
| `diam`, `L` | 3.5 m, 40 m [3.7, 45] | size |
| `Cd_up` / `Cd_down` | 0.5 / 1.0 | drag coefficients |
| `engAscent` / `engBoost` / `engEntry` / `engLand` | 1/1/1/1 [9/3/3/1] | engines lit per phase |

**Attitude:** `gimbalMaxDeg` 6 · `tauRCS` 40 kN·m [150] · `attWn` 1.0 rad/s · `attZeta` 0.8

**Ascent:** `h_target` 110 km · `tKick` 10 s · `kickDeg` 5° · `reserve` 2500 kg

**Return**

| Field | Value | Meaning |
|---|---|---|
| `xPad` | 0 [NaN = drone ship] | landing target |
| `flipTolDeg` / `flipRateTol` | 2° / 0.5°/s | flip-done thresholds |
| `reserveLand` | 900 kg [2500] | boostback safety stop |
| `entryAlt` / `entryEndSpeed` | 55 km / 900 m/s | entry burn start and stop |

**Grid fins:** `finAlt` 40 km · `finCLA` 4 m² · `tauFin` 3 s

**Landing**

| Field | Value | Meaning |
|---|---|---|
| `ignFrac` | 0.75 | ignition margin |
| `ignAlt` | 10 km | highest altitude for ignition |
| `v_td` | 1 m/s | target touchdown speed |
| `hFloor` | 0.2 m | altitude floor in the guidance math |
| `dragCredit` | 0.5 | share of drag counted on for braking |
| `maxTiltDeg` | 15° | max thrust lean |
| `tiltFadeAlt` | 20 m | tilt limit fades to 0 below this |
| `wnDivert` | 0.3 | sideways correction gain |
| `divertFadeAlt` | 50 m | stop chasing pad position below this |
| `divertTauMin` / `divertTauMax` | 2.5 / 3 s | sideways-speed correction time constant |

**Success:** `missMax` 20 m · `vVertMax` 3 m/s · `vHorizMax` 5 m/s · `tiltMaxDeg` 6°

**Dispersions:** `thrustScale`, `IspScale`, `CdScale`, `dryScale` (all 1 by default) · `wind` 0

**Solver:** `maxStep` 0.2 s · `tPhaseMax` 3000 s

---

## 9. Expected results

These numbers come from an independent re-implementation of the same model that was used to tune the guidance. Your MATLAB run should match to within a few percent. Small differences come from `ode45`'s adaptive steps.

**Hop (`rocket_hop_sim`)**

| Event | Approx. value |
|---|---|
| Engine cut-off (MECO) | t ≈ 109 s, 44 km, 1,130 m/s |
| Flip | ≈ 35 s on thrusters |
| Boostback | ≈ 5 s, reverses horizontal speed from +146 to about −53 m/s |
| Apogee | ≈ 109 km (about 4 min above 100 km) |
| Max g-load | ≈ 4.4 g (during re-entry) |
| Landing burn | lit at ≈ 300–450 m altitude, ≈ 136 m/s |
| Touchdown | < 2.5 m/s down, < 1 m/s sideways, about 1–4° tilt, within a few meters of the pad |
| Propellant left | ≈ 2.9 t |

**Monte Carlo (20-run check):** every run landed. Worst cases were a 4 m miss, 2.2 m/s vertical, 4 m/s sideways and 1.6° tilt.

**Two-stage (`rocket_two_stage_sim`)**

| Event | Approx. value |
|---|---|
| MECO | t ≈ 131 s, 60 km, ≈ 1,680 m/s, flight path 42° |
| Orbit | ≈ 254 × 264 km, period ≈ 90 min, about 0.6 t of Stage 2 propellant spare |
| Booster entry burn | 55 → 31 km, slowed to 900 m/s |
| Drone ship | ≈ 385 km downrange |
| Booster touchdown | within about 1 m of the ship, about 1 m/s, ≈ 2.9 t propellant left |

---

## 10. Experiments to try

| Change | What you'll see |
|---|---|
| `P.kickDeg = 10` | Lands farther away, so a longer boostback and less fuel margin. |
| `P.wind = 20` | Grid fins work hard; check panel 8 and the final approach. |
| `P.thrMin = 0.6` | Harder hoverslam; may fail. Shows why deep throttling matters. |
| `P.attWn = 0.3` | Sluggish attitude control; the landing gets sloppy or fails. |
| `P.tauRCS = 10e3` | Slow flip; the boostback starts later and costs more fuel. |
| `P.ignFrac = 0.95` | Very late ignition; may run out of thrust margin. |
| `P.dragCredit = 0` | Lights the engine early, which wastes fuel but is safe. |
| `P.h_target = 200e3` | Higher re-entry speed and higher g-loads. |
| In the Monte Carlo: `sig.wind = 15` | Watch the success rate drop. |
| In the two-stage script: `S.mPay = 2500` | Can it still reach orbit? |
| In the two-stage script: `S.reserve1 = 8000` | More payload capacity, but can the booster still land? |

---

## 11. Assumptions and limitations

- **2-D only.** Flight stays in one vertical plane, with no crossrange.
- **Non-rotating Earth.** Earth's spin, which helps eastward launches by about 400 m/s, is ignored.
- **Exponential atmosphere.** It isn't the full US Standard Atmosphere, and there's no Mach-dependent drag.
- **Simplified aerodynamics:**
  - no aerodynamic torque on the body
  - no lift except the grid-fin side force
  - Cd changes only between "climbing" and "falling"
- **Rigid rod.** There's no propellant slosh, no bending, and no change in the center of gravity as fuel burns. Inertia uses `m·L²/12`.
- **Actuators are instantaneous.** The gimbal and RCS have no lag or rate limits. Engine ignition and shutdown are instant.
- **No heating model.** Entry-burn timing is fixed by parameters, not by a heat-load limit.
- **Ideal steering for the two-stage stack and Stage 2.** Point masses with no attitude dynamics; only the booster has full rotation.
- **Drone ship placement** uses the predicted impact point after the entry burn, not a pre-launch plan.

These are the natural next upgrades if you want to go further. Section 4 shows where each one would plug in.

---

## 12. Troubleshooting on MATLAB Online

| Problem | Fix |
|---|---|
| `Undefined function 'simulateBooster'` | All `.m` files must be in the **Current Folder**. Double-click the folder in the Files panel. |
| Animation is choppy | Browser rendering is slower than desktop MATLAB. Raise `speedup` (e.g. 20), or set `playAnimation = false`. |
| `exportgraphics` error | GIF export needs R2022a or newer. MATLAB Online is always current, so this mainly applies to older desktop versions. |
| Monte Carlo is slow | Lower `N`, or raise `P.maxStep` to 0.5 (less accurate attitude). |
| Free-tier time limit | MATLAB Online Basic gives about 20 hours per month. A 50-run Monte Carlo uses a few minutes. |
| `ode45` warnings about tolerances | Usually harmless near touchdown. If a phase "timed out", read `out.status`. |

---

## 13. What changed from version 1

| Version 1 (single file) | Version 2 (this project) |
|---|---|
| One script with local functions | Modular functions, reused by four entry points |
| Flat Earth | Round Earth (curvature terms), needed for orbit |
| Point mass, thrust points anywhere instantly | **Rigid body with pitch dynamics**, PD control, gimbal and RCS |
| Throttle 0–100% | **Minimum throttle**, gimbal limit, engine count per phase |
| Lands wherever it falls (~18 km away) | **Flip and boostback** with onboard impact predictor, plus **grid fins**; lands on the pad |
| Vertical-only landing law | Landing law with drag credit, pad targeting, tilt fade |
| No wind, perfect knowledge | **Wind**, plus **Monte Carlo** with truth vs guidance separation |
| Single stage only | **Two-stage to orbit** with gravity turn, orbit insertion and **drone-ship** landing |
| Script only | **Interactive app** |

---

## 14. Glossary

| Term | Meaning |
|---|---|
| **Apogee** | Highest point of the trajectory. |
| **ASDS / drone ship** | Autonomous spaceport drone ship: a floating landing pad at sea. |
| **Boostback** | Burn that reverses the booster's horizontal velocity to return toward the launch site. |
| **Dynamic pressure (q)** | ½ρv², the aerodynamic "push". Max-Q is the point of peak stress. |
| **Entry burn** | Burn before re-entering dense air, to reduce heating and loads. |
| **Gimbal** | Swiveling the engine nozzle to steer. |
| **Gravity turn** | Ascent where thrust follows velocity and gravity bends the path over. |
| **Grid fins** | Lattice-shaped control surfaces used during descent. |
| **Hoverslam / suicide burn** | Landing burn timed so the speed hits zero exactly at the ground, needed when the booster can't hover. |
| **Isp** | Specific impulse, a measure of engine efficiency in seconds. |
| **Kármán line** | 100 km, the conventional edge of space. |
| **MECO / SECO** | Main- / second-engine cut-off. |
| **PD controller** | Feedback law: correction = gain × error − gain × rate. |
| **RCS** | Reaction control system, small thrusters for turning. |
| **RTLS** | Return to launch site. |
