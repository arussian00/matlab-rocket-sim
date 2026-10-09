# Roadmap: what to add next

A pick-up list for the next session. Each row is a self-contained addition: pick one, and paste its "Ask" text to Claude to start.

**Effort:** 🟢 an hour or so · 🟡 an afternoon · 🔴 a big project (changes how the whole simulator works)

## 1. Next up: realism upgrades

| # | Addition | Effort | Why it matters | Where it plugs in | Ask |
|---|---|---|---|---|---|
| 1 | **Earth's rotation** | 🟢 | About 420 m/s of free eastward speed at Starbase's latitude. The ship would splash down somewhere else, and the booster's catch would need retuning. | Starting `vx` in the scripts, plus the Coriolis and centrifugal terms in `boosterDynamics` and `starshipStack` | "Add Earth's rotation to all missions" |
| 2 | **Booster entry burn or lofted return** | 🟡 | The only way to make Super Heavy's descent really gentle: it hits 10 km at ~1,170 m/s and ~10 g. Try a short entry burn, or a lower boostback trajectory. | A new phase before `brake` in `simulateBooster.m` | "Make Super Heavy arrive slower so its landing is gentler" |
| 3 | **Engine spool-up time and gimbal rate limits** | 🟡 | Real Raptors take 1–3 s to reach full thrust, and the nozzle can only swivel so fast. Landings get harder and more realistic. | Two new states (actual throttle, actual gimbal) in `boosterDynamics` and `attitudeControl` | "Add engine spool-up and gimbal rate limits" |
| 4 | **Heat-shield temperature and total heat load** | 🟢 | `heatFlux.m` gives heating *rate* only. Adding it up over the flight shows whether the tiles would survive, and could set the entry-burn timing. | Post-processing in `rocket_starship_sim.m`, plus a panel in `animateStarship.m` | "Add heat-shield temperature and total heat load" |
| 5 | **Booster approach from beside the tower** | 🟡 | SpaceX aims the booster *next to* the tower and slides it in at the last moment, so a failure doesn't hit the tower. | The target position in `landingAccel` | "Make Super Heavy approach offset from the tower and slide in" |
| 6 | **Moving centre of gravity and propellant slosh** | 🟡 | As the tanks drain, the rocket's balance point moves and it turns differently. Slosh can upset a landing. | Inertia and lever arm in `attitudeControl` | "Add a moving centre of gravity and propellant slosh" |
| 7 | **Wind and weather profiles** | 🟢 | Wind is now one simple bump at 10 km. Real launch days have jet streams and gusts at many altitudes. | `windAt` in `simulateBooster.m`; random profiles in `rocket_monte_carlo.m` | "Add realistic wind profiles and gusts" |
| 8 | **Ship tower catch** (instead of a splashdown) | 🟡 | SpaceX's plan is to catch the ship as well. The `hLand` machinery already exists, but it needs a target planned before launch, not placed at the flip. | `'starship'` preset in `rocket_params.m`; guidance during the belly-flop | "Catch Starship on a tower instead of splashing down" |
| 9 | **3-D flight with bank-angle steering** | 🔴 | The real ship steers its lift sideways by banking (like the Space Shuttle) to hit a target. It needs a full 3-D model. | New state vector everywhere (a big rewrite) | "Plan a 3-D version of the simulator" |
| 10 | **Real orbit to a target and a deorbit burn** | 🟡 | Today the ship stops just short of orbit. A real mission would reach orbit, coast, then fire a deorbit burn at a chosen time to land at a chosen place. | `rocket_starship_sim.m` (new phases) | "Put the ship in orbit and add a deorbit burn" |

## 2. Fun features

| # | Addition | Effort | What you'd get | Ask |
|---|---|---|---|---|
| 11 | **Starship slider app** | 🟡 | Like `rocket_app.m`, but for Starship: sliders for payload, kick angle, entry angle of attack, booster reserve, then LAUNCH. | "Make a slider app for the Starship mission" |
| 12 | **Starship Monte Carlo** | 🟢 | 50 full Starship missions with random errors: catch and splashdown success rates plus plots. The test scripts already exist; this would package them. | "Add a Monte Carlo study for the Starship mission" |
| 13 | **Save the mission-control animation as a video or GIF** | 🟢 | Share your flight. `animateStarship` already has a `gifFile` option; a video (`VideoWriter`) would be smaller and smoother. | "Let me save the Starship animation as a video" |
| 14 | **Mission presets** | 🟢 | One-line switches for different flights: "Flight 5 style" (catch), "orbital" (real orbit), "heavy payload", "booster failure". | "Add mission presets to the Starship script" |
| 15 | **Failure scenarios** | 🟡 | Engine-out during ascent or landing, a stuck flap, a late flip: see how the vehicle copes (or doesn't). | "Add failure scenarios like engine-out" |

## 3. Housekeeping

| # | Item | Effort | Why | Ask |
|---|---|---|---|---|
| 16 | **Run everything in real MATLAB** | 🟢 | The project now runs fully in GNU Octave, and that's where it has been tested. The changes for Octave were written to work in MATLAB too, but haven't been run there. Run the five scripts below in MATLAB and report any red errors. | "Here are the MATLAB errors I got: ..." |
| 17 | **Automatic tests on GitHub** | 🟡 | A GitHub Action that runs every mission on each pull request (with Octave, which is free) and fails if a landing stops working. It catches mistakes before you merge. | "Set up automatic tests for this repo" |
| 18 | **Speed up the simulations** | 🟡 | The Starship script takes ~30–50 s. Caching the atmosphere and loosening solver tolerances where safe could halve it, and make a Starship Monte Carlo practical. | "Make the simulations run faster" |
| 19 | **Tidy the two-stage script** | 🟢 | `rocket_two_stage_sim` and `rocket_starship_sim` share a lot of point-mass code that could move into one shared function. | "Clean up the duplicated ascent code" |

The scripts to check in MATLAB (item 16): `rocket_hop_sim`, `rocket_monte_carlo`, `rocket_two_stage_sim`, `rocket_starship_sim`, `rocket_app`.

## 4. Already done (for reference)

| Addition | Where |
|---|---|
| Landing drift fix (grid fins aim above the pad) | `finAimAlt` in `rocket_params.m` |
| Starship mission: hot staging, booster tower catch, belly-flop splashdown | `rocket_starship_sim.m` |
| Whole flight from the launch pad, for every printout, graph and animation | `pointMassSegment.m`, `fullFlight.m` |
| Two-stage drawing (Super Heavy + Starship) and live 12-panel mission control | `animateStarship.m` |
| Lifting re-entry (60° angle of attack, ~2.5 g instead of 7.5 g) | `CLside`, `entryAoADeg` |
| Re-entry heating estimate | `heatFlux.m` |
| U.S. Standard Atmosphere 1976 | `earthModel.m` |
| Mach-dependent drag | `machDrag.m` |
| Max-q throttle bucket and 4 g ship limit | `S.thrBucket`, `S.gLimit` |
| g-limited booster landing burn | `brakeGmax` |
| Runs in GNU Octave (free, no time limit) as well as MATLAB | `octaveCompat.m`, `octave/` |

## Where things stand

| Mission | Result |
|---|---|
| Hop | lands 0.08 m from the pad; Monte Carlo 20/20 |
| Two-stage | reaches orbit; booster lands 1.1 m from the drone ship |
| Starship | booster caught 0.08 m from tower centre (8/8 with random errors); ship splashes down 1.5 m from target at T+44 min (6/6 with random errors) |
