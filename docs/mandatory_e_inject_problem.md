# The mandatory-injection regression: diagnosis and candidate fixes

**Status: diagnosis complete, fix NOT yet chosen or implemented.** This document
records a debugging conversation (2026-09-10) about why the planner stopped
producing good paths after the powered-climb (JHC-style) injection sub-model
replaced the constant `E_INJECT_MAX` budget. No code has been changed yet; two
open questions for the user (bottom of this file) gate which fix to apply.

## Symptom

After the uncommitted changes in `config.py` / `hopping_astar_planner.py`
(the "powered-climb injection sub-model", mandatory burst per hop), observed in
`results/demonstration_scenarios/`:

| scenario PNG | result |
|---|---|
| `x_0.5_10_y_2.5_3_..._cliff_gap` | **no path at all** (`plan()` returns None) |
| `x_0.5_4.5_y_4.5_2.5_..._stairs` | **no path at all** |
| `x_0.5_4.75_y_4.5_2.5_..._slope_30deg` | path exists but is a dense **zigzag** of tiny crisscrossing hops grinding up the ramp |
| `x_0.5_14.5_y_2.5_2.5_..._maze` | **fine** — normal-looking path |

Before the change (constant `E_INJECT_MAX`), all these scenarios planned
cleanly (see the older `..._cliff_gap` PNG with a good path).

## What changed in the code (uncommitted diff on `main`)

- Old model: takeoff-speed band per hop was
  `[sqrt(eta)*v_g_in, sqrt(eta*v_g_in^2 + 2*E_INJECT_MAX/m)]` with a constant
  budget `E_INJECT_MAX = ROBOT_MASS * g * 1.0 m = 7.85 J`, injection optional
  (floor = bare stance-exit speed).
- New model (`climb_injection_energy`, `powered_climb_time`,
  `thrust_force_from_pwm` in `hopping_astar_planner.py`; constants
  `DESIRED_HOP_APEX=0.8`, `THRUST_GAIN=0.7`, `T_I=0.06`, `T_P_LOW=0.04`,
  `T_P_HIGH=0.7`, `POWERED_CLIMB_PWM=21000`, `THRUST_MASS=0.3609` in
  `config.py`):
  - Every hop fires a **mandatory** burst sized to reach `DESIRED_HOP_APEX`
    above the landing terrain (clamped to `[T_P_LOW, T_P_HIGH]`). Its energy
    `e_inject_base` is added to the **floor**:
    `W_lo = v_s_min^2 + 2*e_base/mass`.
  - The **ceiling** is one full `T_P_HIGH` burst: `e_inject_ceiling`,
    computed as thrust force x climb distance (`F*d`, F ≈ 1.71 N total at
    PWM 21000, climb kinematics using `THRUST_MASS = 0.3609 kg`).
  - Both energies enter the KE chain via `2*E / ROBOT_MASS`
    (`ROBOT_MASS = 0.8 kg`).
  - `feasible_alpha_interval` now takes `e_inject_base` + `e_inject_ceiling`
    instead of `e_inject_max`; `_injection_bounds(v_g_in)` on the planner
    computes both per state.
- **The edge cost (`_edge_cost`) is unchanged.** The regression is entirely in
  the *feasibility band* (E1 floor / E2 ceiling in `feasible_alpha_interval`),
  not in cost shaping.

## Diagnosis — three interacting mechanisms

Numbers below were computed with the repo's own code
(`.venv/bin/python`, `config._climb_injection_energy` etc.). Old budget for
comparison: **7.85 J on every hop, unconditionally.**

| arrival speed `v_g_in` | `e_base` | `e_ceiling` | `v_s` band | max flat hop | max vertical rise |
|---|---|---|---|---|---|
| 2.0 m/s | 0.47 J | 0.47 J | **[2.00, 2.00]** (single point!) | 0.41 m | 0.20 m |
| 3.0 m/s | 0.95 J | 1.07 J | [2.95, 2.99] | 0.91 m | 0.46 m |
| 4.43 m/s | 0.58 J | 2.32 J | [3.90, 4.42] | 1.99 m | 1.00 m |
| 7.0 m/s | 0.39 J | 4.90 J | [5.94, 6.82] | 4.75 m | 2.37 m |

### Mechanism 1 — the ceiling collapsed, and is now pro-cyclical

The budget dropped from a constant 7.85 J to ~0.5–2.3 J at the speeds the robot
actually travels. Worse, the new ceiling **grows with arrival speed** (thrust
work = force x climb distance, and a faster launch climbs farther during the
burst), while climbing is exactly the regime where you land *slower* than you
took off (`v_g^2 = v_s^2 - 2gZ`). So each uphill hop shrinks the next hop's
budget — a **downward spiral**, the opposite of the old constant refill. On
stairs the chain drains until no angle reaches the next tread → `plan()`
returns None. The cliff gap needs one big hop onto a high plateau — the one
thing a moderately-moving robot can no longer buy. At low speed the band
degenerates to a single point (`e_base == e_ceiling`, the mandatory burst
saturates the whole cycle), which after `SPEED_BIN` quantisation usually
misses: a low-speed **energy trap** the old model didn't have.

### Mechanism 2 — the mandatory floor deletes shallow hops (the zigzag)

The base burst raises E1's floor, deleting every parabola below the
0.8-m-apex one. Short hops are then forced near-vertical (α ≈ 83–86°).

**Important correction established by direct probing** (calling
`feasible_alpha_interval` with the slope's normals, per-state injection
bounds, and its `trace=` argument): the landing **friction cone is NOT what
rejects the uphill hops** — in every rejected probe the cone steps cut
nothing. The actual killers on a 30° slope at steady-state speeds:

| hop | verdict | actual killer (from trace) |
|---|---|---|
| 0.2 m straight uphill | feasible, α ≈ 83–86° | — |
| 0.4 m straight uphill (v=3) | rejected | **E3 min-apex floor (77.9°) > E2 injection ceiling (74.2°)** |
| ≥0.8 m straight uphill | rejected | **E2 ceiling cannot reach (X, Z) at any angle** |
| 0.4–1.2 m cross-slope (Z=0) | feasible | — |
| 1.0 m diagonal up-across | rejected | min-apex vs. ceiling again |

The min-apex interaction: `MIN_APEX_HEIGHT = 0.3 m` is drop from apex to
**landing**, so an uphill hop landing `Z` above takeoff must apex at
`Z + 0.3` above takeoff — an energy requirement scaling with terrain that the
tiny new ceiling can't meet for `Z ≳ 0.2 m`. Under 7.85 J it was trivial.

Result: on the slope the feasible menu is "0.2 m uphill baby-step or ~1 m
cross-slope, nothing in between", so A* stitches mostly-lateral hops that each
sneak in a little climb → the crisscross zigzag. (Note: the trace's `fatal`
flag lands on the step where emptiness is *detected*; read `run_lo`/`run_hi`
per step to find the constraint that actually emptied the interval.)

### Why the maze still works

The maze is **flat** — walls are OBSTACLE cells routed *around* in-plane,
never terrain to gain elevation over. With Z=0 both killers vanish: min-apex
is free (steady-state apex ≈ `DESIRED_HOP_APEX` = 0.8 m ≫ 0.3 m), and the
ceiling is never asked to lift terrain (flat 1.2–1.6 m hops have wide α
bands, e.g. [40°, 50°] at 1.6 m / v=4). The flat steady state is exactly the
regime the JHC sub-model is calibrated for (the mandatory burst replaces the
30% stance loss at the fixed point), so the chain sits at v ≈ 4+ m/s where the
band is widest.

## Constraint from the user

Injection at every hop must stay — that's how the real robot behaves. The goal
is to keep mandatory injection while restoring old-quality plans.

## Candidate fixes (recommended order)

### Fix 2 first: resolve the mass inconsistency (possible outright bug)

`THRUST_MASS = 0.3609 kg` (real vehicle) sets the climb kinematics `F/m`, but
the resulting Joules convert to speed via `2*E / ROBOT_MASS` with
`ROBOT_MASS = 0.8 kg`. If the real hopping vehicle is 0.36 kg, the chain mass
is wrong and every burst is worth ~2.2x less Δv² than it should be — a large
chunk of the anemic ceiling. Note the old `E_INJECT_MAX = m*g*1.0` was sized
using 0.8 kg, so old and new models describe *different robots*. Fixing this
also invalidates `W_ENERGY` (derived as 1/1.197 J from the flat steady-state
hop energy — stale either way now) and the `config.py` first-hop asserts.

### Fix 1: rebuild the floor around `T_P_LOW`, treat desired apex as a per-hop command

The behavioral fix for the zigzag. The truly *hardware-mandatory* part of
injection is the `T_P_LOW = 0.04 s` minimum burst (a few hundredths of a
Joule), IF the desired apex is a command input to the real
JumpingHeightController rather than a firmware constant. Then:

- floor = stance-exit speed + `T_P_LOW` burst (≈ the old floor),
- ceiling = stance-exit speed + `T_P_HIGH` burst (as now),
- the planner's chosen `v_s` per hop *implies* the apex command to send —
  each entry in `path_hops` becomes directly executable.

This restores nearly the whole old feasible band, kills the zigzag, and keeps
injection genuinely mandatory. If the 0.8 m aim IS hardwired and untouchable,
the zigzag is *true robot behavior* and the fix belongs on the robot/firmware,
not the planner.

### Fix 3: calibrate the ceiling empirically, then accept the verdict

Even with fixes 1–2, check whether one `T_P_HIGH` burst at PWM 21000 really
buys only ~0.5 m of extra apex: fly the real robot or the Gazebo sim with a
max burst and measure apex gain over an unpowered hop. If measured gain is
larger, one of `POWERED_CLIMB_PWM` / motor-curve coefficients
(`omega = 0.04076521*pwm + 380.8359`, `F = 4*28e-8*omega^2`) / `T_P_HIGH` is
off. If it agrees, the stairs and cliff gap are **genuinely beyond this
vehicle**, the old pretty paths were fiction, and the maps/expectations should
change rather than the model. Escape hatch worth checking: the ceiling grows
with arrival speed, so a run-up (drop or fast approach) raises it — the
`(cell, speed_bin)` A* state already exploits this if the map offers room.

### Non-fixes

- `MIN_APEX_HEIGHT` needs no change — it only bit because the ceiling was too
  low; with fixes 1+2 it recedes to its documented fallback role. It is a real
  mechanism requirement (leg compression detection), don't relax it.
- Do NOT touch `_edge_cost` — it was never the problem.

## Open questions blocking implementation (ask the user)

1. **Is the desired hop apex commandable on the real robot**, or hardwired to
   0.8 m in the controller? (Gates Fix 1.)
2. **What is the real robot's hop mass** — 0.36 kg or 0.8 kg? (Gates Fix 2;
   also determines whether `ROBOT_MASS`, `E_INJECT` conversions, `W_ENERGY`,
   and the `config.py` asserts need re-deriving.)

## Repro / tooling notes for the next session

- The failing/weird PNGs are the four newest files in
  `results/demonstration_scenarios/` (by mtime).
- Interpreter: `.venv/bin/python` (bare `python` is not on PATH).
- To see which constraint kills a hop, call `feasible_alpha_interval` with
  `trace=[]` and inspect per-step `run_lo`/`run_hi` (interval empty when
  `run_lo > run_hi`); remember the per-state injection bounds come from
  `HoppingAStarPlanner._injection_bounds(v_g_in)`, mirrored by the private
  helpers `config._powered_climb_time` / `config._climb_injection_energy`.
- Per CLAUDE.md: any diagnostic re-scoring hops outside the planner must chain
  energy hop-by-hop (`demo_common.diagnose_path`), never score against
  start-of-chain energy.
