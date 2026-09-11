# Plan: JHC-style energy injection in the standalone planner

Replace the current "inject energy only to clear obstacles" behavior in `hopping_astar_planner.py` with a **desired/nominal-height law** that mirrors `hopcopter.py`'s `JumpingHeightController`: every hop fires a mandatory powered burst, and obstacles inject *more* on top. Scope is strictly the `hopcopter-ballistic-planning/` folder — nothing in `ros2_ws/` or the Gazebo controller changes.

**The mapping is rigorous (verified during research).** JHC's `nominal_height = leg_efficiency·v_apex²/(2g)` equals `v_s_min²/(2g)` in planner terms *exactly*, because `leg_efficiency == eta` and `v_s_min = sqrt(eta)·v_g_in`. So `take_off_speed = sqrt(2g·nominal_height) = v_s_min`, and the two models line up with no fudging.

## Steps

### Phase 1 — New physics helpers
`hopping_astar_planner.py`, pure functions near the top, next to `injection_energy` / `max_hop_radius`:

1. `nominal_height(v_g_in, eta, g)` → `eta·v_g_in²/(2g)` (= `v_s_min²/2g`).
2. `thrust_force_from_pwm(pwm)` → Gazebo motor model: `omega = 0.04076521·pwm + 380.8359`, `F = 4·28e-8·omega²`. At PWM 21000 ≈ **1.7135 N**. Mirror of `hopcopter.py`'s `_total_prop_force`.
3. `powered_climb_time(nominal_h, desired_h, take_off_speed, thrust_gain, t_i, t_p_low, t_p_high)` → the JHC branch logic verbatim (`t_p_low` when `desired_h < nominal_h`; else `(desired_h - nominal_h)/(take_off_speed·thrust_gain) + t_i`, clamped to `[t_p_low, t_p_high]`).
4. `climb_injection_energy(take_off_speed, t_p, thrust_force, mass, g)` → `F·d`, with `d = v₀·t_p − 0.5·(g − F/m)·t_p²` (clamp `t_p` at apex time so `d` stays monotone; clamp `d ≥ 0`). This is exactly what `hopcopter.py` accumulates as `∫F·v_z dt`.

### Phase 2 — Wire the law into the energy chain
*Depends on Phase 1.*

5. In `_validate_and_cost`: compute `nominal_h`, `take_off_speed (= v_s_min)`, `t_p`, then `e_inject_base = climb_injection_energy(v_s_min, t_p_JHC)` and `e_inject_ceiling = climb_injection_energy(v_s_min, t_p_high)`. Pass both down. Leave the least-injection angle selection and the `injection_energy(v_s, v_s_min)` cost untouched — the raised floor does the work.
6. In `feasible_alpha_interval`: change **(E1)** floor to `W_lo = v_s_min² + 2·e_inject_base/m` (mandatory injection raises the floor), and **(E2)** ceiling to `W_hi = min(v_s_min² + 2·e_inject_ceiling/m, V_max²)`. Replace the `e_inject_max` parameter with `e_inject_base` + `e_inject_ceiling`.
7. In `max_hop_radius`: use the per-state `e_inject_ceiling` instead of the constant `e_inject_max`.
8. In `HoppingAStarPlanner.__init__`: add `desired_hop_apex`, `thrust_gain`, `t_i`, `t_p_low`, `t_p_high`, `thrust_force`; retire `e_inject_max` (superseded by the thrust-model ceiling).

### Phase 3 — Config + call sites
*Depends on Phase 2.*

9. `config.py`: add `DESIRED_HOP_APEX`, `THRUST_GAIN=0.7`, `T_I=0.06`, `T_P_LOW=0.04`, `T_P_HIGH=0.7`, `POWERED_CLIMB_PWM=21000` (+ motor constants or a precomputed `THRUST_FORCE_N`). Reconcile `E_INJECT_MAX`'s role and **update the import-time asserts** (`_T_CEIL`/`_T_FLOOR` first-hop feasibility and the `V_MAX` derivation) — they currently assume a constant `e_inject_max`.
10. `main.py`: pass the new params to the constructor and fix the `max_hop_radius(...)` viz call for the new signature.

### Phase 4 — Tests & docs
*Parallel with Phase 3 once signatures are fixed.*

11. Update the blast radius (18 files reference the old model): `test/test_hop_energy_chain.py`, `test/test_friction_cone.py`, `test/test_clearance_rejection.py`, `test/test_edge_cost_energy.py`, `test/demo_common.py`, `test/demo_takeoff_angle_range.py`, `test/demo_clearance_sweep.py`; and prose in `docs/alpha_range_new.md`, `CLAUDE.md`, `CHANGELOG.md`.
12. Add new unit tests for the four helpers (see Verification).

## Relevant files

- `hopcopter-ballistic-planning/hopping_astar_planner.py` — new helpers; edit `feasible_alpha_interval`, `max_hop_radius`, `_validate_and_cost`, `__init__`.
- `hopcopter-ballistic-planning/config.py` — new constants; fix `E_INJECT_MAX`/`V_MAX`/assert block.
- `hopcopter-ballistic-planning/main.py` — constructor + viz call.
- `hopcopter-ballistic-planning/test/*`, `docs/*` — update to the new energy sub-model.

## Verification

1. `python main.py flat` → every printed hop shows `E_inj > 0` (mandatory burst) and the steady-state apex settles near `DESIRED_HOP_APEX` (injection balances the 30% stance loss at the fixed point).
2. `python main.py stairs` and `python main.py platform_0p4m` → injection rises above the base where clearance forces a steeper arc.
3. `python -c "import config"` imports with no assert failure.
4. New unit tests: `nominal_height == v_s_min²/2g`; `powered_climb_time` matches both JHC branches incl. the `t_p_low` minimum; `climb_injection_energy` monotone in `t_p` up to apex; the feasible-interval floor rises with `e_inject_base`.

## Decisions

- desired height = **constant apex above landing terrain** (`DESIRED_HOP_APEX`).
- Injection energy = faithful `thrust_force × distance` over `powered_climb_time`, including the `t_p_low` minimum when `nominal ≥ desired`.
- **Every hop injects** the base burst; obstacle clearance can inject up to `e_inject_ceiling`.
- All Campana gates (friction cones, clearance, `min_apex`, landing-speed cap) stay unchanged.
- Cost keeps charging all injection above stance exit (battery is really used).

## Further Considerations

1. **Mass/g for the `E_base` kinematics.** Use the planner's `ROBOT_MASS=0.8` / `G_ACCEL=9.81` (so injected Joules add coherently to the planner's own KE chain) — *Recommended* — **A**; or use `hopcopter.py`'s `hop_mass=0.3609` / `9.8` for byte-faithfulness to the real bookkeeping — **B**. Mixing them is physically inconsistent, so pick one.
2. **`d` formula fidelity.** Include thrust in the deceleration (`g − F/m`, net) — *Recommended* — vs. pure ballistic (`d = v₀·t_p − 0.5·g·t_p²`). Either way, clamp `t_p` at apex so `d` never turns over.
3. **Is `e_inject_ceiling` being state-dependent acceptable?** It now varies with `v_g_in` (through `take_off_speed`), which is physically right (F·v power) but changes `V_MAX`'s meaning to a loose backstop rather than a tight per-hop cap. Recommend keeping `V_MAX` as a documented backstop.
