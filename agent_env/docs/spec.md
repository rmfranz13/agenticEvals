# Lunar Lander Flight Software — Tier 0 Specification

You are writing the flight software for a 2-D lunar lander. It starts in lunar
orbit and must come to rest, upright, on both footpads, at the landing site.

## Deliverable

`/workspace/controller.py` defining:

```python
class Controller:
    def reset(self, mission: dict) -> None: ...
    def step(self, obs: dict) -> dict:   # {"throttle": float, "gimbal": float}
```

A fresh `Controller()` is created for every flight. `reset` is called once
with the mission dict, then `step` is called at `mission["control_hz"]`
(10 Hz). The command is held constant until the next call.

Constraints when graded: pure Python (stdlib + numpy only), deterministic,
no file, network or subprocess access. A `step` call should take well under
50 ms. Exceptions, NaNs or infinities end the flight as a failure.

## Frames and conventions

- **Site frame:** origin on the surface at the landing site; +z is local
  vertical at the site, +x is downrange (the direction of orbital motion).
  The Moon's centre is at (0, -R). The frame is inertial (no lunar rotation).
- **theta:** angle of the vehicle's body "up" axis from site +z, radians,
  positive counter-clockwise (positive theta tilts the nose toward -x).
- **Gimbal:** radians relative to the body axis. Positive gimbal produces a
  positive (counter-clockwise) torque; the engine is below the centre of mass.
- Positions and velocities are those of the vehicle's **centre of mass**.

## Observations (`obs`), all available in Tier 0

| key | meaning |
|---|---|
| `t` | mission time, s |
| `x`, `z` | COM position in site frame, m |
| `vx`, `vz` | COM velocity in site frame, m/s |
| `theta`, `omega` | attitude (rad) and angular rate (rad/s) |
| `mass`, `fuel` | current total mass and remaining propellant, kg |
| `engine_on`, `throttle`, `gimbal` | actual engine state |
| `altitude` | COM height above the ground directly below (terrain near the site, sphere elsewhere), m |
| `gravity` | local gravitational acceleration vector [gx, gz], m/s² |
| `pad_contact` | [left, right] footpad contact switches |

## Mission dict (given to `reset`)

Moon (`mu`, `radius`), target (site-frame origin), full vehicle parameters,
`nominal_rest_com_height` (COM height above flat ground when standing on its
pads), `control_hz`, success `criteria`, `terrain_half_width`.

## Vehicle (nominal; graded flights are dispersed — see below)

| parameter | value |
|---|---|
| dry mass | 700 kg (cannot go below) |
| wet mass | 1600 kg (±1% dispersion) |
| max thrust | 7000 N (±1%) |
| Isp | 320 s (±0.5%); mass flow = throttle × max_thrust / (Isp × g0) |
| throttle | commanded < 0.10 → engine OFF; otherwise min(cmd, 1.0) |
| relight | allowed |
| gimbal limit | ±30° |
| inertia | I = mass × (1.1 m)² |
| engine gimbal point | 1.0 m below COM |
| footpad hinges | (±1.8, −1.6) m in body frame; pads 0.6 m long, conform to terrain up to ±30° |
| hull | polygon from z_b = −1.3 m (engine bell) to +1.8 m; any hull contact with terrain is a crash |

## Initial conditions (dispersed)

Periapsis of a 15 km × 100 km orbit (periapsis altitude ±0.5 km),
480 km (±10 km) of surface arc uprange of the site, moving toward +x.
Velocity error ±1 m/s per axis (1σ), attitude roughly retrograde (±2°),
small angular rate.

## Terrain

Within ±2000 m of the site, the surface is a profile of straight segments
(1.5–6 m long) with lunar curvature, craters (none within 40 m of the site)
and small steep-sided rocks up to 0.25 m tall. Outside that patch the Moon
is a bare sphere and touching it is a failure. Footpads have Coulomb friction
(μ ≈ 0.6), so a pad can catch on a rock and trip the vehicle.

## Success (all required)

1. No mission-ending event during the flight:
   - any footpad touching down faster than **2.0 m/s into the surface** or
     **1.0 m/s along it**,
   - any hull contact with the terrain,
   - tilt beyond 45° while a pad is in contact,
   - contact outside the terrain patch.
2. Final state: both pads in contact, the COM above and between the pads,
   **engine off**, |vx| < 0.01 m/s, |vz| < 0.02 m/s, |ω| < 0.01 rad/s,
   held continuously for 3 s.
3. COM horizontal distance from the site ≤ **50 m**.

Flights time out at 2500 s. Score = fraction of hidden dispersed flights that
succeed. Distance from the site and fuel remaining are also recorded.

## Tools in your sandbox

- `sim/lander_sim.py` — the simulator (same physics as the grader).
- `sim/run_sim.py controller.py [--seeds A B] [--nominal] [--csv out.csv]`
  flies your controller on dev seeds and prints outcomes.
