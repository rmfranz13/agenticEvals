"""
2-D lunar lander simulation -- Tier 0.

Rigid-body planar vehicle descending from lunar orbit to a landing site on a
spherical Moon (point-mass gravity). Near the site, a local terrain patch made
of straight segments replaces the sphere, and the two footpads get penalty-
based contact with Coulomb (stick/slip) friction.

Frames
------
Site frame (all observations): origin on the terrain surface at the landing
site, +z = local vertical at the site, +x = downrange (direction of orbital
motion). It is the Moon-centred inertial frame translated by R_moon along z,
so it is inertial (non-rotating Moon). theta = angle of the vehicle's body
"up" axis from site +z, positive counter-clockwise (tilting toward -x).

Body frame: origin at the centre of mass, +z_b = vehicle up, +x_b = right.
Positive gimbal angle tilts the thrust vector toward +x_b, which produces a
positive (counter-clockwise) torque because the engine sits below the COM.

The physics backend is meant to be swappable (e.g. Trick/JEOD/GUNNS later):
the contract is simulate(controller, scenario) -> result, with the
controller API Controller.reset(mission) / Controller.step(obs) -> cmd.
"""
from __future__ import annotations

import bisect
import math
import time
from dataclasses import dataclass, field, asdict

import numpy as np

MU_MOON = 4.9048695e12   # m^3/s^2
R_MOON = 1737.4e3        # m
G0 = 9.80665             # m/s^2 (Isp reference)


# --------------------------------------------------------------------------- config
@dataclass
class Vehicle:
    dry_mass: float = 700.0            # kg, cannot go below
    wet_mass: float = 1600.0           # kg, dry + propellant at start (nominal)
    max_thrust: float = 7000.0         # N
    isp: float = 320.0                 # s; mdot = throttle * max_thrust / (isp * g0)
    min_throttle: float = 0.10         # commanded throttle below this -> engine OFF
    allow_relight: bool = True
    gimbal_limit_deg: float = 30.0     # +/- from body axis
    radius_of_gyration: float = 1.1    # m; I = m * k^2 (COM fixed in tier 0)
    engine_offset: float = 1.0         # m, gimbal point below COM along -z_b
    pad_hinges: tuple = ((-1.8, -1.6), (1.8, -1.6))  # body-frame hinge points (L, R)
    pad_length: float = 0.6            # m, flat 2-D pad, free to conform
    pad_tilt_limit_deg: float = 30.0   # pad rotation limit relative to body
    # Hull polygon (body frame). Any vertex touching terrain = mission failure.
    # Lowest points are the engine bell exit at z_b = -1.3.
    hull: tuple = ((-1.0, -0.9), (-0.35, -1.3), (0.35, -1.3), (1.0, -0.9),
                   (1.0, 1.8), (-1.0, 1.8))


@dataclass
class ContactModel:
    k_n: float = 1.5e5        # N/m per contact point (lumped leg + soil compliance)
    c_n: float = 4.0e3        # N s/m
    k_t: float = 1.5e5        # tangential "bristle" stiffness for static friction
    c_t: float = 2.0e3
    mu: float = 0.6           # pad-regolith Coulomb friction (placeholder, configurable)
    engage_altitude: float = 25.0   # COM height above terrain where contact mode switches on
    dt: float = 1.0e-3        # contact-mode integration step (s)


@dataclass
class Terrain:
    seed: int = 0
    half_width: float = 2000.0        # m each side of site; outside -> bare sphere, no landing
    seg_len: tuple = (1.5, 6.0)       # straight-segment length range (m)
    roughness: float = 0.03           # m, small random height noise per node
    n_craters: int = 14
    crater_radius: tuple = (6.0, 80.0)
    crater_depth_ratio: tuple = (0.05, 0.12)   # depth / radius
    keepout: float = 40.0             # no craters within this distance of the site
    n_rocks: int = 25                 # small step-like blocks (things to trip on)
    rock_height: tuple = (0.05, 0.25)
    rock_width: tuple = (0.3, 1.5)


@dataclass
class Criteria:
    max_touchdown_vn: float = 2.0     # m/s into the surface at first pad contact
    max_touchdown_vt: float = 1.0     # m/s along the surface at first pad contact
    rest_v_horizontal: float = 0.01   # m/s
    rest_v_vertical: float = 0.02     # m/s
    rest_omega: float = 0.01          # rad/s
    rest_hold_time: float = 3.0       # s continuously at rest -> landed
    target_radius: float = 50.0       # m, COM horizontal distance from site for success
    max_tilt_in_contact_deg: float = 45.0
    max_sim_time: float = 2500.0      # s


@dataclass
class Rates:
    control_hz: float = 10.0
    physics_dt: float = 0.02          # free-flight RK4 step (s)


@dataclass
class Scenario:
    seed: int = 0
    vehicle: Vehicle = field(default_factory=Vehicle)
    contact: ContactModel = field(default_factory=ContactModel)
    terrain: Terrain = field(default_factory=Terrain)
    criteria: Criteria = field(default_factory=Criteria)
    rates: Rates = field(default_factory=Rates)
    # Initial orbit: start at periapsis of a periapsis x apoapsis ellipse,
    # `uprange` metres of surface arc before the site, moving toward +x.
    periapsis_alt: float = 15.0e3
    apoapsis_alt: float = 100.0e3
    uprange: float = 480.0e3
    dv_err: tuple = (0.0, 0.0)        # velocity error added at t0 (site frame, m/s)
    theta_err: float = 0.0            # rad, added to retrograde-aligned attitude
    omega0: float = 0.0               # rad/s
    initial: list | None = None       # optional override [x, z, vx, vz, theta, omega, mass] (tests)


def make_scenario(seed: int, dispersed: bool = True) -> Scenario:
    """Tier-0 scenario generator. Dispersion distributions are public; draws are not."""
    rng = np.random.default_rng(seed)
    sc = Scenario(seed=seed)
    sc.terrain = Terrain(seed=int(rng.integers(0, 2**31 - 1)))
    if dispersed:
        sc.periapsis_alt = 15.0e3 + rng.normal(0, 500.0)
        sc.uprange = 480.0e3 + rng.normal(0, 10.0e3)
        sc.dv_err = tuple(rng.normal(0, 1.0, 2))
        sc.theta_err = rng.normal(0, math.radians(2.0))
        sc.omega0 = rng.normal(0, math.radians(0.2))
        sc.vehicle.wet_mass = 1600.0 * (1 + rng.normal(0, 0.01))
        sc.vehicle.max_thrust = 7000.0 * (1 + rng.normal(0, 0.01))
        sc.vehicle.isp = 320.0 * (1 + rng.normal(0, 0.005))
    return sc


# --------------------------------------------------------------------------- terrain
class TerrainProfile:
    """Polyline surface z(x) in the site frame; x strictly increasing."""

    def __init__(self, cfg: Terrain):
        rng = np.random.default_rng(cfg.seed)
        xs = [-cfg.half_width]
        while xs[-1] < cfg.half_width:
            xs.append(xs[-1] + rng.uniform(*cfg.seg_len))
        xs = np.array(xs)
        z = np.sqrt(R_MOON**2 - xs**2) - R_MOON          # lunar curvature
        z += rng.normal(0, cfg.roughness, xs.size)
        for _ in range(cfg.n_craters):
            rad = rng.uniform(*cfg.crater_radius)
            while True:
                xc = rng.uniform(-cfg.half_width, cfg.half_width)
                if abs(xc) > cfg.keepout + 1.3 * rad:
                    break
            depth = rad * rng.uniform(*cfg.crater_depth_ratio)
            d = np.abs(xs - xc) / rad
            bowl = np.where(d < 1, depth * (d**2 - 1), 0.0)
            rim = 0.25 * depth * np.exp(-((d - 1) / 0.25) ** 2)
            z += bowl + rim
        xs, z = list(xs), list(z)
        # Rocks: steep-sided blocks (near-vertical risers, still single-valued in x).
        for _ in range(cfg.n_rocks):
            w = rng.uniform(*cfg.rock_width)
            h = rng.uniform(*cfg.rock_height)
            while True:
                xc = rng.uniform(-cfg.half_width + 5, cfg.half_width - 5)
                if abs(xc) > cfg.keepout:
                    break
            x0, x1 = xc - w / 2, xc + w / 2
            base0, base1 = float(np.interp(x0, xs, z)), float(np.interp(x1, xs, z))
            keep = [(xi, zi) for xi, zi in zip(xs, z) if not (x0 - 0.05 <= xi <= x1 + 0.05)]
            new = [(x0 - 0.02, base0), (x0 + 0.02, base0 + h),
                   (x1 - 0.02, base1 + h), (x1 + 0.02, base1)]
            pts = sorted(keep + new)
            xs, z = [p[0] for p in pts], [p[1] for p in pts]
        z0 = float(np.interp(0.0, xs, z))
        self.xs = [float(v) for v in xs]
        self.zs = [float(v) - z0 for v in z]                # site surface at z = 0
        self.xmin, self.xmax = self.xs[0], self.xs[-1]
        n = len(self.xs) - 1
        self._t, self._n, self._len = [], [], []
        for i in range(n):
            dx, dz = self.xs[i + 1] - self.xs[i], self.zs[i + 1] - self.zs[i]
            L = math.hypot(dx, dz)
            self._t.append((dx / L, dz / L))
            self._n.append((-dz / L, dx / L))            # outward (up-ish) normal
            self._len.append(L)

    def contains(self, x: float) -> bool:
        return self.xmin <= x <= self.xmax

    def _seg(self, x: float) -> int:
        i = bisect.bisect_right(self.xs, x) - 1
        return min(max(i, 0), len(self.xs) - 2)

    def height(self, x: float) -> float:
        i = self._seg(x)
        x0, x1 = self.xs[i], self.xs[i + 1]
        f = (x - x0) / (x1 - x0)
        return self.zs[i] + f * (self.zs[i + 1] - self.zs[i])

    def tangent_angle(self, x: float) -> float:
        tx, tz = self._t[self._seg(x)]
        return math.atan2(tz, tx)

    def penetration(self, px: float, pz: float):
        """If (px, pz) is below the surface, return (depth, n, t) of nearest segment."""
        if not self.contains(px) or pz >= self.height(px):
            return None
        i = self._seg(px)
        best = None
        for j in range(max(0, i - 3), min(len(self._t), i + 4)):
            ax, az = self.xs[j], self.zs[j]
            tx, tz = self._t[j]
            s = min(max((px - ax) * tx + (pz - az) * tz, 0.0), self._len[j])
            cx, cz = ax + s * tx, az + s * tz
            d = math.hypot(px - cx, pz - cz)
            if best is None or d < best[0]:
                best = (d, self._n[j], self._t[j])
        return best


# --------------------------------------------------------------------------- helpers
def _rot(th, bx, bz):
    c, s = math.cos(th), math.sin(th)
    return bx * c - bz * s, bx * s + bz * c


def _gravity(x, z):
    X, Z = x, z + R_MOON
    r = math.hypot(X, Z)
    k = -MU_MOON / r**3
    return k * X, k * Z


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def initial_state(sc: Scenario):
    if sc.initial is not None:
        return list(sc.initial)
    rp, ra = R_MOON + sc.periapsis_alt, R_MOON + sc.apoapsis_alt
    a = 0.5 * (rp + ra)
    vp = math.sqrt(MU_MOON * (2 / rp - 1 / a))
    phi = -sc.uprange / R_MOON
    X, Z = rp * math.sin(phi), rp * math.cos(phi)
    vx, vz = vp * math.cos(phi), -vp * math.sin(phi)
    vx += sc.dv_err[0]
    vz += sc.dv_err[1]
    # body up axis aligned with -v (retrograde): up = (-sin th, cos th)
    sp = math.hypot(vx, vz)
    th = math.atan2(vx / sp, -vz / sp) + sc.theta_err
    return [X, Z - R_MOON, vx, vz, _wrap(th), sc.omega0, sc.vehicle.wet_mass]


def mission_info(sc: Scenario) -> dict:
    """Everything the controller is told up front (tier 0 'freebies')."""
    v = sc.vehicle
    return {
        "moon": {"mu": MU_MOON, "radius": R_MOON,
                 "note": "point-mass gravity; Moon centre at site-frame (0, -radius)"},
        "target": {"x": 0.0, "z": 0.0, "note": "landing site surface point = site-frame origin"},
        "vehicle": asdict(v),
        "nominal_rest_com_height": -v.pad_hinges[0][1],
        "control_hz": sc.rates.control_hz,
        "criteria": asdict(sc.criteria),
        "terrain_half_width": sc.terrain.half_width,
    }


# --------------------------------------------------------------------------- simulate
class Result(dict):
    pass


def simulate(controller, sc: Scenario, record: bool = False, record_hz: float = 2.0) -> Result:
    v, cm, cr = sc.vehicle, sc.contact, sc.criteria
    terr = TerrainProfile(sc.terrain)
    x, z, vx, vz, th, om, m = initial_state(sc)
    mdot_max = v.max_thrust / (v.isp * G0)
    gim_lim = math.radians(v.gimbal_limit_deg)
    pad_lim = math.radians(v.pad_tilt_limit_deg)
    tilt_lim = math.radians(cr.max_tilt_in_contact_deg)

    controller.reset(mission_info(sc))
    t, t_ctrl = 0.0, 0.0
    dt_ctrl = 1.0 / sc.rates.control_hz
    throttle, gimbal, engine_on, ever_shut = 0.0, 0.0, False, False
    anchors = {}                      # (pad, point) -> world anchor for static friction
    pad_was_contact = [False, False]
    pad_contact = [False, False]
    pad_center = [(0.0, 0.0), (0.0, 0.0)]
    rest_t = 0.0
    ctrl_time_max, ctrl_overruns = 0.0, 0
    traj, next_rec = [], 0.0
    max_touch_vn, max_touch_vt = 0.0, 0.0
    status, reason = "timeout", "max_sim_time reached"

    def ground_alt(x_, z_):
        if terr.contains(x_):
            return z_ - terr.height(x_)
        return math.hypot(x_, z_ + R_MOON) - R_MOON

    def thrust_force(th_, m_):
        if not engine_on or m_ <= v.dry_mass:
            return 0.0, 0.0, 0.0, 0.0
        T = throttle * v.max_thrust
        bx, bz = math.sin(gimbal), math.cos(gimbal)
        fx, fz = _rot(th_, T * bx, T * bz)
        torque = v.engine_offset * T * math.sin(gimbal)
        return fx, fz, torque, throttle * mdot_max

    def deriv(s):
        x_, z_, vx_, vz_, th_, om_, m_ = s
        fx, fz, tq, md = thrust_force(th_, m_)
        gx, gz = _gravity(x_, z_)
        I = m_ * v.radius_of_gyration**2
        return [vx_, vz_, fx / m_ + gx, fz / m_ + gz, om_, tq / I, -md]

    while t < cr.max_sim_time:
        # ---------------------------------------------------------- control (ZOH)
        if t >= t_ctrl - 1e-9:
            alt = ground_alt(x, z)
            gx, gz = _gravity(x, z)
            obs = {
                "t": t, "x": x, "z": z, "vx": vx, "vz": vz, "theta": th, "omega": om,
                "mass": m, "fuel": max(m - v.dry_mass, 0.0),
                "engine_on": engine_on, "throttle": throttle if engine_on else 0.0,
                "gimbal": gimbal, "altitude": alt, "gravity": [gx, gz],
                "pad_contact": list(pad_contact),
            }
            t0 = time.perf_counter()
            try:
                cmd = controller.step(obs) or {}
                thr_c = float(cmd.get("throttle", 0.0))
                gim_c = float(cmd.get("gimbal", 0.0))
                if not (math.isfinite(thr_c) and math.isfinite(gim_c)):
                    raise ValueError("non-finite command")
            except Exception as e:  # noqa: BLE001
                status, reason = "error", f"controller exception at t={t:.2f}: {e!r}"
                break
            el = time.perf_counter() - t0
            ctrl_time_max = max(ctrl_time_max, el)
            if el > 0.05:
                ctrl_overruns += 1
            gimbal = min(max(gim_c, -gim_lim), gim_lim)
            want_on = thr_c >= v.min_throttle and m > v.dry_mass
            if want_on and not engine_on and ever_shut and not v.allow_relight:
                want_on = False
            if engine_on and not want_on:
                ever_shut = True
            engine_on = want_on
            throttle = min(thr_c, 1.0) if engine_on else 0.0
            t_ctrl += dt_ctrl

        if record and t >= next_rec - 1e-9:
            traj.append([t, x, z, vx, vz, th, om, m, throttle, gimbal, int(pad_contact[0]), int(pad_contact[1])])
            next_rec += 1.0 / record_hz

        alt = ground_alt(x, z)
        if alt > cm.engage_altitude:
            # ------------------------------------------------------ free flight RK4
            h = min(sc.rates.physics_dt, max(t_ctrl - t, 1e-6))
            s0 = [x, z, vx, vz, th, om, m]
            k1 = deriv(s0)
            k2 = deriv([a + 0.5 * h * b for a, b in zip(s0, k1)])
            k3 = deriv([a + 0.5 * h * b for a, b in zip(s0, k2)])
            k4 = deriv([a + h * b for a, b in zip(s0, k3)])
            x, z, vx, vz, th, om, m = [a + h / 6 * (b1 + 2 * b2 + 2 * b3 + b4)
                                       for a, b1, b2, b3, b4 in zip(s0, k1, k2, k3, k4)]
            t += h
            pad_contact = [False, False]
            anchors.clear()
            if not terr.contains(x) and alt < 5.0:
                status, reason = "crash", f"impacted outside modelled terrain patch at x={x:.0f} m"
                break
        else:
            # ------------------------------------------------------ contact mode
            h = cm.dt
            fx, fz, tq, md = thrust_force(th, m)
            gx, gz = _gravity(x, z)
            Fx, Fz, Tq = fx + m * gx, fz + m * gz, tq
            pad_contact = [False, False]
            for p, (hbx, hbz) in enumerate(v.pad_hinges):
                rx, rz = _rot(th, hbx, hbz)
                hx, hz = x + rx, z + rz
                hvx, hvz = vx - om * rz, vz + om * rx
                if terr.contains(hx) and hz - terr.height(hx) < 0.5:
                    rel = max(min(_wrap(terr.tangent_angle(hx) - th), pad_lim), -pad_lim)
                    pa = th + rel
                else:
                    pa = th
                ca, sa = math.cos(pa), math.sin(pa)
                pfx = pfz = 0.0
                touching = False
                for j, s in enumerate((-0.5, 0.0, 0.5)):
                    px, pz = hx + s * v.pad_length * ca, hz + s * v.pad_length * sa
                    pen = terr.penetration(px, pz)
                    key = (p, j)
                    if pen is None:
                        anchors.pop(key, None)
                        continue
                    d, (nx, nz), (tx, tz) = pen
                    vn, vt = hvx * nx + hvz * nz, hvx * tx + hvz * tz
                    if not pad_was_contact[p] and not touching:
                        max_touch_vn, max_touch_vt = max(max_touch_vn, -vn), max(max_touch_vt, abs(vt))
                        if -vn > cr.max_touchdown_vn or abs(vt) > cr.max_touchdown_vt:
                            status = "crash"
                            reason = (f"leg {'LR'[p]} failed at touchdown: v_normal={-vn:.2f} m/s, "
                                      f"v_tangential={abs(vt):.2f} m/s")
                    touching = True
                    Fn = max(cm.k_n * d - cm.c_n * vn, 0.0)
                    ax_, az_ = anchors.setdefault(key, (px, pz))
                    st = (px - ax_) * tx + (pz - az_) * tz
                    Ft = -cm.k_t * st - cm.c_t * vt
                    lim = cm.mu * Fn
                    if abs(Ft) > lim:
                        Ft = math.copysign(lim, Ft)
                        anchors[key] = (px + Ft / cm.k_t * tx, pz + Ft / cm.k_t * tz)
                    pfx += Fn * nx + Ft * tx
                    pfz += Fn * nz + Ft * tz
                if status == "crash":
                    break
                pad_contact[p] = touching
                pad_center[p] = (hx, hz)
                Fx, Fz = Fx + pfx, Fz + pfz
                Tq += rx * pfz - rz * pfx
            if status == "crash":
                break
            pad_was_contact = list(pad_contact)
            I = m * v.radius_of_gyration**2
            vx += Fx / m * h
            vz += Fz / m * h
            om += Tq / I * h
            x += vx * h
            z += vz * h
            th += om * h
            m = max(m - md * h, v.dry_mass)
            t += h
            # hull strike
            hit = False
            for bx, bz in v.hull:
                ox, oz = _rot(th, bx, bz)
                px, pz = x + ox, z + oz
                if terr.contains(px) and pz < terr.height(px):
                    hit = True
                    break
            if hit:
                status, reason = "crash", f"hull struck terrain at t={t:.2f} s"
                break
            if any(pad_contact) and abs(_wrap(th)) > tilt_lim:
                status, reason = "crash", f"tipped over (theta={math.degrees(_wrap(th)):.1f} deg)"
                break
            # rest / landed check
            between = (pad_contact[0] and pad_contact[1]
                       and min(pad_center[0][0], pad_center[1][0]) < x < max(pad_center[0][0], pad_center[1][0])
                       and z > max(pad_center[0][1], pad_center[1][1]))
            at_rest = (between and not engine_on and abs(vx) < cr.rest_v_horizontal
                       and abs(vz) < cr.rest_v_vertical and abs(om) < cr.rest_omega)
            rest_t = rest_t + h if at_rest else 0.0
            if rest_t >= cr.rest_hold_time:
                status, reason = "landed", "at rest on both pads, COM above and between pads, engine off"
                break
        if m <= v.dry_mass:
            m = v.dry_mass

    dist = abs(x)
    success = status == "landed" and dist <= cr.target_radius
    if status == "landed" and not success:
        reason += f"; but {dist:.1f} m from site (> {cr.target_radius} m)"
    res = Result(
        seed=sc.seed, status=status, success=success, reason=reason, t_end=round(t, 3),
        final_state={"x": x, "z": z, "vx": vx, "vz": vz, "theta": _wrap(th), "omega": om, "mass": m},
        distance_from_site=dist, fuel_remaining=m - v.dry_mass,
        max_touchdown_vn=max_touch_vn, max_touchdown_vt=max_touch_vt,
        controller_max_step_s=ctrl_time_max, controller_overruns=ctrl_overruns,
    )
    if record:
        res["trajectory_columns"] = ["t", "x", "z", "vx", "vz", "theta", "omega", "mass",
                                     "throttle", "gimbal", "padL", "padR"]
        res["trajectory"] = traj
    return res
