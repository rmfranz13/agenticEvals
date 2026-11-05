"""
Reference controller (HIDDEN from the agent). Exists to prove Tier 0 is landable.

Guidance, in the local horizontal / vertical frame at the vehicle:
  1. Braking: horizontal decel = v_h^2 / (2 s_togo), i.e. the constant
     deceleration that stops exactly over the site. Vertical channel tracks an
     altitude profile that falls linearly with range-to-go, with centrifugal
     relief (g - v_h^2 / r) as feed-forward.
  2. Terminal (below the gate): PD hold on x = 0, scheduled descent rate.
  3. Cutoff: engine off as soon as either pad reports contact.
Attitude: PD on (theta_desired - theta) through the gimbal, gains scaled by
current thrust so loop bandwidth stays constant as mass/throttle change.
"""
import math

GATE_ALT = 80.0      # m above ground at end of braking
GATE_RANGE = 15.0    # m range-to-go at which terminal phase takes over


class Controller:
    def reset(self, mission):
        self.R = mission["moon"]["radius"]
        v = mission["vehicle"]
        self.Tmax, self.thr_min = v["max_thrust"], v["min_throttle"]
        self.gim_lim = math.radians(v["gimbal_limit_deg"])
        self.L, self.k = v["engine_offset"], v["radius_of_gyration"]
        self.leg = mission["nominal_rest_com_height"]
        self.phase = "coast"
        self.s0 = self.h0 = None

    def _attitude(self, o, ax, az, thr_floor=None):
        m = o["mass"]
        th_d = math.atan2(-ax, az)
        err = (th_d - o["theta"] + math.pi) % (2 * math.pi) - math.pi
        thr = math.hypot(ax, az) * m / self.Tmax
        if abs(err) > 0.35:
            thr = self.thr_min
        thr = min(max(thr, self.thr_min), 1.0)
        T = thr * self.Tmax
        I = m * self.k**2
        wn, zeta = 1.5, 0.9
        gim = I / (self.L * T) * (wn**2 * err - 2 * zeta * wn * o["omega"])
        gim = max(min(gim, self.gim_lim), -self.gim_lim)
        return {"throttle": thr, "gimbal": gim}

    def step(self, o):
        x, z, vx, vz, m = o["x"], o["z"], o["vx"], o["vz"], o["mass"]
        X, Z = x, z + self.R
        r = math.hypot(X, Z)
        ux, uz = X / r, Z / r                 # local up
        hx, hz = uz, -ux                      # local horizontal (+downrange)
        vh, vr = vx * hx + vz * hz, vx * ux + vz * uz
        s = -self.R * math.atan2(X, Z)        # surface range to go (+ = uprange)
        g = math.hypot(*o["gravity"])
        alt = o["altitude"]
        amax = self.Tmax / m

        if self.phase == "coast":
            # Ignite when the stopping decel reaches 60% of available accel.
            if s > 0 and vh * vh / (2 * s) >= 0.6 * amax:
                self.phase, self.s0, self.h0 = "brake", s, alt
            else:
                return {"throttle": 0.0, "gimbal": 0.0}

        if self.phase == "brake":
            if s < GATE_RANGE or vh < 1.0:
                self.phase = "terminal"
            else:
                a_h = -vh * vh / (2 * max(s, 1.0))
                frac = max(s, 0.0) / self.s0
                h_ref = GATE_ALT + (self.h0 - GATE_ALT) * frac
                vr_ref = -(self.h0 - GATE_ALT) / self.s0 * vh
                a_v = (g - vh * vh / r) + 0.004 * (h_ref - alt) + 0.12 * (vr_ref - vr)
                a_v = max(a_v, 0.0)
                # Prioritise the vertical channel if saturated.
                lim = 0.98 * amax
                if math.hypot(a_h, a_v) > lim:
                    a_h = -math.sqrt(max(lim * lim - a_v * a_v, 0.0))
                ax, az = a_h * hx + a_v * ux, a_h * hz + a_v * uz
                return self._attitude(o, ax, az)

        if self.phase == "terminal":
            if any(o["pad_contact"]):
                self.phase = "down"
            else:
                hp = alt - self.leg                       # pad height above ground
                aligned = abs(x) < 1.0 and abs(vx) < 0.15
                if aligned or hp < 3.0:
                    vz_ref = -min(max(0.04 * hp + 0.3, 0.4), 2.0)
                else:
                    vz_ref = 0.0 if hp < 40 else -1.0      # hover-translate until aligned
                ax = -0.12 * x - 0.8 * vx
                ax = max(min(ax, 0.5), -0.5)
                az = g + 1.2 * (vz_ref - vz)
                return self._attitude(o, ax, az)

        return {"throttle": 0.0, "gimbal": 0.0}
