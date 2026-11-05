#!/usr/bin/env python3
"""Fly a controller against the dev simulator.

  python sim/run_sim.py controller.py                 # 5 dispersed scenarios, seeds 0-4
  python sim/run_sim.py controller.py --seeds 0 20    # seeds 0..19
  python sim/run_sim.py controller.py --seeds 3 4 --csv traj.csv   # one run, log trajectory
  python sim/run_sim.py controller.py --nominal       # undispersed scenario

The grader flies the same physics, but on hidden scenario seeds drawn from the
same dispersion distributions (see make_scenario in lander_sim.py).
"""
import argparse
import csv
import importlib.util
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lander_sim as L  # noqa: E402


def load_controller(path):
    spec = importlib.util.spec_from_file_location("agent_controller", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Controller


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("controller")
    ap.add_argument("--seeds", nargs=2, type=int, default=[0, 5])
    ap.add_argument("--nominal", action="store_true")
    ap.add_argument("--csv", help="write trajectory of the (last) run to this file")
    ap.add_argument("--json", action="store_true", help="print full JSON results")
    a = ap.parse_args()
    Ctrl = load_controller(a.controller)
    seeds = [0] if a.nominal else range(*a.seeds)
    results = []
    for s in seeds:
        sc = L.make_scenario(s, dispersed=not a.nominal)
        r = L.simulate(Ctrl(), sc, record=bool(a.csv), record_hz=10.0)
        if a.csv:
            with open(a.csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(r.pop("trajectory_columns"))
                w.writerows(r.pop("trajectory"))
        results.append(r)
        if not a.json:
            print(f"seed {s:4d}  {r['status']:8s} success={r['success']!s:5s} "
                  f"dist={r['distance_from_site']:9.2f} m  fuel={r['fuel_remaining']:6.1f} kg  "
                  f"t={r['t_end']:7.1f} s  | {r['reason']}")
    n_ok = sum(r["success"] for r in results)
    if a.json:
        print(json.dumps(results, indent=1, default=float))
    else:
        print(f"\nsuccess {n_ok}/{len(results)}")


if __name__ == "__main__":
    main()
