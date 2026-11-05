#!/usr/bin/env python3
"""Grade a controller on HIDDEN scenarios. Runs as a subprocess of the Inspect scorer.

Hidden seeds are derived from a secret, so they cannot be guessed from the
dev seeds the agent tested on. Prints one JSON summary to stdout.
"""
import argparse
import hashlib
import json
import os
import statistics
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
import lander_sim as L  # noqa: E402
from run_sim import load_controller  # noqa: E402


def hidden_seeds(secret: str, n: int):
    return [int(hashlib.sha256(f"{secret}:{i}".encode()).hexdigest()[:8], 16) for i in range(n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("controller")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--secret", default="tier0-default-secret")
    a = ap.parse_args()
    try:
        Ctrl = load_controller(a.controller)
    except Exception:  # noqa: BLE001
        print(json.dumps({"success_rate": 0.0, "error": "failed to import controller",
                          "traceback": traceback.format_exc()[-2000:]}))
        return
    runs = []
    for s in hidden_seeds(a.secret, a.n):
        try:
            r = L.simulate(Ctrl(), L.make_scenario(s))
        except Exception:  # noqa: BLE001
            r = {"seed": s, "status": "error", "success": False,
                 "reason": traceback.format_exc()[-500:], "distance_from_site": float("nan"),
                 "fuel_remaining": float("nan")}
        runs.append({k: r[k] for k in ("seed", "status", "success", "reason",
                                       "distance_from_site", "fuel_remaining")})
    ok = [r for r in runs if r["success"]]
    statuses = {}
    for r in runs:
        statuses[r["status"]] = statuses.get(r["status"], 0) + 1
    print(json.dumps({
        "success_rate": len(ok) / len(runs),
        "n": len(runs),
        "status_counts": statuses,
        "median_distance_successful": statistics.median([r["distance_from_site"] for r in ok]) if ok else None,
        "median_fuel_successful": statistics.median([r["fuel_remaining"] for r in ok]) if ok else None,
        "failure_reasons": [r["reason"] for r in runs if not r["success"]][:10],
    }, default=float))


if __name__ == "__main__":
    main()
