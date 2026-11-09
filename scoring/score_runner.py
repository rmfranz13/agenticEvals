#!/usr/bin/env python3
"""Grade a controller on HIDDEN scenarios. Prints one JSON summary to stdout.

In the eval this runs inside the network-less `grader` container (see
compose.yaml / task.py); it also runs locally for sanity checks:

  python scoring/score_runner.py controller.py --n 20 --secret test
  python scoring/score_runner.py - --n 20 < controller.py      # code on stdin

Hidden seeds are derived from a secret (--secret, or $LANDER_SECRET), so they
cannot be guessed from the dev seeds the agent tested on.

The physics runs in THIS process. The agent's controller runs in a separate
worker process per flight (worker/controller_worker.py), optionally as an
unprivileged user (--worker-user), and only exchanges obs/commands over a
pipe -- so it cannot monkeypatch the simulator or read the secret.
"""
import argparse
import hashlib
import json
import os
import select
import statistics
import subprocess
import sys
import tempfile
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
import lander_sim as L  # noqa: E402

WORKER = os.path.join(HERE, "worker", "controller_worker.py")
LOAD_TIMEOUT_S = 30.0   # import + construct + reset
STEP_TIMEOUT_S = 5.0    # hard cap per step (spec asks for well under 50 ms)


def hidden_seeds(secret: str, n: int):
    return [int(hashlib.sha256(f"{secret}:{i}".encode()).hexdigest()[:8], 16) for i in range(n)]


class RemoteController:
    """Controller proxy: same reset/step API, real controller in a subprocess."""

    def __init__(self, path: str, user: str | None = None):
        kw = {}
        if user:
            kw = {"user": user, "group": user, "extra_groups": []}
        self.proc = subprocess.Popen(
            [sys.executable, "-I", WORKER, path],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            env={"PATH": "/usr/local/bin:/usr/bin:/bin"}, cwd=os.path.dirname(path),
            text=True, bufsize=1, **kw)
        self._recv(LOAD_TIMEOUT_S)

    def _recv(self, timeout):
        ready, _, _ = select.select([self.proc.stdout], [], [], timeout)
        if not ready:
            raise RuntimeError(f"controller did not respond within {timeout:g} s")
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError("controller process exited")
        msg = json.loads(line)
        if "error" in msg:
            raise RuntimeError(msg["error"])
        return msg

    def _call(self, msg, timeout):
        try:
            self.proc.stdin.write(json.dumps(msg) + "\n")
            self.proc.stdin.flush()
        except BrokenPipeError:
            raise RuntimeError("controller process exited") from None
        return self._recv(timeout)

    def reset(self, mission):
        self._call({"op": "reset", "mission": mission}, LOAD_TIMEOUT_S)

    def step(self, obs):
        return self._call({"op": "step", "obs": obs}, STEP_TIMEOUT_S)["cmd"]

    def close(self):
        self.proc.kill()
        self.proc.wait()


def fly(path: str, seed: int, user: str | None):
    ctrl = None
    try:
        ctrl = RemoteController(path, user)
        return L.simulate(ctrl, L.make_scenario(seed))
    except Exception:  # noqa: BLE001
        return {"seed": seed, "status": "error", "success": False,
                "reason": traceback.format_exc()[-500:], "distance_from_site": float("nan"),
                "fuel_remaining": float("nan")}
    finally:
        if ctrl is not None:
            ctrl.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("controller", help="path to controller.py, or - to read the code from stdin")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--secret", default=os.environ.get("LANDER_SECRET", "tier0-default-secret"))
    ap.add_argument("--worker-user", help="run the controller as this (unprivileged) user; needs root")
    a = ap.parse_args()

    with tempfile.TemporaryDirectory() as d:
        os.chmod(d, 0o755)              # readable by the worker user
        path = os.path.join(d, "controller.py")
        if a.controller == "-":
            code = sys.stdin.read()
        else:
            with open(a.controller) as f:
                code = f.read()
        with open(path, "w") as f:
            f.write(code)
        os.chmod(path, 0o644)

        runs = []
        for s in hidden_seeds(a.secret, a.n):
            r = fly(path, s, a.worker_user)
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
