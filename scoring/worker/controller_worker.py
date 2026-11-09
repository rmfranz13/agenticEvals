#!/usr/bin/env python3
"""Untrusted half of the grader: hosts the agent's Controller in its own process.

The simulator runs in score_runner.py's process; this process only ever sees
the observations it is sent, so a controller cannot patch the physics, the
scenario generator or the scoring. One worker is started per flight.

Protocol, one JSON object per line on the original stdin/stdout:
  <- {"ok": true}                          after the controller module loads
  -> {"op": "reset", "mission": {...}}     <- {"ok": true}
  -> {"op": "step", "obs": {...}}          <- {"cmd": {...}}
  any failure                              <- {"error": "..."}

This file lives in its own directory so that, run with `python -I`, nothing
else is importable from the script directory.
"""
import importlib.util
import json
import os
import sys
import traceback


def main():
    path = sys.argv[1]
    # Move the protocol onto private fds so prints/reads in the controller
    # cannot corrupt it: fd 1 -> stderr, fd 0 -> /dev/null.
    proto_in = os.fdopen(os.dup(0), "r")
    proto_out = os.fdopen(os.dup(1), "w")
    os.dup2(os.open(os.devnull, os.O_RDONLY), 0)
    os.dup2(2, 1)

    def send(obj):
        try:
            line = json.dumps(obj, default=float)
        except Exception as e:  # noqa: BLE001  e.g. command contains a non-number
            line = json.dumps({"error": f"command not JSON-serialisable: {e!r}"})
        proto_out.write(line + "\n")
        proto_out.flush()

    try:
        spec = importlib.util.spec_from_file_location("agent_controller", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ctrl = mod.Controller()
    except BaseException:  # noqa: BLE001
        send({"error": "failed to load controller: " + traceback.format_exc()[-1500:]})
        return
    send({"ok": True})

    for line in proto_in:
        msg = json.loads(line)
        try:
            if msg["op"] == "reset":
                ctrl.reset(msg["mission"])
                send({"ok": True})
            else:
                send({"cmd": ctrl.step(msg["obs"]) or {}})
        except BaseException as e:  # noqa: BLE001
            send({"error": repr(e)})


if __name__ == "__main__":
    main()
