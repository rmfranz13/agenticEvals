# Lunar Lander Flight Software Eval — Tier 0

An agentic Inspect eval: the model writes 2-D lunar lander flight software
that brings a vehicle from lunar orbit to rest, upright, at a landing site.

## Layout

| path | who sees it | what |
|---|---|---|
| `sim/lander_sim.py` | agent + grader | physics: orbit, rigid body, gimbaled engine, terrain, footpad contact |
| `sim/run_sim.py` | agent | fly a controller on dev seeds |
| `agent_env/docs/spec.md` | agent | the task specification |
| `agent_env/controller.py` | agent | template the agent replaces |
| `prompts/tier0.md` | agent (as prompt) | task prompt |
| `scoring/score_runner.py` | grader only | flies the controller on secret-derived hidden seeds |
| `scoring/reference_controller.py` | **never ship to the sandbox** | proof that Tier 0 is landable |
| `task.py`, `compose.yaml`, `Dockerfile` | Inspect | sandbox + react agent + custom scorer |

## Run

```bash
uv pip install inspect-ai numpy          # Docker must be running
uv run inspect eval task.py --model anthropic/claude-haiku-5-5
uv run inspect eval task.py -T n_hidden=50 -T secret=<yours> --model anthropic/claude-opus-5-5
uv run inspect view
```

Sanity checks without an agent:

```bash
python sim/run_sim.py scoring/reference_controller.py --seeds 0 10
python scoring/score_runner.py scoring/reference_controller.py --n 20 --secret test
python scoring/score_runner.py agent_env/controller.py --n 3        # should score 0
```

## Reference controller results (Tier 0)

90/90 dispersed flights landed (dev and hidden seeds), median miss ~2 cm,
~100–120 kg propellant margin of 900 kg, touchdown 0.5–0.8 m/s vertical,
~0.2 m/s horizontal. About 1.4 s of wall time per flight.

## Known limitations / next steps

- The scorer runs agent code on the host in a subprocess; move grading into a
  separate network-less container before publishing.
- Leg struts are not collision geometry; only the hull polygon and pads are.
- COM is fixed and inertia scales with mass (no propellant CG shift yet).
- Engine response is instantaneous (no throttle/gimbal rate limits yet).
- Friction coefficient 0.6 is a placeholder pending a sourced regolith value.
