# Lunar Lander Flight Software Eval — Tier 0

An agentic Inspect eval: the model writes 2-D lunar lander flight software
that brings a vehicle from lunar orbit to rest, upright, at a landing site.

## Requirements

- **Docker** (with the Compose plugin), running. Inspect starts two
  containers per sample: the agent's sandbox and a separate grader.
- **[uv](https://docs.astral.sh/uv/)**. Inspect itself runs on the host
  (it has to, to drive Docker); uv installs it at the pinned versions in
  `uv.lock`.
- **A model API key** for whichever provider you evaluate, e.g.
  `ANTHROPIC_API_KEY` (you are billed for the agent's usage). Copy
  `.env.example` to `.env` and fill it in; Inspect loads it automatically.
  For other providers, add their SDK first (e.g. `uv add openai`).

## Run

```bash
cp .env.example .env               # then add your API key
uv sync                            # creates .venv from uv.lock
uv run inspect eval task.py --model anthropic/claude-haiku-5-5
uv run inspect eval task.py -T n_hidden=50 -T secret=<yours> --model anthropic/claude-opus-5-5
uv run inspect view
```

The first run builds both Docker images (a minute or two). To smoke-test the
whole pipeline without spending tokens:

```bash
uv run inspect eval task.py --model mockllm/model -T n_hidden=2 -T message_limit=6   # scores 0
```

Task parameters (`-T`): `n_hidden` (hidden flights, default 20), `secret`
(string the hidden seeds are derived from -- **not** a credential, but change
it from the default and keep it private before publishing), `time_limit`
(s, default 3600), `message_limit` (default 200).

Sanity checks without an agent or Docker:

```bash
uv run python sim/run_sim.py scoring/reference_controller.py --seeds 0 10
uv run python scoring/score_runner.py scoring/reference_controller.py --n 20 --secret test
uv run python scoring/score_runner.py agent_env/controller.py --n 3        # should score 0
```

## Layout

| path | who sees it | what |
|---|---|---|
| `sim/lander_sim.py` | agent + grader | physics: orbit, rigid body, gimbaled engine, terrain, footpad contact |
| `sim/run_sim.py` | agent | fly a controller on dev seeds (in-process, for fast iteration) |
| `agent_env/docs/spec.md` | agent | the task specification |
| `agent_env/controller.py` | agent | template the agent replaces |
| `prompts/tier0.md` | agent (as prompt) | task prompt |
| `scoring/score_runner.py` | grader only | flies the controller on secret-derived hidden seeds |
| `scoring/worker/controller_worker.py` | grader only | hosts the untrusted controller in its own process |
| `scoring/reference_controller.py` | **never ship to the sandbox** | proof that Tier 0 is landable |
| `task.py` | Inspect | react agent + custom scorer |
| `compose.yaml`, `Dockerfile`, `grader.Dockerfile` | Inspect | agent sandbox (`default`) and `grader` containers |
| `pyproject.toml`, `uv.lock` | uv | host-side Python dependencies |

## How grading is isolated

```
agent sandbox (default)          grader container
  no network                       no network, read-only FS, tmpfs /tmp
  sim/, docs/, controller.py       sim/, scoring/ (no reference controller)
        |                                  ^
        |  scorer reads controller.py      |  scorer pipes code in on stdin,
        +------------> Inspect (host) -----+  secret via env, reads JSON out
```

Inside the grader, `score_runner.py` runs the physics in its own (root)
process and starts a fresh worker per flight, as the unprivileged `ctrl`
user, that hosts the agent's `Controller`. The two exchange only obs and
commands over a pipe, so a controller cannot monkeypatch the simulator, carry
state between flights, read the secret or write to disk. A step that takes
more than 5 s (or a load/reset over 30 s) fails the flight. The agent's
container never contains `scoring/` and cannot reach the grader.

## Reference controller results (Tier 0)

90/90 dispersed flights landed (dev and hidden seeds), median miss ~2 cm,
~100–120 kg propellant margin of 900 kg, touchdown 0.5–0.8 m/s vertical,
~0.2 m/s horizontal. About 2.5–3 s of wall time per flight.

## Known limitations / next steps

- The 50 ms per-step budget is measured and reported (`controller_overruns`)
  but only the 5 s hard cap is enforced.
- Leg struts are not collision geometry; only the hull polygon and pads are.
- COM is fixed and inertia scales with mass (no propellant CG shift yet).
- Engine response is instantaneous (no throttle/gimbal rate limits yet).
- Friction coefficient 0.6 is a placeholder pending a sourced regolith value.
