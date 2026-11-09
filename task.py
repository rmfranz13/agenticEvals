"""
Inspect task: 2-D lunar lander flight software, Tier 0.

  uv run inspect eval task.py --model anthropic/claude-haiku-5-5
  uv run inspect eval task.py -T n_hidden=50 --model anthropic/claude-opus-5-5

The agent works in a Docker sandbox containing the dev simulator and spec.
When it submits, the scorer reads /workspace/controller.py and pipes it into
a separate, network-less `grader` container (compose.yaml) that holds its own
copy of the simulator. There the physics runs in a trusted process and the
controller in an unprivileged worker process, so nothing the agent edits or
writes can change the grading physics or see the hidden-seed secret.
"""
import json
from pathlib import Path

from inspect_ai import Task, task
from inspect_ai.agent import react
from inspect_ai.dataset import Sample
from inspect_ai.scorer import Score, Target, mean, scorer, stderr
from inspect_ai.solver import TaskState
from inspect_ai.tool import bash, python
from inspect_ai.util import sandbox

HERE = Path(__file__).parent
GRADER = "/grader/scoring/score_runner.py"   # inside the grader container


@task
def lander_tier0(n_hidden: int = 20, secret: str = "change-me-before-publishing",
                 time_limit: int = 3600, message_limit: int = 200):
    return Task(
        dataset=[Sample(id="tier0", input=(HERE / "prompts" / "tier0.md").read_text(),
                        metadata={"tier": 0})],
        solver=react(tools=[bash(timeout=600), python(timeout=600)]),
        scorer=landing_scorer(n_hidden=n_hidden, secret=secret),
        sandbox=("docker", str(HERE / "compose.yaml")),
        time_limit=time_limit,
        message_limit=message_limit,
    )


@scorer(metrics=[mean(), stderr()])
def landing_scorer(n_hidden: int, secret: str):
    async def score(state: TaskState, target: Target) -> Score:
        try:
            code = await sandbox().read_file("/workspace/controller.py")
        except FileNotFoundError:
            return Score(value=0.0, explanation="No /workspace/controller.py found.")
        try:
            result = await sandbox("grader").exec(
                ["python", GRADER, "-", "--n", str(n_hidden), "--worker-user", "ctrl"],
                input=code, env={"LANDER_SECRET": secret}, timeout=60 * n_hidden)
        except TimeoutError:
            return Score(value=0.0, explanation="Grading timed out (controller too slow).")
        out, err = result.stdout, result.stderr
        try:
            res = json.loads(out.strip().splitlines()[-1])
        except Exception:  # noqa: BLE001
            return Score(value=0.0, explanation=f"Grader error: {err[-1500:]}")
        expl = (f"{res['success_rate']:.0%} of {res.get('n', n_hidden)} hidden flights succeeded. "
                f"Outcomes: {res.get('status_counts')}. "
                f"Median miss distance (successful): {res.get('median_distance_successful')}.")
        if res.get("failure_reasons"):
            expl += " Example failures: " + " | ".join(res["failure_reasons"][:3])
        return Score(value=res["success_rate"], answer=code[:4000], explanation=expl, metadata=res)

    return score
