# Agent sandbox. numpy is pinned to match grader.Dockerfile so dev and graded
# physics are bit-for-bit the same library.
FROM python:3.12-slim
RUN pip install --no-cache-dir numpy==2.5.4 matplotlib scipy
WORKDIR /workspace
# Agent gets the dev simulator, the spec and a template -- NOT scoring/ or prompts/.
COPY sim/ /workspace/sim/
COPY agent_env/ /workspace/
