FROM python:3.12-slim
RUN pip install --no-cache-dir numpy matplotlib scipy
WORKDIR /workspace
# Agent gets the dev simulator, the spec and a template -- NOT scoring/ or prompts/.
COPY sim/ /workspace/sim/
COPY agent_env/ /workspace/
