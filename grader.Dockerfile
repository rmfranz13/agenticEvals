# Grader sandbox: trusted simulator + score runner. The agent never sees this
# container (no network, not reachable from the agent's sandbox). Controllers
# are flown in a worker process running as the unprivileged `ctrl` user.
FROM python:3.12-slim
RUN pip install --no-cache-dir numpy==2.5.4 \
 && useradd --system --user-group --no-create-home --shell /usr/sbin/nologin ctrl
COPY sim/lander_sim.py /grader/sim/lander_sim.py
COPY scoring/score_runner.py /grader/scoring/score_runner.py
COPY scoring/worker/controller_worker.py /grader/scoring/worker/controller_worker.py
WORKDIR /grader
