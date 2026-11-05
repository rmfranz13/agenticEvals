"""Flight software. See docs/spec.md. Replace this placeholder."""


class Controller:
    def reset(self, mission):
        self.mission = mission

    def step(self, obs):
        return {"throttle": 0.0, "gimbal": 0.0}
