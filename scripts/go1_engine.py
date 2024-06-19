import random
import time

from engine_utils import timeout
from go1_env import Go1Env

step_frequency = 25  # 25 Hz
step_interval = 1.0 / step_frequency


class CustomTrainingAlgorithm:
    def __init__(self):
        self.env = Go1Env()

    @timeout(step_interval)
    def step_with_timeout(self):
        # test timeout func
        # sleep_time = random.uniform(0.03, 0.05)
        # time.sleep(sleep_time)

        return self.env.step()

    def run(self):
        terminated = False
        while not terminated:
            result = self.step_with_timeout()
            if result is None:
                print("Step exceeded 40ms and was skipped.")
            else:
                print("Finished step.")
                terminated = result


if __name__ == "__main__":
    engine = CustomTrainingAlgorithm()
    engine.run()
