from engine_utils import timeout
from go1_env import Go1Env


class CustomTrainingAlgorithm:
    def __init__(self):
        self.env = Go1Env()

    @timeout(0.04)  # 设置超时时间为40ms
    def step_with_timeout(self):
        return self.env.step()

    def run(self):
        terminated = False
        while not terminated:
            result = self.step_with_timeout()
            if result is None:
                print("Step exceeded 40ms and was skipped.")
            else:
                terminated = result


if __name__ == "__main__":
    engine = CustomTrainingAlgorithm()
    engine.run()
