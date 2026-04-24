import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor

from teleop_dual.config import LeftSpaceMouseTeleopConfig, RightSpaceMouseTeleopConfig
np.set_printoptions(precision=4, linewidth=np.inf, suppress=True)

def main():
    env_left = LeftSpaceMouseTeleopConfig().get_environment()
    # env_right = RightSpaceMouseTeleopConfig().get_environment()
    with ThreadPoolExecutor(max_workers=2) as pool:
        f_left = pool.submit(env_left.reset)
        # f_right = pool.submit(env_right.reset)
        _ = f_left.result()
        # _ = f_right.result()
    print("Reset done")

    # 线程池：两个工作线程分别负责左右环境
    with ThreadPoolExecutor(max_workers=2) as pool:
        while True:
            # 这里按原逻辑构造动作向量
            actions_left = np.zeros(env_left.action_space.sample().shape, dtype=np.float32)
            # actions_right = np.zeros(env_right.action_space.sample().shape, dtype=np.float32)

            # 并行提交 step
            f_left = pool.submit(env_left.step, actions_left)
            # f_right = pool.submit(env_right.step, actions_right)

            # 等待本轮两个环境都完成
            _ = f_left.result()
            # _ = f_right.result()

            time.sleep(0.01)

if __name__ == "__main__":
    main()
