import numpy as np
import time
import sys
sys.path.append("/home/ubuntu/FR3/")
from teleop.config import SpaceMouseTeleopConfig
np.set_printoptions(precision=4, linewidth=np.inf, suppress=True)

def main():

    env = SpaceMouseTeleopConfig()
    env = env.get_environment()
    env.reset()
    print("Reset done")
    
    while True:
        actions = np.zeros(env.action_space.sample().shape) 
        env.step(actions)
        time.sleep(0.01)
    
        
if __name__ == "__main__":
    main()
