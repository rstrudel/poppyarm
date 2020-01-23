import time
import meshcat

from poppyarm.poppy import Poppy

arm = Poppy()
vis = meshcat.Visualizer()
arm.wrapper.initMeshcatDisplay(vis, robot_color=[1.0, 1.0, 1.0, 1.0])

# Sample a random collision-free configuration and displays it
for _ in range(100):
    theta, q = arm.random_configuration()
    arm.wrapper.display(q)
    time.sleep(1)
