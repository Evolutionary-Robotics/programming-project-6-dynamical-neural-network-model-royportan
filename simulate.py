import pybullet as p
import time
import numpy as np
import pybullet_data
import pyrosim.pyrosim as prs
import matplotlib.pyplot as plt
physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# set gravity to 9.81 in Z direction
def toAngle(signal):
    signal = signal*2-1
    return signal*np.pi

p.setGravity(0,0,-20.81)
duration = 1000
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("NewRobot.urdf")

prs.Prepare_To_Simulate(robotId)

for t in range(duration):
    p.stepSimulation()
    time.sleep(1/500)

p.disconnect()
