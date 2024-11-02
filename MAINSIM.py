import pybullet as p
import time
import numpy as np
import pybullet_data
import pyrosim.pyrosim as prs
import matplotlib.pyplot as plt
import CTRNN

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

neuronNum = 20
Neuron1 = CTRNN.CTRNN(neuronNum, 0.01)
Neuron1.load("neuron6.npz")
duration = 500
Time = np.arange(0.0,duration,0.01)
outputs = np.zeros((len(Time),neuronNum))
states = np.zeros((len(Time),neuronNum))
step = 0
for t in range(len(Time)):
    Neuron1.Step()
    states[t] = Neuron1.State
    outputs[t] = Neuron1.Output
    
#1 2 3 4 - > leg
leg1 = 1
leg2 = 2
leg3 = 3
leg4 = 4
# 5 6 7 8 - > wrist
wrist1 = 5
wrist2 = 6
wrist3 = 7
wrist4 = 8
# 9 10 11 12 - > w11 w12 w21 w22
w11 = 9
w12 = 10
w21 = 11
w22 = 12
# 13 14 15 16 -> w31 w32 w41 w42
w31 = 13
w32 =14
w41 = 15
w42 = 16

activity = np.sum(np.abs(np.diff(outputs,axis=0)))/(duration*neuronNum*0.01)

def toAngle(signal):
    signal = signal*2-1
    return signal*np.pi

def toLegAngle(signal):
    signal = signal/2
    return signal*np.pi

p.setGravity(0,0,-9.81)
Duration = len(Time)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("NewRobot.urdf")

prs.Prepare_To_Simulate(robotId)

for t in range(Duration):
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName =b'Body_Leg1' ,
            controlMode = p.POSITION_CONTROL, targetPosition = toLegAngle(outputs[t][leg1]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Body_Leg2' ,
            controlMode = p.POSITION_CONTROL, targetPosition = toLegAngle(outputs[t][leg2]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Body_Leg3' ,
            controlMode = p.POSITION_CONTROL, targetPosition = toLegAngle(outputs[t][leg3]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Body_Leg4',
            controlMode = p.POSITION_CONTROL, targetPosition = toLegAngle(outputs[t][leg4]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Leg1_wrist1',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][wrist1]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Leg2_wrist2',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][wrist2]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Leg3_wrist3',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][wrist3]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Leg4_wrist4',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][wrist4]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Leg1_wrist1',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][wrist1]),maxForce = 500)
    
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'wrist1_wheel11',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][w11]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'wrist1_wheel12',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][w12]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'wrist2_wheel21',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][w21]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'wrist2_wheel22',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][w22]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'wrist3_wheel31',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][w31]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'wrist3_wheel32',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][w32]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'wrist4_wheel41',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][w41]),maxForce = 500)
    prs.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'wrist4_wheel42',
            controlMode = p.POSITION_CONTROL, targetPosition = toAngle(outputs[t][w42]),maxForce = 500)
    
    p.stepSimulation()
    time.sleep(1/500)

p.disconnect()