import numpy as np
import CTRNN
import matplotlib.pyplot as plt
neuronNum = 20
Neuron1 = CTRNN.CTRNN(neuronNum, 0.01)

duration = 10000
time = np.arange(0.0,duration,0.01)
outputs = np.zeros((len(time),neuronNum))
states = np.zeros((len(time),neuronNum))
step = 0
for t in range(len(time)):
    Neuron1.Step()
    states[t] = Neuron1.State
    outputs[t] = Neuron1.Output


# plt.plot(time,outputs)
# plt.xlabel("Time")
# plt.ylabel("Outputs")
# plt.title(" Output plot over time ")
# plt.show()

# # Plot activity
# plt.plot(time,states)
# plt.xlabel("Time")
# plt.ylabel("States")
# plt.title(" State plot over time ")
# plt.show()

Neuron1.save("neuron7")