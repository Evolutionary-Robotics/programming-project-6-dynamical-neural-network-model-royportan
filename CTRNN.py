
import numpy as np

def sigmoid(x):
    return 1/(1+np.exp(-x))
class CTRNN:
    def __init__(self, neuronNum, stepSize):
        self.neuronNum = neuronNum
        self.State = np.zeros((neuronNum))
        self.W = np.random.uniform(-10,10,size=(neuronNum,neuronNum))
        self.Bias = np.random.uniform(-10,10,size=(self.neuronNum)) 
        self.Input = np.zeros(neuronNum)
        self.Output = np.zeros(neuronNum)
        self.timeCons =  np.ones(neuronNum)
        self.cons = 1/self.timeCons
        self.stepSize  = stepSize
        self.DS = np.zeros(neuronNum)
    def setParams(self, W,Bias,timeCons):
        self.W = W
        self.Bias = Bias
        self.timeCons = timeCons
        self.cons = 1/self.timeCons
    def initState(self, State):
        self.Input = np.zeros(self.neuronNum)
        self.State = State
        self.Output = sigmoid(self.State + self.Bias)
    def getSignal(self):
        pass
    def Step(self):
        sDot = self.cons*(-self.State+self.W.dot(self.Output)+self.Input)
        self.State += sDot*self.stepSize
        self.Output = sigmoid(self.State + self.Bias)
    def save(self, fileName):
        np.savez(fileName, neuronNum = self.neuronNum, W = self.W, Bias = self.Bias, timeCons = self.timeCons)
    def load(self, fileName):
        params = np.load(fileName)
        self.Bias = params['Bias']
        self.neuronNum = params['neuronNum']
        self.W = params['W']
        self.timeCons = params['timeCons']
        self.cons = 1/self.timeCons