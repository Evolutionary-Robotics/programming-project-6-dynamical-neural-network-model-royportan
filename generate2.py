import pyrosim.pyrosim as ps 
import numpy as np

#1 body

bodyL = 6
bodyW = 6
bodyH = 2

xb = 0
yb = 0
zb = bodyH/2
# 4 mainleg # wrist 
LegL = 4
LegW = 1.5
LegH = 1


# 4 wrist

WriL = 1.5
WriW = 1.5
WriH = 1.5

# 2 wheels each wrist
WheL = 1.5
WheW = 1.5
WheH = 0.5
# Wh

def Create_Robot_3():
    ps.Start_URDF("NewRobot.urdf")
    ps.Send_Cube(name = "Body",pos = [xb,yb,zb],size= [6,6,2])
    #Legs
    ps.Send_Cube(name = "Leg1",pos=[0,LegL/2,LegH/2],size=[4,1.5,1],rpy=[0,0,np.pi/2])
    ps.Send_Cube(name = "Leg2",pos=[-LegL/2,0,LegH/2],size=[4,1.5,1])
    ps.Send_Cube(name = "Leg3",pos=[0,-LegL/2,LegH/2],size=[4,1.5,1],rpy=[0,0,np.pi/2])
    ps.Send_Cube(name = "Leg4",pos=[LegL/2,0,LegH/2],size=[4,1.5,1])
    #Wrists
    ps.Send_Cube(name = "wrist1",pos=[0,0,-1.5/2],size=[1.5,1.5,1.5])
    ps.Send_Cube(name = "wrist2",pos=[0,0,-1.5/2],size=[1.5,1.5,1.5])
    ps.Send_Cube(name = "wrist3",pos=[0,0,-1.5/2],size=[1.5,1.5,1.5])
    ps.Send_Cube(name = "wrist4",pos=[0,0,-1.5/2],size=[1.5,1.5,1.5])
    #wheels
    ps.Send_Cube(name = "wheel11",pos=[0.5/2,0,0],size=[WheL,WheW,WheH],rpy=[0,-np.pi/2,0,])
    ps.Send_Cube(name = "wheel12",pos=[-0.5/2,0,0],size=[WheL,WheW,WheH],rpy=[0,-np.pi/2,0])
    ps.Send_Cube(name = "wheel21",pos=[0,0.5/2,0],size=[WheL,WheW,WheH],rpy=[-np.pi/2,0,0])
    ps.Send_Cube(name = "wheel22",pos=[0,-0.5/2,0],size=[WheL,WheW,WheH],rpy=[-np.pi/2,0,0])
    ps.Send_Cube(name = "wheel31",pos=[-0.5/2,0,0],size=[WheL,WheW,WheH],rpy=[0,-np.pi/2,0,])
    ps.Send_Cube(name = "wheel32",pos=[0.5/2,0,0],size=[WheL,WheW,WheH],rpy=[0,-np.pi/2,0,])
    ps.Send_Cube(name = "wheel41",pos=[0,-0.5/2,0],size=[WheL,WheW,WheH],rpy=[-np.pi/2,0,0])
    ps.Send_Cube(name = "wheel42",pos=[0,0.5/2,0],size=[WheL,WheW,WheH],rpy=[-np.pi/2,0,0])
    #ewqeq
    ps.Send_Joint(name="Body_Leg1",parent="Body",child="Leg1",type="revolute",position=[xb,yb+bodyW/2,zb],orientation=[-1,0,0])
    ps.Send_Joint(name="Body_Leg2",parent="Body",child="Leg2",type="revolute",position=[xb-bodyL/2,yb,zb],orientation =[0,-1,0])
    ps.Send_Joint(name="Body_Leg3",parent="Body",child="Leg3",type="revolute",position=[xb,yb-bodyW/2,zb],orientation =[1,0,0])
    ps.Send_Joint(name="Body_Leg4",parent="Body",child="Leg4",type="revolute",position=[xb+bodyL/2,yb,zb],orientation =[0,1,0])
    #Wrist Joint
    ps.Send_Joint(name="Leg1_wrist1",parent="Leg1",child="wrist1",type="revolute",position=[xb,LegL-1.5/2,0],orientation=[0,0,1])
    ps.Send_Joint(name="Leg2_wrist2",parent="Leg2",child="wrist2",type="revolute",position=[-(LegL*2-1.5)/2,yb,0],orientation =[0,0,1])
    ps.Send_Joint(name="Leg3_wrist3",parent="Leg3",child="wrist3",type="revolute",position=[xb,-LegL+1.5/2,0],orientation =[0,0,1])
    ps.Send_Joint(name="Leg4_wrist4",parent="Leg4",child="wrist4",type="revolute",position=[(LegL*2-1.5)/2,yb,0],orientation =[0,0,1])
    #wheel Joint
    ps.Send_Joint(name="wrist1_wheel11",parent="wrist1",child="wheel11",type="revolute",position=[1.5/2,0,-1.5],orientation=[1,0,0])
    ps.Send_Joint(name="wrist1_wheel12",parent="wrist1",child="wheel12",type="revolute",position=[-1.5/2,0,-1.5],orientation=[-1,0,0])
    ps.Send_Joint(name="wrist2_wheel21",parent="wrist2",child="wheel21",type="revolute",position=[0,1.5/2,-1.5],orientation=[0,1,0])
    ps.Send_Joint(name="wrist2_wheel22",parent="wrist2",child="wheel22",type="revolute",position=[0,-1.5/2,-1.5],orientation=[0,-1,0])
    ps.Send_Joint(name="wrist3_wheel31",parent="wrist3",child="wheel31",type="revolute",position=[-1.5/2,0,-1.5],orientation=[-1,0,0])
    ps.Send_Joint(name="wrist3_wheel32",parent="wrist3",child="wheel32",type="revolute",position=[1.5/2,0,-1.5],orientation=[1,0,0])
    ps.Send_Joint(name="wrist4_wheel41",parent="wrist4",child="wheel41",type="revolute",position=[0,-1.5/2,-1.5],orientation=[0,-1,0])
    ps.Send_Joint(name="wrist4_wheel42",parent="wrist4",child="wheel42",type="revolute",position=[0,1.5/2,-1.5],orientation=[0,1,0])


    ps.End()
Create_Robot_3()
