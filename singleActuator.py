import pybullet as p
import time
import matplotlib.pyplot as plt
import math
import pybullet_data
from pybullet_utils import urdfEditor as ed
from pybullet_utils import bullet_client as bc
import numpy as np
import matplotlib.animation as animation

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

p.setPhysicsEngineParameter(numSolverIterations=1000)

planeId = p.loadURDF("plane.urdf")

startZ = 1.225
startZUP = 1.465
startZUPPER = startZ + startZUP

#starting locations for all actuators
cubeStartPos = [0,0,0.005]


cubeStartPos1 = [0,0,0]
cubeStartPos2 = [0,0,1.925]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

robot0 = p.loadURDF("models/actuatorBot/actuatorFixed.urdf",cubeStartPos,
    cubeStartOrientation,useFixedBase = 0)

tester = p.loadURDF("models/window/testpoint.urdf", [0,0.05,0.01],cubeStartOrientation,useFixedBase = 0)

robotArray = [robot0]
numJoints = p.getNumJoints(robot0)

joints = {}
links = {}

#Create constraints for actuators
for r in robotArray:
    for i in range(numJoints):
        #print(p.getJointInfo(r,i))
        num = p.getJointInfo(r,i)[0]
        print(p.getJointInfo(r,i))
        jName = str(p.getJointInfo(r,i)[1]).replace("'",'').replace("b",'')
        lName = str(p.getJointInfo(r,i)[12]).replace("'",'').replace("b",'')
        joints[jName] = num
        links[lName] = num
        p.enableJointForceTorqueSensor(r,i,1)
    #blue
    #p.createConstraint(r,links["armr4"],r,links["middle"],p.JOINT_POINT2POINT,
    #    [1,0,0],[0.0024, 0.04,0],[-0.06,-0.0275,-0.0045])
    #green
    p.createConstraint(r,links["arml2"],r,links["middle"],p.JOINT_POINT2POINT,
        [1,0,0],[-0.0024, 0.04,0],[0.06, -0.0325,-0.0015])

#p.createConstraint(robot,links["armr4"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, 0.04,0],[-0.06,-0.03,0])
#p.createConstraint(robot,links["arml3"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, -0.04,0],[0.06,-0.02,0])
#p.createConstraint(robot,links["arml4"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, 0.04,0],[0.06,-0.05,0])
#p.createConstraint(robot,links["armr1"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, -0.04,0],[-0.06,0,0])

while True:
    p.stepSimulation()
    time.sleep(1./2400.)
    qKey = ord('q')

    keys = p.getKeyboardEvents()

    #Start Delay to have the actuators settle in

    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
        break
            
p.disconnect()
