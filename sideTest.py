import pybullet as p
import time
import matplotlib.pyplot as plt
import math
import pybullet_data
from pybullet_utils import urdfEditor as ed
from pybullet_utils import bullet_client as bc
import numpy as np
import matplotlib.animation as animation

import sys, getopt
import tkinter

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

p.setPhysicsEngineParameter(numSolverIterations=1000)

planeId = p.loadURDF("plane.urdf")

cubeStartPos = [0,0,0.005]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF("models/actuatorSide/actuatorSide.urdf",cubeStartPos,
    cubeStartOrientation,useFixedBase = 1)

numJoints = p.getNumJoints(robot)


test = p.loadURDF("models/window/testpoint.urdf",cubeStartPos,
    cubeStartOrientation,useFixedBase = 0)
test1 = p.loadURDF("models/window/testpoint.urdf",cubeStartPos,
    cubeStartOrientation,useFixedBase = 0)
joints = {}
links = {}
for i in range(numJoints):
    print(p.getJointInfo(robot,i))
    num = p.getJointInfo(robot,i)[0]
    jName = str(p.getJointInfo(robot,i)[1]).replace("'",'').replace("b",'')
    lName = str(p.getJointInfo(robot,i)[12]).replace("'",'').replace("b",'')
    joints[jName] = num
    links[lName] = num
    p.enableJointForceTorqueSensor(robot,i,1)

#p.createConstraint(test,-1,robot,links["middle_link"],p.JOINT_POINT2POINT,
#        [1,0,0],[0, 0,0],[-0.045, 0.07,0])

ori = [0,0,0]
pos = [0,0,0.005]
#p.createConstraint(robot,links["arm2"],robot,links["middle_link"],p.JOINT_POINT2POINT,
#        [1,0,0],[0, -0.085,0],[-0.045, 0.07,0])


p.createConstraint(robot,links["arm2"],robot,links["middle_link"],p.JOINT_POINT2POINT,
        [1,0,0],[0,-0.0875,0],[-0.048, 0.07,0])

#p.createConstraint(robot,links["arm4"],robot,links["arm2"],p.JOINT_POINT2POINT,
#        [1,0,0],[0.0035, -0.0425,0],[-0.0035,-0.045,0])

#p.createConstraint(test,-1,robot,links["arm2"],p.JOINT_POINT2POINT,
#        [1,0,0],[0, 0,0],[-0.00525, -0.045,0])
#p.createConstraint(robot,links["arm4"],test1,-1,p.JOINT_POINT2POINT,
#        [1,0,0],[0.00525, -0.045,0],[0,0,0])


jointPos = p.calculateInverseKinematics(robot,links['middle_link'], pos, 
        p.getQuaternionFromEuler(ori),solver=p.IK_DLS,maxNumIterations=2000)
while True:
    p.stepSimulation()
    time.sleep(1./2400.)
    for i in range(numJoints):
        p.setJointMotorControl2(bodyIndex=robot,jointIndex=i,controlMode=p.POSITION_CONTROL,
            targetPosition = jointPos[i], targetVelocity = 0, force = 5000, positionGain = 0.03, 
            velocityGain = 1)

