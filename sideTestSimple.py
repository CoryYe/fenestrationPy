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

p.setPhysicsEngineParameter(numSolverIterations=5000)

planeId = p.loadURDF("plane.urdf")

cubeStartPos = [0,0,0.005]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF("models/actuatorSide/actuatorSideSimple.urdf",cubeStartPos,
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


ori = [0,0,0]
pos = [0,0,0.05]
jointPos = p.calculateInverseKinematics(robot,links['middle_link'], pos, 
        p.getQuaternionFromEuler(ori),solver=p.IK_DLS,maxNumIterations=10000)

        
c = p.createConstraint(robot,links["arm2"],robot,links["middle_link"],p.JOINT_POINT2POINT,
        [1,0,0],[0,-0.0875,0],[0.048, -0.005,0])

#c1 = p.createConstraint(robot,links["arm2"],robot,links["middle_link"],p.JOINT_POINT2POINT,
#        [1,0,0],[0,-0.0875,0],[-0.048, -0.005,0])



while True:
    p.stepSimulation()

    time.sleep(1./300.)
    for i in range(numJoints):
        p.setJointMotorControl2(bodyIndex=robot,jointIndex=i,controlMode=p.POSITION_CONTROL,
            targetPosition = jointPos[i], targetVelocity = 0, force = 5000, positionGain = 0.03, 
            velocityGain = 1)
    print(joints)
    arml2Joint = joints["holder_to_arm2"]
    print(jointPos)
    print(arml2Joint)
    arml2Pos = p.getJointState(robot,arml2Joint)[0]
    print(arml2Pos)
    p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["holder_to_arm1"],
            controlMode=p.POSITION_CONTROL,targetPosition = arml2Pos, 
            targetVelocity = 0, force = 5000, positionGain = 0.03, 
            velocityGain = 1)

    

