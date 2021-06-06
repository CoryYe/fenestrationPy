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



def robotMove(pos,robot,ori,links):
    jointPos = p.calculateInverseKinematics(robot,links['EE'], pos, 
        p.getQuaternionFromEuler(ori),solver=p.IK_DLS,maxNumIterations=2000)
    arml1Joint = joints["carriage1_to_arml1"]
    arml1Pos = jointPos[arml1Joint]
    
    #graphing information data collection
    #print(jointPos)
    #carriagePos1.append(p.getJointState(robot,0)[0])
    #carriagePos2.append(p.getJointState(robot,7)[0])
    #print(p.getJointState(robot,1)[0])
    #middleH = 0.05* math.sin(0.785398-p.getJointState(robot,1)[0])
    #middleY = (p.getJointState(robot,0)[0]+p.getJointState(robot,0)[0])/2
    #middlePos.append(middleY)
    forces0 = p.getContactPoints(robot,window)
    #print(forces0)
    
    forceList = []
    for f in forces0:
        if f[3]==2:
            forceList.append(f[9])
    if len(forceList)>0:
        maxForce = max(forceList)
    
    for i in range(numJoints):
        p.setJointMotorControl2(bodyIndex=robot,jointIndex=i,controlMode=p.POSITION_CONTROL,
            targetPosition = jointPos[i], targetVelocity = 0, force = 5000, positionGain = 0.03, 
            velocityGain = 1)
        p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage1_to_arml3"],
            controlMode=p.POSITION_CONTROL,targetPosition = arml1Pos, 
            targetVelocity = 0, force = 5000, positionGain = 0.03, 
            velocityGain = 1)
        p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage2_to_arml4"],
            controlMode=p.POSITION_CONTROL,targetPosition = -arml1Pos, 
            targetVelocity = 0, force = 5000, positionGain = 0.03, 
            velocityGain = 1)
        p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage1_to_armr1"],
            controlMode=p.POSITION_CONTROL,targetPosition = arml1Pos, 
            targetVelocity = 0, force = 5000, positionGain = 0.03, 
            velocityGain = 1)
        p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage1_to_armr3"],
            controlMode=p.POSITION_CONTROL,targetPosition = arml1Pos, 
            targetVelocity = 0, force = 5000, positionGain = 0.03, 
            velocityGain = 1)
        p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage2_to_armr2"],
            controlMode=p.POSITION_CONTROL,targetPosition = -arml1Pos, 
            targetVelocity = 0, force = 5000, positionGain = 0.03, 
            velocityGain = 1)
    return forceList


    



root = tkinter.Tk()

delay = 0
startDelay = 0
argv = sys.argv[1:]

if len(sys.argv)>0:
    opts, args = getopt.getopt(argv,"d:s:")
    for opt, arg in opts:
        if opt in ['-d']:
            delay = arg
        if opt in ['-s']:
            startDelay = arg
    print("Delay")
    print(delay)
    print("Start Delay")
    print(startDelay)
        




physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

p.setPhysicsEngineParameter(numSolverIterations=1000)

planeId = p.loadURDF("plane.urdf")

startZ = 1.23
startZUP = 1.465
startZSIDE = .25

#starting locations for all actuators
cubeStartPos = [-0.45,0,startZ]
cubeStartPos3 = [0.45,0.0,startZ]
cubeStartPos4 = [-0.45,0.0,startZ + startZUP]
cubeStartPos5 = [0.45,0.0,startZ + startZUP]

cubeStartPos6 = [0.875,0, startZ + startZSIDE]
cubeStartPos7 = [-0.875,0, startZ + startZSIDE]
cubeStartPos8 = [0.875,0, startZ + startZUP- startZSIDE]
cubeStartPos9 = [-0.875,0, startZ + startZUP- startZSIDE]

cubeStartPos1 = [0,0,0]
cubeStartPos2 = [0,0,1.925]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

cubeStartOrientationSideR = p.getQuaternionFromEuler([1.57,1.57,-1.57])
cubeStartOrientationSideL = p.getQuaternionFromEuler([1.57,1.57,1.57])

robot0 = p.loadURDF("models/actuatorBot/actuatorFixed.urdf",cubeStartPos,
    cubeStartOrientation,useFixedBase = 1)
frame = p.loadURDF("models/window/frame.urdf", cubeStartPos1,cubeStartOrientation,useFixedBase = 1)
window = p.loadURDF("models/window/window_nosupport.urdf", cubeStartPos2,p.getQuaternionFromEuler([0,0,1.571]),useFixedBase = 0)


robot1 = p.loadURDF("models/actuatorBot/actuatorFixed.urdf",cubeStartPos3,
    cubeStartOrientation,useFixedBase = 1)
robot2  = p.loadURDF("models/actuatorBot/actuatorFixed.urdf",cubeStartPos4,
    cubeStartOrientation,useFixedBase = 0)
robot3 = p.loadURDF("models/actuatorBot/actuatorFixed.urdf",cubeStartPos5,
    cubeStartOrientation,useFixedBase = 0)
    
robot4 = p.loadURDF("models/actuatorSide/actuatorSideSimple.urdf",cubeStartPos6,
    cubeStartOrientationSideR,useFixedBase = 1)
robot5 = p.loadURDF("models/actuatorSide/actuatorSideSimple.urdf",cubeStartPos7,
    cubeStartOrientationSideL,useFixedBase = 1)

robot6 = p.loadURDF("models/actuatorSide/actuatorSideSimple.urdf",cubeStartPos8,
    cubeStartOrientationSideR,useFixedBase = 1)
robot7 = p.loadURDF("models/actuatorSide/actuatorSideSimple.urdf",cubeStartPos9,
    cubeStartOrientationSideL,useFixedBase = 1)

tester = p.loadURDF("models/window/testpoint.urdf", [0,0.05,0.01],cubeStartOrientation,useFixedBase = 0)

robotArray = [robot0,robot1,robot2,robot3]

robotBot = [robot0,robot1]
robotTop = [robot2,robot3]

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
    p.createConstraint(r,links["armr4"],r,links["middle"],p.JOINT_POINT2POINT,
        [1,0,0],[0.0024, 0.04,0],[-0.06,-0.0275,-0.0045])
    p.createConstraint(r,links["arml2"],r,links["middle"],p.JOINT_POINT2POINT,
        [1,0,0],[-0.0024, 0.04,0],[0.06, -0.0275,-0.0045])

#Create constraints for top actuator
p.createConstraint(window,-1,robot2,-1,p.JOINT_FIXED,[0,0,0],[0, 0,0],[0.45,0,-0.77],p.getQuaternionFromEuler([0,0,-1.57079]))
p.createConstraint(window,-1,robot3,-1,p.JOINT_FIXED,[0,0,0],[0, 0,0],[-0.450,0,-0.77],p.getQuaternionFromEuler([0,0,-1.57079]))

#p.createConstraint(robot,links["armr4"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, 0.04,0],[-0.06,-0.03,0])
#p.createConstraint(robot,links["arml3"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, -0.04,0],[0.06,-0.02,0])
#p.createConstraint(robot,links["arml4"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, 0.04,0],[0.06,-0.05,0])
#p.createConstraint(robot,links["armr1"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, -0.04,0],[-0.06,0,0])


maxforce = 5000

pos = [0,-1,0.06]
ori = [0,0,0]

t=0
carriagePos1 = []
carriagePos2 = []
middlePos = []

y = 0


#best 0.0525
z = 0.053
zU = 0.053
#y (-0.055 - 0.055)
#z (0.01 - 0.055)


startY = 0
n = 0
xs = []
ys = []

timeStep = 0
while True:
    p.stepSimulation()
    time.sleep(1./2400.)

    #startup Procedure
    
    
    
    #l 65295
    #u 65297
    #r 65296
    #d 65298
    qKey = ord('q')
    uKey = 65297
    lKey = 65295
    rKey = 65296
    dKey = 65298
    keys = p.getKeyboardEvents()

    #Start Delay to have the actuators settle in
    if timeStep>startDelay:
        if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
            print(keys)
        if uKey in keys and keys[uKey]&p.KEY_WAS_TRIGGERED:
            print("Up")
            z+=0.001
            zU-=0.001
        if lKey in keys and keys[lKey]&p.KEY_WAS_TRIGGERED:
            print("Left")
            y-=0.001
        if rKey in keys and keys[rKey]&p.KEY_WAS_TRIGGERED:
            print("Right")
            y+=0.001
        if dKey in keys and keys[dKey]&p.KEY_WAS_TRIGGERED:
            print("Down")
            z-=0.001
            zU+=0.001
            
    startZ = 1.225
    startZUP = 1.465
    pos = [0, startY, z+startZ]
    posU = [0, startY, zU+startZ + startZUP]

    forceList = robotMove(pos,robotArray[0],ori,links)
    robotMove(pos,robotArray[1],ori,links)
    robotMove(posU,robotArray[2],ori,links)
    robotMove(posU,robotArray[3],ori,links)

    #Move to Position

    #plt.axis([0, 100, 0, 100])

    windowPos = p.getBasePositionAndOrientation(window)[0]
    #print(windowPos)
    if windowPos[0]>0.1 or windowPos[2] <1.0:
        p.resetSimulation()
    
    
    if len(forceList)>0:
        maxForce = max(forceList)
    else:
        maxForce = 0
    ys.append(maxForce)
    print(maxForce)
    xs.append(z)
    plt.scatter(xs, ys, c="blue")

    #print(z, maxForce)
    #plt.draw()
    #plt.pause(0.001)
    n+=1
    timeStep+=1
    if n>200:
        n=0

    
p.disconnect()

sc = plt.scatter(xs,ys)
plt.show()

