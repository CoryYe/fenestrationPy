import pybullet as p
import time
import matplotlib.pyplot as plt
import math
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

p.setPhysicsEngineParameter(numSolverIterations=10000)


planeId = p.loadURDF("plane.urdf")

startZ = 1.225
cubeStartPos = [0,0,0.005+startZ]
cubeStartPos1 = [0,0,0]
cubeStartPos2 = [0,0,1.95]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF("models/actuatorBot/actuatorFixed.urdf",cubeStartPos,
    cubeStartOrientation,useFixedBase = 1)
frame = p.loadURDF("models/window/frame.urdf", cubeStartPos1,cubeStartOrientation,useFixedBase = 1)
window = p.loadURDF("models/window/window.urdf", cubeStartPos2,p.getQuaternionFromEuler([0,0,1.571]),useFixedBase = 0)


numJoints = p.getNumJoints(robot)
print(numJoints)

joints = {}
links = {}
for i in range(numJoints):
    num = p.getJointInfo(robot,i)[0]
    jName = str(p.getJointInfo(robot,i)[1]).replace("'",'').replace("b",'')
    lName = str(p.getJointInfo(robot,i)[12]).replace("'",'').replace("b",'')
    joints[jName] = num
    links[lName] = num
    p.enableJointForceTorqueSensor(robot,i,1)

print(joints)
print(links)

p.createConstraint(robot,links["armr2"],robot,links["middle"],p.JOINT_POINT2POINT,
    [0,0,0],[0, 0.04,0],[-0.06,-0.05,0])
p.createConstraint(robot,links["arml2"],robot,links["middle"],p.JOINT_POINT2POINT,
    [0,0,0],[0, 0.04,0],[0.06,-0.03,0])

#p.createConstraint(robot,links["armr3"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, -0.04,0],[-0.06,-0.02,0])
#p.createConstraint(robot,links["armr4"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, 0.04,0],[-0.06,-0.03,0])
#p.createConstraint(robot,links["arml3"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, -0.04,0],[0.06,-0.02,0])
#p.createConstraint(robot,links["arml4"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, 0.04,0],[0.06,-0.05,0])
#p.createConstraint(robot,links["armr1"],robot,links["middle"],p.JOINT_POINT2POINT,
#    [0,0,0],[0, -0.04,0],[-0.06,0,0])


maxForce = 500

pos = [0,-1,0.06]
ori = [0,0,0]

t=0
carriagePos1 = []
carriagePos2 = []
middlePos = []

y = -0.055



while y<=0.055:
    z= 0.01
    print(y)
    while z<=0.065:
#while t<100:
    #i+=1
    #print(t)
    #t = t+0.01
        p.stepSimulation()
        time.sleep(1./2400.)
    #print("0")
    #print(p.getJointState(robot,0))
    #print("1")
    #print(p.getJointState(robot,joints["middle_to_EE"]))

    #for :

        #z = 0.01+0.05*abs(math.sin(t))
        #y = -0.05 * math.cos(t)
        #y = 0  
        pos = [0, y, z+startZ]
        #print(pos)
        jointPos = p.calculateInverseKinematics(robot,links['EE'], pos, 
            p.getQuaternionFromEuler(ori),solver=p.IK_DLS,maxNumIterations=5000)
        arml1Joint = joints["carriage1_to_arml1"]
        arml1Pos = jointPos[arml1Joint]
        #print(jointPos)
        carriagePos1.append(p.getJointState(robot,0)[0])
        carriagePos2.append(p.getJointState(robot,7)[0])
        #print(p.getJointState(robot,1)[0])
        #middleH = 0.05* math.sin(0.785398-p.getJointState(robot,1)[0])
        middleY = (p.getJointState(robot,0)[0]+p.getJointState(robot,0)[0])/2
        middlePos.append(middleY)

        for i in range(numJoints):
            p.setJointMotorControl2(bodyIndex=robot,jointIndex=i,controlMode=p.POSITION_CONTROL,
                targetPosition = jointPos[i], targetVelocity = 0, force = 500, positionGain = 0.03, 
                velocityGain = 1)
            p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage1_to_arml3"],
                controlMode=p.POSITION_CONTROL,targetPosition = arml1Pos, 
                targetVelocity = 0, force = 500, positionGain = 0.03, 
                velocityGain = 1)
            p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage2_to_arml4"],
                controlMode=p.POSITION_CONTROL,targetPosition = -arml1Pos, 
                targetVelocity = 0, force = 500, positionGain = 0.03, 
                velocityGain = 1)
            p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage1_to_armr1"],
                controlMode=p.POSITION_CONTROL,targetPosition = arml1Pos, 
                targetVelocity = 0, force = 500, positionGain = 0.03, 
                velocityGain = 1)
            p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage1_to_armr3"],
                controlMode=p.POSITION_CONTROL,targetPosition = arml1Pos, 
                targetVelocity = 0, force = 500, positionGain = 0.03, 
                velocityGain = 1)
            p.setJointMotorControl2(bodyIndex=robot,jointIndex=joints["carriage2_to_armr4"],
                controlMode=p.POSITION_CONTROL,targetPosition = -arml1Pos, 
                targetVelocity = 0, force = 500, positionGain = 0.03, 
                velocityGain = 1)
        z+=0.0001
    y+=0.001

p.disconnect()
sc = plt.scatter(carriagePos1,carriagePos2,c=middlePos,cmap = 'plasma')
plt.xlabel('Carriage1 Position')
plt.ylabel('Carriage2 Position')
plt.colorbar(sc)
plt.show()

