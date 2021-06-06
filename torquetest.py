import pybullet as p
import time
import matplotlib.pyplot as plt
import math
import pybullet_data

startZ = 1.225

def buildWorld():
    
    cubeStartPos = [0,0,0.005+startZ]
    cubeStartPos1 = [0,0,0]
    cubeStartPos2 = [0,0,1.95]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    robot = p.loadURDF("models/actuatorBot/actuatorFixed.urdf",cubeStartPos,
        cubeStartOrientation,useFixedBase = 1)
    frame = p.loadURDF("models/window/frame.urdf", cubeStartPos1,cubeStartOrientation,useFixedBase = 1)
    window = p.loadURDF("models/window/window.urdf", cubeStartPos2,p.getQuaternionFromEuler([0,0,1.571]),useFixedBase = 0)
    return robot



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

p.setPhysicsEngineParameter(numSolverIterations=10000)


planeId = p.loadURDF("plane.urdf")

robot = buildWorld()

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


maxForce = 500

pos = [0,-1,0.06]
ori = [0,0,0]

t=0
carriagePos1 = []
carriagePos2 = []
middlePos = []
torques = []
ts = []
y = -0.055



#while y<=0.055:
#    z= 0.01
#    print(y)
#    while z<=0.065:
maxForce = 0
maxForceAch = []


while maxForce<50:
    t = 0
    zH = 0
    maxForce+=1
    print(maxForce)
    zH=0
    while t<10 and zH<0.055:
        i+=1
        t = t+0.01
        p.stepSimulation()
        time.sleep(1./2400.)
        #print("0")
        #print(p.getJointState(robot,0))
        #print("1")
        #print(p.getJointState(robot,joints["middle_to_EE"]))
        #maxForce+=0.1
        for j in range(1):
            #z = 0.01+0.05*abs(math.sin(0.5*t))
            #y = -0.05 * math.cos(t)
            #y = 0  
            z = 0.05
            pos = [0, 0, z+startZ]
            #print(pos)
            jointPos = p.calculateInverseKinematics(robot,links['EE'], pos, 
                p.getQuaternionFromEuler(ori),solver=p.IK_DLS,maxNumIterations=5000)
            arml1Joint = joints["carriage1_to_arml1"]
            arml1Pos = jointPos[arml1Joint]
            #print(jointPos)
            carriagePos1.append(p.getJointState(robot,0)[0])
            carriagePos2.append(p.getJointState(robot,7)[0])
            #print(p.getJointState(robot,1))
            middleH = 0.05* math.sin(0.785398-p.getJointState(robot,1)[0])
            zH = middleH       
            print(zH) 
            #middleY = (p.getJointState(robot,0)[0]+p.getJointState(robot,7)[0])/2
            #torques.append(p.getJointState(robot,1)[3])
            middlePos.append(middleH)
            ts.append(t)
            for i in range(numJoints):
                p.setJointMotorControl2(bodyIndex=robot,jointIndex=i,controlMode=p.TORQUE_CONTROL,
                    targetPosition = jointPos[i], targetVelocity = 0, force = maxForce,
                    positionGain = 0.03, velocityGain = 1)
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
        if zH>0.055:
            maxForceAch.append(z)
            torques.append(maxForce)
        else:
            maxForceAch.append(0)
            torques.append(maxForce)
    p.resetSimulation()
    robot = buildWorld()
    print("opo")

p.disconnect()
#sc = plt.scatter(ts,middlePos,c=torques,cmap = 'plasma')
sc = plt.plot(torques, maxForceAch)
plt.xlabel('Torques')
plt.ylabel('maxZ')
#plt.colorbar(sc)
plt.show()

