import pybullet as p
import time
import matplotlib.pyplot as plt
import math
import pybullet_data
from pybullet_utils import urdfEditor as ed
from pybullet_utils import bullet_client as bc

p0 = bc.BulletClient(connection_mode=p.DIRECT)
physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p0.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

p.setPhysicsEngineParameter(numSolverIterations=1000)

p1 = bc.BulletClient(connection_mode=p.DIRECT)
p1.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p2 = bc.BulletClient(connection_mode=p.DIRECT)
p2.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally


planeId = p.loadURDF("plane.urdf")

startZ = 1.225
cubeStartPos = [-0.25,0,0.005+startZ]
cubeStartPos3 = [0.25,0.0,0.005+startZ]
cubeStartPos4 = [-0.25,0.0,0.005+startZ+1.45]
cubeStartPos5 = [0.25,0.0,0.005+startZ+1.45]

cubeStartPos1 = [0,0,0]
cubeStartPos2 = [0,0,1.9]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

window = p0.loadURDF("models/window/window.urdf", cubeStartPos2,p.getQuaternionFromEuler([0,0,1.571]),useFixedBase = 0)




robot2  = p2.loadURDF("models/actuatorBot/actuatorFixedTop1.urdf",cubeStartPos4,
    cubeStartOrientation,useFixedBase = 0)
robot3 = p1.loadURDF("models/actuatorBot/actuatorFixedTop.urdf",cubeStartPos5,
    cubeStartOrientation,useFixedBase = 0)



editor = ed.UrdfEditor()
editor1 = ed.UrdfEditor()
editor2 = ed.UrdfEditor()
editor.initializeFromBulletBody(robot3, p1._client)
editor2.initializeFromBulletBody(robot2, p2._client)
editor1.initializeFromBulletBody(window, p0._client)


parentLinkIndex = 0

jointPivotXYZInParent = [0, 0, 0]
jointPivotRPYInParent = [0, 0, 0]

jointPivotXYZInChild = [0.25,0.0,0.005+startZ+1.45]
jointPivotRPYInChild = [0, 0, 0]
newJ = editor1.joinUrdf(editor,parentLinkIndex, jointPivotXYZInParent, jointPivotRPYInParent,
                        jointPivotXYZInChild, jointPivotRPYInChild, p0._client, p1._client)
#newJ = editor1.joinUrdf(editor2,parentLinkIndex, jointPivotXYZInParent, jointPivotRPYInParent,
#                        jointPivotXYZInChild, jointPivotRPYInChild, p0._client, p2._client)

editor1.saveUrdf("combined.urdf")
