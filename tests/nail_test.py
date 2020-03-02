import pybullet as p
import time
import pybullet_data
from random import uniform
from math import sin, cos
from scipy.spatial.transform import Rotation
import numpy as np

def randomPosAndOrn():
    r = uniform(0.1, 0.8)
    theta = uniform(-3.14, 3.14)
    phi = uniform(0,3.14/2)
    pos = [r*sin(phi)*cos(theta), r*sin(phi)*sin(theta), r*cos(phi)]
    orn = Rotation.random().as_quat()
    return pos, orn

def inverseKinematics(bodyID, endEffector, pos): 
    return p.calculateInverseKinematics(bodyID, endEffector, pos)

def activateNailgun(nailGunID, object1ID, object2ID):
    #check to see if they're touching
    gunContact = p.getContactPoints(nailGunID, object1ID)
    objectContact = p.getContactPoints(object1ID, object2ID)
    if not len(gunContact) or not len(objectContact):
        print('No Contact')
        return False
    #use ray tracing to determine locations of constrain
    rayPos = np.array(gunContact[0][6])
    #may need to reverse orientation to get it in the right direction
    rayOrn = np.array(gunContact[0][7])
    rayDist = 0.1
    #Check and see if the ray intersects both objects
    rayTest = p.rayTest(rayPos, rayPos + rayDist*rayOrn)
    hitID = raytest[0]
    hitPos = rayTest[3]
    hitNorm = rayTest[4]
    #Add constraint
    #Add in failsafe in case the ray does not intersect anything
    return True

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeID = p.loadURDF("plane.urdf")
kukaStartPos=[0,0,0]
kukaStartOrn = p.getQuaternionFromEuler([0,0,0])
kukaID = p.loadURDF('kuka_iiwa/model.urdf', kukaStartPos, kukaStartOrn)
cubeID = p.loadURDF('cube_small.urdf', [0,0.5,0.2], p.getQuaternionFromEuler([0,0,0]))
cube2ID = p.loadURDF('cube_small.urdf', [0,0.5,0.25], p.getQuaternionFromEuler([0,0,0]))
nailgunID = p.loadURDF('nailgun.urdf', [0,0.5,0.1], p.getQuaternionFromEuler([0,0,0]))
#Touches the cube to see if it's nailed together or not
for i in range(20):
    p.stepSimulation()
    time.sleep(1./240.)
activateNailgun(nailgunID, cubeID, cube2ID)

num_joints = p.getNumJoints(kukaID)
for i in range(4):
    
    activateNailgun(nailgunID, cubeID, cube2ID)
    pos, _ = p.getBasePositionAndOrientation(cubeID)
    targetPositions = inverseKinematics(kukaID,6,pos)
    for j in range(500):
        p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices = range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetPositions, forces = num_joints*[100])
        p.stepSimulation()
        time.sleep(1./240.)
    contact = p.getContactPoints(cubeID, kukaID)
    if(len(contact)==1):
        print(contact[0])
    #for i in range(len(contact)):
        #print(contact[i])

p.disconnect()
