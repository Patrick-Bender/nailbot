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

def inverseKinematics(bodyID, endEffector, pos, orn): 
    return p.calculateInverseKinematics(bodyID, endEffector, pos)

def getAngleInterpolation(bodyID, finalAngles, k=1000):
    #starting angles are the angles that you begin at
    #final angles are the angles you want to end at
    #target angles is the big array that has the interpolation of all the angles between the starting and final angles
    num_joints = p.getNumJoints(bodyID)
    jointStates= p.getJointStates(bodyID, range(num_joints))
    startingAngles = []
    for i in range(num_joints):
        startingAngles.append(p.getJointState(bodyID, i)[0])
        #startingAngles.append(jointStates[i][0])
    startingAngles = np.array(startingAngles)
    finalAngles = np.array(finalAngles)
    targetAngles = np.zeros((k, num_joints))
    targetAngles[0] = startingAngles
    for i in range(1,k+1):
        targetAngles[i-1] = startingAngles + finalAngles*(i/k)
    return targetAngles

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeID = p.loadURDF("plane.urdf")
kukaStartPos=[0,0,0]
kukaStartOrn = p.getQuaternionFromEuler([0,0,0])
kukaID = p.loadURDF('kuka_iiwa/model.urdf', kukaStartPos, kukaStartOrn)
#each i gives a different random pos and orn for the arm to get to
num_joints = p.getNumJoints(kukaID)
num_iterations = 1000
for q in range(5):
    finalPos, finalOrn = randomPosAndOrn()
    finalAngles = p.calculateInverseKinematics(kukaID, 6, finalPos, finalOrn)
    targetAngles = getAngleInterpolation(kukaID, finalAngles, num_iterations)
    for i in range(num_iterations):
        p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices = range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetAngles[i], forces = num_joints*[100])
        for j in range(1):
            p.stepSimulation()
            time.sleep(1./240.)
p.disconnect()
