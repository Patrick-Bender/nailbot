import pybullet as p
import time
import pybullet_data
from random import uniform
from math import sin, cos
from scipy.spatial.transform import Rotation
import numpy as np
from math import ceil

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
    startingAngles = []
    for i in range(num_joints):
        startingAngles.append(p.getJointState(bodyID, i)[0])
    startingAngles = np.array(startingAngles)
    finalAngles = np.array(finalAngles)
    targetAngles = np.zeros((k, num_joints))
    targetAngles[0] = startingAngles
    for i in range(1,k+1):
        targetAngles[i-1] = startingAngles + (finalAngles-startingAngles)*(i/k)
    return targetAngles

def moveToPos(kukaID, finalPos, finalOrn, k = 500):
    num_joints = p.getNumJoints(kukaID)
    finalAngles = p.calculateInverseKinematics(kukaID, 6, finalPos, finalOrn)
    targetAngles = getAngleInterpolation(kukaID, finalAngles, k)
    for i in range(k):
        p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices = range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetAngles[i], forces = num_joints*[100])
        for j in range(ceil(500/k)):
            p.stepSimulation()
            time.sleep(1./240.)

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
jointPositions = [0,0,0,1.57,0,-1.04,0]
for jointIndex in range(p.getNumJoints(kukaID)):
    p.resetJointState(kukaID, jointIndex, jointPositions[jointIndex])

moveToPos(kukaID, [-0.5,0,0.5], p.getQuaternionFromEuler([3.14,0,3.14]))
moveToPos(kukaID, [-0.1,0.5,0.5], p.getQuaternionFromEuler([3.14,0,0]))


while True:
    p.stepSimulation()

for q in range(5):
    finalPos, finalOrn = randomPosAndOrn()
    finalAngles = p.calculateInverseKinematics(kukaID, 6, finalPos, finalOrn)
    targetAngles = getAngleInterpolation(kukaID, finalAngles, num_iterations)
    for i in range(num_iterations):
        p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices = range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetAngles[i], forces = num_joints*[100])
        for j in range(2):
            p.stepSimulation()
            time.sleep(1./240.)
p.disconnect()
