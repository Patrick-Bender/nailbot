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


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeID = p.loadURDF("plane.urdf")
kukaStartPos=[0,0,0]
kukaStartOrn = p.getQuaternionFromEuler([0,0,0])
kukaID = p.loadURDF('kuka_iiwa/model.urdf', kukaStartPos, kukaStartOrn)
#each i gives a different random pos and orn for the arm to get to
num_joints = p.getNumJoints(kukaID)
for i in range(10):
    pos, orn = randomPosAndOrn()
    targetPositions = inverseKinematics(kukaID,6,pos, orn)
    for j in range(500):
        p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices = range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetPositions, forces = num_joints*[100])
        p.stepSimulation()
        time.sleep(1./240.)
    actualPos, actualOrn, _,_,_,_ = p.getLinkState(kukaID, 6)
    pos = np.array(pos)
    orn = np.array(orn)
    actualPos = np.array(actualPos)
    acualOrn = np.array(actualOrn)
    print("Position Accuracy:")
    print(np.linalg.norm(pos-actualPos))
    print("Orientation Accuracy:")
    print(np.linalg.norm(orn-actualOrn))
p.disconnect()
