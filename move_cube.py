import pybullet as p
import pybullet_data

import numpy as np
import time

def toCube(kukaID, endEffector, cubePos, orn):
    endPos,_,_,_,_,_ = p.getLinkState(kukaID, endEffector)
    num_joints = p.getNumJoints(kukaID)
    pos = np.array(cubePos)+np.array([0,0,0.3])
    pos = np.array((np.array(endPos)+pos)/2)
    targetPositions = p.calculateInverseKinematics(kukaID, 6, pos, targetOrientation = p.getQuaternionFromEuler([3.14,0,0]))
    p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices=range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetPositions, forces = num_joints*[100])
    return p.calculateInverseKinematics(kukaID, endEffector, pos, targetOrientation = orn)
def stepSim(k):
    for j in range(k):
        p.stepSimulation()
        time.sleep(1./240.)
def closeGripper(gripperID):
    targetPosition = 0.03
    p.setJointMotorControl2(gripperID, 4, p.POSITION_CONTROL, targetPosition = targetPosition)
    p.setJointMotorControl2(gripperID, 6, p.POSITION_CONTROL, targetPosition = targetPosition)
    stepSim(200)

def openGripper(gripperID):
    targetPosition = 0
    p.setJointMotorControl2(gripperID, 4, p.POSITION_CONTROL, targetPosition = targetPosition)
    p.setJointMotorControl2(gripperID, 6, p.POSITION_CONTROL, targetPosition = targetPosition)
    stepSim(200)

def moveToPos(kukaID, endeffector, targetPos, orn, currentPos):
    targetPos = np.array(targetPos) + np.array([0,0,0.35])
    for i in range(3):
        pos = (np.array(targetPos) + np.array(currentPos))/2
        targetPositions = p.calculateInverseKinematics(kukaID, 6, pos, targetOrientation = p.getQuaternionFromEuler([3.14,0,0]))
        p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices=range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetPositions, forces = num_joints*[100])
        stepSim(200) 
        currentPos = pos


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeID = p.loadURDF("plane.urdf")
cubePos = [-0.5,0,0]
cubeOrn = p.getQuaternionFromEuler([0,0,0])
kukaPos = [0,0,0]
kukaOrn = p.getQuaternionFromEuler([0,0,0])
cubeID = p.loadURDF("cube_small.urdf", cubePos, cubeOrn)
kukaID = p.loadURDF("kuka_iiwa/model.urdf", kukaPos, kukaOrn)
jointPositions = [0,0,0,1.57,0,-1.04,0]
for jointIndex in range(p.getNumJoints(kukaID)):
    p.resetJointState(kukaID, jointIndex, jointPositions[jointIndex])
gripperID = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
jointPositions = [0.0,-0.011130, -0.206421,0.205143,-0.009999,0.0,-0.010055,0.0]
for jointIndex in range(p.getNumJoints(gripperID)):
    p.resetJointState(gripperID, jointIndex, jointPositions[jointIndex])
    p.setJointMotorControl2(gripperID, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex])
kuka_cid = p.createConstraint(kukaID, 6, gripperID, 0, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])
num_joints = p.getNumJoints(kukaID)
for j in range(3):
    cubePos, _ = p.getBasePositionAndOrientation(cubeID)
    toCube(kukaID, 6, cubePos, p.getQuaternionFromEuler([3.14,0,0]))
    stepSim(200)
contact = p.getContactPoints(cubeID, gripperID)

while not contact:
    closeGripper(gripperID)
    contact = p.getContactPoints(cubeID, gripperID)

moveToPos(kukaID, 6, [-0.5,0,0.5], p.getQuaternionFromEuler([3.14,0,0]), [-0.5,0,0])
moveToPos(kukaID, 6, [0,0.5,0.5], p.getQuaternionFromEuler([3.14,0,0]), [-0.5,0,0.5])
moveToPos(kukaID, 6, [0,0.5,0], p.getQuaternionFromEuler([3.14,0,0]), [0,0.5,0.5])
openGripper(gripperID)
moveToPos(kukaID, 6, [0,0.5,0.5], p.getQuaternionFromEuler([3.14,0,0]), [0,0.5,0])


while True:
    p.stepSimulation()
p.disconnect()
