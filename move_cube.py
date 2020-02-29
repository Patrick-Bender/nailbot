import pybullet as p
import pybullet_data

import numpy as np
import time

def toCube(kukaID, endEffector, cubePos, orn):
    pos = np.array(cubePos)+np.array([0,0,0.1])
    return p.calculateInverseKinematics(kukaID, endEffector, pos)


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
kuka_cid = p.createConstraint(kukaID, 6, gripperID, 0, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])
num_joints = p.getNumJoints(kukaID)
for j in range(5):
    cubePos, _ = p.getBasePositionAndOrientation(cubeID)
    targetPositions = toCube(kukaID, 6, cubePos, p.getQuaternionFromEuler([-1,0,0]))
    print(targetPositions, cubePos)
    p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices=range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetPositions, forces = num_joints*[100])
    for i in range(200):
        p.stepSimulation()
        time.sleep(1./240.)
p.disconnect()
