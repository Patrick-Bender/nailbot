import pybullet as p
import pybullet_data

import numpy as np
import time

def toCube(kukaID, endEffector, cubePos, orn):
    endPos,_,_,_,_,_ = p.getLinkState(kukaID, endEffector)
    num_joints = p.getNumJoints(kukaID)
    pos = np.array(cubePos)+np.array([0,0,0.3])
    pos = np.array((np.array(endPos)+pos)/2)
    targetPositions = p.calculateInverseKinematics(kukaID, 6, pos, targetOrientation = orn)
    p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices=range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetPositions, forces = num_joints*[100])
    return p.calculateInverseKinematics(kukaID, endEffector, pos, targetOrientation = orn)
def stepSim(k):
    for j in range(k):
        p.stepSimulation()
        time.sleep(1./240.)
def closeGripper(gripperID, targetID):
    contact = p.getContactPoints(gripperID, targetID)
    fourPos = p.getJointState(gripperID, 4)[0]
    sixPos = p.getJointState(gripperID, 6)[0]
    targetVelocity = 0.02
    while not contact:
        p.setJointMotorControl2(gripperID, 4, p.VELOCITY_CONTROL, targetVelocity = targetVelocity)
        p.setJointMotorControl2(gripperID, 6, p.VELOCITY_CONTROL, targetVelocity = targetVelocity)
        stepSim(50)
        contact = p.getContactPoints(gripperID, targetID)
    p.setJointMotorControl2(gripperID, 4, p.VELOCITY_CONTROL, targetVelocity = 0)
    p.setJointMotorControl2(gripperID, 6, p.VELOCITY_CONTROL, targetVelocity = 0)
    fourPos = p.getJointState(gripperID, 4)[0]
    sixPos = p.getJointState(gripperID, 6)[0]
    p.setJointMotorControl2(gripperID, 4, p.POSITION_CONTROL, targetPosition = fourPos + 0.005)
    p.setJointMotorControl2(gripperID, 6, p.POSITION_CONTROL, targetPosition = sixPos + 0.005)

def openGripper(gripperID):
    targetPosition = 0
    p.setJointMotorControl2(gripperID, 4, p.POSITION_CONTROL, targetPosition = targetPosition)
    p.setJointMotorControl2(gripperID, 6, p.POSITION_CONTROL, targetPosition = targetPosition)
    stepSim(200)

def moveToPos(kukaID, endeffector, targetPos, targetOrn, k = 1000):
    currentPos = p.getLinkState(kukaID, endeffector)[0]
    currentOrn = p.getLinkState(kukaID, endeffector)[1]
    targetPos = np.array(targetPos) + np.array([0,0,0.35])
    for i in range(k):
        pos = (np.array(targetPos) + np.array(currentPos))/2
        #converts the orientation to euler angles for the purpose of averaging then turns them back into quaternions
        orn = p.getQuaternionFromEuler((np.array(p.getEulerFromQuaternion(targetOrn)) + np.array(p.getEulerFromQuaternion(currentOrn)))/2)
        targetPositions = p.calculateInverseKinematics(kukaID, 6, pos, targetOrientation = orn)
        p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices=range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetPositions, forces = num_joints*[60])
        stepSim(2) 
        currentPos = pos
        currentOrn = orn
def activateNailgun(nailGunID, object1ID):
    #check to see if they're touching
    gunContact = p.getContactPoints(nailGunID, object1ID)
    if not len(gunContact):
        print('No Contact')
        return False
    #use ray tracing to determine locations of constrain
    rayPos = np.array(gunContact[0][6])+np.array([0,0,0.005])
    rayOrn = np.array(gunContact[0][7])
    rayDist = 0.1
    #rayOrn = np.array([0,0,1])
    #Check and see if the ray intersects both objects
    rayTest = p.rayTest(rayPos, rayPos + rayDist*rayOrn)[0]
    hitID = rayTest[0]
    hitPos = np.array(rayTest[3])
    hitNorm = np.array(rayTest[4])
    #Get centers of mass for the objects, and find the center of mass relative to the joint frame
    try:
        parentCOM = np.array(p.getBasePositionAndOrientation(object1ID)[0])
        childCOM = np.array(p.getBasePositionAndOrientation(hitID)[0])
        parentRefCOM = hitPos - parentCOM
        childRefCOM = hitPos - childCOM
    except:
        print("Nothing to nail the object to")
        return False
    print(parentRefCOM, [0,0,0.025])
    print(childRefCOM, [0,0,-0.025])
    #Add constraint
    constraintID = p.createConstraint(object1ID, -1, hitID, -1, p.JOINT_FIXED, [0,0,0], parentRefCOM, childRefCOM)
    #Add in failsafe in case the ray does not intersect anything
    print("Finished")
    print(p.getConstraintInfo(constraintID))
    return constraintID



physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeID = p.loadURDF("plane.urdf")
cubePos = [-0.5,0,0]
straightUp = p.getQuaternionFromEuler([0,0,0])
kukaPos = [0,0,0]
kuka2Pos = [0,1,0]
nailGunPos = [0,0.5,0.2]
roof1ID = p.loadURDF("urdfs/roof.urdf", [-0.5,0,0.1], straightUp)
roof2ID = p.loadURDF("urdfs/roof.urdf", [0.5,1,0.1], straightUp)
kukaID = p.loadURDF("kuka_iiwa/model.urdf", kukaPos, straightUp)
kuka2ID = p.loadURDF("kuka_iiwa/model.urdf", kuka2Pos, straightUp)
nailGunID = p.loadURDF('urdfs/nailgun.urdf', nailGunPos, straightUp)
jointPositions = [-0,0,0,1.57,0,-1.04,0]
for jointIndex in range(p.getNumJoints(kukaID)):
    p.resetJointState(kukaID, jointIndex, jointPositions[jointIndex])
    p.setJointMotorControl2(kukaID, jointIndex, p.POSITION_CONTROL, targetPosition = jointPositions[jointIndex])
jointPositions = [0,0,0,-1.57,0,1.04,0]
for jointIndex in range(p.getNumJoints(kukaID)):
    p.resetJointState(kuka2ID, jointIndex, jointPositions[jointIndex])
    p.setJointMotorControl2(kuka2ID, jointIndex, p.POSITION_CONTROL, targetPosition = jointPositions[jointIndex])
gripperID = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
gripper2ID = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
jointPositions = [0.0,-0.011130, -0.206421,0.205143,-0.009999,0.0,-0.010055,0.0]
for jointIndex in range(p.getNumJoints(gripperID)):
    p.resetJointState(gripperID, jointIndex, jointPositions[jointIndex])
    p.setJointMotorControl2(gripperID, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex])
    p.resetJointState(gripper2ID, jointIndex, jointPositions[jointIndex])
    p.setJointMotorControl2(gripper2ID, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex])
kuka_cid = p.createConstraint(kukaID, 6, gripperID, 0, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])
kuka2_cid = p.createConstraint(kuka2ID, 6, gripper2ID, 0, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])
stepSim(20)
num_joints = p.getNumJoints(kukaID)
#moveToPos(kukaID, 6, np.array([-0.5,0,0.1]) + np.array([0,0,0.2]), p.getQuaternionFromEuler([3.14,0,0]))
for j in range(3):
    roof1Pos, _ = p.getBasePositionAndOrientation(roof1ID)
    toCube(kukaID, 6, roof1Pos, p.getQuaternionFromEuler([3.14,0,0]))
    roof2Pos, _ = p.getBasePositionAndOrientation(roof2ID)
    toCube(kuka2ID, 6, roof2Pos, p.getQuaternionFromEuler([3.14,0,0]))
    stepSim(200)

closeGripper(gripperID, roof1ID)
closeGripper(gripper2ID, roof2ID)

moveToPos(kukaID, 6, [-0.5,0,0.5], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kuka2ID, 6, [0.5,1,0.5], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, 6, [0,0.5,0.5], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, 6, [-0.4,0.5,0.5], p.getQuaternionFromEuler([3.14/2,3.14/4,0]))
moveToPos(kukaID, 6, [-0.4,0.49,-0.1], p.getQuaternionFromEuler([0,3.14/2,0]), 5)
moveToPos(kuka2ID, 6, [0,0.5,0.5], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kuka2ID, 6, [0.2, 0.6, 0.3], p.getQuaternionFromEuler([3.14/2, -3.14/4,0]))
moveToPos(kuka2ID, 6, [0.4,0.51,0], p.getQuaternionFromEuler([0,-3.14/2,0]), 5)


activateNailgun(nailGunID, roof1ID)
#think about moving the kuka arms a little farther back
#something is fucked up about the orientation change, it's way to fast


while True:
    p.stepSimulation()
p.disconnect()
'''
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
'''

