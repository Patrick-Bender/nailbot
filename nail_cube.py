import pybullet as p
import pybullet_data

import numpy as np
import time
from math import ceil

def stepSim(k):
    for j in range(k):
        p.stepSimulation()
        #time.sleep(1./240.)
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
    pos = max(fourPos, sixPos)
    p.setJointMotorControl2(gripperID, 4, p.POSITION_CONTROL, targetPosition = pos + 0.0125, force = 100)
    p.setJointMotorControl2(gripperID, 6, p.POSITION_CONTROL, targetPosition = pos + 0.0125, force = 100)

def openGripper(gripperID):
    targetPosition = 0
    p.setJointMotorControl2(gripperID, 4, p.POSITION_CONTROL, targetPosition = targetPosition)
    p.setJointMotorControl2(gripperID, 6, p.POSITION_CONTROL, targetPosition = targetPosition)
    stepSim(200)

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def activateNailgun(nailGunID, object1ID):
    #check to see if they're touching
    gunContact = p.getContactPoints(nailGunID, object1ID)
    if not len(gunContact):
        print('No Contact')
        return False
    #use ray tracing to determine locations of constrain
    rayPos = np.array(gunContact[0][6])+np.array([0,0,0.05])
    rayOrn = np.array(gunContact[0][7])
    rayDist = 0.25
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
    #these are correct, need to figure out a way to do this generically
    parentRefCOM = [-0.0125, -0.0375, 0]
    childRefCOM = [0,0.05,0]
    childFrameOrn = p.getQuaternionFromEuler([0,3.14,-3.14/2])
    #Add constrain
    constraintID = p.createConstraint(object1ID, -1, hitID, -1, p.JOINT_FIXED, hitPos, parentRefCOM, childRefCOM, childFrameOrientation = childFrameOrn)
    print("Finished")
    print(p.getConstraintInfo(constraintID))
    return constraintID

def getAngleInterpolation(bodyID, finalAngles, k=1000):
    #starting angles are the angles that you begin at
    #final angles are the angles you want to end at
    #target angles is the big array that has the interpolation of all the angles between the starting and final angles
    num_joints = p.getNumJoints(bodyID)
    jointStates= p.getJointStates(bodyID, range(num_joints))
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
        for j in range(ceil(1000/k)):
            p.stepSimulation()
            #time.sleep(1./240.)

#setup
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeID = p.loadURDF("plane.urdf")
cubePos = [-0.5,0,0]
straightUp = p.getQuaternionFromEuler([0,0,0])
kukaPos = [0,0,0]
kuka2Pos = [0,1,0]
nailGunPos = [0,0.5,0.2]
roof1ID = p.loadURDF("urdfs/roof.urdf", [-0.6,0,0.15], straightUp)
roof2ID = p.loadURDF("urdfs/roof.urdf", [0.6,1,0.15], straightUp)
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
roof1Pos, _ = p.getBasePositionAndOrientation(roof1ID)
roof2Pos, _ = p.getBasePositionAndOrientation(roof2ID)

#move to grasp roofs
moveToPos(kukaID, np.array(roof1Pos) + np.array([0,0,0.35]), p.getQuaternionFromEuler([3.14,0,3.14]),50)
moveToPos(kuka2ID, np.array(roof2Pos) + np.array([0,0,0.35]), p.getQuaternionFromEuler([3.14,0,3.14]),50)

#close grippers
closeGripper(gripperID, roof1ID)
closeGripper(gripper2ID, roof2ID)

#move up
moveToPos(kukaID, [-0.5,0,0.7], p.getQuaternionFromEuler([3.14,0,3.14]))
moveToPos(kuka2ID, [0.5,1,0.7], p.getQuaternionFromEuler([3.14,0,3.14]))

#move closer to nailgun and orient planks
moveToPos(kuka2ID, [0.35,0.5,0.7], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))
moveToPos(kukaID, [-0.5,0.3,0.3], p.getQuaternionFromEuler([3.14,-3.14/2,0]))
moveToPos(kukaID, [-0.35,0.45,0.5], p.getQuaternionFromEuler([3.14,-3.14/2,0]))


#lower plank
moveToPos(kukaID, [-0.35,0.45,0.3125], p.getQuaternionFromEuler([3.14,-3.14/2,0]))
moveToPos(kuka2ID, [0.35,0.5,0.3625], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))

activateNailgun(nailGunID, roof1ID)

openGripper(gripper2ID)
#move kuka 2 out of the way
moveToPos(kuka2ID, [0.4,0.5,0.5], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))
moveToPos(kuka2ID, [0.5,1,0.7], p.getQuaternionFromEuler([3.14,0,3.14]))

#move nailed thing front

moveToPos(kukaID, [0.4,0.3,0.5], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, [0.5,0.2,0.3], p.getQuaternionFromEuler([3.14,0,0]))
openGripper(gripperID)
moveToPos(kukaID, [0.5,0.2,0.7], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, [-0.5,0.3,0.3], p.getQuaternionFromEuler([3.14,-3.14/2,0]))


while True:
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()

