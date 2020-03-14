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
    p.setJointMotorControl2(gripperID, 4, p.POSITION_CONTROL, targetPosition = pos + 0.0125, force = 500)
    p.setJointMotorControl2(gripperID, 6, p.POSITION_CONTROL, targetPosition = pos + 0.0125, force = 500)

def openGripper(gripperID):
    targetPosition = 0
    p.setJointMotorControl2(gripperID, 4, p.POSITION_CONTROL, targetPosition = targetPosition)
    p.setJointMotorControl2(gripperID, 6, p.POSITION_CONTROL, targetPosition = targetPosition)
    stepSim(200)


def activateNailgun(nailGunID, object1ID, parentRefCOM, childRefCOM, childFrameOrn=[0,0,0], hitID = []):
    #check to see if they're touching
    gunContact = p.getContactPoints(nailGunID, object1ID)
    if not len(gunContact):
        print('No Contact')
        return False
    #get rid of the nasty contact points-just use COM of nailgun + some z component to get pos, and just use straight up as orn
    #use ray tracing to determine locations of constrain
    if hitID == []:
        gunPos, _ = p.getBasePositionAndOrientation(nailGunID)
        rayPos = np.array(gunPos)+np.array([0,0,0.165])
        rayOrn = np.array([0,0,1])
        rayDist = 0.25
        #rayOrn = np.array([0,0,1])
        #Check and see if the ray intersects both objects
        rayTest = p.rayTest(rayPos, rayPos + rayDist*rayOrn)[0]
        hitID = rayTest[0]
        hitPos = np.array(rayTest[3])
    else:
        hitPos,_ = p.getBasePositionAndOrientation(hitID)
    #Add constrain
    constraintID = p.createConstraint(object1ID, -1, hitID, -1, p.JOINT_FIXED, hitPos, parentRefCOM, childRefCOM, childFrameOrientation = childFrameOrn)
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

def moveToPos(kukaID, finalPos, finalOrn, k = 500, s = 0):
    if s == 0:
        s = ceil(1000/k)
    num_joints = p.getNumJoints(kukaID)
    finalAngles = p.calculateInverseKinematics(kukaID, 6, finalPos, finalOrn)
    targetAngles = getAngleInterpolation(kukaID, finalAngles, k)
    for i in range(k):
        p.setJointMotorControlArray(bodyUniqueId = kukaID, jointIndices = range(num_joints), controlMode = p.POSITION_CONTROL, targetPositions = targetAngles[i], forces = num_joints*[100])
        for j in range(s):
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
roof1ID = p.loadURDF("urdfs/roof1.urdf", [-0.6,0,0.15], straightUp)
roof2ID = p.loadURDF("urdfs/roof2.urdf", [0.6,1,0.15], straightUp)
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
roof1Pos, roof1Orn = p.getBasePositionAndOrientation(roof1ID)
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
moveToPos(kuka2ID, [0.325,0.5,0.35], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))

activateNailgun(nailGunID, roof1ID, [-0.0125,-0.0375,0], [0,0.05,0], p.getQuaternionFromEuler([0,3.14,-3.14/2]))

openGripper(gripper2ID)
#move kuka 2 out of the way
moveToPos(kuka2ID, [0.4,0.5,0.5], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))
moveToPos(kuka2ID, [0.5,1,0.7], p.getQuaternionFromEuler([3.14,0,3.14]))

#move roof out of the way

moveToPos(kukaID, [-0.4,0,0.8], p.getQuaternionFromEuler([3.14,-3.14/2,0]))
moveToPos(kukaID, [-0.5,-0.4,0.6], p.getQuaternionFromEuler([3.14,0,0]))
openGripper(gripperID)
moveToPos(kukaID, [-0.5,-0.4,0.7], p.getQuaternionFromEuler([3.14,0,0]))









#load side and bottom
side1Pos = [0.5,1,0.1]
side1ID = p.loadURDF("urdfs/side.urdf", side1Pos, straightUp)
bottomPos = [-0.5, 0, 0.15]
bottomID = p.loadURDF("urdfs/bottom.urdf", bottomPos, straightUp)

#grasp both side and bottom

moveToPos(kukaID, [-0.5,0,0.8], p.getQuaternionFromEuler([3.14,0,3.14]))
moveToPos(kukaID, np.array(bottomPos) + np.array([0.015,0,0.3]), p.getQuaternionFromEuler([3.14,0,3.14]))

moveToPos(kuka2ID, [0.5,1,0.7], p.getQuaternionFromEuler([3.14,0,3.14]))
moveToPos(kuka2ID, np.array(side1Pos) + np.array([0,0,0.35]), p.getQuaternionFromEuler([3.14,0,3.14]))
closeGripper(gripper2ID, side1ID)

#move side and bottom into position

moveToPos(kuka2ID, [0.5,1,0.7], p.getQuaternionFromEuler([-3.14/2,0,3.14]))
moveToPos(kuka2ID, [0.325,0.5,0.7], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))

closeGripper(gripperID, bottomID)
moveToPos(kukaID, [-0.5,0,0.7], p.getQuaternionFromEuler([3.14,0,3.14]))
moveToPos(kukaID, [-0.5,0.3,0.3], p.getQuaternionFromEuler([3.14,-3.14/2,0]))
moveToPos(kukaID, [-0.35,0.45,0.5], p.getQuaternionFromEuler([3.14,-3.14/2,0]))

#lower the planks and activate nailgun

moveToPos(kukaID, [-0.35,0.45,0.31], p.getQuaternionFromEuler([3.14,-3.14/2,0]))
moveToPos(kuka2ID, [0.325,0.5,0.4], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))

activateNailgun(nailGunID, bottomID, [-0.025,-0.05,0], [0,0.05,0], p.getQuaternionFromEuler([0,3.14,-3.14/2]), hitID = side1ID)
openGripper(gripperID)
moveToPos(kukaID, [-0.6,0.45,0.3125], p.getQuaternionFromEuler([3.14,-3.14/2,0]))
moveToPos(kuka2ID, [0.325,0.5,0.5], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))
moveToPos(kukaID, [-0.5,0,0.7], p.getQuaternionFromEuler([3.14,0,0]))








#load and grasp second side, reposition bottom+first side, and nail it together
#load side 2
side2Pos = [-0.5, 0.1, 0.15]
side2ID = p.loadURDF("urdfs/side.urdf", side2Pos, straightUp)

#move kuka1 to pick up second side of the roof
#moveToPos(kukaID, np.array(side2Pos) + np.array([0.0861, -0.005,0.35]) , p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, np.array(side2Pos) + np.array([0,0,0.5]) , p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, np.array(side2Pos) + np.array([0,0,0.3]) , p.getQuaternionFromEuler([3.14,0,0]))
closeGripper(gripperID, side2ID)
moveToPos(kukaID, [-0.5, 0, 0.7], p.getQuaternionFromEuler([3.14, 0, 0]))


#drop bottom + side1 to reposition it
'''
moveToPos(kuka2ID, [0.5,0.6,0.7], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))
moveToPos(kuka2ID, [0.5,0.6,0.6], p.getQuaternionFromEuler([3.14,0,0]))
openGripper(gripper2ID)
stepSim(200)
side1Pos, _ = p.getBasePositionAndOrientation(side1ID)
moveToPos(kuka2ID, np.array(side1Pos) + np.array([0,-0.01,0.35]) , p.getQuaternionFromEuler([3.14,0,0]))
closeGripper(gripper2ID, side1ID)
moveToPos(kuka2ID, [0.5,0.6,0.6], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kuka2ID, [0.325,0.5,0.5], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))
'''

#move kuka2 then kuka1 into position, then activate nailgun
moveToPos(kuka2ID, [0.325,0.55,0.4], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))
moveToPos(kuka2ID, [0.325,0.57,0.4], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))


moveToPos(kukaID, [-0.4,0.5,0.7], p.getQuaternionFromEuler([3.14/2*3,0,-3.14/2]))
moveToPos(kukaID, [-0.35,0.5,0.5], p.getQuaternionFromEuler([3.14/2*3,0,-3.14/2]))
moveToPos(kukaID, [-0.35,0.5,0.4], p.getQuaternionFromEuler([3.14/2*3,0,-3.14/2]))

activateNailgun(nailGunID, bottomID, [0.025,0.05,0], [-0.0125,-0.1,0], p.getQuaternionFromEuler([0,0,-3.14/2]))
openGripper(gripper2ID)






#load front-creating constraint
frontPos = [0.5, 1, 0.05]
frontID = p.loadURDF("urdfs/front.urdf", frontPos, straightUp)
frontAtticID = p.loadURDF("urdfs/attic.urdf", [0.5, 1, 0.5], straightUp)
frontConstraintID = p.createConstraint(frontID, -1, frontAtticID, -1, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0], childFrameOrientation = p.getQuaternionFromEuler([3.14/4,0,0]))

#move kuka 2 and grasp the front

moveToPos(kuka2ID, [0.45,0.57,0.6], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))
moveToPos(kuka2ID, np.array(frontPos) + np.array([-0.0625,0,0.5]), p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kuka2ID, np.array(frontPos) + np.array([-0.0625,0,0.335]), p.getQuaternionFromEuler([3.14,0,0]))
closeGripper(gripper2ID, frontID)
moveToPos(kuka2ID, np.array(frontPos) + np.array([0,0,0.5]), p.getQuaternionFromEuler([3.14,0,0]))

#reset kuka 1 position and grasp

moveToPos(kukaID, [-0.5,0.5,0.8], p.getQuaternionFromEuler([3.14/2*3,0,-3.14/2]))
moveToPos(kukaID, [-0.5,0.4,0.6], p.getQuaternionFromEuler([3.14,0,0]))
openGripper(gripperID)
stepSim(200)
side2Pos, side2Orn = p.getBasePositionAndOrientation(side2ID)
moveToPos(kukaID, np.array(side2Pos) + np.array([0,0,0.35]), p.getQuaternionFromEuler([3.14,0,0]))
closeGripper(gripperID, side2ID)
moveToPos(kukaID, [-0.5,0.4,0.6], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, [-0.5,0.5,0.8], p.getQuaternionFromEuler([3.14/2*3,0,-3.14/2]))

#move kuka 2 and 1 into position and activate nailgun

moveToPos(kukaID, [-0.45,0.45,0.5], p.getQuaternionFromEuler([3.14/2*3,0,-3.14/2]))
moveToPos(kukaID, [-0.45,0.45,0.365], p.getQuaternionFromEuler([3.14/2*3,0,-3.14/2]))
moveToPos(kuka2ID, [0,0.375,0.9], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kuka2ID, [0,0.45,0.685], p.getQuaternionFromEuler([3.14,0,0]))

activateNailgun(nailGunID, bottomID, [0,0,0], [-.1,0,-.075], childFrameOrn = p.getQuaternionFromEuler([0,3.14/2,3.14]), hitID = frontID )

openGripper(gripperID)

moveToPos(kukaID, [-0.6,0.425,0.7], p.getQuaternionFromEuler([3.14/2*3,0,-3.14/2]))

#reset kuka2

moveToPos(kuka2ID, [0.5,.5,0.9], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kuka2ID, [0.75,1,0.5], p.getQuaternionFromEuler([3.14,0,0]))
openGripper(gripper2ID)
stepSim(500)
frontPos, _ = p.getBasePositionAndOrientation(frontID)

moveToPos(kuka2ID, np.array(frontPos) + np.array([0,0,0.4]), p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kuka2ID, np.array(frontPos) + np.array([-0.05,0.0,0.3]), p.getQuaternionFromEuler([3.14,0,0]))
closeGripper(gripper2ID, frontID)
moveToPos(kuka2ID, [0.75,1,0.9], p.getQuaternionFromEuler([3.14,0,0]))


#load back and move it into position
backPos = [-0.5, 0, 0.05]
backID = p.loadURDF("urdfs/front.urdf", backPos, straightUp)
backAtticID = p.loadURDF("urdfs/attic.urdf", [-0.5, 0, 0.1], straightUp)
backConstraintID = p.createConstraint(backID, -1, backAtticID, -1, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0], childFrameOrientation = p.getQuaternionFromEuler([3.14/4,0,0]))
handleID = p.loadURDF("urdfs/handle.urdf", [-0.55, 0, 0.05], straightUp)
p.createConstraint(backID, -1, handleID, -1, p.JOINT_FIXED, [0,0,0], [-0.05, 0, -0.025], [0,0,0], childFrameOrientation = p.getQuaternionFromEuler([0, 3.14/2, 0]))
moveToPos(kukaID, np.array(backPos) + np.array([0.025,0,0.6]), p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, np.array(backPos) + np.array([0.025,0,0.325]), p.getQuaternionFromEuler([3.14,0,0]))
closeGripper(gripperID, backID)
moveToPos(kukaID, np.array(backPos) + np.array([0,0,0.6]), p.getQuaternionFromEuler([3.14,0,0]))

#position kuka 1 and 2 and activate nailgun


moveToPos(kuka2ID, [0.75,0.5,0.9], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kuka2ID, [0.175,0.575,0.65], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kuka2ID, [0.11,0.7,0.55], p.getQuaternionFromEuler([3.14,0,0]))

while True:
    p.stepSimulation()
    time.sleep(1./240.)

moveToPos(kukaID, [-.2,0.5,0.8], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, [0,0.5,0.65], p.getQuaternionFromEuler([3.14,0,0]))



#reset kuka 2 pos


#grasp handle and reposition






while True:
    p.stepSimulation()
    time.sleep(1./240.)
#gasp roof and move it to position
roof1Pos, _ = p.getBasePositionAndOrientation(roof1ID)
moveToPos(kukaID, np.array(roof1Pos) + np.array([-0.01,0,0.35]), p.getQuaternionFromEuler([3.14,0,0]))
closeGripper(gripperID, roof1ID)
moveToPos(kukaID, [-0.5,0.2,0.7], p.getQuaternionFromEuler([3.14,0,0]))
moveToPos(kukaID, [-0.5,0.45,0.4], p.getQuaternionFromEuler([-3.14/2,-3.14/4*3,3.14/2*3]))
moveToPos(kukaID, [-0.35,0.4,0.3], p.getQuaternionFromEuler([-3.14/2,-3.14/4*3,3.14/2*3]))

#lower side, activate nailgun, release grip
moveToPos(kuka2ID, [0.325,0.5,0.4], p.getQuaternionFromEuler([-3.14/2,0,3.14/2]))
activateNailgun(nailGunID, roof2ID, [-0.02,-0.05,0], [0,0.07,0], p.getQuaternionFromEuler([0,0,-3.14/4]))
openGripper(gripperID)
#sideConstraintID = p.createConstraint(gripper2ID, -1, side1ID, -1, p.JOINT_FIXED, [0,0,0], [0,0,0.3], [0,0,0], childFrameOrientation = p.getQuaternionFromEuler([0,0,-3.14/2])) 


while True:
    p.stepSimulation()
    #for i in range(50):
    time.sleep(1./240.)
p.disconnect()

