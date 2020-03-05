import pybullet as p
import time
import pybullet_data
import numpy as np

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
cubeID = p.loadURDF('cube_small.urdf', [0,0,0.225], p.getQuaternionFromEuler([0,0,0]))
cube2ID = p.loadURDF('cube_small.urdf', [0,0,0.275], p.getQuaternionFromEuler([0,0,0]))
nailgunID = p.loadURDF('nailgun.urdf', [0,0,0.1], p.getQuaternionFromEuler([0,0,0]))
print("cubeID"+str(cubeID))
#Touches the cube to see if it's nailed together or not
'''
for i in range(200):
    p.stepSimulation()
    time.sleep(1./240.)
activateNailgun(nailgunID, cubeID, cube2ID)
'''
childFrame = [0,0,0,0]
parentFrame = [0,0,0,0]

for i in range(200):
    p.stepSimulation()
    time.sleep(1./240.)
activateNailgun(nailgunID, cubeID)
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()
