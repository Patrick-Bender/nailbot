import pybullet as p
import pybullet_data
import time
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeID = p.loadURDF("plane.urdf")
startPos = [0,0,0.2]
startOrn = p.getQuaternionFromEuler([0,0,0])
#print(pybullet_data.getDataPath())
botID = p.loadURDF("kuka_iiwa/model_free_base.urdf", startPos, startOrn)
botID = p.loadURDF("kuka_iiwa/model.urdf", [1,1,0], startOrn)
jointPositions = [-0.000000, -0.000000, 0.000000, 1.5, 0.000000, -1.036725, 0.000001]
for jointIndex in range(p.getNumJoints(botID)):
  p.resetJointState(botID, jointIndex, jointPositions[jointIndex])
  p.setJointMotorControl2(botID, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)
for i in range(10000):
	p.stepSimulation
	time.sleep(1./240.)
p.disconnect()
