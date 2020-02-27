import pybullet as p
import time

import pybullet_data

#cid = p.connect(p.UDP,"192.168.86.100")
cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()

objects = [
    p.loadURDF("plane.urdf", 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)
]
kuka = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", [1.40000, -0.200000, 0], [0.000000, 0.000000, 0.000000, 1.000000])
jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
for jointIndex in range(p.getNumJoints(kuka)):
  p.resetJointState(kuka, jointIndex, jointPositions[jointIndex])
  p.setJointMotorControl2(kuka, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)
#I have no idea why it needs the [0] at the end, but loadSDF loads a tuple and the first one is what we want for some reason
kuka_gripper = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
print("kuka gripper=")
print(kuka_gripper)
print("Kuka # joints: %d " % p.getNumJoints(kuka))

for i in range(p.getNumJoints(kuka)):
    print(p.getJointInfo(kuka, i)) 


print("Gripper # joints: %d" % p.getNumJoints(kuka_gripper))
print("Gripper joint types:")
for i in range(p.getNumJoints(kuka_gripper)):
    print(p.getJointInfo(kuka_gripper, i))
jointPositions = [
    0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000
]

for jointIndex in range(p.getNumJoints(kuka_gripper)):
  p.resetJointState(kuka_gripper, jointIndex, jointPositions[jointIndex])
  p.setJointMotorControl2(kuka_gripper, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex],0)

kuka_cid = p.createConstraint(kuka, 6, kuka_gripper, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05],[0, 0, 0])
p.setGravity(0, 0, -10)

p.setRealTimeSimulation(1)
ref_time = time.time()

running_time = 4  # seconds
while (time.time() < ref_time + running_time):
  p.setGravity(0, 0, -10)
  p.stepSimulation()

p.disconnect()
