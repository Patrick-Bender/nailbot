import pybullet as p
import time
import pybullet_data
from random import uniform
from math import sin, cos
from scipy.spatial.transform import Rotation
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeID = p.loadURDF("plane.urdf")
cubeID = p.loadURDF('cube_small.urdf', [0,0,0.025], p.getQuaternionFromEuler([0,0,0]))
cube2ID = p.loadURDF('cube_small.urdf', [0,0,0.075], p.getQuaternionFromEuler([0,0,0]))
#Touches the cube to see if it's nailed together or not

p.createConstraint(cubeID, -1, cube2ID, -1, p.JOINT_FIXED, [0,0,1], [0,0,0.025], [0,0,-0.025])
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()
