import os
import math
import numpy as np
import random
import time

import gym
from gym import error,spaces,utils
from gym.utils import seeding

import pybullet as p
import pybullet_data
'''
TO DO

'''
'''
TO IMPROVE
Add contact detection to observation space using p.getcontactpoints
change cube to plank
'''

class NailbotEnv(gym.GoalEnv):
    metadata={'render.modes':['human']}
    def __init__(self):
        self._observation = []
        #need to get action space, 8 gripper joints, 7 kuka joints
        #first 7 are kuka joints- all revolute(?)
        kuka_lower_bound = 7*[-math.pi] 
        kuka_upper_bound = 7*[math.pi]
        #gripper bounds are bounds for prismatic joints, fixed are set to 0
        gripper_lower_bound = [0, -0.055, -20, -20, -0.01, 0, -0.01, 0]
        gripper_upper_bound = [0, 0.001, 20, 20, 0.05, 0, 0.05, 0]
        self.action_space = spaces.Box(np.array(kuka_lower_bound + gripper_lower_bound), np.array(kuka_upper_bound + gripper_upper_bound)) 
        obs_shape = 108
        self.observation_space = spaces.Box(-1*np.inf*np.ones(obs_shape), np.inf*np.ones(obs_shape))
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._seed()
    def _step(self,action):
        self._assign_target_position(action)
        p.stepSimulation()
        #time.sleep(1./240.)
        self._observation = self._compute_observation()
        reward = self._compute_reward()
        done = self._compute_done()
        self._envStepCounter += 100
        return np.array(self._observation), reward, done, {}

    def _reset(self):
        self.vt=0
        self.vd=0
        self._envStepCounter=0

        p.resetSimulation()
        p.setGravity(0,0,-9.8)
        p.setTimeStep(0.01)
        
        planeId = p.loadURDF("plane.urdf")
        kukaStartPos=[0,0,0]
        kukaStartOrientation = p.getQuaternionFromEuler([0,0,0])
        cubeStartPos = [-0.5,0,0]
        #load small cube
        self.cubeId = p.loadURDF('cube_small.urdf',cubeStartPos, p.getQuaternionFromEuler([0,0,0])) 
        #load the kuka arm
        self.kukaId = p.loadURDF('kuka_iiwa/model.urdf', kukaStartPos, kukaStartOrientation)
        jointPositions = [0,0,0,1.57,0,-1.04,0]
        for jointIndex in range(p.getNumJoints(self.kukaId)):
            p.resetJointState(self.kukaId, jointIndex, jointPositions[jointIndex])
            #p.setJointMotorControl2(self.kukaId, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex],0)
        #load the gripper
        self.gripperId = p.loadSDF('gripper/wsg50_one_motor_gripper_new_free_base.sdf')[0]
        jointPositions = [0.000000,-0.011130,-0.206421,0.205143,-0.009999,0.000000,-0.010055,0.000000]
        for jointIndex in range(p.getNumJoints(self.gripperId)):
            p.resetJointState(self.gripperId, jointIndex, jointPositions[jointIndex])
            p.setJointMotorControl2(self.gripperId, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex],0)
        #attach gripper to kuka
        kuka_cid = p.createConstraint(self.kukaId, 6, self.gripperId, 0, p.JOINT_FIXED,[0,0,0], [0,0,0.05],[0,0,0])
        p.setRealTimeSimulation(1)
        self._observation = self._compute_observation()
        return np.array(self._observation)
    def _render(self, mode = 'human', close=False):
        pass
    def _assign_target_position(self, action):
        p.setJointMotorControlArray(bodyUniqueId = self.kukaId, jointIndices = range(p.getNumJoints(self.kukaId)), controlMode = p.POSITION_CONTROL, targetPositions = action[:p.getNumJoints(self.kukaId)], forces = p.getNumJoints(self.kukaId)*[60])
        p.setJointMotorControlArray(bodyUniqueId = self.gripperId, jointIndices = range(p.getNumJoints(self.gripperId)), controlMode = p.POSITION_CONTROL, targetPositions = action[p.getNumJoints(self.kukaId):p.getNumJoints(self.kukaId)+p.getNumJoints(self.gripperId)])
        return(p.getNumJoints(self.kukaId)+1, p.getNumJoints(self.kukaId)+p.getNumJoints(self.gripperId), p.getNumJoints(self.gripperId))
    def _compute_observation(self):
        #includes positions of the kuka, cube, and gripper, and whether or not the gripper is contacting the cube
        cubePos, _ = p.getBasePositionAndOrientation(self.cubeId)
        observation = []
        for i in range (len(cubePos)):
            observation.append(cubePos[i])
        for i in range (p.getNumJoints(self.kukaId)):
            kukaPos,kukaOrn,_,_,_,_ = p.getLinkState(self.kukaId, i)
            for j in range(len(kukaPos)):
                observation.append(kukaPos[j])
            for j in range(len(kukaOrn)):
                observation.append(kukaOrn[j])
        for i in range (p.getNumJoints(self.gripperId)):
            gripperPos,gripperOrn,_,_,_,_ = p.getLinkState(self.gripperId, i)
            for j in range(len(gripperPos)):
                observation.append(gripperPos[j])
            for j in range(len(gripperOrn)):
                observation.append(gripperOrn[j])
        return observation
    def _compute_reward(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.cubeId)
        goalPos = [0,0.5,0]
        distance = 0
        for i in range(len(cubePos)):
            distance += (cubePos[0]-goalPos[0])**2
        distance = math.sqrt(distance)
        return 1/distance
    def _compute_done(self):
        reward = self._compute_reward()
        return reward>1000 or self._envStepCounter >= 15000
    def _seed(self, seed = None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
