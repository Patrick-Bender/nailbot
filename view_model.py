import gym
import nail_bot
from stable_baselines import DDPG

env = gym.make('nailbot-v0')
model = DDPG.load('ddpg_nailbot', env = env)
obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render('human')
