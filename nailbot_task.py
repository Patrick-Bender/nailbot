import gym
from stable_baselines import DDPG
from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
import nail_bot
import numpy as np

def callback(lcl, glb):
        #stop training if reward goes over 199
        is_solved = lcl['t'] >100 and sum(lcl['episode_rewards'][-101:-1])/100>=199
        return is_solved

def main():
    env = gym.make('nailbot-v0')
    n_actions = env.action_space.shape[-1]
    param_noise = None
    action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

    model = DDPG(MlpPolicy, env, verbose=1, param_noise=param_noise, action_noise=action_noise)
    model.learn(total_timesteps=400000)


    model = DDPG.load("ddpg_nailbot")

    obs = env.reset()
    done_count = 0
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render('human')
        model.save('ddpg_nailbot')
            

if __name__=='__main__':
        main()
