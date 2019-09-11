
import otter.gym as gym
import numpy as np

env_params = {
    "environment_name": "Reach",
    "random_start": False,
    "random_target": False,
    'is_image_state': True,
    "image": True,
    "image_dim": 128,
    "goal_point": [0.5, 0, 0.5],
    'isAbsolute_control': False,
    '_render':True
}
data_params=dict(
        num_rollouts=10,
        init_std=0.5,
        smooth_noise=False,
    )
horizon = 50

env = gym.from_config(env_params)

print('Env state dim: ', env.get_state_dim())
print('Env action dim: ', env.get_action_dim())

from scipy.ndimage import filters
def generate_noise(dims, std=1.0, smooth=False):
    if std == 0.0:
        return np.zeros(dims)
    noise = std * np.random.randn(*dims)
    if smooth:
        for j in range(dims[-1]):
            noise[..., j] = filters.gaussian_filter(noise[..., j], 2.0)
        emp_std = np.std(noise, axis=0)
        noise = std * (noise / emp_std)
    return noise
def noise_function():
    return generate_noise((horizon, env.get_action_dim()),
                               std= data_params['init_std'],
                               smooth= data_params['smooth_noise'])

obs = env.reset()

while True:
    action = [1,1,1]  #env.action_space.sample()
    #action =  env.gym_env.action_space.sample()

    obs, reward, done, _ = env.step(action)
    env.render()


