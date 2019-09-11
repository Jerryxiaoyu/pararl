
import otter.gym as gym
import numpy as np

import sys
import pygame
from pygame.locals import QUIT, KEYDOWN

env_params = {
    "environment_name": "ImageKinovaReacherJointEnv-v0",
    "random_target": True,
    "debug": True,
    "isImageObservation": False,
    "image_dim": 64,

    'isRender': True,
    'debug': True
}


pygame.init()
screen = pygame.display.set_mode((400, 300))

speed = 1
char_to_action = {
    '1': np.array([speed, 0, 0, 0, 0, 0, 0 ]),
    '2': np.array([0, speed, 0, 0, 0, 0, 0 ]),
    '3': np.array([0, 0, speed, 0, 0, 0, 0 ]),
    '4': np.array([0, 0, 0, speed, 0, 0, 0 ]),
    '5': np.array([0, 0, 0, 0, speed, 0, 0 ]),
    '6': np.array([0, 0, 0, 0, 0, speed, 0 ]),
    '7': np.array([0, 0, 0, 0, 0, 0, speed ]),

    'q': np.array([-speed, 0, 0, 0, 0, 0, 0 ]),
    'w': np.array([0, -speed, 0, 0, 0, 0, 0 ]),
    'e': np.array([0, 0, -speed, 0, 0, 0, 0 ]),
    'r': np.array([0, 0, 0, -speed, 0, 0, 0 ]),
    't': np.array([0, 0, 0, 0, -speed, 0, 0 ]),
    'y': np.array([0, 0, 0, 0, 0, -speed, 0 ]),
    'u': np.array([0, 0, 0, 0, 0, 0, -speed ]),

    'z': np.array([0,  0, 0, 0, 0, 0, 0 ]),

}


env = gym.from_config(env_params)

obs = env.reset()

print(obs.shape)
action = np.zeros(7)
while True:


    for event in pygame.event.get():
        event_happened = True
        if event.type == QUIT:
            sys.exit()
        if event.type == KEYDOWN:
            char = event.dict['key']
            action = char_to_action.get(chr(char), None)
            if action is None:
                action = np.zeros(7)


    obs, reward, done, _ = env.step(action)

    #print("state info :", obs.shape)

    if done:

        obs = env.reset()

    env.render()
    #print( 'reward :', reward)