import pickle
import tensorflow as tf
from deepx import T
import numpy as np
import random
from path import Path

import otter.util as util
import otter.gym as gym
# import parasol.control
# import parasol.model
import sys
from .common import Experiment



sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import ros.src.otter_kinova_grasping.otter_kinova_grasping.scripts.kinova_cup as kc

gfile = tf.gfile


class Myexp_real(Experiment):

    experiment_type = 'myexp_real'

    def __init__(self, experiment_name, env, control, model,
                 seed=0,
                 data={},
                 dump_data = False,
                 horizon=50,
                 num_videos=1,
                 rollouts_per_iter=100,
                 num_iters=10,
                 buffer_size=None,
                 smooth_noise=True,
                 model_train={},
                 **kwargs):

        super(Myexp_real, self).__init__(experiment_name, **kwargs)
        self.env_params = env
        self.control_params = control
        self.horizon = horizon
        self.model_path = model
        self.seed = seed
        self.num_videos = num_videos
        self.rollouts_per_iter = rollouts_per_iter
        self.buffer_size = buffer_size if buffer_size is not None else rollouts_per_iter
        self.num_iters = num_iters
        self.smooth_noise = smooth_noise
        self.model_train_params = model_train
        self.data_params = data
        self.dump_data = dump_data

    def initialize(self, out_dir):
        if not gfile.Exists(out_dir / "data"):
            gfile.MakeDirs(out_dir / "data")

        self.env = kc.ROSImageKinovaCupPusherEnv()
        self.initialize_params()

        print('initializing the experiment..')

    def initialize_params(self):
        # if self.model_path is not None:
        #     with gfile.GFile(self.model_path, 'rb') as fp:
        #         self.model = pickle.load(fp)
        # else:
        #     self.model = parasol.model.NoModel(self.env.get_state_dim(),
        #                                        self.env.get_action_dim(), self.horizon)
        # self.control = parasol.control.from_config(self.model, self.control_params, self.env)
        pass

    def to_dict(self):
        return {
            'experiment_name': self.experiment_name,
            'experiment_type': self.experiment_type,
            'env': self.env_params,
            'seed': self.seed,
            'env': self.env_params.copy(),
            'control': self.control_params.copy(),
            'horizon': self.horizon,
            "dump_data": self.dump_data,
            'model': self.model_path,
            'model_train': self.model_train_params,
            'out_dir': self.out_dir,
            'buffer_size': self.buffer_size,
            'rollouts_per_iter': self.rollouts_per_iter,
            'num_videos': self.num_videos,
            'num_iters': self.num_iters,
            'smooth_noise': self.smooth_noise,
        }

    @classmethod
    def from_dict(cls, params):
        return Myexp_real(
            params['experiment_name'],
            params['env'],
            params['control'],
            params['model'],
            out_dir=params['out_dir'],
            seed=params['seed'],
            data=params['data'],
            dump_data=params['dump_data'],
            num_videos=params['num_videos'],
            num_iters=params['num_iters'],
            buffer_size=params['buffer_size'],
            smooth_noise=params['smooth_noise'],
            model_train=params['model_train'],
            horizon=params['horizon'],
            rollouts_per_iter=params['rollouts_per_iter']
        )

    def run_experiment(self, out_dir):

        out_dir = Path(out_dir)

        T.core.set_random_seed(self.seed)
        np.random.seed(self.seed)
        random.seed(self.seed)

        def noise_function():
            return util.generate_noise((self.horizon, self.env.get_action_dim()),
                                       std=self.data_params['init_std'],
                                       smooth=self.data_params['smooth_noise'])

        num_rollouts = self.data_params['num_rollouts']
        policy = lambda _, __, ___, noise: noise

        rollouts = self.env.rollouts(num_rollouts, self.horizon, policy=policy, noise=noise_function, show_progress=True, )

        rollouts = (
            rollouts[0],
            rollouts[1],
            rollouts[2],# - 0.5 * np.einsum('nta,ab,ntb->nt', rollouts[1], self.env.torque_matrix(), rollouts[1])
            rollouts[3]
        )


        if self.dump_data and 'load_data' not in self.data_params:
            with gfile.GFile(out_dir / "data" / "rollouts.pkl", 'wb') as fp:
                pickle.dump(rollouts, fp)

        print('This is a experiment on Myexp!')
