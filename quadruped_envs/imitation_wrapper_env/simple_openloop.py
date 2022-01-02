from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import inspect
import numpy as np

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from quadruped_envs.od import ACTION_CONFIG

def normalize_action(input_action):
    output_action = []
    for i in range(0, len(input_action)):
        output_action.append((input_action[i] * (ACTION_CONFIG[i].upper_bound - ACTION_CONFIG[i].lower_bound)) / 2)
    return np.array(output_action)


class SimpleNormalizer(object):

    def __init__(
            self
    ):
        """Initializes the controller."""

    def reset(self):
        pass

    def get_action(self, current_time=None, input_action=None):
        """Limits actions.
        Args:
          current_time: The time in gym env since reset.
          input_action: A numpy array. The input leg pose from a NN controller.
        Returns:
          A numpy array. The desired motor angles.
        """
        # del current_time
        # output_action = normalize_action(input_action)
        # return output_action
        return input_action

    def get_observation(self, input_observation):
        """Get the trajectory generator's observation."""
        return input_observation
