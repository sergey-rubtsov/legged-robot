"""Simple openloop trajectory generators."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


class SimpleOpenLoopGenerator(object):

    def __init__(
            self
    ):
        """Initializes the controller."""

    def reset(self):
        pass

    def get_action(self, current_time=None, input_action=None):
        """Computes the trajectory according to input time and action.
        Args:
          current_time: The time in gym env since reset.
          input_action: A numpy array. The input leg pose from a NN controller.
        Returns:
          A numpy array. The desired motor angles.
        """
        # del current_time
        return input_action

    def get_observation(self, input_observation):
        """Get the trajectory generator's observation."""
        return input_observation
