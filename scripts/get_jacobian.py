#!/usr/bin/env python3

import numpy as np

# Differential robot
def get_jacobian(tractor_yaw, tractor_tyre_radius):
      
      J = np.array([[ tractor_tyre_radius * np.sin(tractor_yaw), 0.0],
                    [ tractor_tyre_radius * np.cos(tractor_yaw), 0.0],
                    [ 0.0,                                       1.0]])
      
      return J