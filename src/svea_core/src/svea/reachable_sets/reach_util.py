"""
Module of helper function for handling reachable sets
"""

import os
import numpy as np
from copy import deepcopy
from scipy.io import loadmat
from grid import GridData
from svea.models.bicycle import SimpleBicycleModel
path = os.path.dirname(os.path.abspath(__file__))

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se "
__status__ = "Development"

def load_TTR_from_mat(filename):
    '''Load a minimum time-to-reach value function which can also be
    used as a reachable set'''
    file_path = path + '/' + filename
    TTR_data = loadmat(file_path)
    grid = TTR_data['g']
    val_func = TTR_data['TTR']
    val_func[val_func == 1000000.0] = float('inf')
    TTR = GridData(grid, val_func)
    return TTR

def check_safe_with_control(TTR, rel_state, next_states):
    xy_offset = np.array([rel_state.x, rel_state.y, 0.0, 0.0])
    in_grid_next_states = []
    for i, next_state in enumerate(next_states):
        curr_state = next_state + xy_offset
        if TTR.in_grid(curr_state):
            in_grid_next_states.append(curr_state)

    # compare time to reach after control
    rel_state = rel_state.array
    if TTR.in_grid(rel_state) and len(in_grid_next_states) > 0:
        next_states = in_grid_next_states
        # compute time to reach for given state, and following  states
        rel_TTR = TTR.get_raw_value(rel_state)
        next_TTRs = np.array([TTR.get_raw_value(next_state)
                              for next_state in next_states])
        if rel_TTR != 0.0:
            result = "Ctrl Unsafe" if np.any(next_TTRs == 0.0) else "Safe"
        else:
            # already unsafe
            result = "Ctrl Safer" if np.min(next_TTRs) > rel_TTR else "Unsafe"
    else:
        # not even in computation domain
        rel_TTR = float('inf')
        result = "Safe"
    return result, rel_TTR
