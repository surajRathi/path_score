from typing import Generator, List

import numpy as np

from path_score.helpers import path_t


def generate_paths(env, n=10, n_pts=20) -> Generator[path_t, None, None]:
    """
    Generates straight lines around the direction the vehicle faces
    TODO: Replace with an actual path generator

    :param env: Environment
    :param n: Number of paths
    :param n_pts: Number of points in each path
    :return:
    """
    r = np.linspace(0, 5, n_pts)
    veh = env.state
    for i in range(n):
        a = np.arctan(-0.8 + (0.8 - (-0.8)) * (1.0 * i / n))
        x = veh[0] + r * np.cos(a + veh[2])
        y = veh[1] + r * np.sin(a + veh[2])
        yield np.stack([x, y], axis=1)
