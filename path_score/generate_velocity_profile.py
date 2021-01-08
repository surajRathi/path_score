import numpy as np

from path_score.helpers import Env, path_t


def generate_velocity_profile(env: Env, path: path_t) -> np.ndarray:
    """

    :param env: Env
    :param path: [n x 2] float
    :return: [n-1] float
    """
    # TODO: Many repeated calculations, store value
    # e.g.
    # if a > foo(a): a = foo(a)
    # if a > (a_max := foo(a)): a = a_max)

    v = np.zeros(len(path) - 1)
    deltav = np.zeros(len(path) - 1)
    theta = np.zeros(len(path) - 1)
    deltav[0] = 0.0
    v[0] = env.state[3]

    x, y = path[:, 0], path[:, 1]

    for xx in range(1, len(path) - 1):
        params = env.vel_params
        m = params.m
        mu = params.mu
        rolling_friction = params.rolling_friction

        # TODO: What are these magic numbers? g and __ Cd ?
        normal_reaction = m * 9.81 * np.cos(theta[xx]) + 0.96552 * np.square(v[xx])
        d = 4.0
        u = v[xx - 1]

        a = np.sqrt(np.square(x[xx] - x[xx - 1]) +
                    np.square(y[xx] - y[xx - 1]))
        # assert (a == np.linalg.norm(path[xx] - path[xx - 1]))

        b = np.sqrt(np.square(x[xx] - x[xx + 1]) +
                    np.square(y[xx] - y[xx + 1]))
        # assert (b == np.linalg.norm(path[xx] - path[xx + 1]))

        c = np.sqrt(np.square(x[xx + 1] - x[xx - 1]) +
                    np.square(y[xx + 1] - y[xx - 1]))
        # assert (c == np.linalg.norm(path[xx + 1] - path[xx - 1]))

        if -1e-9 <= (a + b + c) * (a + b - c) * (a + c - b) * (b + c - a) <= 0:  # colinear
            radius = 1000000000000000000.0
        else:
            radius = a * b * c / (np.sqrt((a + b + c) * (a + b - c) * (a + c - b) * (b + c - a)))

        possible_velocities = np.roots(
            [
                np.square(m / radius) + np.square(m) / (np.square(d) * 4) - np.square(0.96552),
                0,
                -2 * np.square(m) * 9.81 * np.sin(theta[xx]) / radius +
                m * (rolling_friction + 0.445 * np.square(u) - m * np.square(u) / (2 * d)) / d
                - 2 * 0.96552 * mu * m * 9.81 * np.cos(theta[xx]),
                0,
                np.square(m * 9.81 * np.sin(theta[xx])) +
                np.square(rolling_friction + 0.445 * np.square(u) - m * np.square(u) / (2 * d))
                - np.square(mu * m * 9.81 * np.cos(theta[xx]))
            ]
        )
        possible_velocities = np.sort(possible_velocities)
        delta = possible_velocities[3] - v[xx - 1]
        # assert (delta == np.max(possible_velocities) - v[xx - 1])

        if delta >= (np.sqrt(
                np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1]) - 1550.0) / m) - v[xx - 1]):
            deltav[xx] = (np.sqrt(np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1]) - 1550.0) / m) - v[xx - 1])
        else:
            if delta < (
                    np.sqrt(np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1])) / m) -
                    v[xx - 1]):
                b = 2
                for changes in range(1, b):
                    u = np.sqrt(np.square(np.sqrt(
                        radius * (mu * normal_reaction / m + 9.81 * np.sin(theta[xx])))) - 2 * d * (
                                        rolling_friction + 0.445 * np.square(v[xx - 1])) / m)
                    # TODO: we are changing the previous deltas, but not the previous velocities !?
                    for i in range(1, changes + 1):
                        deltav[xx - changes + i - 1] = max(deltav[xx - changes + i - 1] - (v[xx - 1] - u) / changes,
                                                           (np.sqrt(np.square(v[xx - 1]) - 2 * d * (
                                                                   rolling_friction + 0.445 * np.square(
                                                               v[xx - 1])) / m) - v[xx - 1]))
                    # find delta again
                    if delta < (np.sqrt(
                            np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1])) / m) -
                                v[xx - 1]):
                        if xx > (b - 1):
                            b += 1
                    else:
                        if delta >= (np.sqrt(np.square(v[xx - 1]) - 2 * d * (
                                rolling_friction + 0.445 * np.square(v[xx - 1]) - 1550.0) / m) - v[xx - 1]):
                            deltav[xx] = (np.sqrt(np.square(v[xx - 1]) - 2 * d * (
                                    rolling_friction + 0.445 * np.square(v[xx - 1]) - 1550.0) / m) - v[xx - 1])
                        else:
                            deltav[xx] = delta
            else:
                deltav[xx] = delta

        v[xx] = v[xx - 1] + deltav[xx]

    return v
