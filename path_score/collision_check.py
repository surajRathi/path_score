import numpy as np
import scipy.spatial

import math

from std_msgs.msg import ColorRGBA

from path_score.generate_markers import visualize
from path_score.helpers import Env, path_t, state_t


# TODO: Fix docstrings

class CollisionChecker:
    def __init__(self, env: Env, path_length: int, time_step: float):
        self.info = env.info

        params = env.collision_params
        self._circle_offset = params.circle_offset
        self._circle_radii = params.circle_radii
        self._growth_factor = params.growth_factor
        self._obstacles = env.obstacles
        self._time_step = time_step
        self._path_length = path_length
        other_vehicle_states = env.other_vehicle_states
        self._other_vehicle_paths = np.zeros((len(other_vehicle_states), 3, path_length), dtype=float)

        for i in range(len(self._other_vehicle_paths)):
            vehicle_state: state_t = other_vehicle_states[i]
            vehicle_path = np.zeros((3, path_length), dtype=float)

            time = np.arange(1, path_length + 1)  # , 1)
            vehicle_path[0] = time_step * vehicle_state[3] * math.cos(vehicle_state[2]) * time + vehicle_state[0]
            vehicle_path[1] = time_step * vehicle_state[3] * math.sin(vehicle_state[2]) * time + vehicle_state[1]
            vehicle_path[2] = vehicle_state[2]
            visualize(env.m_pub, env.nh.get_clock(), 75 + i, vehicle_path.T[:, :2],
                      color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))

            self._other_vehicle_paths[i] = vehicle_path

        # Takes in a set of obstacle borders and path waypoints and returns
        # a boolean collision check array that tells if a path has an obstacle
        # or not

    def _static_collision_check(self, path: path_t):
        """Returns a bool array on whether each path is collision free.

        args:
            paths: A list of paths in the global frame.
                A path is a list of points of the following format:
                    [x_points, y_points, t_points]:
                        x_points: List of x values (m)
                        y_points: List of y values (m)
                        t_points: List of yaw values (rad)
                    Example of accessing the ith path, jth point's t value:
                        paths[i][2][j]
            obstacles: A list of [x, y] points that represent points along the
                border of obstacles, in the global frame.
                Format: [[x0, y0],
                         [x1, y1],
                         ...,
                         [xn, yn]]
                , where n is the number of obstacle points and units are [m, m]

        returns:
            collision_check_array: A list of boolean values which classifies
                whether the path is collision-free (true), or not (false). The
                ith index in the collision_check_array list corresponds to the
                ith path in the paths list.
        """
        if len(self._obstacles) == 0:
            return True

        if len(path) == 0:
            assert (False, "Empty Path")

        # Iterate over the points in the path.
        for (i, pt) in enumerate(path):

            circle_locations = np.zeros((1, 2))

            # TODO: Find better approximation of theta
            # Take two indices to find slope
            i1 = i - 1 if i > 0 else i
            i2 = i + 1 if i < len(path) - 1 else i
            theta = np.arctan2(path[i2, 1] - path[i1, 1], path[i2, 1] - path[i, 1])

            circle_locations[0, 0] = pt[0] + self._circle_offset * math.cos(theta)
            circle_locations[0, 1] = pt[1] + self._circle_offset * math.sin(theta)

            for obstacle in self._obstacles:
                collision_dists = scipy.spatial.distance.cdist(np.array([obstacle]), circle_locations)
                collision_dists = np.subtract(collision_dists, self._circle_radii)
                if np.any(collision_dists < 0):
                    return False

            # # Loop not required right?
            # collision_dists = scipy.spatial.distance.cdist(np.array(self._obstacles), circle_locations)
            # collision_dists = np.subtract(collision_dists, self._circle_radii)
            # if np.any(collision_dists < 0):
            #     return False

        return True

    def _dynamic_collision_check(self, path: path_t):
        """ Returns a bool array on whether each path is collision free.

        args:
                paths: A list of paths in the global frame.
                    A path is a list of points of the following format:
                        [x_points, y_points, t_points]:
                            x_points: List of x values (m)
                            y_points: List of y values (m)
                            t_points: List of yaw values (rad)
                        Example of accessing the ith path, jth point's t value:
                            paths[i][2][j]
                ego_state: ego state vector for the vehicle. (global frame)
                    format: [ego_x, ego_y, ego_yaw, ego_speed]
                        ego_x and ego_y     : position (m)
                        ego_yaw             : top-down orientation [-pi to pi] (ground frame)
                        ego_speed : speed (m/s)
                other_vehicle_states: other vehicles' state vectors
                    Each state vector is of the format (global frame):
                        [pos_x, pos_y, yaw, speed]
                            pos_x and pos_y	: position (m)
                            yaw 			: top-down orientation [-pi to pi] (ground frame)
                            speed 			: speed (m/s)
                        Example of accessing the ith car's speed would be:
                            other_vehicle_states[i][3]
                look_ahead_time: The look ahead time to which the paths have been generated (s)

            returns:
                collision_check_array: A list of boolean values which classifies
                    whether the path is collision-free (true), or not (false). The
                    ith index in the collision_check_array list corresponds to the
                    ith path in the paths list.
        """
        if len(self._other_vehicle_paths) == 0:
            return True

        if len(path) == 0:
            assert (False, "Empty Path")

        # time step between each point in path
        time_step = self._time_step

        for j in range(len(path[0])):

            # generating ego vehicle's circle location from the given offset
            ego_circle_locations = np.zeros((1, 2))

            circle_offset = self._circle_offset
            ego_circle_locations[:, 0] = path[0][j] + circle_offset * math.cos(path[2][j])
            ego_circle_locations[:, 1] = path[1][j] + circle_offset * math.sin(path[2][j])

            for k in range(len(self._other_vehicle_paths)):
                # generating other vehicles' circle locations based on circle offset
                other_circle_locations = np.zeros((1, 2))

                other_circle_locations[:, 0] = self._other_vehicle_paths[k][0][j] + circle_offset * math.cos(
                    self._other_vehicle_paths[k][2][j])
                other_circle_locations[:, 1] = self._other_vehicle_paths[k][1][j] + circle_offset * math.sin(
                    self._other_vehicle_paths[k][2][j])

                # calculating if any collisions occur
                collision_dists = scipy.spatial.distance.cdist(other_circle_locations, ego_circle_locations)
                collision_dists = np.subtract(collision_dists,
                                              self._circle_radii *
                                              (2 + self._growth_factor * time_step * (j + 1)))

                if np.any(collision_dists < 0):
                    return False

        return True

    def check_collisions(self, path: path_t):
        return self._static_collision_check(path) and self._dynamic_collision_check(path)
