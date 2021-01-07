import numpy as np
import scipy.spatial
from math import sin, cos


class CollisionChecker:
    def __init__(self, circle_offset, circle_radii, growth_factor):
        self._circle_offset = circle_offset
        self._circle_radii = circle_radii
        self._growth_factor = growth_factor

    # Takes in a set of obstacle borders and path waypoints and returns
    # a boolean collision check array that tells if a path has an obstacle
    # or not
    def static_collision_check(self, paths, obstacles):
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
        collision_check_array = np.zeros(len(paths), dtype=bool)
        for i in range(len(paths)):
            collision_free = True
            path = paths[i]

            # Iterate over the points in the path.
            for j in range(len(path[0])):

                circle_locations = np.zeros((1, 2))

                circle_locations[0, 0] = path[0][j] + self._circle_offset * cos(path[2][j])
                circle_locations[0, 1] = path[1][j] + self._circle_offset * sin(path[2][j])

                for k in range(len(obstacles)):
                    collision_dists = scipy.spatial.distance.cdist(np.array([obstacles[k]]), circle_locations)
                    collision_dists = np.subtract(collision_dists, self._circle_radii)
                    collision_free = collision_free and not np.any(collision_dists < 0)

                    if not collision_free:
                        break
                if not collision_free:
                    break

            collision_check_array[i] = collision_free

        return collision_check_array

    # Takes in a set of paths and obstacles, and returns an array
    # of bools that says whether or not each path is collision free.
    def dynamic_collision_check(self, paths, ego_state, other_vehicle_states, look_ahead_time):
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

        # timesetop between each point in path
        time_step = look_ahead_time / (len(paths[0][0]) - 1)

        # generate the other vehicles' paths assuming same yaw and same velocity over the look ahead time
        other_vehicle_paths = np.zeros((len(other_vehicle_states), 3, len(paths[0][0])), dtype=float)

        for i in range(len(other_vehicle_paths)):
            vehicle_state = other_vehicle_states[i]
            vehicle_path = np.zeros((3, len(paths[0][0])), dtype=float)

            vehicle_path[0] = vehicle_state[0] + \
                              time_step * vehicle_state[3] * cos(vehicle_state[2]) * range(1, len(paths[0][0]) + 1, 1)
            vehicle_path[1] = vehicle_state[1] + \
                              time_step * vehicle_state[3] * sin(vehicle_state[2]) * range(1, len(paths[0][0]) + 1, 1)
            vehicle_path[2] = vehicle_state[2]

            other_vehicle_paths[i] = vehicle_path

        # checking for any collisions
        collision_check_array = np.zeros(len(paths), dtype=bool)
        for i in range(len(paths)):
            collision_free = True
            path = paths[i]

            for j in range(len(path[0])):

                # generating ego vehicle's circle location from the given offset
                ego_circle_locations = np.zeros((1, 2))

                ego_circle_locations[:, 0] = path[0][j] + self._circle_offset * cos(path[2][j])
                ego_circle_locations[:, 1] = path[1][j] + self._circle_offset * sin(path[2][j])

                for k in range(len(other_vehicle_states)):
                    # generating other vehicles' circle locations based on circle offset
                    other_circle_locations = np.zeros((1, 2))

                    other_circle_locations[:, 0] = other_vehicle_paths[k][0][j] + self._circle_offset * cos(
                        other_vehicle_paths[k][2][j])
                    other_circle_locations[:, 1] = other_vehicle_paths[k][1][j] + self._circle_offset * sin(
                        other_vehicle_paths[k][2][j])

                    # calculating if any collisions occour
                    collision_dists = scipy.spatial.distance.cdist(other_circle_locations, ego_circle_locations)
                    collision_dists = np.subtract(collision_dists,
                                                  self._circle_radii * (2 + self._growth_factor * time_step * (j + 1)))
                    collision_free = collision_free and not np.any(collision_dists < 0)

                    if not collision_free:
                        break
                if not collision_free:
                    break

            # updating collision_check_array based on the bool collision_free
            collision_check_array[i] = collision_free

        return collision_check_array

    # implements both static and dynamic under one function for ease of operation
    def check_collisions(self, paths, obstacles, ego_state, other_vehicle_states, look_ahead_time):

        static_collision_check_array = np.array(self.static_collision_check(paths, obstacles))
        new_paths = paths[np.where(static_collision_check_array)[0]]
        new_paths_index = np.where(static_collision_check_array)[0]

        if True not in static_collision_check_array:
            viable_paths = None
            return viable_paths

        dynamic_collision_check_array = np.array(
            self.dynamic_collision_check(new_paths, ego_state, other_vehicle_states, look_ahead_time))

        static_collision_check_array[new_paths_index[np.where(dynamic_collision_check_array == False)[0]]] = False
        collision_check_array = static_collision_check_array

        if True in collision_check_array:
            viable_paths = paths[collision_check_array]
        else:
            viable_paths = None

        return viable_paths


class VelocityProfile:
    def __init__(self):
        pass

    # Generated the velocity profile for a single path given initial
    # velocity of ego vehicle
    def single_velocity_profile(self, path, initial_velocity):
        v = deltav = theta = [0.0] * len(path)
        deltav[0] = 0.0
        v[0] = initial_velocity

        waypoints = np.zeros((len(path) - 1, 3))

        arr = np.zeros(4)
        x = y = [0.0] * len(path)
        t, a, b, c = 0, 0.0, 0.0, 0.0

        x, y = path[:, 0], path[:, 1]
        waypoints[0][0] = x[0]
        waypoints[0][1] = y[0]
        waypoints[0][2] = v[0]

        for xx in range(1, len(path) - 1):
            # params = env.vel_params
            # m = params.m
            # mu = params.mu
            # rolling_friction = params.rolling_friction
            m = 730.0
            mu = 0.6
            rolling_friction = 65.0

            # TODO: What are these magic numbers? g and __
            N = m * 9.81 * np.cos(theta[xx]) + 0.96552 * np.square(v[xx])
            d = 4.0
            u = v[xx - 1]
            a = np.sqrt(np.square(x[xx] - x[xx - 1]) +
                        np.square(y[xx] - y[xx - 1]))

            b = np.sqrt(np.square(x[xx] - x[xx + 1]) +
                        np.square(y[xx] - y[xx + 1]))

            c = np.sqrt(np.square(x[xx + 1] - x[xx - 1]) +
                        np.square(y[xx + 1] - y[xx - 1]))

            if (a + b + c) * (a + b - c) * (a + c - b) * (b + c - a) == 0:
                R = 1000000000000000000.0
            else:
                R = a * b * c / (np.sqrt((a + b + c) * (a + b - c) * (a + c - b) * (b + c - a)))

            arr = np.roots(
                [
                    np.square(m / R) + np.square(m) / (np.square(d) * 4) - np.square(0.96552),
                    0,

                    -2 * np.square(m) * 9.81 * np.sin(theta[xx]) / R +
                    m * (rolling_friction + 0.445 * np.square(u) - m * np.square(u) / (2 * d)) / d
                    - 2 * 0.96552 * mu * m * 9.81 * np.cos(theta[xx]),
                    0,

                    np.square(m * 9.81 * np.sin(theta[xx])) +
                    np.square(rolling_friction + 0.445 * np.square(u) - m * np.square(u) / (2 * d))
                    - np.square(mu * m * 9.81 * np.cos(theta[xx]))
                ]
            )
            arr = np.sort(arr)
            delta = arr[3] - v[xx - 1]

            if delta >= (np.sqrt(np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1]) - 1550.0) / m) - v[xx - 1]):
                deltav[xx] = (np.sqrt(np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1]) - 1550.0) / m) - v[xx - 1])
            else:
                if delta < (np.sqrt(np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1])) / m) - v[xx - 1]):
                    b = 2
                    for changes in range(1, b):
                        u = np.sqrt(np.square(np.sqrt(
                            R * (mu * N / m + 9.81 * np.sin(theta[xx])))) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1])) / m)
                        for a in range(1, changes + 1):
                            deltav[xx - changes + a - 1] = max(deltav[xx - changes + a - 1] - (v[xx - 1] - u) / changes,
                                                               (np.sqrt(np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1])) / m) - v[xx - 1]))
                        # find delta again
                        if delta < (np.sqrt(np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1])) / m) - v[xx - 1]):
                            if xx > (b - 1):
                                b += 1
                        else:
                            if delta >= (np.sqrt(np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1]) - 1550.0) / m) - v[xx - 1]):
                                deltav[xx] = (np.sqrt(np.square(v[xx - 1]) - 2 * d * (rolling_friction + 0.445 * np.square(v[xx - 1]) - 1550.0) / m) - v[xx - 1])
                            else:
                                deltav[xx] = delta
                else:
                    deltav[xx] = delta

            v[xx] = v[xx - 1] + deltav[xx]

            waypoints[xx][0] = x[xx]
            waypoints[xx][1] = y[xx]
            waypoints[xx][2] = v[xx]
        # waypoints = waypoints[0:-1]
        # print(waypoints[:][2])
        # print(waypoints)
        # print(v)
        return waypoints[:][2]
        # return v

    # generates velocity profile for all paths passed and returns the path along
    # with the velocity profile
    def generate_velocity_profile(self, paths, ego_state):
        """Returns paths along with velocity profile

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
            returns:
                vel_paths: A list of all paths + their veloicty profile
                    Is of the format:
                        [x_points, y_points, t_points, v_points]
                            x_points: List of x values (m)
                            y_points: List of y values (m)
                            t_points: List of yaw values (rad)
                            v_points: List of all the velocities from profile (m/s)

        """

        # initializing vel_paths
        vel_paths = np.zeros((len(paths), 4, len(paths[0][0])), dtype=float)

        # loop over all paths
        for i in range(len(paths)):
            path = paths[i]

            # generate velocity profile for each path
            vel_profile = self.single_velocity_profile(np.dstack((path[0], path[1]))[0], ego_state[3])

            # add profile along with path onto vel_paths
            vel_paths[i, :3, :] = path
            vel_paths[i, -1, 0] = ego_state[3]
            vel_paths[i, -1, 1:-1] = vel_profile
            vel_paths[i, -1, -1] = vel_profile[-1]

        return vel_paths


# TestBench
if __name__ == '__main__':
    collision_checker = CollisionChecker(0, 1, 0)

    paths = np.zeros((2, 3, 5))
    paths[0] = np.array([[0, 1 / 4, 2 / 4, 3 / 4, 1], [1, 1, 1, 1, 1], [0, 0, 0, 0, 0]])
    paths[1] = np.array(
        [[0, 1 / 4, 2 / 4, 3 / 4, 1], [1, 5 / 4, 6 / 4, 7 / 4, 2],
         [np.pi / 4, np.pi / 4, np.pi / 4, np.pi / 4, np.pi / 4]])

    look_ahead_time = 1

    ego_state = np.array([0, 1, 0, 1])

    other_vehicle_states = np.zeros((1, 4))
    other_vehicle_states[0] = np.array([0, -1.5, np.pi / 4, 1])

    obstacles = np.array([[3 / 4, -4]])

    viable_paths = collision_checker.check_collisions(paths, obstacles, ego_state, other_vehicle_states,
                                                      look_ahead_time)

    # print(viable_paths)

    velocity_gen = VelocityProfile()

    vel_paths = velocity_gen.generate_velocity_profile(viable_paths, ego_state)
    print(all(np.abs((vel_paths - [[[0., 0.25, 0.5, 0.75, 1.],
                         [1., 1.25, 1.5, 1.75, 2.],
                         [0.78539816, 0.78539816, 0.78539816, 0.78539816, 0.78539816],
                         [1., 0.5, 1.5, 5.78436272, 5.78436272]]]).flatten()) < 1e-8))
    # print(vel_paths)
