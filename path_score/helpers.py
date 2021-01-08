from dataclasses import dataclass
from typing import TypeVar, Callable, List

import numpy as np
from rclpy.node import Node
from rclpy.publisher import Publisher


@dataclass
class Weights:
    cte: float = 1
    vel: float = 1
    slope: float = 1


@dataclass
class VelParams:
    m: float = 730.0
    mu: float = 0.6
    rolling_friction: float = 65.0


@dataclass
class CollisionParams:
    circle_offset = 0
    circle_radii = 1
    growth_factor = 0


state_t = TypeVar('state_t')  # np.ndarray[[3], float]
path_t = TypeVar('path_t')  # np.ndarray[[-1, 2], float]


@dataclass
class Env:
    nh: Node
    m_pub: Publisher = None

    weights: Weights = Weights()
    vel_params: VelParams = VelParams()
    collision_params: CollisionParams = CollisionParams()
    info: Callable[[str], None] = None

    state: state_t = np.zeros(4)  # [x, y, theta, v]
    path: path_t = None  # [ [x, y], ... ]

    obstacles: np.ndarray = np.zeros((0, 2))
    other_vehicle_states: List[state_t] = ()
