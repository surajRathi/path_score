import dataclasses
from dataclasses import dataclass
from typing import TypeVar

import numpy as np
from rclpy.node import Node
from rclpy.publisher import Publisher


@dataclass
class Weights:
    cte: float = 1
    vel: float = 1
    slope: float = 1


state_t = TypeVar('state_t')  # np.ndarray[[3], float]
path_t = TypeVar('path_t')  # np.ndarray[[-1, 2], float]


@dataclass
class VelParams:
    m: float = 730.0
    mu: float = 0.6
    rolling_friction: float = 65.0


@dataclass
class Env:
    nh: Node
    m_pub: Publisher = None

    weights: Weights = Weights()
    vel_params: VelParams = VelParams()
    info = None

    state: state_t = np.zeros(4)  # [x, y, theta, v]
    path: path_t = None  # [ [x, y], ... ]
