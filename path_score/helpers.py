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
path_t = TypeVar('path_t')    # np.ndarray[[-1, 2], float]


@dataclass
class Env:
    nh: Node
    m_pub: Publisher = None

    weights: Weights = Weights()

    state: state_t = np.zeros(3)  # [x, y, theta]
    path: path_t = None  # [ [x, y], ... ]
