import dataclasses
from dataclasses import dataclass
from typing import List

from rclpy.node import Node
from rclpy.publisher import Publisher


# Helper Data types
@dataclass
class Position:
    x: float
    y: float

    def __getitem__(self, item):
        return (self.x, self.y)[item]


@dataclass
class State(Position):
    theta: float


@dataclass
class Weights:
    cte: float = 1
    vel: float = 1
    slope: float = 1


@dataclass
class Env:
    nh: Node
    m_pub: Publisher = None

    weights: Weights = Weights()

    state: State = State(0, 0, 0)
    path: List[Position] = dataclasses.field(default_factory=list)  # []
