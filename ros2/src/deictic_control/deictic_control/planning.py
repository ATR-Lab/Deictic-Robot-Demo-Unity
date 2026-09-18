"""Copied planning requests; workers never access ROS or controller state."""
from dataclasses import dataclass
import copy
import time
from threading import Event
import numpy as np
from .kinematics import make_plan
from .teleop import check_bimanual_geometry


def frozen_array(value):
    value = np.array(value, dtype=float, copy=True)
    value.setflags(write=False)
    return value


@dataclass(frozen=True)
class PlanningRequest:
    generation: int
    goal_stamp: tuple
    created: float
    deadline: float
    epoch: int
    transform: np.ndarray
    target: np.ndarray
    joints: np.ndarray
    model: object
    velocity: float
    acceleration: float
    table_top: float
    obstacles: tuple
    cancelled: Event
    held_model: object = None
    held_joints: np.ndarray = None

    @classmethod
    def capture(cls, generation, goal_stamp, created, deadline, epoch, transform, target,
                joints, model, velocity, acceleration, table_top, obstacles,
                held_model=None, held_joints=None):
        return cls(generation, tuple(goal_stamp), created, deadline, epoch,
                   frozen_array(transform), frozen_array(target), frozen_array(joints),
                   copy.deepcopy(model), float(velocity), float(acceleration), float(table_top),
                   tuple(tuple(float(v) for v in box) for box in obstacles), Event(),
                   copy.deepcopy(held_model), frozen_array(held_joints) if held_joints is not None else None)


def solve_request(request):
    cancelled = lambda: request.cancelled.is_set() or time.monotonic() > request.deadline
    plan = make_plan(request.model, request.joints, request.target,
                     velocity=request.velocity, acceleration=request.acceleration,
                     table_top=request.table_top, obstacles=request.obstacles,
                     cancel=cancelled)
    if request.held_joints is not None:
        for q in plan.positions:
            if cancelled():
                raise ValueError('planning_cancelled')
            check_bimanual_geometry((request.held_model, request.model), (request.held_joints, q),
                                     request.table_top, request.obstacles)
    return plan
