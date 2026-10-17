# core/algorithms/control/pid.py
"""PID 控制器：单级 + 串级。"""
from dataclasses import dataclass

import numpy as np

__all__ = ["PIDParams", "PID", "CascadedPID"]


@dataclass
class PIDParams:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    max_output: float = np.inf
    max_integral: float = np.inf
    deadband: float = 0.0


class PID:
    def __init__(self, params: PIDParams):
        self.params = params
        self.last_error = 0.0
        self.integral = 0.0
        self.output = 0.0

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.output = 0.0

    def update(self, error: float, dt: float) -> float:
        if abs(error) < self.params.deadband:
            error = 0.0

        p_term = self.params.kp * error
        self.integral += error * dt
        if self.integral > self.params.max_integral:
            self.integral = self.params.max_integral
        elif self.integral < -self.params.max_integral:
            self.integral = -self.params.max_integral
        i_term = self.params.ki * self.integral
        d_term = (self.params.kd * (error - self.last_error) / dt
                  if dt > 0 else 0.0)

        output = p_term + i_term + d_term
        if output > self.params.max_output:
            output = self.params.max_output
        elif output < -self.params.max_output:
            output = -self.params.max_output

        self.last_error = error
        self.output = output
        return output


class CascadedPID:
    def __init__(self, outer_params: PIDParams, inner_params: PIDParams):
        self.outer = PID(outer_params)
        self.inner = PID(inner_params)

    def reset(self):
        self.outer.reset()
        self.inner.reset()

    def update(self,
               setpoint: float,
               measurement_outer: float,
               measurement_inner: float,
               feedforward_outer: float = 0.0,
               feedforward_inner: float = 0.0,
               dt: float = 0.01) -> float:
        outer_error = setpoint - measurement_outer
        inner_setpoint = self.outer.update(outer_error, dt) + feedforward_outer
        inner_error = inner_setpoint - measurement_inner
        control = self.inner.update(inner_error, dt) + feedforward_inner
        return control