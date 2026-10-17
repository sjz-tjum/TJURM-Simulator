# core/entities/property/motion.py
"""运动学：运动常量、快照、键盘意图的方向量。

概念：
    MotionConfig   —— 从 cfg.sim.motion 读出来的运动量（平移速度、旋转步长、物理参数）。
    MotionSnapshot —— 某一帧某一物理实体的运动状态副本。
    Motion         —— 键盘意图用到的方向与旋转量常量容器（场地轴向，见类注释）。
"""
from dataclasses import dataclass, field

import numpy as np

from config import cfg


class MotionConfig:
    """运动量：平移速度、各级旋转步长、物理参数。全部来自 cfg.sim.motion。"""
    TAR_SPEED = float(cfg.sim.motion.tar_speed)
    R_STEP_SLOW = float(cfg.sim.motion.r_step_slow)
    R_STEP_FAST = float(cfg.sim.motion.r_step_fast)
    G_STEP_SLOW = float(cfg.sim.motion.g_step_slow)
    G_STEP_PITCH = float(cfg.sim.motion.g_step_pitch)

    MASS = float(cfg.sim.motion.mass)
    FRIC_COEF = float(cfg.sim.motion.fric_coef)
    MOTOR_GAIN = float(cfg.sim.motion.motor_gain)
    MAX_FORCE = float(cfg.sim.motion.max_force)
    MAX_SPEED = float(cfg.sim.motion.max_speed)
    MAX_ROTATE_SPEED = float(cfg.sim.motion.max_rotate_speed)


@dataclass
class MotionSnapshot:
    """某一帧某一物理实体的运动状态副本。

    - 状态字段：由 MotionManager 从实体复制进来，物理步进后回写。
    - tar_vel / tar_omg：由意图累加得到，物理步进消费。
    - acc / alp：物理步进中间量，不回写实体。
    - 物理参数（mass / fric_coef / ...）：默认值来自 cfg.sim.motion。
    """
    pos: np.ndarray
    vel: np.ndarray
    rpy: np.ndarray
    omg: np.ndarray

    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))
    alp: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tar_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tar_omg: np.ndarray = field(default_factory=lambda: np.zeros(3))

    mass: float = MotionConfig.MASS
    fric_coef: float = MotionConfig.FRIC_COEF
    motor_gain: float = MotionConfig.MOTOR_GAIN
    max_force: float = MotionConfig.MAX_FORCE
    max_speed: float = MotionConfig.MAX_SPEED
    max_rotate_speed: float = MotionConfig.MAX_ROTATE_SPEED


class Motion:
    """键盘运动方向与旋转量常量（场地轴向）。

    tar_vel 物理上就是世界系（W 系）的目标速度 —— 物理步进算的是 tar_vel - world_vel，
    所以这些平移常量是**场地轴**单位向量：按 ↑ 就是沿世界 +x 走，与车头朝向无关。
    这是既定决策：物体在场地上移动以 W 为参照。

    只给方向与旋转量、不持状态；怎么组合、怎么归一化由 KeyboardManager 负责
    （MotionManager 只把 tar_vel 当绝对目标速度消费）。
    """
    FORWARD = np.array([1.0, 0.0, 0.0])
    BACKWARD = np.array([-1.0, 0.0, 0.0])
    LEFT = np.array([0.0, 1.0, 0.0])
    RIGHT = np.array([0.0, -1.0, 0.0])
    UP = np.array([0.0, 0.0, 1.0])
    DOWN = np.array([0.0, 0.0, -1.0])

    CHASSIS_CCW_SLOW = MotionConfig.R_STEP_SLOW
    CHASSIS_CW_SLOW = -MotionConfig.R_STEP_SLOW
    CHASSIS_CCW_FAST = MotionConfig.R_STEP_FAST
    CHASSIS_CW_FAST = -MotionConfig.R_STEP_FAST

    GIMBAL_YAW_LEFT = MotionConfig.G_STEP_SLOW
    GIMBAL_YAW_RIGHT = -MotionConfig.G_STEP_SLOW
    GIMBAL_PITCH_UP = -MotionConfig.G_STEP_PITCH
    GIMBAL_PITCH_DOWN = MotionConfig.G_STEP_PITCH