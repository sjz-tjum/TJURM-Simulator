# schema.py — 由 config/generator.py 生成，勿手改。
from dataclasses import dataclass
from typing import Any, List, Literal


@dataclass
class SimCameraConfig:
    pixel_decimals: int
    focal_len: float
    max_range: float
    image_width: int
    image_height: int
    min_edge_px: float


@dataclass
class SimAimConfig:
    stale_limit: float
    slew_limit_deg: float
    pitch_slew_deg: float
    cooldown: float
    rune_cooldown: float


@dataclass
class SimGimbalYawOuterConfig:
    kp: float
    ki: float
    kd: float
    max_output: float


@dataclass
class SimGimbalYawInnerConfig:
    kp: float
    ki: float
    kd: float
    max_output: float


@dataclass
class SimGimbalPitchOuterConfig:
    kp: float
    ki: float
    kd: float
    max_output: float


@dataclass
class SimGimbalPitchInnerConfig:
    kp: float
    ki: float
    kd: float
    max_output: float


@dataclass
class SimGimbalConfig:
    alpha_max_yaw: float
    alpha_max_pitch: float
    yaw_outer: SimGimbalYawOuterConfig
    yaw_inner: SimGimbalYawInnerConfig
    pitch_outer: SimGimbalPitchOuterConfig
    pitch_inner: SimGimbalPitchInnerConfig


@dataclass
class SimMotionConfig:
    tar_speed: float
    r_step_slow: float
    r_step_fast: float
    g_step_slow: float
    g_step_pitch: float
    mass: float
    fric_coef: float
    motor_gain: float
    max_force: float
    max_speed: float
    max_rotate_speed: float
    spin_slow: float
    spin_fast: float
    patrol_radius: float
    patrol_speed: float
    patrol_radial_gain: float


@dataclass
class SimCustomConfig:
    queue_size: int
    target_fps: float


@dataclass
class SimBulletConfig:
    fire_delay: float
    manual_interval: float
    max_age: float
    max_range: float
    hit_retention: float


@dataclass
class SimConfig:
    frame: str
    sim_fps: int
    sensor_fps: int
    world_scale: float
    pixel_noise_sigma: float
    pnp_method: str
    air_density: float
    gravity: float
    seed: int
    camera: SimCameraConfig
    aim: SimAimConfig
    gimbal: SimGimbalConfig
    motion: SimMotionConfig
    custom: SimCustomConfig
    bullet: SimBulletConfig


@dataclass
class RobotSpecConfig:
    length: float
    width: float
    armor_count: int
    high_height: float
    low_height: float
    armor_size: str
    light_bar_interval: float
    light_bar_length: float
    hp: int
    priority: int
    pitch_offset: List[float]
    gimbal_mount_pos: List[float]
    camera_mount_pos: List[float]
    muzzle_mount_pos: List[float]


@dataclass
class RobotConfig:
    armor_tilt: float
    hero: RobotSpecConfig
    infantry: RobotSpecConfig
    sentry: RobotSpecConfig
    engineer: RobotSpecConfig

    def __getitem__(self, key: Literal["hero", "infantry", "sentry", "engineer"]) -> RobotSpecConfig:
        return getattr(self, key)


@dataclass
class BaseConfig:
    pass


@dataclass
class OutpostConfig:
    radius: float
    armor_count: int
    base_height: float
    height_step: float
    rotate_speed: float
    armor_size: str
    light_bar_interval: float
    light_bar_length: float
    hp: int
    priority: int
    armor_tilt: float


@dataclass
class RuneConfig:
    leaf_count: int
    radius: float
    height: float
    target_diameter: float
    armor_size: str
    light_bar_interval: float
    light_bar_length: float
    ring_inner_diameter: float
    ring_count: int
    small_speed: float
    big_speed: float
    activation_a: List[float]
    activation_omega: List[float]
    activation_b_base: float
    hit_window: float
    second_window: float
    groups: int
    min_bullet_speed: float
    ring_gate: bool
    ring_min_after_activation: List[int]
    flash_period: float
    flash_count: int
    mark_bands: List[List[float]]
    mark_cross_half_angle: float
    mark_cross_inner_diameter: float
    priority: int


@dataclass
class Bullet17mmConfig:
    mass: float
    caliber: float
    drag_cd: float
    speed: float
    damage: int


@dataclass
class Bullet42mmConfig:
    mass: float
    caliber: float
    drag_cd: float
    speed: float
    damage: int


@dataclass
class BulletConfig:
    _17mm: Bullet17mmConfig
    _42mm: Bullet42mmConfig


@dataclass
class UiPanelsMainConfig:
    rect: List[int]


@dataclass
class UiPanelsCameraConfig:
    rect: List[int]


@dataclass
class UiPanelsInfoConfig:
    rect: List[int]


@dataclass
class UiPanelsCurveConfig:
    rect: List[int]


@dataclass
class UiPanelsConfig:
    main: UiPanelsMainConfig
    camera: UiPanelsCameraConfig
    info: UiPanelsInfoConfig
    curve: UiPanelsCurveConfig


@dataclass
class UiColorsConfig:
    robot_alive: List[int]
    robot_dead: List[int]
    armor: List[int]
    observation: List[int]
    prediction: List[int]
    bullet: List[int]
    hit: List[int]
    camera: List[int]
    rune_activatable: List[int]
    rune_activated: List[int]


@dataclass
class UiConfig:
    screen_width: int
    screen_height: int
    panels: UiPanelsConfig
    colors: UiColorsConfig


@dataclass
class Config:
    sim: SimConfig
    robot: RobotConfig
    base: BaseConfig
    outpost: OutpostConfig
    rune: RuneConfig
    bullet: BulletConfig
    ui: UiConfig
