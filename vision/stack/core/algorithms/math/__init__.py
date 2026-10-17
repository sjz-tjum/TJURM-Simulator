# core/algorithms/math/__init__.py
"""数学工具：角度、距离、旋转/坐标变换、四元数、五次多项式。"""
from core.algorithms.math.angle import (  # noqa: F401
    TWO_PI,
    limit_rad, safe_angle_sub, unwrap_near, unwrap_angle,
    is_angle_step, is_angle_trans, angle_step_index, trans_near,
    get_angle_trans, get_toggle, nearest_armor_angle, get_angle_min,
    get_weight_by_theta,
)
from core.algorithms.math.distance import (  # noqa: F401
    distance, get_distance, distance_squared,
    point_to_segment_distance,
)
from core.algorithms.math.transform import (  # noqa: F401
    rpy_to_R, R_to_rpy, rpyt_to_T,
    FLU2RDF, RDF2FLU,
    vec3d_FLU2RDF, vec3d_RDF2FLU,
    vec2d_FLU2RDF, vec2d_RDF2FLU,
    world_to_gimbal, gimbal_to_world,
    world_to_camera_FLU, camera_FLU_to_world,
    gimbal_to_camera, camera_to_gimbal,
    gimbal_to_muzzle, muzzle_to_gimbal,
    camera_RDF_to_normalized,
    pixel_to_normalized_RDF, normalized_RDF_to_pixel,
    abs_camera_to_gimbal, gimbal_to_abs_camera,
    abs_camera_to_abs_armor, abs_armor_to_abs_camera,
    armor_to_abs_armor, abs_armor_to_armor,
)
from core.algorithms.math.intersect import (  # noqa: F401
    segment_rect_hit, segment_disc_hit,
)
from core.algorithms.math.quaternion import (  # noqa: F401
    Quat,
    quat_identity, quat_from_rpy, quat_to_rpy,
    quat_from_axis_angle, quat_from_rotation_matrix,
    quat_to_rotation_matrix,
    quat_multiply, quat_conjugate, quat_inverse,
    quat_normalize, quat_norm, quat_dot,
    quat_rotate, quat_slerp, quat_nlerp,
    quat_is_unit, quat_almost_equal,
)
from core.algorithms.math.quintic import Quintic  # noqa: F401