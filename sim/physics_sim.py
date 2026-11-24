"""3200 Hz master-grid 6-DOF chest-mounted hockey rink physics simulator.

Generates physically self-consistent .hks binary telemetry logs and isolated
ground-truth dictionaries across NHL, IIHF, and CUSTOM rink geometries.
Strictly isolated from sim.fusion_pipeline (zero fusion imports).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from .flash_codec import (
    CANONICAL_R_BS,
    DEFAULT_ACCEL_SCALE,
    DEFAULT_GYRO_SCALE,
    DEFAULT_HIGH_G_SCALE,
    DEFAULT_MAG_SCALE,
    GRAVITY_MPS2,
    IDENTITY_3X3,
    AuxEnvPacket,
    AuxValidityFlag,
    CalibrationBlock,
    EventMarkerPacket,
    FrameFlag,
    HighGBatchPacket,
    HighGSample,
    ImuBatchPacket,
    ImuSample,
    SessionHeader,
    encode_hks_session,
)

# ---------------------------------------------------------------------------
# Rink Geometry & Signed-Distance Containment
# ---------------------------------------------------------------------------

RINK_PRESET_SPECS: dict[str, dict[str, Any]] = {
    "NHL_200X85FT": {
        "length_m": 60.96,
        "width_m": 25.908,
        "corner_radius_m": 8.5344,
        "goal_line_from_boards_m": 3.3528,
        "blue_line_from_boards_m": 22.86,
        "default_bench_gate_xy_m": (-6.0, 12.954),
    },
    "IIHF_60X30M": {
        "length_m": 60.0,
        "width_m": 30.0,
        "corner_radius_m": 8.5,
        "goal_line_from_boards_m": 4.0,
        "blue_line_from_boards_m": 22.5,
        "default_bench_gate_xy_m": (-6.0, 15.0),
    },
    "CUSTOM": {
        "length_m": 56.0,
        "width_m": 26.0,
        "corner_radius_m": 7.5,
        "goal_line_from_boards_m": 3.5,
        "blue_line_from_boards_m": 21.0,
        "default_bench_gate_xy_m": (-5.0, 13.0),
    },
}


@dataclass(frozen=True)
class RinkGeometry:
    preset_name: str
    length_m: float
    width_m: float
    corner_radius_m: float
    goal_line_from_boards_m: float
    blue_line_from_boards_m: float
    bench_gate_xy_m: tuple[float, float]
    r_torso_m: float = 0.25

    @classmethod
    def from_preset(
        cls,
        preset_name: str,
        custom_rink: dict[str, Any] | None = None,
        r_torso_m: float = 0.25,
    ) -> "RinkGeometry":
        if preset_name not in RINK_PRESET_SPECS:
            raise ValueError(f"Unknown rink_preset {preset_name!r}")
        spec = dict(RINK_PRESET_SPECS[preset_name])
        if custom_rink:
            spec.update(custom_rink)
        gate = tuple(float(v) for v in spec["default_bench_gate_xy_m"])
        return cls(
            preset_name=preset_name,
            length_m=float(spec["length_m"]),
            width_m=float(spec["width_m"]),
            corner_radius_m=float(spec["corner_radius_m"]),
            goal_line_from_boards_m=float(spec["goal_line_from_boards_m"]),
            blue_line_from_boards_m=float(spec["blue_line_from_boards_m"]),
            bench_gate_xy_m=(gate[0], gate[1]),
            r_torso_m=float(r_torso_m),
        )

    @property
    def bench_pose_xy(self) -> tuple[float, float]:
        gx, gy = self.bench_gate_xy_m
        sgn_y = 1.0 if gy >= 0.0 else -1.0
        return (gx, gy - sgn_y * self.r_torso_m)

    @property
    def bench_facing_yaw_rad(self) -> float:
        gy = self.bench_gate_xy_m[1]
        return -math.pi / 2.0 if gy >= 0.0 else math.pi / 2.0

    def signed_distance(self, x: float, y: float) -> float:
        """Exact signed distance to inner board wall (< 0 inside ice, 0 on wall)."""
        a = self.length_m / 2.0 - self.corner_radius_m
        b = self.width_m / 2.0 - self.corner_radius_m
        qx = abs(x) - a
        qy = abs(y) - b
        return (
            math.hypot(max(qx, 0.0), max(qy, 0.0))
            + min(max(qx, qy), 0.0)
            - self.corner_radius_m
        )

    def outward_normal(self, x: float, y: float) -> tuple[float, float]:
        a = self.length_m / 2.0 - self.corner_radius_m
        b = self.width_m / 2.0 - self.corner_radius_m
        ax = abs(x)
        ay = abs(y)
        qx = ax - a
        qy = ay - b
        if ax > a and ay > b:
            sx = 1.0 if x >= 0.0 else -1.0
            sy = 1.0 if y >= 0.0 else -1.0
            rx = x - sx * a
            ry = y - sy * b
            n = max(math.hypot(rx, ry), 1e-12)
            return (rx / n, ry / n)
        if qx > qy:
            return (1.0 if x >= 0.0 else -1.0, 0.0)
        return (0.0, 1.0 if y >= 0.0 else -1.0)


# ---------------------------------------------------------------------------
# Configuration & Output Dataclasses
# ---------------------------------------------------------------------------


@dataclass
class SimConfig:
    rink_preset: str = "NHL_200X85FT"
    custom_rink: dict[str, Any] | None = None
    mount_location: str = "STERNUM_POCKET"
    player_height_cm: int = 183
    session_id: int = 1001
    device_id: int = 0x00005CC800010203
    start_utc_ms: int = 1763852500000
    imu_odr_hz: int = 200
    high_g_odr_hz: int = 3200
    high_g_mode: str = "IMPACT_BURST"  # "IMPACT_BURST" or "CONTINUOUS"
    mag_odr_hz: int = 50
    baro_odr_hz: int = 25
    duration_s: float = 140.0
    num_shifts: int = 2
    start_mode: str = "CENTER_ICE_CAL"  # "CENTER_ICE_CAL" or "BENCH_START"
    seed: int = 42
    scenario: str = "full_game"
    cal_block_overrides: dict[str, Any] | None = None
    enable_imperfections: bool = True
    output_gt_hz: int = 50


@dataclass
class SimResult:
    hks_bytes: bytes
    ground_truth: dict[str, Any]


# ---------------------------------------------------------------------------
# Quaternion & Vector Helpers (Hamilton scalar-first [qw, qx, qy, qz])
# ---------------------------------------------------------------------------


def _euler_zyx_to_quat_batch(psi: np.ndarray, theta: np.ndarray, phi: np.ndarray) -> np.ndarray:
    """Vectorized ZYX Euler (yaw psi, pitch theta, roll phi) -> Hamilton [qw, qx, qy, qz]."""
    cy = np.cos(0.5 * psi)
    sy = np.sin(0.5 * psi)
    cp = np.cos(0.5 * theta)
    sp = np.sin(0.5 * theta)
    cr = np.cos(0.5 * phi)
    sr = np.sin(0.5 * phi)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    q = np.stack([qw, qx, qy, qz], axis=-1)
    q /= np.linalg.norm(q, axis=-1, keepdims=True)
    return q


def _quat_mul_batch(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1[..., 0], q1[..., 1], q1[..., 2], q1[..., 3]
    w2, x2, y2, z2 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]
    return np.stack(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        axis=-1,
    )


def _quat_conj_batch(q: np.ndarray) -> np.ndarray:
    out = q.copy()
    out[..., 1:] *= -1.0
    return out


def _quat_to_rotmat_batch(q: np.ndarray) -> np.ndarray:
    """Active Body->World rotation matrices R(q) of shape (N, 3, 3)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((q.shape[0], 3, 3), dtype=np.float64)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    R[:, 0, 0] = 1.0 - 2.0 * (yy + zz)
    R[:, 0, 1] = 2.0 * (xy - wz)
    R[:, 0, 2] = 2.0 * (xz + wy)
    R[:, 1, 0] = 2.0 * (xy + wz)
    R[:, 1, 1] = 1.0 - 2.0 * (xx + zz)
    R[:, 1, 2] = 2.0 * (yz - wx)
    R[:, 2, 0] = 2.0 * (xz - wy)
    R[:, 2, 1] = 2.0 * (yz + wx)
    R[:, 2, 2] = 1.0 - 2.0 * (xx + yy)
    return R


def _quat_delta_to_omega(q0: np.ndarray, q1: np.ndarray, dt: float) -> np.ndarray:
    dq = _quat_mul_batch(_quat_conj_batch(q0), q1)
    sign = np.where(dq[:, 0] < 0.0, -1.0, 1.0)[:, None]
    dq = dq * sign
    v = dq[:, 1:4]
    v_norm = np.linalg.norm(v, axis=1)
    alpha = 2.0 * np.arctan2(v_norm, np.clip(dq[:, 0], -1.0, 1.0))
    scale = np.where(v_norm > 1e-12, alpha / (dt * np.maximum(v_norm, 1e-12)), 0.0)
    return v * scale[:, None]


# ---------------------------------------------------------------------------
# Quintic Hermite & Smooth Segment Builders
# ---------------------------------------------------------------------------


def _quintic_hermite_eval(
    p0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    p1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
    T: float,
    tau: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate C^2 quintic Hermite spline p(tau), v(tau), a(tau) on tau in [0, T]."""
    s = np.clip(tau / T, 0.0, 1.0)[:, None]
    s2 = s * s
    s3 = s2 * s
    s4 = s3 * s
    s5 = s4 * s

    # Position basis
    h0 = 1.0 - 10.0 * s3 + 15.0 * s4 - 6.0 * s5
    h1 = (s - 6.0 * s3 + 8.0 * s4 - 3.0 * s5) * T
    h2 = (0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5) * (T * T)
    h3 = 10.0 * s3 - 15.0 * s4 + 6.0 * s5
    h4 = (-4.0 * s3 + 7.0 * s4 - 3.0 * s5) * T
    h5 = (0.5 * s3 - 1.0 * s4 + 0.5 * s5) * (T * T)

    # First derivative w.r.t. tau
    dh0 = (-30.0 * s2 + 60.0 * s3 - 30.0 * s4) / T
    dh1 = 1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4
    dh2 = (s - 4.5 * s2 + 6.0 * s3 - 2.5 * s4) * T
    dh3 = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T
    dh4 = -12.0 * s2 + 28.0 * s3 - 15.0 * s4
    dh5 = (1.5 * s2 - 4.0 * s3 + 2.5 * s4) * T

    # Second derivative w.r.t. tau
    ddh0 = (-60.0 * s + 180.0 * s2 - 120.0 * s3) / (T * T)
    ddh1 = (-36.0 * s + 96.0 * s2 - 60.0 * s3) / T
    ddh2 = 1.0 - 9.0 * s + 18.0 * s2 - 10.0 * s3
    ddh3 = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (T * T)
    ddh4 = (-24.0 * s + 84.0 * s2 - 60.0 * s3) / T
    ddh5 = (3.0 * s - 12.0 * s2 + 10.0 * s3)

    p = h0 * p0 + h1 * v0 + h2 * a0 + h3 * p1 + h4 * v1 + h5 * a1
    v = dh0 * p0 + dh1 * v0 + dh2 * a0 + dh3 * p1 + dh4 * v1 + dh5 * a1
    a = ddh0 * p0 + ddh1 * v0 + ddh2 * a0 + ddh3 * p1 + ddh4 * v1 + ddh5 * a1
    return p, v, a


def _hann_pulse_eval(
    dv: np.ndarray,
    T_imp: float,
    tau: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Analytical position, velocity, and acceleration of a Hann impulse over [0, T_imp]."""
    u = np.clip(tau / T_imp, 0.0, 1.0)
    ang = 2.0 * math.pi * u
    a_s = (1.0 - np.cos(ang)) / T_imp
    v_s = u - np.sin(ang) / (2.0 * math.pi)
    p_s = T_imp * (0.5 * u * u - (1.0 - np.cos(ang)) / (4.0 * math.pi * math.pi))
    return p_s[:, None] * dv[None, :], v_s[:, None] * dv[None, :], a_s[:, None] * dv[None, :]


def _smooth_window(tau: np.ndarray, T: float, edge: float = 0.6) -> np.ndarray:
    """C^2 smooth Tukey window in [0, 1] fading in/out at segment ends."""
    e = min(edge, 0.45 * T)
    if e <= 1e-6:
        return np.ones_like(tau)
    w = np.ones_like(tau)
    m_in = tau < e
    m_out = tau > (T - e)
    s_in = tau[m_in] / e
    s_out = (T - tau[m_out]) / e
    w[m_in] = 0.5 * (1.0 - np.cos(math.pi * s_in))
    w[m_out] = 0.5 * (1.0 - np.cos(math.pi * s_out))
    return w


def _smoothstep5(u: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Quintic smoothstep s(u)=10u^3-15u^4+6u^5 and first/second derivatives w.r.t u."""
    uc = np.clip(u, 0.0, 1.0)
    s = 10.0 * uc**3 - 15.0 * uc**4 + 6.0 * uc**5
    ds = 30.0 * uc**2 - 60.0 * uc**3 + 30.0 * uc**4
    dds = 60.0 * uc - 180.0 * uc**2 + 120.0 * uc**3
    return s, ds, dds


# ---------------------------------------------------------------------------
# Magnetic Field & Calibration Helpers
# ---------------------------------------------------------------------------


def _build_calibration_block(cfg: SimConfig) -> CalibrationBlock:
    rng = np.random.default_rng(cfg.seed + 101)
    if cfg.enable_imperfections:
        m_a = (
            (1.0, 0.0018, -0.0012),
            (-0.0015, 0.9992, 0.0011),
            (0.0010, -0.0009, 1.0008),
        )
        b_a = (0.012, -0.008, 0.015)
        m_g = (
            (1.0006, 0.0008, -0.0005),
            (-0.0007, 0.9996, 0.0006),
            (0.0004, -0.0005, 1.0004),
        )
        b_g = (0.0011, -0.0008, 0.0005)
        b_hg = (0.15, -0.10, 0.05)
        h_m = (12.4, -5.8, 3.1)
        s_m = (
            (1.02, 0.01, 0.0),
            (0.01, 0.98, 0.0),
            (0.0, 0.0, 1.0),
        )
    else:
        _ = rng
        m_a = IDENTITY_3X3
        b_a = (0.0, 0.0, 0.0)
        m_g = IDENTITY_3X3
        b_g = (0.0, 0.0, 0.0)
        b_hg = (0.0, 0.0, 0.0)
        h_m = (0.0, 0.0, 0.0)
        s_m = IDENTITY_3X3

    kwargs: dict[str, Any] = {
        "cal_version": 1,
        "flags": 0x000F,
        "cal_utc_ms": max(0, cfg.start_utc_ms - 100_000),
        "sensor_to_body_rotation_matrix": CANONICAL_R_BS,
        "accel_misalign_scale": m_a,
        "accel_bias_mps2": b_a,
        "gyro_misalign_scale": m_g,
        "gyro_bias_rads": b_g,
        "high_g_to_body_rotation_matrix": CANONICAL_R_BS,
        "high_g_misalign_scale": IDENTITY_3X3,
        "high_g_bias_mps2": b_hg,
        "mag_hard_iron_ut": h_m,
        "mag_soft_iron_matrix": s_m,
        "baseline_mag_norm_ut": 51.2,
        "baseline_mag_dip_deg": 66.4,
    }
    if cfg.cal_block_overrides:
        kwargs.update(cfg.cal_block_overrides)

    cal = CalibrationBlock(**kwargs)
    crc = cal.compute_crc32()
    return CalibrationBlock(**{**kwargs, "cal_crc32": crc})


def _rink_magnetic_field_world(
    x: np.ndarray,
    y: np.ndarray,
    rink: RinkGeometry,
    *,
    b0_ut: float = 51.2,
    dip_deg: float = 66.4,
    az_deg: float = 22.0,
    enable_anomalies: bool = True,
) -> np.ndarray:
    """Compute world magnetic flux vector B^W(x, y) in uT with rink steel anomalies."""
    dip = math.radians(dip_deg)
    az = math.radians(az_deg)
    bx0 = b0_ut * math.cos(dip) * math.cos(az)
    by0 = b0_ut * math.cos(dip) * math.sin(az)
    bz0 = -b0_ut * math.sin(dip)

    B = np.empty((x.shape[0], 3), dtype=np.float64)
    B[:, 0] = bx0
    B[:, 1] = by0
    B[:, 2] = bz0
    if not enable_anomalies:
        return B

    # 1. Dasher-board steel rebar anomaly within 1.5 m of boards
    a = rink.length_m / 2.0 - rink.corner_radius_m
    b = rink.width_m / 2.0 - rink.corner_radius_m
    qx = np.abs(x) - a
    qy = np.abs(y) - b
    phi = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0)) + np.minimum(np.maximum(qx, qy), 0.0) - rink.corner_radius_m
    d_wall = np.maximum(-phi, 0.0)
    rebar_mask = d_wall < 1.5
    if np.any(rebar_mask):
        amp = 24.0 * np.exp(-d_wall[rebar_mask] / 0.55)
        sx = np.sign(x[rebar_mask])
        sy = np.sign(y[rebar_mask])
        B[rebar_mask, 0] += amp * 0.6 * sx
        B[rebar_mask, 1] += amp * 0.7 * sy
        B[rebar_mask, 2] += amp * 0.9

    # 2. Zamboni door dipole & goal frame steel dipoles
    zamb_x = -rink.length_m / 2.0 + 1.0
    zamb_y = -rink.width_m / 2.0 + 1.0
    goal_x = rink.length_m / 2.0 - rink.goal_line_from_boards_m
    dipoles = (
        (zamb_x, zamb_y, 75.0),
        (goal_x, 0.0, 55.0),
        (-goal_x, 0.0, 55.0),
    )
    for dx0, dy0, strength in dipoles:
        rx = x - dx0
        ry = y - dy0
        r2 = rx * rx + ry * ry + 0.36
        inv_r3 = strength / (r2 ** 1.5)
        B[:, 0] += inv_r3 * (0.4 * rx + 0.3)
        B[:, 1] += inv_r3 * (0.5 * ry - 0.2)
        B[:, 2] += inv_r3 * 0.85
    return B


# ---------------------------------------------------------------------------
# Trajectory & Kinematics Synthesis (3200 Hz Master Grid)
# ---------------------------------------------------------------------------


def _build_shift_trajectory_3200hz(
    rink: RinkGeometry,
    fs: int,
    start_xy: np.ndarray,
    start_yaw: float,
    start_pitch: float,
    start_roll: float,
    shift_dur: float,
    shift_idx: int,
    include_body_check: bool,
    include_board_bounce: bool,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Synthesize one complete on-ice shift at 3200 Hz returning:
    p(N,3), v(N,3), a(N,3), yaw(N), pitch(N), roll(N), states(N,int), z_floor(N).
    State enum: 0=UNKNOWN, 1=BENCH, 2=GLIDING, 3=STRIDING, 4=STOPPING, 5=IMPACT
    """
    dt = 1.0 / fs
    n_tot = int(round(shift_dur * fs))
    half_l = rink.length_m / 2.0
    half_w = rink.width_m / 2.0
    gate_xy = np.array(rink.bench_pose_xy, dtype=np.float64)

    # Scale interior waypoints safely to any rink preset
    sx = (half_l - rink.corner_radius_m) / (60.96 / 2.0 - 8.5344)
    sy = (half_w - rink.corner_radius_m) / (25.908 / 2.0 - 8.5344)

    p_list: list[np.ndarray] = []
    v_list: list[np.ndarray] = []
    a_list: list[np.ndarray] = []
    yaw_list: list[np.ndarray] = []
    pitch_list: list[np.ndarray] = []
    roll_list: list[np.ndarray] = []
    state_list: list[np.ndarray] = []

    cur_p = np.array([start_xy[0], start_xy[1], 0.0], dtype=np.float64)
    cur_v = np.zeros(3, dtype=np.float64)
    cur_a = np.zeros(3, dtype=np.float64)
    cur_yaw = float(start_yaw)
    cur_yaw_rate = 0.0
    cur_pitch = float(start_pitch)
    cur_roll = float(start_roll)
    stride_phase = 0.0

    def append_hermite(
        target_xy: tuple[float, float],
        target_vxy: tuple[float, float],
        dur: float,
        mode: str,
        *,
        target_yaw_override: float | None = None,
        add_stride_sway: bool = True,
    ) -> None:
        nonlocal cur_p, cur_v, cur_a, cur_yaw, cur_yaw_rate, cur_pitch, cur_roll, stride_phase
        n = max(1, int(round(dur * fs)))
        tau = np.arange(1, n + 1, dtype=np.float64) * dt
        T = n * dt
        p1 = np.array([target_xy[0], target_xy[1], 0.0], dtype=np.float64)
        v1 = np.array([target_vxy[0], target_vxy[1], 0.0], dtype=np.float64)
        a1 = np.zeros(3, dtype=np.float64)
        p_seg, v_seg, a_seg = _quintic_hermite_eval(cur_p, cur_v, cur_a, p1, v1, a1, T, tau)

        # C^2-smooth heading tracking on velocity with bounded angular rate (eliminates corners & gyro saturation)
        spd = np.hypot(v_seg[:, 0], v_seg[:, 1])
        stride_sub = max(1, fs // 400)  # 400 Hz ODE subgrid -> exact C^1 cubic Hermite upsample
        sub_idx = np.arange(0, n, stride_sub)
        if sub_idx[-1] != n - 1:
            sub_idx = np.append(sub_idx, n - 1)
        tau_sub = tau[sub_idx]
        spd_sub = spd[sub_idx]
        vx_sub = v_seg[sub_idx, 0]
        vy_sub = v_seg[sub_idx, 1]
        u_g = np.clip((spd_sub - 0.35) / 0.85, 0.0, 1.0)
        g_sub = 10.0 * u_g**3 - 15.0 * u_g**4 + 6.0 * u_g**5
        psi_vel_sub = np.arctan2(vy_sub, vx_sub)

        g_list = g_sub.tolist()
        pv_list = psi_vel_sub.tolist()
        t_list = tau_sub.tolist()
        m_sub = len(t_list)
        psi_sub = [0.0] * m_sub
        dpsi_sub = [0.0] * m_sub
        psi_c = cur_yaw
        dpsi_c = cur_yaw_rate
        t_prev = 0.0
        for idx_s in range(m_sub):
            dt_s = t_list[idx_s] - t_prev
            t_prev = t_list[idx_s]
            err = (pv_list[idx_s] - psi_c + math.pi) % (2.0 * math.pi) - math.pi
            dpsi_ref = 5.0 * g_list[idx_s] * err
            dpsi_ref = 5.5 * math.tanh(dpsi_ref / 5.5)
            ddpsi = (dpsi_ref - dpsi_c) / 0.12
            dpsi_c += ddpsi * dt_s
            psi_c += dpsi_c * dt_s
            psi_sub[idx_s] = psi_c
            dpsi_sub[idx_s] = dpsi_c

        psi_sub_arr = np.asarray(psi_sub, dtype=np.float64)
        dpsi_sub_arr = np.asarray(dpsi_sub, dtype=np.float64)

        # Cubic Hermite interpolation of (psi_sub, dpsi_sub) onto exact 3200 Hz tau grid
        # Prepend knot at tau=0: (cur_yaw, cur_yaw_rate)
        t_knots = np.concatenate(([0.0], tau_sub))
        y_knots = np.concatenate(([cur_yaw], psi_sub_arr))
        dy_knots = np.concatenate(([cur_yaw_rate], dpsi_sub_arr))
        k_int = np.clip(np.searchsorted(t_knots, tau, side="right") - 1, 0, len(t_knots) - 2)
        h_int = t_knots[k_int + 1] - t_knots[k_int]
        u_int = (tau - t_knots[k_int]) / h_int
        u2 = u_int * u_int
        u3 = u2 * u_int
        h00 = 2.0 * u3 - 3.0 * u2 + 1.0
        h10 = u3 - 2.0 * u2 + u_int
        h01 = -2.0 * u3 + 3.0 * u2
        h11 = u3 - u2
        yaw_seg = (
            h00 * y_knots[k_int]
            + h10 * h_int * dy_knots[k_int]
            + h01 * y_knots[k_int + 1]
            + h11 * h_int * dy_knots[k_int + 1]
        )
        dh00 = 6.0 * u2 - 6.0 * u_int
        dh10 = 3.0 * u2 - 4.0 * u_int + 1.0
        dh01 = -6.0 * u2 + 6.0 * u_int
        dh11 = 3.0 * u2 - 2.0 * u_int
        yaw_rate = (
            dh00 * y_knots[k_int]
            + dh10 * h_int * dy_knots[k_int]
            + dh01 * y_knots[k_int + 1]
            + dh11 * h_int * dy_knots[k_int + 1]
        ) / h_int

        if target_yaw_override is not None:
            s_yaw, ds_yaw, _ = _smoothstep5(tau / T)
            diff_end = (target_yaw_override - yaw_seg[-1] + math.pi) % (2.0 * math.pi) - math.pi
            yaw_seg = yaw_seg + s_yaw * diff_end
            yaw_rate = yaw_rate + (ds_yaw / T) * diff_end

        end_yaw_rate_base = float(yaw_rate[-1])
        s_att, _, _ = _smoothstep5(np.minimum(tau / min(0.45, 0.5 * T), 1.0))

        if mode == "STRIDING" and add_stride_sway:
            f_s = np.clip(0.85 + 0.16 * spd, 0.85, 2.45)
            dphi = 2.0 * math.pi * f_s * dt
            phi_s = stride_phase + np.cumsum(dphi)
            stride_phase = float(phi_s[-1] % (2.0 * math.pi))

            e_win = min(0.6, 0.25 * T)
            w_in, _, _ = _smoothstep5(tau / e_win)
            w_out, _, _ = _smoothstep5((T - tau) / e_win)
            w = w_in * w_out
            ay_amp = 0.052
            az_amp = 0.030

            ny_x = -np.sin(yaw_seg)
            ny_y = np.cos(yaw_seg)
            lat_p = w * ay_amp * np.sin(phi_s)
            z_p = w * az_amp * (np.sin(phi_s) ** 2)

            dp_sway = np.column_stack((lat_p * ny_x, lat_p * ny_y, z_p))
            dv_sway = np.gradient(dp_sway, dt, axis=0, edge_order=2)
            da_sway = np.gradient(dv_sway, dt, axis=0, edge_order=2)
            p_seg = p_seg + dp_sway
            v_seg = v_seg + dv_sway
            a_seg = a_seg + da_sway

            lean = math.radians(15.5) + w * math.radians(2.0) * np.sin(2.0 * phi_s)
            pitch_seg = cur_pitch + s_att * (lean - cur_pitch)
            roll_target = -np.arctan2(spd * yaw_rate, GRAVITY_MPS2) + w * math.radians(7.5) * np.sin(phi_s)
            roll_seg = cur_roll + s_att * (roll_target - cur_roll)
            yaw_seg = yaw_seg + w * math.radians(4.0) * np.sin(phi_s + 0.3)
            st_code = 3
        elif mode == "GLIDING":
            pitch_seg = cur_pitch + s_att * (math.radians(11.0) - cur_pitch)
            roll_target = -np.arctan2(spd * yaw_rate, GRAVITY_MPS2)
            roll_seg = cur_roll + s_att * (roll_target - cur_roll)
            st_code = 2
        else:
            pitch_seg = cur_pitch + s_att * (math.radians(6.0) - cur_pitch)
            roll_target = -np.arctan2(spd * yaw_rate, GRAVITY_MPS2)
            roll_seg = cur_roll + s_att * (roll_target - cur_roll)
            st_code = 4 if mode == "STOPPING" else 2

        p_list.append(p_seg)
        v_list.append(v_seg)
        a_list.append(a_seg)
        yaw_list.append(yaw_seg)
        pitch_list.append(pitch_seg)
        roll_list.append(roll_seg)
        state_list.append(np.full(n, st_code, dtype=np.int32))

        cur_p = p_seg[-1].copy()
        cur_v = v_seg[-1].copy()
        cur_a = a_seg[-1].copy()
        cur_yaw = float(yaw_seg[-1])
        cur_yaw_rate = end_yaw_rate_base
        cur_pitch = float(pitch_seg[-1])
        cur_roll = float(roll_seg[-1])

    def append_hockey_stop(dur: float = 1.2) -> None:
        nonlocal cur_p, cur_v, cur_a, cur_yaw, cur_yaw_rate, cur_pitch, cur_roll
        n = max(1, int(round(dur * fs)))
        tau = np.arange(1, n + 1, dtype=np.float64) * dt
        T = n * dt
        v0 = cur_v.copy()
        spd0 = float(np.hypot(v0[0], v0[1]))
        if spd0 < 0.5:
            append_hermite((cur_p[0], cur_p[1]), (0.0, 0.0), dur, "STOPPING", add_stride_sway=False)
            return
        p_s, v_s, a_s = _hann_pulse_eval(-v0, T, tau)
        p_seg = cur_p[None, :] + v0[None, :] * tau[:, None] + p_s
        v_seg = v0[None, :] + v_s
        a_seg = a_s

        # 90 deg yaw snap over first 0.6 s (peak |omega_z| ~ 300 deg/s >= 90 deg/s)
        T_yaw = min(0.6, T * 0.65)
        u_y = np.clip(tau / T_yaw, 0.0, 1.0)
        s_y = u_y - np.sin(2.0 * math.pi * u_y) / (2.0 * math.pi)
        yaw_seg = cur_yaw + (math.pi / 2.0) * s_y
        s_att, _, _ = _smoothstep5(np.minimum(tau / 0.25, 1.0))
        pitch_seg = cur_pitch + s_att * (math.radians(8.0) - cur_pitch)
        roll_target = -math.radians(12.0) * np.sin(math.pi * np.clip(tau / T, 0.0, 1.0))
        roll_seg = cur_roll + s_att * (roll_target - cur_roll)

        p_list.append(p_seg)
        v_list.append(v_seg)
        a_list.append(a_seg)
        yaw_list.append(yaw_seg)
        pitch_list.append(pitch_seg)
        roll_list.append(roll_seg)
        state_list.append(np.full(n, 4, dtype=np.int32))

        cur_p = p_seg[-1].copy()
        cur_v = np.zeros(3, dtype=np.float64)
        cur_a = np.zeros(3, dtype=np.float64)
        cur_yaw = float(yaw_seg[-1])
        cur_yaw_rate = 0.0
        cur_pitch = float(pitch_seg[-1])
        cur_roll = float(roll_seg[-1])

    def append_impact_pulse(dv_xy: np.ndarray, T_imp: float, whiplash_rad: float = 0.14) -> None:
        nonlocal cur_p, cur_v, cur_a, cur_yaw, cur_yaw_rate, cur_pitch, cur_roll
        n = max(1, int(round(T_imp * fs)))
        tau = np.arange(1, n + 1, dtype=np.float64) * dt
        T = n * dt
        dv = np.array([dv_xy[0], dv_xy[1], 0.0], dtype=np.float64)
        p_imp, v_imp, a_imp = _hann_pulse_eval(dv, T, tau)
        p_seg = cur_p[None, :] + cur_v[None, :] * tau[:, None] + p_imp
        v_seg = cur_v[None, :] + v_imp
        a_seg = a_imp

        # Smooth C^1 rotational whiplash (peak |omega| <= 12 rad/s, well within +-2000 dps = 34.9 rad/s)
        whip = whiplash_rad * (np.sin(math.pi * tau / T) ** 2)
        yaw_seg = cur_yaw + 0.35 * whip
        pitch_seg = cur_pitch + whip
        roll_seg = cur_roll - 0.5 * whip

        p_list.append(p_seg)
        v_list.append(v_seg)
        a_list.append(a_seg)
        yaw_list.append(yaw_seg)
        pitch_list.append(pitch_seg)
        roll_list.append(roll_seg)
        state_list.append(np.full(n, 5, dtype=np.int32))

        cur_p = p_seg[-1].copy()
        cur_v = v_seg[-1].copy()
        cur_a = np.zeros(3, dtype=np.float64)
        cur_yaw = float(yaw_seg[-1])
        cur_yaw_rate = 0.0
        cur_pitch = float(pitch_seg[-1])
        cur_roll = float(roll_seg[-1])

    # Build segments inside shift_dur
    t_used = 0.0
    ret_dur = min(6.0, 0.25 * shift_dur)
    avail = shift_dur - ret_dur

    if shift_idx % 2 == 0:
        segs = [
            ("STRIDING", (10.0 * sx, -1.5 * sy), (5.2, 0.8), 6.5),
            ("STRIDING", (19.5 * sx, 5.2 * sy), (-1.2, 4.6), 5.0),
            ("STRIDING", (10.0 * sx, 8.8 * sy), (-5.2, -0.4), 4.5),
            ("GLIDING", (-9.0 * sx, 5.5 * sy), (-3.8, -1.0), 5.0),
        ]
        for mode, wp, tv, d in segs:
            if t_used + d <= avail - 9.0:
                append_hermite(wp, tv, d, mode)
                t_used += d
        if t_used + 1.3 <= avail - 6.0:
            append_hockey_stop(1.2)
            t_used += 1.2
        if include_board_bounce and t_used + 7.0 <= avail:
            wall_y = -half_w + rink.r_torso_m + 0.08
            append_hermite((2.0 * sx, wall_y + 0.02), (4.0, -2.8), 3.6, "STRIDING")
            t_used += 3.6
            vn_in = -2.8
            vn_out = -0.45 * vn_in
            dv_bounce = np.array([-0.15 * 1.45 * abs(vn_in), vn_out - vn_in], dtype=np.float64)
            append_impact_pulse(dv_bounce, 0.030, whiplash_rad=0.10)
            t_used += 0.030
            append_hermite((5.0 * sx, -3.0 * sy), (1.0, 3.4), 3.2, "GLIDING", add_stride_sway=False)
            t_used += 3.2
    else:
        segs = [
            ("STRIDING", (-14.0 * sx, 4.0 * sy), (-5.8, -2.0), 5.5),
            ("STRIDING", (-20.5 * sx, -6.5 * sy), (1.2, -4.4), 5.0),
            ("GLIDING", (-7.0 * sx, -8.0 * sy), (5.5, 1.0), 4.5),
            ("STRIDING", (3.5 * sx, -4.0 * sy), (5.8, 1.8), 3.5),
        ]
        for mode, wp, tv, d in segs:
            if t_used + d <= avail - 10.0:
                append_hermite(wp, tv, d, mode)
                t_used += d
        if include_body_check and t_used + 5.5 <= avail:
            append_impact_pulse(np.array([-5.2, 1.4], dtype=np.float64), 0.028, whiplash_rad=0.14)
            t_used += 0.028
            append_hermite((2.5 * sx, 1.5 * sy), (3.2, 1.6), 2.8, "GLIDING", add_stride_sway=False)
            t_used += 2.8
        if t_used + 1.3 <= avail - 4.0:
            append_hockey_stop(1.2)
            t_used += 1.2
        if t_used + 5.0 <= avail:
            append_hermite((10.0 * sx, 4.2 * sy), (-1.8, 3.4), 4.8, "STRIDING")
            t_used += 4.8

    rem_mid = avail - t_used
    extra_wps = [
        ((-12.0 * sx, 6.0 * sy), (-4.5, -1.2)),
        ((-18.0 * sx, -4.5 * sy), (1.8, -4.2)),
        ((-4.0 * sx, -7.0 * sy), (5.0, 0.8)),
        ((11.0 * sx, -3.5 * sy), (4.2, 3.0)),
        ((15.0 * sx, 5.5 * sy), (-3.5, 2.5)),
    ]
    ew_idx = 0
    while rem_mid > 6.5:
        wp_e, tv_e = extra_wps[ew_idx % len(extra_wps)]
        ew_idx += 1
        append_hermite(wp_e, tv_e, 5.0, "STRIDING")
        t_used += 5.0
        rem_mid = avail - t_used
    if rem_mid > 0.5:
        append_hermite((-2.0 * sx, 5.0 * sy), (-3.0, 2.2), rem_mid, "STRIDING")
        t_used += rem_mid

    rem_ret = max(0.5, shift_dur - t_used)
    append_hermite(
        (float(gate_xy[0]), float(gate_xy[1])),
        (0.0, 0.0),
        rem_ret,
        "STRIDING" if rem_ret > 3.0 else "GLIDING",
        target_yaw_override=-rink.bench_facing_yaw_rad,
    )

    p = np.vstack(p_list)
    v = np.vstack(v_list)
    a = np.vstack(a_list)
    yaw = np.concatenate(yaw_list)
    pitch = np.concatenate(pitch_list)
    roll = np.concatenate(roll_list)
    st = np.concatenate(state_list)

    if p.shape[0] > n_tot:
        p, v, a = p[:n_tot], v[:n_tot], a[:n_tot]
        yaw, pitch, roll, st = yaw[:n_tot], pitch[:n_tot], roll[:n_tot], st[:n_tot]
    elif p.shape[0] < n_tot:
        pad = n_tot - p.shape[0]
        p = np.vstack([p, np.repeat(p[-1:], pad, axis=0)])
        v = np.vstack([v, np.zeros((pad, 3), dtype=np.float64)])
        a = np.vstack([a, np.zeros((pad, 3), dtype=np.float64)])
        yaw = np.concatenate([yaw, np.full(pad, yaw[-1])])
        pitch = np.concatenate([pitch, np.full(pad, pitch[-1])])
        roll = np.concatenate([roll, np.full(pad, roll[-1])])
        st = np.concatenate([st, np.full(pad, st[-1], dtype=np.int32)])

    a_r = rink.length_m / 2.0 - rink.corner_radius_m
    b_r = rink.width_m / 2.0 - rink.corner_radius_m
    qx_v = np.abs(p[:, 0]) - a_r
    qy_v = np.abs(p[:, 1]) - b_r
    phi_v = np.hypot(np.maximum(qx_v, 0.0), np.maximum(qy_v, 0.0)) + np.minimum(np.maximum(qx_v, qy_v), 0.0) - rink.corner_radius_m
    out_idx = np.where(phi_v > -rink.r_torso_m)[0]
    for i in out_idx:
        nx, ny = rink.outward_normal(float(p[i, 0]), float(p[i, 1]))
        p[i, 0] -= (float(phi_v[i]) + rink.r_torso_m + 1e-4) * nx
        p[i, 1] -= (float(phi_v[i]) + rink.r_torso_m + 1e-4) * ny

    z_floor = np.zeros(n_tot, dtype=np.float64)
    return p, v, a, yaw, pitch, roll, st, z_floor


def _build_bench_or_cal_3200hz(
    fs: int,
    dur: float,
    pos_xy: tuple[float, float],
    yaw_start: float,
    yaw_end: float,
    pitch_start: float,
    roll_start: float,
    z_floor_start: float,
    z_floor_end: float,
    state_code: int,
    *,
    is_static_cal: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    dt = 1.0 / fs
    n = max(1, int(round(dur * fs)))
    tau = np.arange(1, n + 1, dtype=np.float64) * dt

    p = np.zeros((n, 3), dtype=np.float64)
    v = np.zeros((n, 3), dtype=np.float64)
    a = np.zeros((n, 3), dtype=np.float64)
    p[:, 0] = pos_xy[0]
    p[:, 1] = pos_xy[1]

    if is_static_cal:
        yaw = np.full(n, yaw_end, dtype=np.float64)
        pitch = np.zeros(n, dtype=np.float64)
        roll = np.zeros(n, dtype=np.float64)
        z_floor = np.full(n, z_floor_start, dtype=np.float64)
    else:
        z_floor = np.full(n, z_floor_end, dtype=np.float64)
        t_step = min(1.5, dur * 0.5)
        m_step = tau <= t_step
        u = tau[m_step] / t_step
        s_turn, _, _ = _smoothstep5(u)
        if abs(z_floor_end - z_floor_start) > 0.05:
            dz = z_floor_end - z_floor_start
            _, ds, dds = _smoothstep5(u)
            z_floor[m_step] = z_floor_start + dz * s_turn
            vib_env = np.sin(math.pi * u) ** 2
            p_step_z = z_floor_start + dz * s_turn - 0.00015 * vib_env * np.sin(2.0 * math.pi * 15.0 * tau[m_step])
            p[m_step, 2] = p_step_z
            v[m_step, 2] = (dz / t_step) * ds
            a[m_step, 2] = (dz / (t_step * t_step)) * dds + 1.33 * vib_env * np.sin(2.0 * math.pi * 15.0 * tau[m_step])
            if not np.all(m_step):
                p[~m_step, 2] = z_floor_end

        # Smooth yaw/pitch/roll transition onto bench (zero gyro spikes!)
        dyaw = (yaw_end - yaw_start + math.pi) % (2.0 * math.pi) - math.pi
        yaw = np.full(n, yaw_start + dyaw, dtype=np.float64)
        yaw[m_step] = yaw_start + dyaw * s_turn

        pitch = np.full(n, math.radians(2.5), dtype=np.float64)
        pitch[m_step] = pitch_start + (math.radians(2.5) - pitch_start) * s_turn
        breath_mask = tau > t_step
        if np.any(breath_mask):
            b_ramp, _, _ = _smoothstep5((tau[breath_mask] - t_step) / 0.5)
            pitch[breath_mask] += b_ramp * math.radians(0.4) * np.sin(2.0 * math.pi * 0.25 * (tau[breath_mask] - t_step))

        roll = np.zeros(n, dtype=np.float64)
        roll[m_step] = roll_start * (1.0 - s_turn)

    st = np.full(n, state_code, dtype=np.int32)
    return p, v, a, yaw, pitch, roll, st, z_floor


# ---------------------------------------------------------------------------
# Public Simulation API
# ---------------------------------------------------------------------------


_STATE_NAMES = ("UNKNOWN", "BENCH", "GLIDING", "STRIDING", "STOPPING", "IMPACT")


def simulate_session(config: SimConfig | None = None) -> SimResult:
    """Run a 3200 Hz 6-DOF hockey rink simulation and return (.hks bytes, ground_truth)."""
    cfg = config or SimConfig()
    rng = np.random.default_rng(cfg.seed)
    rink = RinkGeometry.from_preset(cfg.rink_preset, cfg.custom_rink)
    cal = _build_calibration_block(cfg)

    if cfg.duration_s > 600.0 and cfg.high_g_mode == "IMPACT_BURST" and cfg.imu_odr_hz <= 400:
        fs_master = 400
    else:
        fs_master = 3200
    dt_master = 1.0 / fs_master

    # Construct timeline segments
    segments_p: list[np.ndarray] = []
    segments_v: list[np.ndarray] = []
    segments_a: list[np.ndarray] = []
    segments_yaw: list[np.ndarray] = []
    segments_pitch: list[np.ndarray] = []
    segments_roll: list[np.ndarray] = []
    segments_st: list[np.ndarray] = []
    segments_zf: list[np.ndarray] = []
    gt_shifts_meta: list[dict[str, Any]] = []

    bench_xy = rink.bench_pose_xy
    bench_yaw = rink.bench_facing_yaw_rad

    # Determine shift and bench intervals
    if cfg.num_shifts == 2 and abs(cfg.duration_s - 140.0) < 1e-3 and cfg.start_mode == "CENTER_ICE_CAL":
        # Exact canonical 140.0 s timeline matching INTERFACE_CONTRACT.json (Shift 1: 5.0..48.5 s)
        plan: list[tuple[str, float]] = [
            ("CAL_ICE", 5.0),
            ("SHIFT_0", 43.5),
            ("BENCH_1", 26.5),
            ("SHIFT_1", 45.0),
            ("BENCH_2", 20.0),
        ]
    else:
        total_d = max(25.0, float(cfg.duration_s))
        pre_d = 5.0
        rem = total_d - pre_d
        n_sh = max(1, int(cfg.num_shifts))
        slot = rem / n_sh
        plan = [("BENCH_0" if cfg.start_mode == "BENCH_START" else "CAL_ICE", pre_d)]
        for idx in range(n_sh):
            sh_d = min(45.0, max(10.0, slot * 0.65))
            bn_d = max(6.0, slot - sh_d)
            plan.append((f"SHIFT_{idx}", sh_d))
            plan.append((f"BENCH_{idx + 1}", bn_d))

    t_cursor = 0.0
    prev_was_bench = cfg.start_mode == "BENCH_START"
    cur_yaw = bench_yaw if prev_was_bench else 0.0
    cur_pitch = math.radians(2.5) if prev_was_bench else 0.0
    cur_roll = 0.0
    for label, dur in plan:
        if label == "CAL_ICE":
            seg = _build_bench_or_cal_3200hz(
                fs_master, dur, (0.0, 0.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, is_static_cal=True
            )
            prev_was_bench = False
        elif label.startswith("BENCH"):
            z0 = 0.30 if (label == "BENCH_0" or prev_was_bench) else 0.0
            seg = _build_bench_or_cal_3200hz(
                fs_master, dur, bench_xy, cur_yaw, bench_yaw, cur_pitch, cur_roll, z0, 0.30, 1, is_static_cal=False
            )
            prev_was_bench = True
        else:
            s_idx = int(label.split("_")[1])
            s_xy = np.array(bench_xy if prev_was_bench else (0.0, 0.0), dtype=np.float64)
            seg = _build_shift_trajectory_3200hz(
                rink,
                fs_master,
                s_xy,
                cur_yaw,
                cur_pitch,
                cur_roll,
                dur,
                s_idx,
                include_body_check=(s_idx % 2 == 1 or cfg.num_shifts == 1),
                include_board_bounce=(s_idx % 2 == 0),
                rng=rng,
            )
            # If coming from bench, prepend 1.5 s step-down (+0.30 -> 0.0 m) into z_floor
            if prev_was_bench:
                n_step = min(int(round(1.5 * fs_master)), seg[7].shape[0])
                u = np.arange(1, n_step + 1, dtype=np.float64) / n_step
                s_step, ds_step, dds_step = _smoothstep5(u)
                seg[7][:n_step] = 0.30 * (1.0 - s_step)
                seg[0][:n_step, 2] += 0.30 * (1.0 - s_step)
                seg[1][:n_step, 2] += (-0.30 / 1.5) * ds_step
                vib = np.sin(math.pi * u) ** 2
                seg[2][:n_step, 2] += (-0.30 / 2.25) * dds_step + 1.8 * vib * np.sin(2.0 * math.pi * 16.0 * (u * 1.5))

            p_s, v_s, a_s = seg[0], seg[1], seg[2]
            spd_s = np.hypot(v_s[:, 0], v_s[:, 1])
            dist_s = float(np.sum(np.hypot(np.diff(p_s[:, 0]), np.diff(p_s[:, 1]))))
            gt_shifts_meta.append(
                {
                    "shift_id": len(gt_shifts_meta) + 1,
                    "start_s": float(t_cursor),
                    "end_s": float(t_cursor + dur),
                    "duration_s": float(dur),
                    "distance_m": dist_s,
                    "peak_speed_mps": float(np.max(spd_s)),
                    "avg_speed_mps": float(np.mean(spd_s)),
                    "peak_accel_mps2": float(np.max(np.linalg.norm(a_s, axis=1))),
                }
            )
            prev_was_bench = False

        cur_yaw = float(seg[3][-1])
        cur_pitch = float(seg[4][-1])
        cur_roll = float(seg[5][-1])
        segments_p.append(seg[0])
        segments_v.append(seg[1])
        segments_a.append(seg[2])
        segments_yaw.append(seg[3])
        segments_pitch.append(seg[4])
        segments_roll.append(seg[5])
        segments_st.append(seg[6])
        segments_zf.append(seg[7])
        t_cursor += dur

    p_3200 = np.vstack(segments_p)
    v_3200 = np.vstack(segments_v)
    a_3200 = np.vstack(segments_a)
    yaw_3200 = np.concatenate(segments_yaw)
    pitch_3200 = np.concatenate(segments_pitch)
    roll_3200 = np.concatenate(segments_roll)
    st_3200 = np.concatenate(segments_st)
    zf_3200 = np.concatenate(segments_zf)

    n_master = p_3200.shape[0]
    # Align n_master to exact multiple of 64 (divisible by 3200/200=16 and 3200/50=64)
    n_master = (n_master // 64) * 64
    p_3200 = p_3200[:n_master]
    v_3200 = v_3200[:n_master]
    a_3200 = a_3200[:n_master]
    yaw_3200 = yaw_3200[:n_master]
    pitch_3200 = pitch_3200[:n_master]
    roll_3200 = roll_3200[:n_master]
    st_3200 = st_3200[:n_master]
    zf_3200 = zf_3200[:n_master]

    # Exact attitude quaternions & angular rates on 3200 Hz master grid
    q_3200 = _euler_zyx_to_quat_batch(yaw_3200, pitch_3200, roll_3200)
    # Enforce hemisphere continuity in O(N) vectorized
    dots = np.sum(q_3200[:-1] * q_3200[1:], axis=1)
    signs = np.cumprod(np.where(dots < 0.0, -1.0, 1.0))
    q_3200[1:] *= signs[:, None]

    R_WB_3200 = _quat_to_rotmat_batch(q_3200)
    omega_B_3200 = np.zeros((n_master, 3), dtype=np.float64)
    omega_B_3200[:-1] = _quat_delta_to_omega(q_3200[:-1], q_3200[1:], dt_master)
    omega_B_3200[-1] = omega_B_3200[-2]

    g_W = np.array([0.0, 0.0, -GRAVITY_MPS2], dtype=np.float64)
    f_W_3200 = a_3200 - g_W[None, :]
    f_B_3200 = np.einsum("nji,nj->ni", R_WB_3200, f_W_3200)

    # -----------------------------------------------------------------------
    # Sensor Frame `S` Inverse Calibration, Imperfections & Quantization
    # -----------------------------------------------------------------------
    R_BS = np.asarray(cal.sensor_to_body_rotation_matrix, dtype=np.float64)
    M_a_inv = np.linalg.inv(np.asarray(cal.accel_misalign_scale, dtype=np.float64))
    b_a = np.asarray(cal.accel_bias_mps2, dtype=np.float64)
    M_g_inv = np.linalg.inv(np.asarray(cal.gyro_misalign_scale, dtype=np.float64))
    b_g = np.asarray(cal.gyro_bias_rads, dtype=np.float64)
    R_BS_hg = np.asarray(cal.high_g_to_body_rotation_matrix, dtype=np.float64)
    M_hg_inv = np.linalg.inv(np.asarray(cal.high_g_misalign_scale, dtype=np.float64))
    b_hg = np.asarray(cal.high_g_bias_mps2, dtype=np.float64)
    S_m_inv = np.linalg.inv(np.asarray(cal.mag_soft_iron_matrix, dtype=np.float64))
    h_m = np.asarray(cal.mag_hard_iron_ut, dtype=np.float64)

    t_3200 = np.arange(n_master, dtype=np.float64) * dt_master
    temp_imu_c = 18.0 + 6.5 * (1.0 - np.exp(-t_3200 / 90.0))

    # Downsample to ICM-42688-P ODR (e.g. 200 Hz -> M_imu = 16)
    M_imu = fs_master // cfg.imu_odr_hz
    n_imu = n_master // M_imu
    dt_imu = 1.0 / cfg.imu_odr_hz

    f_B_imu = f_B_3200.reshape(n_imu, M_imu, 3).mean(axis=1)
    q_imu_starts = q_3200[0::M_imu][:n_imu]
    q_imu_ends = np.vstack([q_3200[M_imu::M_imu], q_3200[-1:]])[:n_imu]
    omega_B_imu = _quat_delta_to_omega(q_imu_starts, q_imu_ends, dt_imu)

    if cfg.enable_imperfections:
        ba_turn = rng.uniform(-0.025, 0.025, size=3)
        ba_rw = np.cumsum(rng.normal(0.0, 1e-4 * math.sqrt(dt_imu), size=(n_imu, 3)), axis=0)
        ba_temp = 0.0012 * (temp_imu_c[0::M_imu][:n_imu, None] - 20.0)
        eta_a = rng.normal(0.0, 0.012, size=(n_imu, 3))
        ba_res = ba_turn[None, :] + ba_rw + ba_temp

        bg_turn = rng.uniform(-0.0025, 0.0025, size=3)
        bg_rw = np.cumsum(rng.normal(0.0, 2e-5 * math.sqrt(dt_imu), size=(n_imu, 3)), axis=0)
        bg_temp = 8e-5 * (temp_imu_c[0::M_imu][:n_imu, None] - 20.0)
        eta_g = rng.normal(0.0, 0.0008, size=(n_imu, 3))
        bg_res = bg_turn[None, :] + bg_rw + bg_temp
    else:
        ba_res = np.zeros((n_imu, 3), dtype=np.float64)
        eta_a = np.zeros((n_imu, 3), dtype=np.float64)
        bg_res = np.zeros((n_imu, 3), dtype=np.float64)
        eta_g = np.zeros((n_imu, 3), dtype=np.float64)

    f_S_imu_raw = (f_B_imu @ R_BS + b_a[None, :] + ba_res + eta_a) @ M_a_inv.T
    w_S_imu_raw = (omega_B_imu @ R_BS + b_g[None, :] + bg_res + eta_g) @ M_g_inv.T

    ax_q = np.clip(np.rint(f_S_imu_raw[:, 0] / DEFAULT_ACCEL_SCALE), -32768, 32767).astype(np.int16)
    ay_q = np.clip(np.rint(f_S_imu_raw[:, 1] / DEFAULT_ACCEL_SCALE), -32768, 32767).astype(np.int16)
    az_q = np.clip(np.rint(f_S_imu_raw[:, 2] / DEFAULT_ACCEL_SCALE), -32768, 32767).astype(np.int16)
    gx_q = np.clip(np.rint(w_S_imu_raw[:, 0] / DEFAULT_GYRO_SCALE), -32768, 32767).astype(np.int16)
    gy_q = np.clip(np.rint(w_S_imu_raw[:, 1] / DEFAULT_GYRO_SCALE), -32768, 32767).astype(np.int16)
    gz_q = np.clip(np.rint(w_S_imu_raw[:, 2] / DEFAULT_GYRO_SCALE), -32768, 32767).astype(np.int16)

    # High-g ADXL372 (3200 Hz)
    eff_hg_rate = min(fs_master, max(400, cfg.high_g_odr_hz))
    M_hg = max(1, fs_master // eff_hg_rate)
    hg_upsample = max(1, max(400, cfg.high_g_odr_hz) // eff_hg_rate)
    f_B_hg = f_B_3200[::M_hg]
    n_hg = f_B_hg.shape[0]
    eta_hg = rng.normal(0.0, 1.4, size=(n_hg, 3)) if cfg.enable_imperfections else np.zeros((n_hg, 3))
    f_S_hg_raw = (f_B_hg @ R_BS_hg + b_hg[None, :] + eta_hg) @ M_hg_inv.T
    hg_x_q = np.clip(np.rint(f_S_hg_raw[:, 0] / DEFAULT_HIGH_G_SCALE), -2048, 2047).astype(np.int16)
    hg_y_q = np.clip(np.rint(f_S_hg_raw[:, 1] / DEFAULT_HIGH_G_SCALE), -2048, 2047).astype(np.int16)
    hg_z_q = np.clip(np.rint(f_S_hg_raw[:, 2] / DEFAULT_HIGH_G_SCALE), -2048, 2047).astype(np.int16)

    # AUX_ENV (50 Hz: Mag 50 Hz, Baro 25 Hz, PMIC 1 Hz)
    M_aux = fs_master // cfg.mag_odr_hz  # 64
    n_aux = n_master // M_aux
    p_aux = p_3200[0::M_aux][:n_aux]
    R_WB_aux = R_WB_3200[0::M_aux][:n_aux]
    zf_aux = zf_3200[0::M_aux][:n_aux]
    t_aux = t_3200[0::M_aux][:n_aux]

    B_W_aux = _rink_magnetic_field_world(
        p_aux[:, 0],
        p_aux[:, 1],
        rink,
        b0_ut=cal.baseline_mag_norm_ut,
        dip_deg=cal.baseline_mag_dip_deg,
        enable_anomalies=cfg.enable_imperfections,
    )
    B_B_aux = np.einsum("nji,nj->ni", R_WB_aux, B_W_aux)
    eta_m = rng.normal(0.0, 0.25, size=(n_aux, 3)) if cfg.enable_imperfections else np.zeros((n_aux, 3))
    m_S_raw = (B_B_aux @ R_BS) @ S_m_inv.T + h_m[None, :] + eta_m
    mx_q = np.clip(np.rint(m_S_raw[:, 0] / DEFAULT_MAG_SCALE), -131072, 131071).astype(np.int32)
    my_q = np.clip(np.rint(m_S_raw[:, 1] / DEFAULT_MAG_SCALE), -131072, 131071).astype(np.int32)
    mz_q = np.clip(np.rint(m_S_raw[:, 2] / DEFAULT_MAG_SCALE), -131072, 131071).astype(np.int32)

    # Baro (101325 - rho*g*z_floor + noise, 25 Hz fresh)
    rho_air = 1.2466
    eta_p = rng.normal(0.0, 0.07, size=n_aux) if cfg.enable_imperfections else np.zeros(n_aux)
    p_pa = 101325.0 - rho_air * GRAVITY_MPS2 * zf_aux + eta_p
    p_x64_all = np.clip(np.rint(p_pa * 64.0), 1920000, 8000000).astype(np.uint32)
    baro_step = max(1, cfg.mag_odr_hz // cfg.baro_odr_hz)
    baro_held = p_x64_all.copy()
    for i in range(n_aux):
        if i % baro_step != 0:
            baro_held[i] = baro_held[i - 1]

    # Battery discharge (1 Hz PMIC)
    soc_init = 95.0
    i_batt_ma = 1.249
    cap_derated_mah = 292.5
    soc_arr = np.clip(soc_init - 100.0 * (i_batt_ma * (t_aux / 3600.0)) / cap_derated_mah, 0.0, 100.0)
    vbatt_arr = np.clip(np.rint(3300.0 + 880.0 * (soc_arr / 100.0) - i_batt_ma * 0.36), 2500, 4400).astype(np.uint16)

    # -----------------------------------------------------------------------
    # Build .hks Packets Strictly Sorted by timestamp_us
    # -----------------------------------------------------------------------
    base_flags = int(FrameFlag.UTC_SYNCED)
    frames_with_ts: list[tuple[int, int, Any]] = []  # (timestamp_us, tie_breaker, packet)

    frames_with_ts.append(
        (
            0,
            0,
            EventMarkerPacket(
                timestamp_us=0,
                event_code="SESSION_START",
                severity=0,
                value_f32=0.0,
                detail_u64=cfg.start_utc_ms,
                flags=base_flags,
            ),
        )
    )

    # 0x02 IMU_BATCH (20 samples per batch @ 10 Hz)
    batch_sz = 20
    dt_imu_us = 1_000_000 // cfg.imu_odr_hz
    temp_imu_all = np.rint(temp_imu_c[0::M_imu][:n_imu] * 100.0).astype(np.int32).tolist()
    ax_l = ax_q.tolist()
    ay_l = ay_q.tolist()
    az_l = az_q.tolist()
    gx_l = gx_q.tolist()
    gy_l = gy_q.tolist()
    gz_l = gz_q.tolist()
    sat_a_per_sample = ((np.abs(ax_q) >= 32767) | (np.abs(ay_q) >= 32767) | (np.abs(az_q) >= 32767)).tolist()
    sat_g_per_sample = ((np.abs(gx_q) >= 32767) | (np.abs(gy_q) >= 32767) | (np.abs(gz_q) >= 32767)).tolist()
    for b_start in range(0, n_imu, batch_sz):
        b_end = min(b_start + batch_sz, n_imu)
        ts_us = b_start * dt_imu_us
        sat_a = any(sat_a_per_sample[b_start:b_end])
        sat_g = any(sat_g_per_sample[b_start:b_end])
        flg = base_flags | (int(FrameFlag.ACCEL_SAT) if sat_a else 0) | (int(FrameFlag.GYRO_SAT) if sat_g else 0)
        samples = tuple(
            ImuSample(ax_l[k], ay_l[k], az_l[k], gx_l[k], gy_l[k], gz_l[k])
            for k in range(b_start, b_end)
        )
        frames_with_ts.append(
            (
                ts_us,
                1,
                ImuBatchPacket(
                    timestamp_us=ts_us,
                    odr_hz=cfg.imu_odr_hz,
                    temp_c_x100=temp_imu_all[b_start],
                    samples=samples,
                    flags=flg,
                ),
            )
        )

    # 0x03 HIGH_G_BATCH (IMPACT_BURST > 12 g or CONTINUOUS)
    dt_hg_us = 1_000_000.0 / eff_hg_rate
    if cfg.high_g_odr_hz > 0:
        if cfg.high_g_mode == "CONTINUOUS":
            hg_batch_sz = 40
            for b_start in range(0, n_hg, hg_batch_sz):
                b_end = min(b_start + hg_batch_sz, n_hg)
                ts_us = int(round(b_start * dt_hg_us))
                samples_hg = tuple(
                    HighGSample(int(hg_x_q[k]), int(hg_y_q[k]), int(hg_z_q[k])) for k in range(b_start, b_end)
                )
                frames_with_ts.append(
                    (
                        ts_us,
                        2,
                        HighGBatchPacket(
                            timestamp_us=ts_us,
                            odr_hz=cfg.high_g_odr_hz,
                            trigger_mode="CONTINUOUS",
                            samples=samples_hg,
                            flags=base_flags | int(FrameFlag.HIGH_G_ACTIVE),
                        ),
                    )
                )
        else:
            # IMPACT_BURST (> 12 g threshold)
            thresh_mps2 = 12.0 * GRAVITY_MPS2
            linf = np.max(np.abs(f_S_hg_raw), axis=1)
            pre_win = max(1, 32 // hg_upsample)
            tot_win = max(1, 512 // hg_upsample)
            idx = 0
            while idx < n_hg:
                if linf[idx] > thresh_mps2:
                    b_start = max(0, idx - pre_win)
                    b_end = min(n_hg, b_start + tot_win)
                    ts_us = int(round(b_start * dt_hg_us))
                    samples_hg = tuple(
                        HighGSample(int(hg_x_q[k]), int(hg_y_q[k]), int(hg_z_q[k]))
                        for k in range(b_start, b_end)
                        for _ in range(hg_upsample)
                    )
                    peak_val = float(np.max(np.linalg.norm(f_S_hg_raw[b_start:b_end], axis=1)))
                    frames_with_ts.append(
                        (
                            ts_us,
                            2,
                            HighGBatchPacket(
                                timestamp_us=ts_us,
                                odr_hz=cfg.high_g_odr_hz,
                                trigger_mode="IMPACT_BURST",
                                samples=samples_hg,
                                flags=base_flags | int(FrameFlag.ACCEL_SAT) | int(FrameFlag.HIGH_G_ACTIVE),
                            ),
                        )
                    )
                    frames_with_ts.append(
                        (
                            ts_us,
                            3,
                            EventMarkerPacket(
                                timestamp_us=ts_us,
                                event_code="IMPACT_SPIKE",
                                severity=3,
                                value_f32=peak_val,
                                detail_u64=ts_us,
                                flags=base_flags | int(FrameFlag.HIGH_G_ACTIVE),
                            ),
                        )
                    )
                    idx = b_end
                else:
                    idx += 1

    # 0x04 AUX_ENV (50 Hz)
    dt_aux_us = 1_000_000 // cfg.mag_odr_hz
    temp_aux_l = np.rint(temp_imu_c[0::M_aux][:n_aux] * 100.0).astype(np.int32).tolist()
    b_norms_l = (np.abs(np.linalg.norm(B_W_aux, axis=1) - cal.baseline_mag_norm_ut) > 0.12 * cal.baseline_mag_norm_ut).tolist()
    mx_l = mx_q.tolist()
    my_l = my_q.tolist()
    mz_l = mz_q.tolist()
    baro_l = baro_held.tolist()
    vbatt_l = vbatt_arr.tolist()
    soc_l = np.rint(soc_arr * 10.0).astype(np.int32).tolist()
    mag_anom_flag = int(FrameFlag.MAG_ANOM)
    for i in range(n_aux):
        ts_us = i * dt_aux_us
        vflags = int(AuxValidityFlag.MAG_FRESH)
        if i % baro_step == 0:
            vflags |= int(AuxValidityFlag.BARO_FRESH)
        if i % cfg.mag_odr_hz == 0:
            vflags |= int(AuxValidityFlag.PMIC_FRESH)
        if i == 0:
            vflags |= int(AuxValidityFlag.MAG_SET_RESET)
        flg = base_flags | (mag_anom_flag if b_norms_l[i] else 0)
        frames_with_ts.append(
            (
                ts_us,
                4,
                AuxEnvPacket(
                    timestamp_us=ts_us,
                    mx_i32=mx_l[i],
                    my_i32=my_l[i],
                    mz_i32=mz_l[i],
                    pressure_pa_x64=baro_l[i],
                    baro_temp_c_x100=temp_aux_l[i],
                    vbatt_mv=vbatt_l[i],
                    batt_temp_c_x100=1500,
                    soc_pct_x10=soc_l[i],
                    pmic_status=0x40,
                    validity_flags=vflags,
                    flags=flg,
                ),
            )
        )

    end_ts_us = int(round(n_master * dt_master * 1_000_000.0))
    frames_with_ts.append(
        (
            end_ts_us,
            5,
            EventMarkerPacket(
                timestamp_us=end_ts_us,
                event_code="SESSION_END",
                severity=0,
                value_f32=0.0,
                detail_u64=end_ts_us,
                flags=base_flags,
            ),
        )
    )

    frames_with_ts.sort(key=lambda item: (item[0], item[1]))
    body_frames = [item[2] for item in frames_with_ts]

    hdr = SessionHeader(
        session_id=cfg.session_id,
        start_utc_ms=cfg.start_utc_ms,
        device_id=cfg.device_id,
        cal_crc32=cal.cal_crc32,
        imu_odr_hz=cfg.imu_odr_hz,
        high_g_odr_hz=cfg.high_g_odr_hz,
        mag_odr_hz=cfg.mag_odr_hz,
        baro_odr_hz=cfg.baro_odr_hz,
        accel_scale_mps2_per_lsb=DEFAULT_ACCEL_SCALE,
        gyro_scale_rads_per_lsb=DEFAULT_GYRO_SCALE,
        high_g_scale_mps2_per_lsb=DEFAULT_HIGH_G_SCALE,
        mag_scale_ut_per_lsb=DEFAULT_MAG_SCALE,
        rink_preset=cfg.rink_preset,
        mount_location=cfg.mount_location,
        player_height_cm=cfg.player_height_cm,
        bench_gate_xy_m=rink.bench_gate_xy_m,
        flags=base_flags,
        seq=0,
        timestamp_us=0,
    )

    end_utc_ms = cfg.start_utc_ms + int(round((n_master * dt_master) * 1000.0))
    hks_bytes = encode_hks_session(
        hdr,
        cal,
        body_frames,
        end_utc_ms=end_utc_ms,
        end_reason="BUTTON_STOP",
        footer_timestamp_us=end_ts_us,
    )

    # -----------------------------------------------------------------------
    # Ground Truth Assembly (Downsampled to output_gt_hz with bin-max accel)
    # -----------------------------------------------------------------------
    M_gt = fs_master // cfg.output_gt_hz
    n_gt = n_master // M_gt
    a_norm_3200 = np.linalg.norm(a_3200, axis=1)
    peak_rel = a_norm_3200[: n_gt * M_gt].reshape(n_gt, M_gt).argmax(axis=1)
    base_idx = np.arange(n_gt) * M_gt
    peak_idx_arr = base_idx + peak_rel

    p_gt = p_3200[base_idx]
    v_gt = v_3200[base_idx]
    q_gt = q_3200[base_idx]
    a_gt = a_3200[peak_idx_arr]
    st_gt = st_3200[peak_idx_arr]
    spd_gt = np.hypot(v_gt[:, 0], v_gt[:, 1])
    amag_gt = np.linalg.norm(a_gt, axis=1)
    t_gt_l = (base_idx * dt_master).tolist()
    p_gt_l = p_gt.tolist()
    v_gt_l = v_gt.tolist()
    q_gt_l = q_gt.tolist()
    a_gt_l = a_gt.tolist()
    st_gt_l = st_gt.tolist()
    spd_gt_l = spd_gt.tolist()
    amag_gt_l = amag_gt.tolist()

    gt_traj: list[dict[str, Any]] = [
        {
            "t": t_gt_l[k],
            "x": p_gt_l[k][0],
            "y": p_gt_l[k][1],
            "z": p_gt_l[k][2],
            "vx": v_gt_l[k][0],
            "vy": v_gt_l[k][1],
            "speed": spd_gt_l[k],
            "ax": a_gt_l[k][0],
            "ay": a_gt_l[k][1],
            "az": a_gt_l[k][2],
            "accel_mag": amag_gt_l[k],
            "q_WB": q_gt_l[k],
            "state": _STATE_NAMES[st_gt_l[k]],
        }
        for k in range(n_gt)
    ]

    total_dist = sum(s["distance_m"] for s in gt_shifts_meta)
    total_ice_s = sum(s["duration_s"] for s in gt_shifts_meta)
    peak_spd = max((s["peak_speed_mps"] for s in gt_shifts_meta), default=0.0)
    peak_acc = float(np.max(a_norm_3200))

    summary_dict = {
        "session_id": f"{cfg.session_id:016x}",
        "device_id": f"HKS-{cfg.device_id:016x}",
        "rink_preset": cfg.rink_preset,
        "gps_used": False,
        "external_anchors_used": False,
        "total_duration_s": float(n_master * dt_master),
        "total_ice_time_s": float(total_ice_s),
        "total_distance_m": float(total_dist),
        "shift_count": len(gt_shifts_meta),
        "peak_speed_mps": float(peak_spd),
        "peak_accel_mps2": float(peak_acc),
        "battery_used_pct": float(round(soc_init - float(soc_arr[-1]), 4)),
    }
    ground_truth = {
        "session_id": cfg.session_id,
        "rink_preset": cfg.rink_preset,
        "total_duration_s": float(n_master * dt_master),
        "total_ice_time_s": float(total_ice_s),
        "total_distance_m": float(total_dist),
        "shift_count": len(gt_shifts_meta),
        "peak_speed_mps": float(peak_spd),
        "peak_accel_mps2": float(peak_acc),
        "session_summary": summary_dict,
        "shifts": gt_shifts_meta,
        "trajectory": gt_traj,
    }

    return SimResult(hks_bytes=hks_bytes, ground_truth=ground_truth)
