"""Anchor-free 6-layer sensor fusion pipeline (15-state ES-EKF + batched RTS smoother).

Strictly isolated from ground truth: consumes ONLY .hks binary data (or DecodedHksSession)
and emits contract-compliant analytics_report JSON with zero GPS or external anchors.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
from scipy.signal import butter, sosfiltfilt

from .flash_codec import (
    CANONICAL_R_BS,
    GRAVITY_MPS2,
    IDENTITY_3X3,
    AuxValidityFlag,
    DecodedHksSession,
    FrameFlag,
    decode_hks_session,
)

# ---------------------------------------------------------------------------
# Rink Presets (from INTERFACE_CONTRACT.json, zero physics_sim dependency)
# ---------------------------------------------------------------------------

_RINK_PRESETS: dict[str, dict[str, Any]] = {
    "NHL_200X85FT": {
        "length_m": 60.96,
        "width_m": 25.908,
        "corner_radius_m": 8.5344,
        "default_bench_gate_xy_m": (-6.0, 12.954),
    },
    "IIHF_60X30M": {
        "length_m": 60.0,
        "width_m": 30.0,
        "corner_radius_m": 8.5,
        "default_bench_gate_xy_m": (-6.0, 15.0),
    },
    "CUSTOM": {
        "length_m": 56.0,
        "width_m": 26.0,
        "corner_radius_m": 7.5,
        "default_bench_gate_xy_m": (-5.0, 13.0),
    },
}


# ---------------------------------------------------------------------------
# Configuration Dataclasses
# ---------------------------------------------------------------------------


@dataclass
class FusionConfig:
    ekf_hz: int = 10
    output_hz: int = 50
    init_window_s: float = 5.0
    r_torso_m: float = 0.25
    # Process noise
    sigma_accel_icm: float = 0.10
    sigma_accel_striding: float = 0.18
    sigma_accel_hg: float = 2.5
    sigma_gyro: float = 0.005
    sigma_ba_rw: float = 2e-4
    sigma_bg_rw: float = 5e-5
    # Measurement noise
    sigma_zupt_v: float = 0.05
    sigma_zaru_w: float = 0.003
    sigma_grav: float = 0.25
    sigma_nhc_vz: float = 0.12
    sigma_nhc_z: float = 0.08
    sigma_nhc_vy_base: float = 0.65
    sigma_nhc_vy_turn: float = 3.5
    sigma_stride_v: float = 0.85
    sigma_bench_xy: float = 2.5
    sigma_mag_yaw_rad: float = 0.08
    # Gating & thresholds
    icm_sat_threshold_mps2: float = 14.5 * GRAVITY_MPS2
    mag_norm_tol: float = 0.12
    mag_dip_tol_deg: float = 8.0
    # Stride biomechanical speed: v_bio = s_h * (alpha*f_s + beta*rms_y + gamma)
    stride_alpha: float = 3.0
    stride_beta: float = 0.45
    stride_gamma: float = -1.9
    custom_rink: dict[str, Any] | None = None


@dataclass
class AblationConfig:
    use_high_g_handoff: bool = True
    use_zupt_zaru: bool = True
    use_nhc_vz: bool = True
    use_nhc_vy: bool = True
    use_stride_speed: bool = True
    use_rink_containment: bool = True
    use_bench_gate_reset: bool = True
    use_mag_gating: bool = True
    use_mag_updates: bool = True
    use_rts_smoother: bool = True


# ---------------------------------------------------------------------------
# SO(3) & Quaternion Math (Hamilton scalar-first [qw, qx, qy, qz])
# ---------------------------------------------------------------------------


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=np.float64,
    )


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def _quat_exp(rotvec: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-10:
        return _quat_normalize(np.array([1.0, 0.5 * rotvec[0], 0.5 * rotvec[1], 0.5 * rotvec[2]], dtype=np.float64))
    half = 0.5 * theta
    s = math.sin(half) / theta
    return np.array([math.cos(half), s * rotvec[0], s * rotvec[1], s * rotvec[2]], dtype=np.float64)


def _quat_log(q: np.ndarray) -> np.ndarray:
    qn = _quat_normalize(q)
    if qn[0] < 0.0:
        qn = -qn
    v = qn[1:4]
    vn = float(np.linalg.norm(v))
    if vn < 1e-12:
        return 2.0 * v
    ang = 2.0 * math.atan2(vn, float(np.clip(qn[0], -1.0, 1.0)))
    return v * (ang / vn)


def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _quat_from_two_vectors(u_from: np.ndarray, u_to: np.ndarray) -> np.ndarray:
    a = u_from / max(float(np.linalg.norm(u_from)), 1e-12)
    b = u_to / max(float(np.linalg.norm(u_to)), 1e-12)
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if dot > 1.0 - 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if dot < -1.0 + 1e-12:
        ortho = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, ortho)
        axis /= np.linalg.norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]], dtype=np.float64)
    c = np.cross(a, b)
    q = np.array([1.0 + dot, c[0], c[1], c[2]], dtype=np.float64)
    return _quat_normalize(q)


def _wrap_angle(ang: float) -> float:
    return (ang + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Rink Containment Geometry Helpers
# ---------------------------------------------------------------------------


def _rink_sdf_and_normal(
    x: float,
    y: float,
    length_m: float,
    width_m: float,
    corner_radius_m: float,
) -> tuple[float, float, float]:
    a = length_m / 2.0 - corner_radius_m
    b = width_m / 2.0 - corner_radius_m
    ax = abs(x)
    ay = abs(y)
    qx = ax - a
    qy = ay - b
    phi = math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0) - corner_radius_m
    if ax > a and ay > b:
        sx = 1.0 if x >= 0.0 else -1.0
        sy = 1.0 if y >= 0.0 else -1.0
        rx = x - sx * a
        ry = y - sy * b
        n = max(math.hypot(rx, ry), 1e-12)
        return phi, rx / n, ry / n
    if qx > qy:
        return phi, (1.0 if x >= 0.0 else -1.0), 0.0
    return phi, 0.0, (1.0 if y >= 0.0 else -1.0)


def _project_inside_rink(
    p_xy: np.ndarray,
    v_xy: np.ndarray,
    length_m: float,
    width_m: float,
    corner_radius_m: float,
    r_torso: float,
    e_wall: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, bool]:
    phi, nx, ny = _rink_sdf_and_normal(float(p_xy[0]), float(p_xy[1]), length_m, width_m, corner_radius_m)
    if phi <= -r_torso:
        return p_xy, v_xy, False
    pen = phi + r_torso
    p_new = p_xy.copy()
    p_new[0] -= pen * nx
    p_new[1] -= pen * ny
    v_new = v_xy.copy()
    vn = float(v_new[0] * nx + v_new[1] * ny)
    if vn > 0.0:
        v_new[0] -= (1.0 + e_wall) * vn * nx
        v_new[1] -= (1.0 + e_wall) * vn * ny
    return p_new, v_new, True


# ---------------------------------------------------------------------------
# Feature Extraction & Stride Cadence Estimator
# ---------------------------------------------------------------------------


def _compute_stride_features(
    t_imu: np.ndarray,
    f_y_B: np.ndarray,
    imu_odr_hz: int,
    player_height_cm: int,
    cfg: FusionConfig,
    t_ekf: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute stride-band RMS, dominant frequency, and biomechanical speed on t_ekf grid."""
    n_imu = f_y_B.shape[0]
    n_ekf = t_ekf.shape[0]
    rms_out = np.zeros(n_ekf, dtype=np.float64)
    freq_out = np.zeros(n_ekf, dtype=np.float64)
    v_bio_out = np.zeros(n_ekf, dtype=np.float64)
    valid_out = np.zeros(n_ekf, dtype=bool)

    if n_imu < max(32, imu_odr_hz):
        return rms_out, v_bio_out, valid_out

    sos = butter(2, [0.7, 3.0], btype="bandpass", fs=float(imu_odr_hz), output="sos")
    y_bp = sosfiltfilt(sos, f_y_B)

    win_len = int(round(1.5 * imu_odr_hz))
    if n_imu < win_len:
        return rms_out, v_bio_out, valid_out

    step = max(1, imu_odr_hz // 10)  # 10 Hz cadence analysis
    windows = sliding_window_view(y_bp, win_len)[::step]
    win_centers = t_imu[win_len // 2 : n_imu - (win_len - win_len // 2) + 1 : step]
    if windows.shape[0] == 0:
        return rms_out, v_bio_out, valid_out

    hann = np.hanning(win_len)
    n_fft = 1024
    spec = np.abs(np.fft.rfft(windows * hann[None, :], n=n_fft, axis=1))
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / float(imu_odr_hz))

    band_mask = (freqs >= 0.8) & (freqs <= 2.5)
    snr_mask = (freqs >= 0.5) & (freqs <= 4.0)
    band_idx = np.where(band_mask)[0]
    spec_band = spec[:, band_idx]
    peak_rel = np.argmax(spec_band, axis=1)
    peak_idx = band_idx[peak_rel]
    peak_amp = spec[np.arange(spec.shape[0]), peak_idx]
    med_amp = np.median(spec[:, snr_mask], axis=1) + 1e-9
    snr = peak_amp / med_amp

    # Parabolic peak refinement
    f_dom = freqs[peak_idx].copy()
    df = freqs[1] - freqs[0]
    for i in range(spec.shape[0]):
        idx = int(peak_idx[i])
        if 0 < idx < len(freqs) - 1:
            a_l, a_c, a_r = spec[i, idx - 1], spec[i, idx], spec[i, idx + 1]
            denom = a_l - 2.0 * a_c + a_r
            if abs(denom) > 1e-12:
                delta = 0.5 * (a_l - a_r) / denom
                f_dom[i] += float(np.clip(delta, -0.5, 0.5)) * df

    rms_win = np.sqrt(np.mean(windows * windows, axis=1))
    s_h = math.sqrt(float(player_height_cm) / 183.0) if player_height_cm > 0 else 1.0
    v_win = np.clip(
        s_h * (cfg.stride_alpha * f_dom + cfg.stride_beta * rms_win + cfg.stride_gamma),
        1.2,
        12.0,
    )
    ok_win = (snr >= 2.2) & (rms_win >= 0.55)

    # Nearest-neighbor map onto t_ekf
    idx_map = np.clip(np.searchsorted(win_centers, t_ekf, side="left"), 0, len(win_centers) - 1)
    prev_idx = np.clip(idx_map - 1, 0, len(win_centers) - 1)
    choose_prev = np.abs(t_ekf - win_centers[prev_idx]) < np.abs(win_centers[idx_map] - t_ekf)
    best = np.where(choose_prev, prev_idx, idx_map)

    rms_out = rms_win[best]
    freq_out = f_dom[best]
    v_bio_out = v_win[best]
    valid_out = ok_win[best]
    _ = freq_out
    return rms_out, v_bio_out, valid_out


# ---------------------------------------------------------------------------
# Main Anchor-Free Fusion Pipeline
# ---------------------------------------------------------------------------


def run_fusion(
    source: DecodedHksSession | bytes | bytearray | memoryview | Path | str,
    *,
    fusion_config: FusionConfig | None = None,
    ablation: AblationConfig | None = None,
) -> dict[str, Any]:
    """Run 6-layer anchor-free ES-EKF + RTS smoother on .hks telemetry and emit analytics_report."""
    cfg = fusion_config or FusionConfig()
    abl = ablation or AblationConfig()

    if isinstance(source, DecodedHksSession):
        sess = source
    elif isinstance(source, (Path, str)):
        sess = decode_hks_session(Path(source).read_bytes(), require_footer=True, verify_so3=True)
    else:
        sess = decode_hks_session(source, require_footer=True, verify_so3=True)

    if not sess.imu_batches:
        raise ValueError("Cannot run fusion on .hks session with zero IMU_BATCH frames")

    hdr = sess.header
    cal = sess.calibration
    preset_spec = dict(_RINK_PRESETS[hdr.rink_preset])
    if hdr.rink_preset == "CUSTOM" and cfg.custom_rink:
        preset_spec.update(cfg.custom_rink)

    rink_L = float(preset_spec["length_m"])
    rink_W = float(preset_spec["width_m"])
    rink_Rc = float(preset_spec["corner_radius_m"])
    gate_x = float(hdr.bench_gate_xy_m[0])
    gate_y = float(hdr.bench_gate_xy_m[1])
    sgn_gate_y = 1.0 if gate_y >= 0.0 else -1.0
    bench_pose_xy = np.array([gate_x, gate_y - sgn_gate_y * cfg.r_torso_m], dtype=np.float64)
    bench_yaw = -math.pi / 2.0 if gate_y >= 0.0 else math.pi / 2.0

    # Calibration matrices
    if cal is not None:
        R_BS = np.asarray(cal.sensor_to_body_rotation_matrix, dtype=np.float64)
        M_a = np.asarray(cal.accel_misalign_scale, dtype=np.float64)
        b_a_cal = np.asarray(cal.accel_bias_mps2, dtype=np.float64)
        M_g = np.asarray(cal.gyro_misalign_scale, dtype=np.float64)
        b_g_cal = np.asarray(cal.gyro_bias_rads, dtype=np.float64)
        R_BS_hg = np.asarray(cal.high_g_to_body_rotation_matrix, dtype=np.float64)
        M_hg = np.asarray(cal.high_g_misalign_scale, dtype=np.float64)
        b_hg_cal = np.asarray(cal.high_g_bias_mps2, dtype=np.float64)
        h_m = np.asarray(cal.mag_hard_iron_ut, dtype=np.float64)
        S_m = np.asarray(cal.mag_soft_iron_matrix, dtype=np.float64)
        B0_ut = float(cal.baseline_mag_norm_ut)
        dip0_deg = float(cal.baseline_mag_dip_deg)
    else:
        R_BS = np.asarray(CANONICAL_R_BS, dtype=np.float64)
        M_a = np.asarray(IDENTITY_3X3, dtype=np.float64)
        b_a_cal = np.zeros(3, dtype=np.float64)
        M_g = np.asarray(IDENTITY_3X3, dtype=np.float64)
        b_g_cal = np.zeros(3, dtype=np.float64)
        R_BS_hg = R_BS
        M_hg = np.asarray(IDENTITY_3X3, dtype=np.float64)
        b_hg_cal = np.zeros(3, dtype=np.float64)
        h_m = np.zeros(3, dtype=np.float64)
        S_m = np.asarray(IDENTITY_3X3, dtype=np.float64)
        B0_ut = 51.2
        dip0_deg = 66.4

    # -----------------------------------------------------------------------
    # 1. Unpack IMU (200 Hz), High-g (3200 Hz) & AUX_ENV (50 Hz) Streams
    # -----------------------------------------------------------------------
    imu_t_list: list[np.ndarray] = []
    imu_a_list: list[np.ndarray] = []
    imu_g_list: list[np.ndarray] = []
    imu_sat_flags: list[np.ndarray] = []

    for b in sess.imu_batches:
        ts, a_s, g_s = b.to_numpy(hdr.accel_scale_mps2_per_lsb, hdr.gyro_scale_rads_per_lsb)
        imu_t_list.append(ts)
        imu_a_list.append(a_s)
        imu_g_list.append(g_s)
        is_sat = (b.flags & int(FrameFlag.ACCEL_SAT)) != 0
        imu_sat_flags.append(np.full(ts.shape[0], is_sat, dtype=bool))

    t_imu = np.concatenate(imu_t_list)
    a_raw_S = np.vstack(imu_a_list)
    w_raw_S = np.vstack(imu_g_list)
    batch_sat_flag = np.concatenate(imu_sat_flags)

    order_imu = np.argsort(t_imu, kind="mergesort")
    t_imu = t_imu[order_imu]
    a_raw_S = a_raw_S[order_imu]
    w_raw_S = w_raw_S[order_imu]
    batch_sat_flag = batch_sat_flag[order_imu]

    t0_s = float(t_imu[0])
    t_rel = t_imu - t0_s

    a_cal_S = a_raw_S @ M_a.T - b_a_cal[None, :]
    w_cal_S = w_raw_S @ M_g.T - b_g_cal[None, :]
    f_icm_B = a_cal_S @ R_BS.T
    w_icm_B = w_cal_S @ R_BS.T

    # High-g ADXL372 (already >>4 unpacked in flash_codec)
    if sess.high_g_batches:
        hg_t_list: list[np.ndarray] = []
        hg_a_list: list[np.ndarray] = []
        for b in sess.high_g_batches:
            ts_hg, a_hg_s = b.to_numpy(hdr.high_g_scale_mps2_per_lsb)
            hg_t_list.append(ts_hg)
            hg_a_list.append(a_hg_s)
        t_hg = np.concatenate(hg_t_list) - t0_s
        a_hg_raw_S = np.vstack(hg_a_list)
        ord_hg = np.argsort(t_hg, kind="mergesort")
        t_hg = t_hg[ord_hg]
        a_hg_raw_S = a_hg_raw_S[ord_hg]
        f_hg_B = (a_hg_raw_S @ M_hg.T - b_hg_cal[None, :]) @ R_BS_hg.T
    else:
        t_hg = np.empty(0, dtype=np.float64)
        f_hg_B = np.empty((0, 3), dtype=np.float64)

    # Layer 1: Dual-IMU Saturation Handoff
    dt_imu = 1.0 / float(hdr.imu_odr_hz)
    linf_S = np.max(np.abs(a_raw_S), axis=1)
    sat_mask = (linf_S > cfg.icm_sat_threshold_mps2) | (batch_sat_flag & (linf_S > 14.0 * GRAVITY_MPS2))

    f_fused_B = f_icm_B.copy()
    f_peak_B = f_icm_B.copy()
    hg_active = np.zeros(t_rel.shape[0], dtype=bool)

    if abl.use_high_g_handoff and t_hg.shape[0] > 0:
        sat_indices = np.where(sat_mask)[0]
        for idx_n in sat_indices:
            t_lo = t_rel[idx_n] - 1e-7
            t_hi = t_rel[idx_n] + dt_imu - 1e-7
            i0 = int(np.searchsorted(t_hg, t_lo, side="left"))
            i1 = int(np.searchsorted(t_hg, t_hi, side="left"))
            if i1 <= i0:
                # Sparse ODR fallback (e.g. 400 Hz)
                i0 = max(0, int(np.searchsorted(t_hg, t_rel[idx_n], side="left")) - 1)
                i1 = min(t_hg.shape[0], i0 + 2)
            if i1 > i0:
                sl = f_hg_B[i0:i1]
                f_fused_B[idx_n] = np.mean(sl, axis=0)
                norms = np.linalg.norm(sl, axis=1)
                f_peak_B[idx_n] = sl[int(np.argmax(norms))]
                hg_active[idx_n] = True

    # Also capture peak high-g sample inside every EKF bin even if ICM didn't stay >14.5g across whole sample
    # (for accurate peak_accel_mps2 reporting)
    # AUX_ENV stream
    if sess.aux_env_packets:
        aux_t = np.array([p.timestamp_us * 1e-6 - t0_s for p in sess.aux_env_packets], dtype=np.float64)
        aux_vflags = np.array([p.validity_flags for p in sess.aux_env_packets], dtype=np.int32)
        aux_m_raw = np.array([[p.mx_i32, p.my_i32, p.mz_i32] for p in sess.aux_env_packets], dtype=np.float64) * hdr.mag_scale_ut_per_lsb
        aux_m_B = ((aux_m_raw - h_m[None, :]) @ S_m.T) @ R_BS.T
        aux_p_pa = np.array([p.pressure_pa for p in sess.aux_env_packets], dtype=np.float64)
        aux_soc = np.array([p.soc_pct for p in sess.aux_env_packets], dtype=np.float64)
        batt_used_pct = float(np.clip(aux_soc[0] - aux_soc[-1], 0.0, 100.0))
    else:
        aux_t = np.empty(0, dtype=np.float64)
        aux_vflags = np.empty(0, dtype=np.int32)
        aux_m_B = np.empty((0, 3), dtype=np.float64)
        aux_p_pa = np.empty(0, dtype=np.float64)
        batt_used_pct = 0.0

    # -----------------------------------------------------------------------
    # 2. EKF Step Grid & Pre-EKF Kinematic / Stride Features
    # -----------------------------------------------------------------------
    n_imu = t_rel.shape[0]
    ekf_hz = cfg.ekf_hz
    if n_imu > 360_000:
        # Memory-safe down-rate for multi-hour logs
        ekf_hz = min(ekf_hz, 10)
    substeps = max(1, hdr.imu_odr_hz // ekf_hz)
    n_ekf = max(1, n_imu // substeps)
    dt_ekf = float(substeps) * dt_imu
    t_ekf = t_rel[: n_ekf * substeps : substeps]

    # Per-EKF-step windows (0.4 s for ZUPT, 1.0 s for RMS)
    f_norm_imu = np.linalg.norm(f_icm_B, axis=1)
    w_norm_imu = np.linalg.norm(w_icm_B, axis=1)
    a_dyn_imu = np.abs(f_norm_imu - GRAVITY_MPS2)

    win_zupt = max(4, int(round(0.4 * hdr.imu_odr_hz)))
    win_rms = max(8, int(round(1.0 * hdr.imu_odr_hz)))

    c_arr = np.arange(n_ekf, dtype=np.int64) * substeps
    i0_r = np.maximum(0, c_arr - (win_rms // 2))
    i1_r = np.minimum(n_imu, i0_r + win_rms)
    len_r = np.maximum(1, i1_r - i0_r)

    cs_adyn2 = np.concatenate(([0.0], np.cumsum(a_dyn_imu * a_dyn_imu)))
    cs_wnorm2 = np.concatenate(([0.0], np.cumsum(w_norm_imu * w_norm_imu)))
    a_dyn_rms_ekf = np.sqrt(np.maximum(0.0, (cs_adyn2[i1_r] - cs_adyn2[i0_r]) / len_r))
    w_rms_ekf = np.sqrt(np.maximum(0.0, (cs_wnorm2[i1_r] - cs_wnorm2[i0_r]) / len_r))

    cs_wz = np.concatenate(([0.0], np.cumsum(w_icm_B[:, 2])))
    i1_wz = np.minimum(n_imu, c_arr + substeps)
    w_z_ekf = (cs_wz[i1_wz] - cs_wz[c_arr]) / np.maximum(1, i1_wz - c_arr)

    i0_z = np.maximum(0, c_arr - (win_zupt // 2))
    i1_z = np.minimum(n_imu, i0_z + win_zupt)
    len_z = np.maximum(1, i1_z - i0_z)
    cs_fn = np.concatenate(([0.0], np.cumsum(f_norm_imu)))
    cs_fn2 = np.concatenate(([0.0], np.cumsum(f_norm_imu * f_norm_imu)))
    mean_fn = (cs_fn[i1_z] - cs_fn[i0_z]) / len_z
    std_fn = np.sqrt(np.maximum(0.0, (cs_fn2[i1_z] - cs_fn2[i0_z]) / len_z - mean_fn * mean_fn))
    zupt_mask_ekf = std_fn < (0.04 * GRAVITY_MPS2)
    w_thresh_rad = math.radians(6.0)
    for k in np.where(zupt_mask_ekf)[0]:
        if float(np.max(w_norm_imu[int(i0_z[k]) : int(i1_z[k])])) >= w_thresh_rad:
            zupt_mask_ekf[k] = False

    stride_rms_ekf, v_bio_ekf, stride_ok_ekf = _compute_stride_features(
        t_rel, f_icm_B[:, 1], hdr.imu_odr_hz, hdr.player_height_cm, cfg, t_ekf
    )

    # Baro & Mag mapped onto EKF steps
    baro_ekf = np.full(n_ekf, 101325.0, dtype=np.float64)
    if aux_t.shape[0] > 0:
        baro_valid = (aux_vflags & int(AuxValidityFlag.BARO_FRESH)) != 0
        bt = aux_t[baro_valid] if np.any(baro_valid) else aux_t
        bp = aux_p_pa[baro_valid] if np.any(baro_valid) else aux_p_pa
        baro_ekf = np.interp(t_ekf, bt, bp)

    # -----------------------------------------------------------------------
    # 3. Static Initialization & Start Mode Detection (Zero Ground Truth)
    # -----------------------------------------------------------------------
    move_idx_arr = np.where((a_dyn_rms_ekf > 0.12 * GRAVITY_MPS2) | (w_rms_ekf > math.radians(15.0)))[0]
    first_move_k = int(move_idx_arr[0]) if move_idx_arr.size > 0 else n_ekf - 1
    t_move = float(t_ekf[first_move_k])
    init_end_k = max(1, min(first_move_k, int(round(cfg.init_window_s * ekf_hz))))
    init_end_imu = max(1, min(n_imu, init_end_k * substeps))

    f_init_mean = np.mean(f_icm_B[:init_end_imu], axis=0)
    w_init_mean_S = np.mean(w_cal_S[:init_end_imu], axis=0)
    w_init_rms = float(np.sqrt(np.mean(w_norm_imu[:init_end_imu] ** 2)))

    active_mask = a_dyn_rms_ekf > 0.12 * GRAVITY_MPS2
    P_ice_est = float(np.percentile(baro_ekf[active_mask], 75)) if np.any(active_mask) else float(np.median(baro_ekf))
    P_0 = float(np.median(baro_ekf[:init_end_k]))

    # Check baro step-down around t_move
    k_lo_step = max(0, first_move_k - int(round(2.5 * ekf_hz)))
    k_hi_step = min(n_ekf, first_move_k + int(round(2.5 * ekf_hz)))
    delta_p_move = float(np.median(baro_ekf[first_move_k:k_hi_step]) - np.median(baro_ekf[k_lo_step : max(k_lo_step + 1, first_move_k)]))

    if (P_ice_est - P_0) >= 2.0 or delta_p_move >= 2.0 or t_move > 8.0:
        start_mode = "BENCH_START"
    else:
        start_mode = "CENTER_ICE_CAL"

    # Pre-EKF macro-state (BENCH vs ON_ICE)
    # 0=CAL_UNKNOWN, 1=BENCH, 2=ON_ICE
    pre_macro = np.full(n_ekf, 2, dtype=np.int32)
    if start_mode == "CENTER_ICE_CAL":
        pre_macro[:first_move_k] = 0
        state_macro = 0
    else:
        pre_macro[:first_move_k] = 1
        state_macro = 1

    still_min_steps = int(round(5.0 * ekf_hz))
    skate_min_steps = int(round(1.5 * ekf_hz))
    still_run = 0
    skate_run = 0
    for k in range(first_move_k, n_ekf):
        is_still = (a_dyn_rms_ekf[k] < 0.055 * GRAVITY_MPS2) and (w_rms_ekf[k] < math.radians(8.5))
        is_skate = a_dyn_rms_ekf[k] > 0.14 * GRAVITY_MPS2
        if state_macro != 1:
            if is_still:
                still_run += 1
                if still_run >= still_min_steps:
                    back_k = max(first_move_k, k - still_run + 1)
                    pre_macro[back_k : k + 1] = 1
                    state_macro = 1
                    skate_run = 0
                else:
                    pre_macro[k] = 2
            else:
                still_run = 0
                pre_macro[k] = 2
        else:
            if is_skate:
                skate_run += 1
                if skate_run >= skate_min_steps:
                    back_k = max(first_move_k, k - skate_run + 1)
                    pre_macro[back_k : k + 1] = 2
                    state_macro = 2
                    still_run = 0
                else:
                    pre_macro[k] = 1
            else:
                skate_run = 0
                pre_macro[k] = 1

    # Initial state
    q_level = _quat_from_two_vectors(f_init_mean, np.array([0.0, 0.0, 1.0], dtype=np.float64))
    if start_mode == "BENCH_START":
        p_cur = np.array([bench_pose_xy[0], bench_pose_xy[1], 0.30], dtype=np.float64)
        psi0 = bench_yaw
    else:
        p_cur = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        psi0 = 0.0

    q_z0 = np.array([math.cos(0.5 * psi0), 0.0, 0.0, math.sin(0.5 * psi0)], dtype=np.float64)
    q_cur = _quat_normalize(_quat_mul(q_z0, q_level))
    v_cur = np.zeros(3, dtype=np.float64)
    ba_cur = np.zeros(3, dtype=np.float64)
    bg_cur = w_init_mean_S.copy() if w_init_rms < math.radians(6.0) else np.zeros(3, dtype=np.float64)

    P = np.diag(
        [
            0.25**2,
            0.25**2,
            0.05**2,
            0.05**2,
            0.05**2,
            0.05**2,
            0.03**2,
            0.03**2,
            0.05**2,
            0.03**2,
            0.03**2,
            0.03**2,
            0.002**2,
            0.002**2,
            0.002**2,
        ]
    ).astype(np.float64)

    # Lock World Magnetic Reference B_ref^W from static init (solving missing psi_mag in cal block)
    B_ref_W: np.ndarray | None = None
    if aux_t.shape[0] > 0:
        mag_fresh = (aux_vflags & int(AuxValidityFlag.MAG_FRESH)) != 0
        init_m_mask = mag_fresh & (aux_t <= float(t_ekf[init_end_k - 1]) + 0.1)
        if np.any(init_m_mask):
            m0_B = np.median(aux_m_B[init_m_mask], axis=0)
            m0_norm = float(np.linalg.norm(m0_B))
            if (1.0 - cfg.mag_norm_tol) * B0_ut <= m0_norm <= (1.0 + cfg.mag_norm_tol) * B0_ut:
                cand_W = _quat_to_rotmat(q_cur) @ m0_B
                cand_dip = math.degrees(math.atan2(-cand_W[2], math.hypot(cand_W[0], cand_W[1])))
                if abs(cand_dip - dip0_deg) < cfg.mag_dip_tol_deg:
                    B_ref_W = cand_W

    # Map mag samples onto EKF steps
    aux_step_map: dict[int, np.ndarray] = {}
    if aux_t.shape[0] > 0:
        for i in range(aux_t.shape[0]):
            if (aux_vflags[i] & int(AuxValidityFlag.MAG_FRESH)) != 0:
                k_step = int(round((aux_t[i] - float(t_ekf[0])) / dt_ekf))
                if 0 <= k_step < n_ekf:
                    aux_step_map[k_step] = aux_m_B[i]

    # -----------------------------------------------------------------------
    # 4. Forward 15-State ES-EKF Loop (200 Hz Midpoint + 50 Hz Scalar Joseph)
    # -----------------------------------------------------------------------
    g_W = np.array([0.0, 0.0, -GRAVITY_MPS2], dtype=np.float64)
    I15 = np.eye(15, dtype=np.float64)
    I3 = np.eye(3, dtype=np.float64)

    # Allocate RTS buffers (only if n_ekf <= 100_000 to bound RAM on multi-hour files)
    use_rts = abl.use_rts_smoother and (n_ekf <= 100_000)

    p_filt = np.zeros((n_ekf, 3), dtype=np.float64)
    v_filt = np.zeros((n_ekf, 3), dtype=np.float64)
    q_filt = np.zeros((n_ekf, 4), dtype=np.float64)
    ba_filt = np.zeros((n_ekf, 3), dtype=np.float64)
    bg_filt = np.zeros((n_ekf, 3), dtype=np.float64)
    a_filt = np.zeros((n_ekf, 3), dtype=np.float64)
    conf_filt = np.ones(n_ekf, dtype=np.float64)

    if use_rts:
        p_pred = np.zeros((n_ekf, 3), dtype=np.float64)
        v_pred = np.zeros((n_ekf, 3), dtype=np.float64)
        q_pred = np.zeros((n_ekf, 4), dtype=np.float64)
        ba_pred = np.zeros((n_ekf, 3), dtype=np.float64)
        bg_pred = np.zeros((n_ekf, 3), dtype=np.float64)
        P_pred_all = np.zeros((n_ekf, 15, 15), dtype=np.float64)
        P_filt_all = np.zeros((n_ekf, 15, 15), dtype=np.float64)
        F_all = np.zeros((n_ekf, 15, 15), dtype=np.float64)

    p_filt[0] = p_cur
    v_filt[0] = v_cur
    q_filt[0] = q_cur
    ba_filt[0] = ba_cur
    bg_filt[0] = bg_cur
    a_filt[0] = _quat_to_rotmat(q_cur) @ (f_fused_B[0] - R_BS @ ba_cur) + g_W
    if use_rts:
        p_pred[0] = p_cur
        v_pred[0] = v_cur
        q_pred[0] = q_cur
        ba_pred[0] = ba_cur
        bg_pred[0] = bg_cur
        P_pred_all[0] = P
        P_filt_all[0] = P
        F_all[0] = I15

    # Preallocated reusable work buffers
    F = I15.copy()
    Q = np.zeros((15, 15), dtype=np.float64)
    dx = np.zeros(15, dtype=np.float64)
    h_vec = np.zeros(15, dtype=np.float64)

    def joseph_scalar_update(h: np.ndarray, y_val: float, var: float) -> None:
        nonlocal P, dx
        Ph = P @ h
        s = float(h @ Ph) + var
        if s <= 1e-18:
            return
        k_gain = Ph / s
        P -= k_gain[:, None] * Ph[None, :]
        dx += k_gain * (y_val - float(h @ dx))

    def axis_scalar_update(idx: int, sign: float, y_val: float, var: float) -> None:
        nonlocal P, dx
        Ph = P[:, idx] if sign > 0.0 else -P[:, idx]
        s = float(P[idx, idx]) + var
        if s <= 1e-18:
            return
        k_gain = Ph / s
        P -= k_gain[:, None] * Ph[None, :]
        dx += k_gain * (y_val - sign * float(dx[idx]))

    last_bench_anchor_k = -1000
    last_impact_k = -1000

    t_rel_l = t_rel.tolist()
    w_cal_S_0 = w_cal_S[:, 0].tolist()
    w_cal_S_1 = w_cal_S[:, 1].tolist()
    w_cal_S_2 = w_cal_S[:, 2].tolist()
    f_fused_B_0 = f_fused_B[:, 0].tolist()
    f_fused_B_1 = f_fused_B[:, 1].tolist()
    f_fused_B_2 = f_fused_B[:, 2].tolist()
    hg_active_l = hg_active.tolist()
    rbs00, rbs01, rbs02 = float(R_BS[0, 0]), float(R_BS[0, 1]), float(R_BS[0, 2])
    rbs10, rbs11, rbs12 = float(R_BS[1, 0]), float(R_BS[1, 1]), float(R_BS[1, 2])
    rbs20, rbs21, rbs22 = float(R_BS[2, 0]), float(R_BS[2, 1]), float(R_BS[2, 2])

    for k in range(1, n_ekf):
        s0 = (k - 1) * substeps
        s1 = s0 + substeps

        # Check for large timestamp gap (e.g. C simulator jumping > 2 hours between shifts)
        dt_gap = (t_rel_l[s0] - t_rel_l[max(0, s0 - 1)]) if s0 > 0 else dt_imu
        if dt_gap > 0.5:
            v_cur = np.zeros(3, dtype=np.float64)
            p_cur[2] = 0.30 if pre_macro[k] == 1 else 0.0

        f_B_sum = np.zeros(3, dtype=np.float64)
        any_hg = False
        a_step_W = np.zeros(3, dtype=np.float64)

        qw, qx, qy, qz = float(q_cur[0]), float(q_cur[1]), float(q_cur[2]), float(q_cur[3])
        px, py, pz = float(p_cur[0]), float(p_cur[1]), float(p_cur[2])
        vx, vy, vz = float(v_cur[0]), float(v_cur[1]), float(v_cur[2])
        ba0, ba1, ba2 = float(ba_cur[0]), float(ba_cur[1]), float(ba_cur[2])
        ba_B_x = rbs00 * ba0 + rbs01 * ba1 + rbs02 * ba2
        ba_B_y = rbs10 * ba0 + rbs11 * ba1 + rbs12 * ba2
        ba_B_z = rbs20 * ba0 + rbs21 * ba1 + rbs22 * ba2
        bg0, bg1, bg2 = float(bg_cur[0]), float(bg_cur[1]), float(bg_cur[2])
        fb_sum0 = fb_sum1 = fb_sum2 = 0.0
        aw0 = aw1 = aw2 = 0.0

        for n in range(s0, s1):
            dt_n = (t_rel_l[n + 1] - t_rel_l[n]) if (n + 1 < n_imu) else dt_imu
            if dt_n <= 0.0 or dt_n > 0.25:
                dt_n = dt_imu

            ws0 = w_cal_S_0[n] - bg0
            ws1 = w_cal_S_1[n] - bg1
            ws2 = w_cal_S_2[n] - bg2
            wx = rbs00 * ws0 + rbs01 * ws1 + rbs02 * ws2
            wy = rbs10 * ws0 + rbs11 * ws1 + rbs12 * ws2
            wz = rbs20 * ws0 + rbs21 * ws1 + rbs22 * ws2

            if hg_active_l[n]:
                fbx = f_fused_B_0[n]
                fby = f_fused_B_1[n]
                fbz = f_fused_B_2[n]
                any_hg = True
            else:
                fbx = f_fused_B_0[n] - ba_B_x
                fby = f_fused_B_1[n] - ba_B_y
                fbz = f_fused_B_2[n] - ba_B_z

            # Midpoint (0.5*dt_n -> half-angle 0.25*dt_n) & full-step (dt_n -> half-angle 0.5*dt_n)
            wnorm = math.sqrt(wx * wx + wy * wy + wz * wz)
            if wnorm > 1e-10:
                hm = 0.25 * dt_n * wnorm
                sm = math.sin(hm) / wnorm
                dmw, dmx, dmy, dmz = math.cos(hm), sm * wx, sm * wy, sm * wz
                hf = 0.5 * dt_n * wnorm
                sf = math.sin(hf) / wnorm
                dfw, dfx, dfy, dfz = math.cos(hf), sf * wx, sf * wy, sf * wz
            else:
                c1 = 0.25 * dt_n
                dmw, dmx, dmy, dmz = 1.0, c1 * wx, c1 * wy, c1 * wz
                c2 = 0.5 * dt_n
                dfw, dfx, dfy, dfz = 1.0, c2 * wx, c2 * wy, c2 * wz

            # q_mid = q * dm
            mw = qw * dmw - qx * dmx - qy * dmy - qz * dmz
            mx = qw * dmx + qx * dmw + qy * dmz - qz * dmy
            my = qw * dmy - qx * dmz + qy * dmw + qz * dmx
            mz = qw * dmz + qx * dmy - qy * dmx + qz * dmw
            mn = math.sqrt(mw * mw + mx * mx + my * my + mz * mz)
            mw, mx, my, mz = mw / mn, mx / mn, my / mn, mz / mn

            # q_next = q * df
            nw = qw * dfw - qx * dfx - qy * dfy - qz * dfz
            nx = qw * dfx + qx * dfw + qy * dfz - qz * dfy
            ny = qw * dfy - qx * dfz + qy * dfw + qz * dfx
            nz = qw * dfz + qx * dfy - qy * dfx + qz * dfw
            nn = math.sqrt(nw * nw + nx * nx + ny * ny + nz * nz)
            nw, nx, ny, nz = nw / nn, nx / nn, ny / nn, nz / nn
            if nw * qw + nx * qx + ny * qy + nz * qz < 0.0:
                nw, nx, ny, nz = -nw, -nx, -ny, -nz

            # R(q_mid) @ f_B + g_W
            r00 = 1.0 - 2.0 * (my * my + mz * mz)
            r01 = 2.0 * (mx * my - mw * mz)
            r02 = 2.0 * (mx * mz + mw * my)
            r10 = 2.0 * (mx * my + mw * mz)
            r11 = 1.0 - 2.0 * (mx * mx + mz * mz)
            r12 = 2.0 * (my * mz - mw * mx)
            r20 = 2.0 * (mx * mz - mw * my)
            r21 = 2.0 * (my * mz + mw * mx)
            r22 = 1.0 - 2.0 * (mx * mx + my * my)

            aw0 = r00 * fbx + r01 * fby + r02 * fbz
            aw1 = r10 * fbx + r11 * fby + r12 * fbz
            aw2 = r20 * fbx + r21 * fby + r22 * fbz - GRAVITY_MPS2

            vx_n = vx + aw0 * dt_n
            vy_n = vy + aw1 * dt_n
            vz_n = vz + aw2 * dt_n
            px += 0.5 * (vx + vx_n) * dt_n
            py += 0.5 * (vy + vy_n) * dt_n
            pz += 0.5 * (vz + vz_n) * dt_n
            vx, vy, vz = vx_n, vy_n, vz_n
            qw, qx, qy, qz = nw, nx, ny, nz
            fb_sum0 += fbx
            fb_sum1 += fby
            fb_sum2 += fbz

        q_cur = np.array([qw, qx, qy, qz], dtype=np.float64)
        p_cur = np.array([px, py, pz], dtype=np.float64)
        v_cur = np.array([vx, vy, vz], dtype=np.float64)
        f_B_sum[0], f_B_sum[1], f_B_sum[2] = fb_sum0, fb_sum1, fb_sum2
        a_step_W[0], a_step_W[1], a_step_W[2] = aw0, aw1, aw2

        if any_hg or float(np.linalg.norm(a_step_W)) > 12.0 * GRAVITY_MPS2:
            last_impact_k = k

        R_k = _quat_to_rotmat(q_cur)
        f_mean_B = f_B_sum / float(substeps)
        f_W = R_k @ f_mean_B
        R_k_Rbs = R_k @ R_BS
        s_ba = 0.0 if any_hg else 1.0

        # Build F_k
        F[:] = I15
        F[0:3, 3:6] = dt_ekf * I3
        F[3:6, 6:9] = -dt_ekf * _skew(f_W)
        F[0:3, 6:9] = 0.5 * dt_ekf * F[3:6, 6:9]
        F[3:6, 9:12] = -dt_ekf * s_ba * R_k_Rbs
        F[0:3, 9:12] = 0.5 * dt_ekf * F[3:6, 9:12]
        F[6:9, 12:15] = -dt_ekf * R_k_Rbs

        # Process noise Q_k
        if any_hg:
            sig_a = cfg.sigma_accel_hg
        elif stride_ok_ekf[k] and pre_macro[k] == 2:
            sig_a = cfg.sigma_accel_striding
        else:
            sig_a = cfg.sigma_accel_icm

        Q[0, 0] = Q[1, 1] = Q[2, 2] = (sig_a * sig_a) * (dt_ekf**3) / 3.0
        Q[3, 3] = Q[4, 4] = Q[5, 5] = (sig_a * sig_a) * dt_ekf
        Q[6, 6] = Q[7, 7] = Q[8, 8] = (cfg.sigma_gyro * cfg.sigma_gyro) * dt_ekf
        Q[9, 9] = Q[10, 10] = Q[11, 11] = (cfg.sigma_ba_rw * cfg.sigma_ba_rw) * dt_ekf
        Q[12, 12] = Q[13, 13] = Q[14, 14] = (cfg.sigma_bg_rw * cfg.sigma_bg_rw) * dt_ekf

        P = F @ P @ F.T + Q
        P = 0.5 * (P + P.T)

        if use_rts:
            p_pred[k] = p_cur
            v_pred[k] = v_cur
            q_pred[k] = q_cur
            ba_pred[k] = ba_cur
            bg_pred[k] = bg_cur
            P_pred_all[k] = P
            F_all[k] = F

        dx.fill(0.0)
        is_bench = pre_macro[k] == 1
        is_low_dyn = bool(zupt_mask_ekf[k])
        is_stationary = is_low_dyn and (
            is_bench
            or pre_macro[k] == 0
            or math.hypot(float(v_cur[0]), float(v_cur[1])) < 0.65
        )

        # --- Layer 3: ZUPT / ZARU + Gravity Leveling ---
        if abl.use_zupt_zaru and is_stationary:
            var_v = cfg.sigma_zupt_v ** 2
            axis_scalar_update(3, 1.0, float(-v_cur[0]), var_v)
            axis_scalar_update(4, 1.0, float(-v_cur[1]), var_v)
            axis_scalar_update(5, 1.0, float(-v_cur[2]), var_v)

            var_w = cfg.sigma_zaru_w ** 2
            w_res_S = w_cal_S[s0] - bg_cur
            axis_scalar_update(12, -1.0, float(-w_res_S[0]), var_w)
            axis_scalar_update(13, -1.0, float(-w_res_S[1]), var_w)
            axis_scalar_update(14, -1.0, float(-w_res_S[2]), var_w)

            # Gravity leveling (updates tilt theta, keeping ba decoupled during short windows)
            g_pred_B = R_k.T @ (-g_W) + R_BS @ ba_cur
            y_grav = f_icm_B[s0] - g_pred_B
            H_grav_theta = R_k.T @ _skew(-g_W)
            var_g = cfg.sigma_grav ** 2
            for row_i in range(3):
                h_vec.fill(0.0)
                h_vec[6:9] = H_grav_theta[row_i]
                h_vec[9:12] = R_BS[row_i]
                joseph_scalar_update(h_vec, float(y_grav[row_i]), var_g)

        # --- Layer 4: Non-Holonomic Constraints (v_z^W, z^W, v_y^B) & Stride Speed ---
        if abl.use_nhc_vz:
            sig_vz = 0.05 if is_bench else cfg.sigma_nhc_vz
            axis_scalar_update(5, 1.0, float(-v_cur[2]), sig_vz * sig_vz)

            z_tgt = 0.30 if is_bench else 0.0
            axis_scalar_update(2, 1.0, float(z_tgt - p_cur[2]), cfg.sigma_nhc_z ** 2)

        if abl.use_nhc_vy and not is_stationary and not is_bench:
            is_sharp_turn = abs(w_z_ekf[k]) >= math.radians(90.0) or (k - last_impact_k) <= int(0.5 * ekf_hz)
            sig_vy = cfg.sigma_nhc_vy_turn if is_sharp_turn else cfg.sigma_nhc_vy_base
            v_B_est = R_k.T @ v_cur
            H_vB = R_k.T @ _skew(v_cur)
            h_vec.fill(0.0)
            h_vec[3:6] = R_k.T[1]
            h_vec[6:9] = H_vB[1]
            joseph_scalar_update(h_vec, float(-v_B_est[1]), sig_vy * sig_vy)

        if abl.use_stride_speed and (not is_bench) and (not is_stationary) and pre_macro[k] == 2 and stride_ok_ekf[k]:
            # Disable during sharp stops/impacts
            if abs(w_z_ekf[k]) < math.radians(85.0) and (k - last_impact_k) > int(0.4 * ekf_hz):
                v_B_est = R_k.T @ v_cur
                H_vB = R_k.T @ _skew(v_cur)
                h_vec.fill(0.0)
                h_vec[3:6] = R_k.T[0]
                h_vec[6:9] = H_vB[0]
                joseph_scalar_update(h_vec, float(v_bio_ekf[k] - v_B_est[0]), cfg.sigma_stride_v ** 2)

        # --- Layer 5: Bench-Gate Coordinate Soft Reset ---
        if abl.use_bench_gate_reset:
            trans_bench = pre_macro[k] != pre_macro[k - 1] and (pre_macro[k] == 1 or pre_macro[k - 1] == 1)
            periodic_bench = is_bench and (k - last_bench_anchor_k >= ekf_hz)
            if trans_bench or periodic_bench:
                sig_b = 1.5 if trans_bench else cfg.sigma_bench_xy
                var_b = sig_b * sig_b
                axis_scalar_update(0, 1.0, float(bench_pose_xy[0] - p_cur[0]), var_b)
                axis_scalar_update(1, 1.0, float(bench_pose_xy[1] - p_cur[1]), var_b)
                last_bench_anchor_k = k

        # --- Layer 6: Adaptive Magnetometer Anomaly Gating ---
        if abl.use_mag_updates and k in aux_step_map:
            m_B = aux_step_map[k]
            m_norm = float(np.linalg.norm(m_B))
            m_W = R_k @ m_B
            dip_meas = math.degrees(math.atan2(-m_W[2], math.hypot(m_W[0], m_W[1])))
            mag_ok = True
            if abl.use_mag_gating:
                norm_ok = (1.0 - cfg.mag_norm_tol) * B0_ut <= m_norm <= (1.0 + cfg.mag_norm_tol) * B0_ut
                dip_ok = abs(dip_meas - dip0_deg) < cfg.mag_dip_tol_deg
                mag_ok = norm_ok and dip_ok
            if mag_ok:
                if B_ref_W is None:
                    B_ref_W = m_W.copy()
                psi_ref = math.atan2(float(B_ref_W[1]), float(B_ref_W[0]))
                psi_meas = math.atan2(float(m_W[1]), float(m_W[0]))
                y_psi = _wrap_angle(psi_ref - psi_meas)
                if (not abl.use_mag_gating) or abs(y_psi) < math.radians(45.0):
                    axis_scalar_update(8, 1.0, float(y_psi), cfg.sigma_mag_yaw_rad ** 2)

        # Symmetrize & floor covariance once per step
        P = 0.5 * (P + P.T)
        np.fill_diagonal(P, np.maximum(np.diag(P), 1e-12))

        # Inject error state into nominal state
        p_cur = p_cur + dx[0:3]
        v_cur = v_cur + dx[3:6]
        q_cur = _quat_normalize(_quat_mul(_quat_exp(dx[6:9]), q_cur))
        ba_cur = ba_cur + dx[9:12]
        bg_cur = bg_cur + dx[12:15]

        # Layer 5: Rink boundary projection (if enabled)
        if abl.use_rink_containment:
            p_xy_proj, v_xy_proj, hit = _project_inside_rink(
                p_cur[:2], v_cur[:2], rink_L, rink_W, rink_Rc, cfg.r_torso_m,
                e_wall=(0.35 if (k - last_impact_k) <= 2 else 0.0),
            )
            if hit:
                p_cur[:2] = p_xy_proj
                v_cur[:2] = v_xy_proj

        p_filt[k] = p_cur
        v_filt[k] = v_cur
        q_filt[k] = q_cur
        ba_filt[k] = ba_cur
        bg_filt[k] = bg_cur
        a_filt[k] = a_step_W
        pos_std = math.sqrt(max(0.0, float(P[0, 0] + P[1, 1])))
        conf_filt[k] = float(np.clip(1.0 / (1.0 + 0.15 * pos_std), 0.1, 0.99))

        if use_rts:
            P_filt_all[k] = P

    # -----------------------------------------------------------------------
    # 5. Layer 6b: Batched Manifold Rauch-Tung-Striebel (RTS) Backward Smoother
    # -----------------------------------------------------------------------
    if use_rts and n_ekf >= 2:
        p_sm = p_filt.copy()
        v_sm = v_filt.copy()
        q_sm = q_filt.copy()
        ba_sm = ba_filt.copy()
        bg_sm = bg_filt.copy()

        # Compute all smoother gains G_k = P_{k|k} F_{k+1}^T P_{k+1|k}^{-1} in one batched call
        P_reg = P_pred_all[1:] + 1e-12 * I15[None, :, :]
        G_all = np.linalg.solve(P_reg, F_all[1:] @ P_filt_all[:-1]).swapaxes(-1, -2)

        dx_diff = np.zeros(15, dtype=np.float64)
        for k in range(n_ekf - 2, -1, -1):
            dx_diff[0:3] = p_sm[k + 1] - p_pred[k + 1]
            dx_diff[3:6] = v_sm[k + 1] - v_pred[k + 1]
            dx_diff[6:9] = _quat_log(_quat_mul(q_sm[k + 1], _quat_conj(q_pred[k + 1])))
            dx_diff[9:12] = ba_sm[k + 1] - ba_pred[k + 1]
            dx_diff[12:15] = bg_sm[k + 1] - bg_pred[k + 1]

            dx_k = G_all[k] @ dx_diff
            p_sm[k] = p_filt[k] + dx_k[0:3]
            v_sm[k] = v_filt[k] + dx_k[3:6]
            q_sm[k] = _quat_normalize(_quat_mul(_quat_exp(dx_k[6:9]), q_filt[k]))
            if np.dot(q_sm[k], q_sm[k + 1]) < 0.0:
                q_sm[k] = -q_sm[k]
            ba_sm[k] = ba_filt[k] + dx_k[9:12]
            bg_sm[k] = bg_filt[k] + dx_k[12:15]
    else:
        p_sm = p_filt
        v_sm = v_filt
        q_sm = q_filt
        ba_sm = ba_filt
        bg_sm = bg_filt

    # Post-RTS rink containment projection
    if abl.use_rink_containment:
        for k in range(n_ekf):
            p_xy_proj, v_xy_proj, hit = _project_inside_rink(
                p_sm[k, :2], v_sm[k, :2], rink_L, rink_W, rink_Rc, cfg.r_torso_m
            )
            if hit:
                p_sm[k, :2] = p_xy_proj
                v_sm[k, :2] = v_xy_proj

    # -----------------------------------------------------------------------
    # 6. Post-RTS Per-Sample Classification, Shift Segmentation & Contract JSON
    # -----------------------------------------------------------------------
    if cfg.output_hz > ekf_hz and cfg.output_hz % ekf_hz == 0:
        up_factor = cfg.output_hz // ekf_hz
        n_out = n_ekf * up_factor
        dt_out = dt_ekf / up_factor
        t_out = np.arange(n_out, dtype=np.float64) * dt_out + float(t_ekf[0])
        k_float = np.clip(np.arange(n_out, dtype=np.float64) / up_factor, 0.0, float(n_ekf - 1))
        k0_arr = np.floor(k_float).astype(np.int32)
        k1_arr = np.minimum(n_ekf - 1, k0_arr + 1)
        alpha = (k_float - k0_arr)[:, None]
        p_out = (1.0 - alpha) * p_sm[k0_arr] + alpha * p_sm[k1_arr]
        v_out = (1.0 - alpha) * v_sm[k0_arr] + alpha * v_sm[k1_arr]
        ba_out = (1.0 - alpha) * ba_sm[k0_arr] + alpha * ba_sm[k1_arr]
        q_out = (1.0 - alpha) * q_sm[k0_arr] + alpha * q_sm[k1_arr]
        q_out /= np.linalg.norm(q_out, axis=1, keepdims=True)
        conf_out = (1.0 - alpha[:, 0]) * conf_filt[k0_arr] + alpha[:, 0] * conf_filt[k1_arr]
        ekf_idx_for_out = k0_arr
    else:
        out_stride = max(1, ekf_hz // cfg.output_hz)
        out_indices = np.arange(0, n_ekf, out_stride)
        n_out = out_indices.shape[0]
        dt_out = dt_ekf * out_stride
        t_out = t_ekf[out_indices]
        p_out = p_sm[out_indices]
        v_out = v_sm[out_indices]
        ba_out = ba_sm[out_indices]
        q_out = q_sm[out_indices]
        conf_out = conf_filt[out_indices]
        ekf_idx_for_out = out_indices

    imu_per_out = max(1, int(round(dt_out / dt_imu)))

    # Reconstruct bin-peak world acceleration (preserving 3200 Hz ADXL372 impacts!)
    qw_o, qx_o, qy_o, qz_o = q_out[:, 0], q_out[:, 1], q_out[:, 2], q_out[:, 3]
    R_out_all = np.empty((n_out, 3, 3), dtype=np.float64)
    R_out_all[:, 0, 0] = 1.0 - 2.0 * (qy_o * qy_o + qz_o * qz_o)
    R_out_all[:, 0, 1] = 2.0 * (qx_o * qy_o - qw_o * qz_o)
    R_out_all[:, 0, 2] = 2.0 * (qx_o * qz_o + qw_o * qy_o)
    R_out_all[:, 1, 0] = 2.0 * (qx_o * qy_o + qw_o * qz_o)
    R_out_all[:, 1, 1] = 1.0 - 2.0 * (qx_o * qx_o + qz_o * qz_o)
    R_out_all[:, 1, 2] = 2.0 * (qy_o * qz_o - qw_o * qx_o)
    R_out_all[:, 2, 0] = 2.0 * (qx_o * qz_o - qw_o * qy_o)
    R_out_all[:, 2, 1] = 2.0 * (qy_o * qz_o + qw_o * qx_o)
    R_out_all[:, 2, 2] = 1.0 - 2.0 * (qx_o * qx_o + qy_o * qy_o)

    ba_B_out = ba_out @ R_BS.T
    n_full_bins = min(n_out, n_imu // imu_per_out)
    f_best_all = np.empty((n_out, 3), dtype=np.float64)
    icm_max_norms = np.empty(n_out, dtype=np.float64)

    if n_full_bins > 0:
        f_bins = (
            f_peak_B[: n_full_bins * imu_per_out].reshape(n_full_bins, imu_per_out, 3)
            - ba_B_out[:n_full_bins, None, :]
        )
        icm_norm_sq = np.sum(f_bins * f_bins, axis=2)
        best_idx = np.argmax(icm_norm_sq, axis=1)
        row_arange = np.arange(n_full_bins)
        f_best_all[:n_full_bins] = f_bins[row_arange, best_idx]
        icm_max_norms[:n_full_bins] = np.sqrt(icm_norm_sq[row_arange, best_idx])

    for out_i in range(n_full_bins, n_out):
        s0 = min(n_imu - 1, out_i * imu_per_out)
        s1 = max(s0 + 1, min(n_imu, (out_i + 1) * imu_per_out))
        f_bin_icm = f_peak_B[s0:s1] - ba_B_out[out_i]
        icm_norms = np.linalg.norm(f_bin_icm, axis=1)
        idx_max = int(np.argmax(icm_norms))
        f_best_all[out_i] = f_bin_icm[idx_max]
        icm_max_norms[out_i] = float(icm_norms[idx_max])

    if abl.use_high_g_handoff and t_hg.shape[0] > 0:
        hg_norms_all = np.linalg.norm(f_hg_B, axis=1)
        for out_i in np.where(hg_norms_all > 12.0 * GRAVITY_MPS2)[0]:
            pass
        s0_arr = np.minimum(n_imu - 1, np.arange(n_out) * imu_per_out)
        s1_arr = np.maximum(s0_arr + 1, np.minimum(n_imu, (np.arange(n_out) + 1) * imu_per_out))
        t_lo_arr = t_rel[s0_arr] - 1e-7
        t_hi_arr = t_rel[np.minimum(n_imu - 1, s1_arr - 1)] + dt_imu - 1e-7
        i0_hg_arr = np.searchsorted(t_hg, t_lo_arr, side="left")
        i1_hg_arr = np.searchsorted(t_hg, t_hi_arr, side="left")
        for out_i in np.where(i1_hg_arr > i0_hg_arr)[0]:
            i0_hg = int(i0_hg_arr[out_i])
            i1_hg = int(i1_hg_arr[out_i])
            sl_norms = hg_norms_all[i0_hg:i1_hg]
            idx_hg_max = int(np.argmax(sl_norms))
            best_hg_norm = float(sl_norms[idx_hg_max])
            if best_hg_norm > float(icm_max_norms[out_i]) and best_hg_norm > 12.0 * GRAVITY_MPS2:
                f_best_all[out_i] = f_hg_B[i0_hg + idx_hg_max]

    a_out_W = np.einsum("nij,nj->ni", R_out_all, f_best_all) + g_W[None, :]

    a_out_mag = np.linalg.norm(a_out_W, axis=1)
    vx_out = v_out[:, 0]
    vy_out = v_out[:, 1]
    spd_out = np.hypot(vx_out, vy_out)
    tangential_acc = np.gradient(spd_out, dt_out) if n_out >= 3 else np.zeros(n_out)

    # Impact halo (+-0.15 s around accel_mag > 12 g)
    imp_halo = np.zeros(n_out, dtype=bool)
    halo_steps = max(1, int(round(0.15 / dt_out)))
    for out_i in np.where(a_out_mag > 12.0 * GRAVITY_MPS2)[0]:
        imp_halo[max(0, out_i - halo_steps) : min(n_out, out_i + halo_steps + 1)] = True

    states_out: list[str] = []
    for out_i in range(n_out):
        k = int(ekf_idx_for_out[out_i])
        macro = int(pre_macro[k])
        if imp_halo[out_i]:
            st_name = "IMPACT"
        elif macro == 0:
            st_name = "UNKNOWN"
        elif macro == 1:
            st_name = "BENCH"
        else:
            if spd_out[out_i] > 0.8 and (abs(w_z_ekf[k]) >= math.radians(90.0) or tangential_acc[out_i] < -3.0):
                st_name = "STOPPING"
            elif stride_ok_ekf[k] and stride_rms_ekf[k] >= 0.55:
                st_name = "STRIDING"
            elif spd_out[out_i] >= 0.45:
                st_name = "GLIDING"
            else:
                st_name = "UNKNOWN"
        states_out.append(st_name)

    # Lightly smoothed speed for clean distance integration (avoids jitter inflation)
    smooth_win = max(1, int(round(0.20 / dt_out)))
    if smooth_win > 1 and n_out >= smooth_win:
        kernel = np.ones(smooth_win, dtype=np.float64) / smooth_win
        vx_sm_dist = np.convolve(vx_out, kernel, mode="same")
        vy_sm_dist = np.convolve(vy_out, kernel, mode="same")
        spd_clean = np.hypot(vx_sm_dist, vy_sm_dist)
    else:
        spd_clean = spd_out

    # Extract contiguous shifts (macro == 2)
    on_ice_mask = pre_macro[ekf_idx_for_out] == 2
    shifts: list[dict[str, Any]] = []
    i = 0
    while i < n_out:
        if not on_ice_mask[i]:
            i += 1
            continue
        j = i
        while j < n_out and on_ice_mask[j]:
            j += 1
        # Ignore sub-2s micro-glitches
        t_start = float(t_out[i])
        t_end = float(t_out[j - 1] + dt_out)
        dur_s = t_end - t_start
        if dur_s >= 2.0:
            sl_spd = spd_clean[i:j]
            sl_raw_spd = spd_out[i:j]
            sl_acc = a_out_mag[i:j]
            dist_m = float(np.sum(np.where(sl_spd >= 0.35, sl_spd, 0.0)) * dt_out)
            peak_spd = float(np.max(sl_raw_spd))
            avg_spd = min(peak_spd, dist_m / dur_s)
            # High intensity bouts (> 5.0 m/s separated by >= 1.0 s)
            hi_mask = sl_raw_spd > 5.0
            bouts = 0
            in_bout = False
            for flag in hi_mask:
                if flag and not in_bout:
                    bouts += 1
                    in_bout = True
                elif not flag:
                    in_bout = False

            shifts.append(
                {
                    "shift_id": len(shifts) + 1,
                    "start_s": round(t_start, 6),
                    "end_s": round(t_end, 6),
                    "duration_s": round(t_end, 6) - round(t_start, 6),
                    "distance_m": float(dist_m),
                    "peak_speed_mps": float(peak_spd),
                    "avg_speed_mps": float(avg_spd),
                    "peak_accel_mps2": float(np.max(sl_acc)),
                    "high_intensity_bouts": int(bouts),
                }
            )
        i = j

    # Trajectory list
    t_out_l = t_out.tolist()
    px_out_l = p_out[:, 0].tolist()
    py_out_l = p_out[:, 1].tolist()
    vx_out_l = vx_out.tolist()
    vy_out_l = vy_out.tolist()
    spd_out_l = spd_out.tolist()
    ax_out_l = a_out_W[:, 0].tolist()
    ay_out_l = a_out_W[:, 1].tolist()
    az_out_l = a_out_W[:, 2].tolist()
    amag_out_l = a_out_mag.tolist()
    q_out_l = q_out.tolist()
    conf_out_l = conf_out.tolist()

    trajectory: list[dict[str, Any]] = [
        {
            "t": t_out_l[out_i],
            "x": px_out_l[out_i],
            "y": py_out_l[out_i],
            "vx": vx_out_l[out_i],
            "vy": vy_out_l[out_i],
            "speed": spd_out_l[out_i],
            "ax": ax_out_l[out_i],
            "ay": ay_out_l[out_i],
            "az": az_out_l[out_i],
            "accel_mag": amag_out_l[out_i],
            "q_WB": q_out_l[out_i],
            "state": states_out[out_i],
            "confidence": conf_out_l[out_i],
        }
        for out_i in range(n_out)
    ]

    total_duration_s = float(t_rel[-1] + dt_imu)
    total_ice_time_s = min(total_duration_s, sum(float(s["duration_s"]) for s in shifts))
    total_distance_m = sum(float(s["distance_m"]) for s in shifts)
    peak_speed_mps = max((float(s["peak_speed_mps"]) for s in shifts), default=float(np.max(spd_out)))
    peak_accel_mps2 = float(np.max(a_out_mag))

    session_summary = {
        "session_id": f"{hdr.session_id:016x}",
        "device_id": f"HKS-{hdr.device_id:016x}",
        "rink_preset": hdr.rink_preset,
        "gps_used": False,
        "external_anchors_used": False,
        "total_duration_s": total_duration_s,
        "total_ice_time_s": total_ice_time_s,
        "total_distance_m": total_distance_m,
        "shift_count": len(shifts),
        "peak_speed_mps": peak_speed_mps,
        "peak_accel_mps2": peak_accel_mps2,
        "battery_used_pct": batt_used_pct,
    }

    return {
        "schema_version": "1.0.0",
        "session_summary": session_summary,
        "shifts": shifts,
        "trajectory": trajectory,
    }
