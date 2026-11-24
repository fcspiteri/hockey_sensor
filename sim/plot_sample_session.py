"""Generate diagnostic plots of raw .hks sensor telemetry and 6-layer ES-EKF + RTS output metrics."""

from __future__ import annotations

from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from .flash_codec import GRAVITY_MPS2, decode_hks_session
from .fusion_pipeline import run_fusion
from .physics_sim import SimConfig, simulate_session


def generate_plots(out_dirs: list[Path]) -> dict[str, float]:
    for d in out_dirs:
        d.mkdir(parents=True, exist_ok=True)

    cfg = SimConfig(rink_preset="NHL_200X85FT", duration_s=140.0, num_shifts=2, seed=42)
    res = simulate_session(cfg)
    sess = decode_hks_session(res.hks_bytes)
    rep = run_fusion(res.hks_bytes)
    gt = res.ground_truth

    # 1. Unpack raw sensor streams
    imu_t, imu_a, imu_g = [], [], []
    for b in sess.imu_batches:
        ts, a_s, g_s = b.to_numpy(sess.header.accel_scale_mps2_per_lsb, sess.header.gyro_scale_rads_per_lsb)
        imu_t.append(ts)
        imu_a.append(a_s)
        imu_g.append(g_s)
    t_imu = np.concatenate(imu_t)
    a_imu = np.vstack(imu_a) / GRAVITY_MPS2
    g_imu = np.degrees(np.vstack(imu_g))

    hg_t, hg_a = [], []
    for b in sess.high_g_batches:
        ts, a_h = b.to_numpy(sess.header.high_g_scale_mps2_per_lsb)
        hg_t.append(ts)
        hg_a.append(a_h)
    t_hg = np.concatenate(hg_t) if hg_t else np.empty(0)
    a_hg = np.vstack(hg_a) / GRAVITY_MPS2 if hg_a else np.empty((0, 3))

    t_aux = np.array([p.timestamp_us * 1e-6 for p in sess.aux_env_packets])
    m_aux = (
        np.array([[p.mx_i32, p.my_i32, p.mz_i32] for p in sess.aux_env_packets])
        * sess.header.mag_scale_ut_per_lsb
    )
    m_norm = np.linalg.norm(m_aux, axis=1)
    p_aux = np.array([p.pressure_pa for p in sess.aux_env_packets])
    mag_anom = np.array([(p.flags & 0x04) != 0 for p in sess.aux_env_packets])

    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")

    # --- FIGURE 1: Raw Sensor Telemetry ---
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    fig.suptitle(
        "Raw Chest-Pod Sensor Telemetry (.hks Binary Stream — 140 s, 2 Shifts, NHL Rink)",
        fontsize=14,
        fontweight="bold",
    )

    axes[0].plot(t_imu, a_imu[:, 0], label="ax (Sensor X)", alpha=0.85, lw=0.8)
    axes[0].plot(t_imu, a_imu[:, 1], label="ay (Sensor Y)", alpha=0.85, lw=0.8)
    axes[0].plot(t_imu, a_imu[:, 2], label="az (Sensor Z)", alpha=0.85, lw=0.8)
    axes[0].axhline(16.0, color="r", ls="--", lw=0.9, label="±16 g ICM-42688-P Saturation Limit")
    axes[0].axhline(-16.0, color="r", ls="--", lw=0.9)
    axes[0].set_ylabel("Primary Accel [g]\n(ICM-42688-P @ 200 Hz)")
    axes[0].legend(loc="upper right", ncol=4, fontsize=8)

    if len(t_hg) > 0:
        hg_mag = np.linalg.norm(a_hg, axis=1)
        axes[1].plot(
            t_imu,
            np.linalg.norm(a_imu, axis=1),
            color="#888888",
            alpha=0.55,
            lw=0.8,
            label="ICM-42688-P ||a|| (clipped @ 16 g)",
        )
        axes[1].scatter(
            t_hg,
            hg_mag,
            color="#d62728",
            s=12,
            zorder=5,
            label="ADXL372 High-g Burst @ 3200 Hz (unclipped ±200 g)",
        )
        for sh_i, sh in enumerate(gt["shifts"]):
            axes[1].axvspan(
                sh["start_s"],
                sh["end_s"],
                color="#1f77b4",
                alpha=0.08,
                label="On-Ice Shift Window" if sh_i == 0 else None,
            )
    axes[1].set_ylabel("Impact Accel ||a|| [g]\n(ADXL372 @ 3200 Hz)")
    axes[1].legend(loc="upper right", fontsize=8)

    axes[2].plot(t_imu, g_imu[:, 0], label="gx [deg/s]", alpha=0.85, lw=0.8)
    axes[2].plot(t_imu, g_imu[:, 1], label="gy [deg/s]", alpha=0.85, lw=0.8)
    axes[2].plot(t_imu, g_imu[:, 2], label="gz [deg/s]", alpha=0.85, lw=0.8)
    axes[2].set_ylabel("Angular Rate [deg/s]\n(ICM-42688-P @ 200 Hz)")
    axes[2].legend(loc="upper right", ncol=3, fontsize=8)

    ax3 = axes[3]
    ax3_r = ax3.twinx()
    l1 = ax3.plot(t_aux, p_aux - 101325.0, color="#2ca02c", lw=1.3, label="Baro ΔP (BMP581 - 101325 Pa)")
    l2 = ax3_r.plot(t_aux, m_norm, color="#9467bd", lw=1.0, alpha=0.85, label="Mag ||B|| [µT] (MMC5983MA)")
    extra_handles = []
    if np.any(mag_anom):
        l3 = ax3_r.scatter(
            t_aux[mag_anom], m_norm[mag_anom], color="#ff7f0e", s=14, zorder=5, label="Gated Steel Mag Anomaly"
        )
        extra_handles.append(l3)
    ax3.set_ylabel("Baro ΔP [Pa]\n(Bench +0.30 m = -3.7 Pa)", color="#2ca02c")
    ax3_r.set_ylabel("Mag Flux ||B|| [µT]", color="#9467bd")
    ax3.set_xlabel("Session Time [s]")
    lines = l1 + l2 + extra_handles
    labels = [l.get_label() for l in lines]
    ax3.legend(lines, labels, loc="upper right", fontsize=8)

    fig.tight_layout()
    for d in out_dirs:
        fig.savefig(d / "raw_sensor_telemetry.png", dpi=160)
    plt.close(fig)

    # --- FIGURE 2: Output Metrics (Speed, Cumulative Distance, Accel, Shifts & States) ---
    t_est = np.array([p["t"] for p in rep["trajectory"]])
    t_gt = np.array([p["t"] for p in gt["trajectory"]])
    spd_est = np.array([p["speed"] for p in rep["trajectory"]])
    spd_gt = np.array([p["speed"] for p in gt["trajectory"]])
    acc_est = np.array([p["accel_mag"] for p in rep["trajectory"]]) / GRAVITY_MPS2
    acc_gt = np.array([p["accel_mag"] for p in gt["trajectory"]]) / GRAVITY_MPS2
    st_est = [p["state"] for p in rep["trajectory"]]

    dt_out = t_est[1] - t_est[0]
    on_ice_est = np.isin(st_est, ["STRIDING", "GLIDING", "STOPPING", "IMPACT"])
    cum_dist_est = np.cumsum(np.where(on_ice_est & (spd_est >= 0.35), spd_est, 0.0) * dt_out)
    on_ice_gt = np.isin([p["state"] for p in gt["trajectory"]], ["STRIDING", "GLIDING", "STOPPING", "IMPACT"])
    cum_dist_gt = np.cumsum(np.where(on_ice_gt, spd_gt, 0.0) * dt_out)

    fig, axes = plt.subplots(4, 1, figsize=(13, 10.5), sharex=True)
    rmse_spd = float(np.sqrt(np.mean((spd_est - spd_gt) ** 2)))
    est_tot_dist = rep["session_summary"]["total_distance_m"]
    gt_tot_dist = gt["total_distance_m"]
    dist_err_pct = 100.0 * abs(est_tot_dist - gt_tot_dist) / gt_tot_dist
    fig.suptitle(
        f"6-Layer Anchor-Free ES-EKF + RTS Output vs. Ground Truth  |  Speed RMSE = {rmse_spd:.2f} m/s  |  Distance Error = {dist_err_pct:.2f}%",
        fontsize=13.5,
        fontweight="bold",
    )

    axes[0].plot(t_gt, spd_gt, color="#222222", lw=1.6, label="Ground Truth Speed [m/s]")
    axes[0].plot(
        t_est, spd_est, color="#1f77b4", lw=1.3, ls="--", label=f"Fused Estimate Speed [m/s] (RMSE = {rmse_spd:.2f} m/s)"
    )
    for i, sh in enumerate(rep["shifts"]):
        sid = sh["shift_id"]
        sdur = sh["duration_s"]
        axes[0].axvspan(
            sh["start_s"],
            sh["end_s"],
            color="#2ca02c",
            alpha=0.10,
            label=f"Detected Shift {sid} ({sdur:.1f} s)" if i < 2 else None,
        )
    axes[0].set_ylabel("Player Speed [m/s]")
    axes[0].legend(loc="upper right", fontsize=8.5)

    axes[1].plot(t_gt, cum_dist_gt, color="#222222", lw=1.6, label=f"Ground Truth Distance ({gt_tot_dist:.1f} m)")
    axes[1].plot(
        t_est, cum_dist_est, color="#ff7f0e", lw=1.5, ls="--", label=f"Fused Distance Skated ({est_tot_dist:.1f} m)"
    )
    axes[1].set_ylabel("Distance Skated [m]")
    axes[1].legend(loc="upper left", fontsize=8.5)

    gt_peak_g = gt["peak_accel_mps2"] / GRAVITY_MPS2
    est_peak_g = rep["session_summary"]["peak_accel_mps2"] / GRAVITY_MPS2
    axes[2].plot(t_gt, acc_gt, color="#222222", lw=1.1, alpha=0.75, label=f"Ground Truth ||a_W|| (Peak = {gt_peak_g:.1f} g)")
    axes[2].plot(
        t_est, acc_est, color="#d62728", lw=1.0, alpha=0.85, label=f"Fused Dual-IMU ||a_W|| (Peak = {est_peak_g:.1f} g)"
    )
    axes[2].axhline(16.0, color="gray", ls=":", lw=0.9, label="16 g Primary IMU Threshold")
    axes[2].set_ylabel("World Accel [g]")
    axes[2].legend(loc="upper right", fontsize=8.5)

    state_order = ["BENCH", "UNKNOWN", "GLIDING", "STRIDING", "STOPPING", "IMPACT"]
    state_to_y = {s: i for i, s in enumerate(state_order)}
    y_est = [state_to_y[s] for s in st_est]
    y_gt = [state_to_y[p["state"]] for p in gt["trajectory"]]
    axes[3].step(t_gt, y_gt, where="post", color="#222222", lw=1.4, alpha=0.7, label="Ground Truth State")
    axes[3].step(t_est, y_est, where="post", color="#1f77b4", lw=1.1, alpha=0.85, label="Classified Motion State")
    axes[3].set_yticks(range(len(state_order)))
    axes[3].set_yticklabels(state_order, fontsize=8.5)
    axes[3].set_ylabel("Motion & Shift State")
    axes[3].set_xlabel("Session Time [s]")
    axes[3].legend(loc="upper right", fontsize=8.5)

    fig.tight_layout()
    for d in out_dirs:
        fig.savefig(d / "fusion_metrics_timeline.png", dpi=160)
    plt.close(fig)

    # --- FIGURE 3: 2D On-Ice Position & Shift Trajectories on NHL Rink ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.2), sharey=True)
    est_xy = np.array([[p["x"], p["y"]] for p in rep["trajectory"]])
    gt_xy = np.array([[p["x"], p["y"]] for p in gt["trajectory"]])
    cep50 = float(np.median(np.linalg.norm(est_xy[on_ice_gt] - gt_xy[on_ice_gt], axis=1)))
    fig.suptitle(
        f"Indoor On-Ice 2D Position Tracking (Zero GPS, Zero External Anchors)  |  On-Ice CEP50 = {cep50:.2f} m",
        fontsize=13.5,
        fontweight="bold",
    )

    def draw_nhl_rink(ax: plt.Axes) -> None:
        L, W, R = 60.96, 25.908, 8.5344
        rink_patch = patches.FancyBboxPatch(
            (-L / 2 + R, -W / 2 + R),
            L - 2 * R,
            W - 2 * R,
            boxstyle=f"round,pad={R}",
            ec="#1a1a1a",
            fc="#f4f9fd",
            lw=2.0,
            zorder=1,
        )
        ax.add_patch(rink_patch)
        ax.axvline(0.0, color="#c8102e", lw=2.0, alpha=0.6, zorder=2)
        for x_bl in (-L / 2 + 22.86, L / 2 - 22.86):
            ax.axvline(x_bl, color="#0033a0", lw=2.0, alpha=0.6, zorder=2)
        for x_gl in (-L / 2 + 3.3528, L / 2 - 3.3528):
            ax.axvline(x_gl, color="#c8102e", lw=1.2, alpha=0.5, zorder=2)
        circle = patches.Circle((0, 0), 4.572, ec="#0033a0", fc="none", lw=1.2, alpha=0.5, zorder=2)
        ax.add_patch(circle)
        ax.scatter(
            [-6.0],
            [12.954 - 0.25],
            marker="s",
            s=70,
            color="#2ca02c",
            zorder=6,
            label="Bench Gate (-6.0, 12.70 m)",
        )
        ax.set_xlim(-33, 33)
        ax.set_ylim(-15, 15)
        ax.set_aspect("equal")
        ax.set_xlabel("Rink X [m] (Center Ice = 0)")

    for idx, (ax, sh_gt, sh_est) in enumerate(zip(axes, gt["shifts"], rep["shifts"])):
        draw_nhl_rink(ax)
        m_g = (t_gt >= sh_gt["start_s"]) & (t_gt <= sh_gt["end_s"])
        m_e = (t_est >= sh_est["start_s"]) & (t_est <= sh_est["end_s"])
        ax.plot(gt_xy[m_g, 0], gt_xy[m_g, 1], color="#222222", lw=2.0, alpha=0.75, zorder=3, label="Ground Truth Path")
        sc = ax.scatter(
            est_xy[m_e, 0],
            est_xy[m_e, 1],
            c=spd_est[m_e],
            cmap="viridis",
            s=10,
            zorder=4,
            label="ES-EKF+RTS Path (color=speed)",
        )
        imp_m = m_e & (acc_est > 15.0)
        if np.any(imp_m):
            ax.scatter(
                est_xy[imp_m, 0],
                est_xy[imp_m, 1],
                marker="*",
                s=160,
                color="#d62728",
                edgecolor="black",
                zorder=7,
                label="High-g Impact (>15 g)",
            )
        s_start = sh_est["start_s"]
        s_end = sh_est["end_s"]
        s_dur = sh_est["duration_s"]
        s_dist = sh_est["distance_m"]
        g_dist = sh_gt["distance_m"]
        s_pk = sh_est["peak_speed_mps"]
        ax.set_title(
            f"Shift {idx+1}: {s_start:.1f}–{s_end:.1f} s ({s_dur:.1f} s)\n"
            f"Est Dist: {s_dist:.1f} m vs GT: {g_dist:.1f} m  |  Peak Speed: {s_pk:.1f} m/s",
            fontsize=10.5,
            fontweight="bold",
        )
        ax.legend(loc="lower right", fontsize=8)
        cb = fig.colorbar(sc, ax=ax, fraction=0.035, pad=0.02)
        cb.set_label("Speed [m/s]", fontsize=8.5)

    axes[0].set_ylabel("Rink Y [m] (+Y = Home Bench)")
    fig.tight_layout()
    for d in out_dirs:
        fig.savefig(d / "rink_2d_trajectories.png", dpi=160)
    plt.close(fig)

    return {
        "speed_rmse": rmse_spd,
        "cep50_m": cep50,
        "dist_err_pct": dist_err_pct,
        "est_dist_m": est_tot_dist,
        "gt_dist_m": gt_tot_dist,
        "peak_accel_g": est_peak_g,
    }


if __name__ == "__main__":
    metrics = generate_plots(
        [
            Path("/Users/frankspiteri/Documents/hockey_sensor/docs/plots"),
            Path("/Users/frankspiteri/.gemini/antigravity/brain/61db1d94-e94e-4b86-acab-08349fb0eebe"),
        ]
    )
    print("PLOTS_GENERATED_OK", metrics)
