"""Command-line interface for hockey_sensor simulation, download, fusion, and e2e."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

from . import DEFAULT_CONTRACT_PATH, load_contract
from .downloader import DownloadClient, SerialCdcTransport, SimulatedDeviceTransport
from .flash_codec import decode_hks_session
from .fusion_pipeline import run_fusion
from .physics_sim import SimConfig, simulate_session


def _validate_report_strict(report: dict[str, Any], contract_path: Path = DEFAULT_CONTRACT_PATH) -> None:
    from docs.validate_contract import validate_analytics_report_semantics

    contract = load_contract(contract_path)
    Draft202012Validator(contract["schemas"]["analytics_report"]).validate(report)
    validate_analytics_report_semantics(
        report,
        presets=contract["rink_geometry_presets"],
        q_tol=contract["coordinate_frames"]["quaternion_convention"]["unit_norm_tolerance"],
    )


def _dump_json_strict(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(data, indent=2, allow_nan=False)
    path.write_text(text + "\n", encoding="utf-8")


def _cmd_simulate(args: argparse.Namespace) -> int:
    cfg = SimConfig(
        rink_preset=args.preset,
        mount_location=args.mount,
        player_height_cm=args.player_height_cm,
        session_id=args.session_id,
        device_id=args.device_id,
        duration_s=args.duration_s,
        num_shifts=args.shifts,
        seed=args.seed,
    )
    res = simulate_session(cfg)
    out_hks = Path(args.out_hks)
    out_hks.parent.mkdir(parents=True, exist_ok=True)
    out_hks.write_bytes(res.hks_bytes)
    if args.out_truth:
        _dump_json_strict(Path(args.out_truth), res.ground_truth)
    print(
        f"SIMULATE_OK preset={cfg.rink_preset} bytes={len(res.hks_bytes)} "
        f"shifts={res.ground_truth['shift_count']} out={out_hks}"
    )
    return 0


def _cmd_download(args: argparse.Namespace) -> int:
    if args.transport == "sim":
        if args.hks_source:
            raw_src = Path(args.hks_source).read_bytes()
        else:
            raw_src = simulate_session(SimConfig(seed=args.seed)).hks_bytes
        transport: Any = SimulatedDeviceTransport([raw_src])
    else:
        if not args.port:
            raise ValueError("--port is required when --transport serial")
        transport = SerialCdcTransport(args.port, baudrate=args.baud)

    try:
        client = DownloadClient(transport, record_trace=bool(args.out_trace))
        if args.status_only:
            st = client.get_device_status()
            print(
                f"DEVICE_STATUS fw=0x{st.fw_version:08X} hw=0x{st.hw_rev:04X} "
                f"vbatt_mv={st.vbatt_mv} soc={st.soc_pct_x10/10.0:.1f}% "
                f"pmic=0x{st.pmic_status:02X} free_kb={st.flash_free_kb}"
            )
            if args.out_trace:
                _dump_json_strict(Path(args.out_trace), client.trace)
            return 0

        sessions = client.list_sessions()
        if args.list_only:
            for s in sessions:
                print(
                    f"SESSION id={s.session_id} bytes={s.byte_size} "
                    f"duration_ms={s.duration_ms} samples={s.sample_count}"
                )
            if args.out_trace:
                _dump_json_strict(Path(args.out_trace), client.trace)
            return 0

        if not sessions:
            raise RuntimeError("No sessions found on device")
        target_id = args.session_id if args.session_id is not None else sessions[-1].session_id
        dl = client.download_session(
            target_id,
            chunk_size=args.chunk_size,
            verify=True,
            delete_after=args.delete_after,
            strict_decode=True,
        )
        if args.out_hks:
            out_hks = Path(args.out_hks)
            out_hks.parent.mkdir(parents=True, exist_ok=True)
            out_hks.write_bytes(dl.raw_bytes)
        if args.out_trace:
            _dump_json_strict(Path(args.out_trace), client.trace)
        print(
            f"DOWNLOAD_OK session_id={dl.session_id} bytes={len(dl.raw_bytes)} "
            f"crc32=0x{dl.file_crc32:08X} hash_mode={dl.hash_mode} deleted={args.delete_after}"
        )
        return 0
    finally:
        if hasattr(transport, "close"):
            transport.close()


def _cmd_analyze(args: argparse.Namespace) -> int:
    decoded = decode_hks_session(Path(args.in_hks).read_bytes(), require_footer=True, verify_so3=True)
    report = run_fusion(decoded)
    if args.strict_contract:
        _validate_report_strict(report)
    _dump_json_strict(Path(args.out_report), report)
    ss = report["session_summary"]
    print(
        f"ANALYZE_OK preset={ss['rink_preset']} shifts={ss['shift_count']} "
        f"ice_s={ss['total_ice_time_s']:.2f} dist_m={ss['total_distance_m']:.2f} "
        f"peak_v={ss['peak_speed_mps']:.2f} peak_a={ss['peak_accel_mps2']:.2f} out={args.out_report}"
    )
    return 0


def _cmd_e2e(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = SimConfig(
        rink_preset=args.preset,
        duration_s=args.duration_s,
        num_shifts=args.shifts,
        seed=args.seed,
    )
    sim_res = simulate_session(cfg)
    sim_hks_path = out_dir / "session.hks"
    sim_hks_path.write_bytes(sim_res.hks_bytes)

    transport = SimulatedDeviceTransport([sim_res.hks_bytes])
    client = DownloadClient(transport, record_trace=True)
    sessions = client.list_sessions()
    if not sessions:
        raise RuntimeError("SimulatedDeviceTransport returned empty session list")
    dl = client.download_session(
        sessions[0].session_id,
        chunk_size=4096,
        verify=True,
        delete_after=args.delete_after,
        strict_decode=True,
    )
    dl_hks_path = out_dir / "downloaded.hks"
    dl_hks_path.write_bytes(dl.raw_bytes)

    decoded = decode_hks_session(dl.raw_bytes, require_footer=True, verify_so3=True, validate_contract_schemas=False)
    report = run_fusion(decoded)
    _validate_report_strict(report)

    report_path = out_dir / "analytics_report.json"
    _dump_json_strict(report_path, report)

    summary = {
        "preset": cfg.rink_preset,
        "seed": cfg.seed,
        "hks_bytes": len(dl.raw_bytes),
        "file_crc32": dl.file_crc32,
        "file_sha256": dl.file_sha256.hex(),
        "hash_mode": dl.hash_mode,
        "deleted_after_verify": args.delete_after,
        "session_summary": report["session_summary"],
        "shifts": report["shifts"],
    }
    summary_path = out_dir / "e2e_summary.json"
    _dump_json_strict(summary_path, summary)

    ss = report["session_summary"]
    print(
        f"E2E_OK preset={ss['rink_preset']} shifts={ss['shift_count']} "
        f"dist_m={ss['total_distance_m']:.2f} peak_v={ss['peak_speed_mps']:.2f} "
        f"peak_a={ss['peak_accel_mps2']:.2f} out_dir={out_dir}"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python3 -m sim.cli", description="hockey_sensor simulation & fusion CLI")
    sub = parser.add_subparsers(dest="subcommand", required=True)

    p_sim = sub.add_parser("simulate", help="Generate synthetic 6-DOF .hks telemetry")
    p_sim.add_argument("--preset", choices=["NHL_200X85FT", "IIHF_60X30M", "CUSTOM"], default="NHL_200X85FT")
    p_sim.add_argument("--duration-s", type=float, default=140.0)
    p_sim.add_argument("--shifts", type=int, default=2)
    p_sim.add_argument("--seed", type=int, default=42)
    p_sim.add_argument("--session-id", type=int, default=1001)
    p_sim.add_argument("--device-id", type=int, default=0x00005CC800010203)
    p_sim.add_argument("--mount", choices=["STERNUM_POCKET", "CHEST_STRAP"], default="STERNUM_POCKET")
    p_sim.add_argument("--player-height-cm", type=int, default=183)
    p_sim.add_argument("--out-hks", required=True)
    p_sim.add_argument("--out-truth")

    p_dl = sub.add_parser("download", help="Download session over USB CDC-ACM or simulated transport")
    p_dl.add_argument("--transport", choices=["sim", "serial"], default="sim")
    p_dl.add_argument("--hks-source")
    p_dl.add_argument("--port")
    p_dl.add_argument("--baud", type=int, default=115200)
    p_dl.add_argument("--seed", type=int, default=42)
    p_dl.add_argument("--session-id", type=int)
    p_dl.add_argument("--chunk-size", type=int, default=4096)
    p_dl.add_argument("--list-only", action="store_true")
    p_dl.add_argument("--status-only", action="store_true")
    p_dl.add_argument("--delete-after", action="store_true")
    p_dl.add_argument("--out-hks")
    p_dl.add_argument("--out-trace")

    p_an = sub.add_parser("analyze", help="Decode .hks and run anchor-free ES-EKF + RTS fusion")
    p_an.add_argument("--in-hks", required=True)
    p_an.add_argument("--out-report", required=True)
    p_an.add_argument("--strict-contract", action=argparse.BooleanOptionalAction, default=True)

    p_e2e = sub.add_parser("e2e", help="Run end-to-end simulate -> download -> analyze -> validate")
    p_e2e.add_argument("--preset", choices=["NHL_200X85FT", "IIHF_60X30M", "CUSTOM"], default="NHL_200X85FT")
    p_e2e.add_argument("--duration-s", type=float, default=140.0)
    p_e2e.add_argument("--shifts", type=int, default=2)
    p_e2e.add_argument("--seed", type=int, default=42)
    p_e2e.add_argument("--out-dir", default="sim_out")
    p_e2e.add_argument("--delete-after", action="store_true")

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        return int(exc.code) if isinstance(exc.code, int) else 2

    try:
        if args.subcommand == "simulate":
            return _cmd_simulate(args)
        if args.subcommand == "download":
            return _cmd_download(args)
        if args.subcommand == "analyze":
            return _cmd_analyze(args)
        if args.subcommand == "e2e":
            return _cmd_e2e(args)
        parser.print_usage(sys.stderr)
        return 2
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
