#!/usr/bin/env python3
"""Dynamic cold-rated power, ohmic sag, NAND storage, and mechanical budget calculator.

Reads INTERFACE_CONTRACT.json, hardware/ee/bom.csv, hardware/mechanical/dimensions.json,
and hardware/mechanical/bom.csv dynamically with zero hardcoded magic numbers.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import struct
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONTRACT_PATH = REPO_ROOT / "INTERFACE_CONTRACT.json"
DEFAULT_EE_BOM_PATH = REPO_ROOT / "hardware" / "ee" / "bom.csv"
DEFAULT_ME_DIMS_PATH = REPO_ROOT / "hardware" / "mechanical" / "dimensions.json"
DEFAULT_ME_BOM_PATH = REPO_ROOT / "hardware" / "mechanical" / "bom.csv"

FORBIDDEN_RF_KEYWORDS = (
    "gps",
    "gnss",
    "uwb",
    "dw3000",
    "dw1000",
    "sr150",
    "ublox",
    "u-blox",
    "rink_anchor",
    "external_anchor",
)


class BudgetError(ValueError):
    """Raised when input files are malformed or contain non-standard values."""


def _reject_non_standard_json_constant(constant: str) -> Any:
    raise BudgetError(f"Non-standard JSON constant '{constant}' is forbidden (strict RFC 8259 required)")


def load_json_strict(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f, parse_constant=_reject_non_standard_json_constant)


def _parse_float(raw: str | None, field: str, row_id: str, default: float | None = None) -> float:
    text = (raw or "").strip()
    if text == "":
        if default is not None:
            return default
        raise BudgetError(f"Missing required float '{field}' in row '{row_id}'")
    val = float(text)
    if not math.isfinite(val):
        raise BudgetError(f"Non-finite float '{text}' for '{field}' in row '{row_id}'")
    return val


def load_ee_bom(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required_cols = {
            "refdes",
            "subsystem_key",
            "category",
            "part_number",
            "qty",
            "rail_v",
            "i_nom_ma",
            "i_max_ma",
            "i_standby_ua",
            "buck_efficiency",
            "r_int_25c_mohm",
            "r_int_0c_mohm",
            "uvlo_mv",
            "unit_mass_g",
            "total_mass_g",
        }
        if reader.fieldnames is None or not required_cols.issubset(set(reader.fieldnames)):
            missing = sorted(required_cols - set(reader.fieldnames or []))
            raise BudgetError(f"EE BOM missing required columns: {missing}")

        for idx, raw in enumerate(reader, start=2):
            row_id = raw.get("refdes") or f"line_{idx}"
            rows.append(
                {
                    "refdes": (raw["refdes"] or "").strip(),
                    "subsystem_key": (raw["subsystem_key"] or "").strip(),
                    "category": (raw["category"] or "").strip(),
                    "part_number": (raw["part_number"] or "").strip(),
                    "manufacturer": (raw.get("manufacturer") or "").strip(),
                    "package": (raw.get("package") or "").strip(),
                    "qty": int(_parse_float(raw["qty"], "qty", row_id)),
                    "rail_v": _parse_float(raw["rail_v"], "rail_v", row_id),
                    "i_nom_ma": _parse_float(raw["i_nom_ma"], "i_nom_ma", row_id, 0.0),
                    "i_max_ma": _parse_float(raw["i_max_ma"], "i_max_ma", row_id, 0.0),
                    "i_standby_ua": _parse_float(raw["i_standby_ua"], "i_standby_ua", row_id, 0.0),
                    "buck_efficiency": (
                        _parse_float(raw["buck_efficiency"], "buck_efficiency", row_id)
                        if (raw["buck_efficiency"] or "").strip()
                        else None
                    ),
                    "r_int_25c_mohm": (
                        _parse_float(raw["r_int_25c_mohm"], "r_int_25c_mohm", row_id)
                        if (raw["r_int_25c_mohm"] or "").strip()
                        else None
                    ),
                    "r_int_0c_mohm": (
                        _parse_float(raw["r_int_0c_mohm"], "r_int_0c_mohm", row_id)
                        if (raw["r_int_0c_mohm"] or "").strip()
                        else None
                    ),
                    "uvlo_mv": (
                        _parse_float(raw["uvlo_mv"], "uvlo_mv", row_id)
                        if (raw["uvlo_mv"] or "").strip()
                        else None
                    ),
                    "unit_mass_g": _parse_float(raw["unit_mass_g"], "unit_mass_g", row_id),
                    "total_mass_g": _parse_float(raw["total_mass_g"], "total_mass_g", row_id),
                    "bus_or_function": (raw.get("bus_or_function") or "").strip(),
                    "notes": (raw.get("notes") or "").strip(),
                }
            )
    if not rows:
        raise BudgetError("EE BOM is empty")
    return rows


def load_me_bom(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required_cols = {
            "item_id",
            "assembly_scope",
            "category",
            "part_number",
            "qty",
            "unit_mass_g",
            "total_mass_g",
        }
        if reader.fieldnames is None or not required_cols.issubset(set(reader.fieldnames)):
            missing = sorted(required_cols - set(reader.fieldnames or []))
            raise BudgetError(f"ME BOM missing required columns: {missing}")

        for idx, raw in enumerate(reader, start=2):
            row_id = raw.get("item_id") or f"line_{idx}"
            rows.append(
                {
                    "item_id": (raw["item_id"] or "").strip(),
                    "assembly_scope": (raw["assembly_scope"] or "").strip(),
                    "category": (raw["category"] or "").strip(),
                    "part_number": (raw["part_number"] or "").strip(),
                    "description": (raw.get("description") or "").strip(),
                    "qty": int(_parse_float(raw["qty"], "qty", row_id)),
                    "unit_mass_g": _parse_float(raw["unit_mass_g"], "unit_mass_g", row_id),
                    "total_mass_g": _parse_float(raw["total_mass_g"], "total_mass_g", row_id),
                    "ip_role": (raw.get("ip_role") or "").strip(),
                    "notes": (raw.get("notes") or "").strip(),
                }
            )
    if not rows:
        raise BudgetError("ME BOM is empty")
    return rows


def _frame_bytes_from_contract(
    bff: dict[str, Any],
    msg_name: str,
    sample_count: int | None = None,
) -> int:
    overhead = int(bff["frame_header_bytes"]) + int(bff["frame_crc_bytes"])
    msg = bff["message_types"][msg_name]
    fmt = msg["struct_format"]
    if "+ N*" in fmt:
        if sample_count is None:
            raise BudgetError(f"sample_count required for variable-length message '{msg_name}'")
        hdr_fmt, smp_fmt = [part.strip() for part in fmt.split("+ N*", 1)]
        payload_bytes = struct.calcsize(hdr_fmt) + struct.calcsize(smp_fmt) * sample_count
    else:
        payload_bytes = struct.calcsize(fmt)
    return overhead + payload_bytes


def _round_opt(val: float | None, digits: int) -> float | None:
    return None if val is None else round(val, digits)


def _battery_current_from_rails(
    rail_currents_ma: dict[float, float],
    v_batt_nom: float,
    buck_eta: float,
) -> float:
    if v_batt_nom <= 0.0 or buck_eta <= 0.0:
        return 0.0
    total_batt_ma = 0.0
    for rail_v, i_ma in rail_currents_ma.items():
        if rail_v <= 0.0 or i_ma == 0.0:
            continue
        if abs(rail_v - v_batt_nom) < 1e-6:
            total_batt_ma += i_ma
        else:
            total_batt_ma += i_ma * rail_v / (v_batt_nom * buck_eta)
    return total_batt_ma


def _detect_forbidden_rf_parts(
    ee_rows: list[dict[str, Any]],
    me_rows: list[dict[str, Any]] | None = None,
) -> list[str]:
    hits: list[str] = []
    pattern = re.compile("(" + "|".join(re.escape(k) for k in FORBIDDEN_RF_KEYWORDS) + ")", re.IGNORECASE)
    for row in ee_rows:
        for field in ("part_number", "subsystem_key", "category", "manufacturer", "bus_or_function", "notes"):
            val = row.get(field, "")
            if pattern.search(val):
                hits.append(f"{row['refdes']}:{field}={val}")
    if me_rows is not None:
        for row in me_rows:
            for field in ("part_number", "assembly_scope", "category", "description", "notes"):
                val = row.get(field, "")
                if pattern.search(val):
                    hits.append(f"{row['item_id']}:{field}={val}")
    return hits


def _validate_r_bs(me_r_bs: Any, canonical_r_bs: Any) -> list[str]:
    errors: list[str] = []
    if (
        not isinstance(me_r_bs, list)
        or len(me_r_bs) != 3
        or not all(isinstance(row, list) and len(row) == 3 for row in me_r_bs)
    ):
        return ["coordinate_frame_markings.R_BS must be a 3x3 matrix"]
    try:
        mat = []
        for row in me_r_bs:
            f_row = []
            for val in row:
                if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(float(val)):
                    return ["coordinate_frame_markings.R_BS contains non-numeric or non-finite entry"]
                f_row.append(float(val))
            mat.append(f_row)
    except (TypeError, ValueError):
        return ["coordinate_frame_markings.R_BS failed numeric conversion"]

    ortho_err = 0.0
    for i in range(3):
        for j in range(3):
            dot = sum(mat[k][i] * mat[k][j] for k in range(3))
            ortho_err = max(ortho_err, abs(dot - (1.0 if i == j else 0.0)))
    if ortho_err >= 1e-6:
        errors.append(f"coordinate_frame_markings.R_BS is not orthogonal (||R^T R - I||_max={ortho_err:.3e} >= 1e-6)")

    det = (
        mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1])
        - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
        + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0])
    )
    if abs(det - 1.0) >= 1e-6:
        errors.append(f"coordinate_frame_markings.R_BS determinant={det:.6f} != +1.0")

    max_diff = max(
        abs(mat[i][j] - float(canonical_r_bs[i][j]))
        for i in range(3)
        for j in range(3)
    )
    if max_diff >= 1e-6:
        errors.append(f"coordinate_frame_markings.R_BS does not match contract canonical_R_BS (max_diff={max_diff:.3e})")
    return errors


def compute_budgets(
    contract: dict[str, Any],
    ee_rows: list[dict[str, Any]],
    me_dims: dict[str, Any],
    me_rows: list[dict[str, Any]] | None = None,
    *,
    cold_derating_frac: float = 0.20,
    aging_derating_frac: float = 0.15,
    ble_beacon_delta_1v8_ma: float = 0.080,
    ble_offload_1v8_ma: float = 14.000,
    ble_offload_throughput_bps: float = 125_000.0,
    usb_offload_throughput_bps: float = 650_000.0,
    soc_10pct_ocv_cold_mv: float = 3550.0,
) -> dict[str, Any]:
    constants = contract["constants"]
    hw = contract["hardware_abstraction"]
    bff = contract["binary_flash_format"]
    u_and_t = contract["units_and_tolerances"]

    v_sys = float(hw["system_voltage_v"])
    min_life_h = float(hw["min_battery_life_hours"])
    batt_cfg = hw["battery"]
    v_batt = float(batt_cfg["nominal_v"])
    cap_nom_mah = float(batt_cfg["capacity_mah"])

    # Cold + aged capacity
    cap_add_mah = cap_nom_mah * (1.0 - cold_derating_frac - aging_derating_frac)
    cap_mul_mah = cap_nom_mah * (1.0 - cold_derating_frac) * (1.0 - aging_derating_frac)

    # Extract PMIC & battery electrical parameters from EE BOM
    buck_eta_vals = [r["buck_efficiency"] for r in ee_rows if r["buck_efficiency"] is not None]
    if not buck_eta_vals:
        raise BudgetError("EE BOM missing buck_efficiency value")
    buck_eta = float(buck_eta_vals[0])
    if not (0.0 < buck_eta <= 1.0):
        raise BudgetError(f"Invalid buck_efficiency={buck_eta}")

    r25_vals = [r["r_int_25c_mohm"] for r in ee_rows if r["r_int_25c_mohm"] is not None]
    r0_vals = [r["r_int_0c_mohm"] for r in ee_rows if r["r_int_0c_mohm"] is not None]
    uvlo_vals = [r["uvlo_mv"] for r in ee_rows if r["uvlo_mv"] is not None]
    if not r25_vals or not r0_vals or not uvlo_vals:
        raise BudgetError("EE BOM missing battery r_int_25c_mohm, r_int_0c_mohm, or uvlo_mv")
    r_int_25c_mohm = float(r25_vals[0])
    r_int_0c_mohm = float(r0_vals[0])
    uvlo_mv = float(uvlo_vals[0])

    batt_param_failures: list[str] = []
    if not math.isfinite(v_batt) or v_batt <= 0.0:
        batt_param_failures.append(f"Battery nominal_v must be > 0 (got {v_batt})")
    if not math.isfinite(cap_nom_mah) or cap_nom_mah <= 0.0:
        batt_param_failures.append(f"Battery capacity_mah must be > 0 (got {cap_nom_mah})")
    if any(float(v) < 0.0 for v in r25_vals):
        batt_param_failures.append(f"Battery r_int_25c_mohm must be >= 0 (got {r25_vals})")
    if any(float(v) < 0.0 for v in r0_vals):
        batt_param_failures.append(f"Battery r_int_0c_mohm must be >= 0 (got {r0_vals})")
    if any(float(v) <= 0.0 for v in uvlo_vals):
        batt_param_failures.append(f"Battery uvlo_mv must be > 0 (got {uvlo_vals})")

    # Aggregate rail currents dynamically from EE BOM
    rail_nom_ma: dict[float, float] = {}
    rail_max_ma: dict[float, float] = {}
    rail_standby_ma: dict[float, float] = {}
    ee_row_failures: list[str] = []

    for row in ee_rows:
        ref = row["refdes"] or "<unknown>"
        qty = int(row["qty"])
        i_nom = float(row["i_nom_ma"])
        i_max = float(row["i_max_ma"])
        i_stby = float(row["i_standby_ua"])
        unit_g = float(row["unit_mass_g"])
        total_g = float(row["total_mass_g"])
        rv = round(float(row["rail_v"]), 4)
        if qty < 0:
            ee_row_failures.append(f"EE BOM row {ref} has negative qty={qty}")
        if i_nom < 0.0 or i_max < 0.0 or i_stby < 0.0:
            ee_row_failures.append(f"EE BOM row {ref} has negative current (nom={i_nom}, max={i_max}, stby={i_stby})")
        if unit_g < 0.0 or total_g < 0.0 or abs(qty * unit_g - total_g) > 1e-4:
            ee_row_failures.append(f"EE BOM row {ref} mass mismatch: qty={qty}, unit={unit_g}, total={total_g}")
        if qty == 0:
            if i_nom != 0.0 or i_max != 0.0 or i_stby != 0.0:
                ee_row_failures.append(
                    f"EE BOM DNI row {ref} (qty=0) has non-zero current "
                    f"(nom={i_nom}, max={i_max}, stby={i_stby})"
                )
            continue
        if rv <= 0.0:
            if i_nom != 0.0 or i_max != 0.0 or i_stby != 0.0:
                ee_row_failures.append(
                    f"EE BOM row {ref} has non-positive rail_v={rv} with non-zero current "
                    f"(nom={i_nom}, max={i_max}, stby={i_stby})"
                )
            continue
        rail_nom_ma[rv] = rail_nom_ma.get(rv, 0.0) + i_nom
        rail_max_ma[rv] = rail_max_ma.get(rv, 0.0) + i_max
        rail_standby_ma[rv] = rail_standby_ma.get(rv, 0.0) + (i_stby / 1000.0)

    # Mode 1: Nominal Play Logging
    i_batt_nom_ma = _battery_current_from_rails(rail_nom_ma, v_batt, buck_eta)
    p_batt_nom_mw = i_batt_nom_ma * v_batt
    t_nom_add_h = cap_add_mah / i_batt_nom_ma if i_batt_nom_ma > 0.0 else None
    t_nom_mul_h = cap_mul_mah / i_batt_nom_ma if i_batt_nom_ma > 0.0 else None

    # Mode 1b: Worst-Case Logging
    i_batt_wc_ma = _battery_current_from_rails(rail_max_ma, v_batt, buck_eta)
    p_batt_wc_mw = i_batt_wc_ma * v_batt
    t_wc_add_h = cap_add_mah / i_batt_wc_ma if i_batt_wc_ma > 0.0 else None
    t_wc_mul_h = cap_mul_mah / i_batt_wc_ma if i_batt_wc_ma > 0.0 else None

    # Mode 2: Play + 1 Hz BLE Beacon
    rail_beacon_ma = dict(rail_nom_ma)
    rail_beacon_ma[round(v_sys, 4)] = rail_beacon_ma.get(round(v_sys, 4), 0.0) + ble_beacon_delta_1v8_ma
    i_batt_beacon_ma = _battery_current_from_rails(rail_beacon_ma, v_batt, buck_eta)
    t_beacon_add_h = cap_add_mah / i_batt_beacon_ma if i_batt_beacon_ma > 0.0 else None

    # Mode 3: BLE 5.3 2M Bulk Offload
    vbat_pmic_nom_ma = rail_nom_ma.get(round(v_batt, 4), 0.0)
    rail_ble_offload_ma = {round(v_sys, 4): ble_offload_1v8_ma, round(v_batt, 4): vbat_pmic_nom_ma}
    i_batt_ble_offload_ma = _battery_current_from_rails(rail_ble_offload_ma, v_batt, buck_eta)
    p_batt_ble_offload_mw = i_batt_ble_offload_ma * v_batt

    # Mode 5a: WoM Standby
    i_batt_standby_ma = _battery_current_from_rails(rail_standby_ma, v_batt, buck_eta)
    t_standby_add_h = cap_add_mah / i_batt_standby_ma if i_batt_standby_ma > 0.0 else None

    # Mode 5b: Ship Mode (PMIC ship quiescent 0.37 uA + battery PCM/self-discharge from BOM)
    batt_standby_ua = sum(
        float(r["i_standby_ua"]) for r in ee_rows if r["subsystem_key"] == "battery" and int(r["qty"]) > 0
    )
    i_batt_ship_ua = 0.37 + batt_standby_ua
    t_ship_h = cap_add_mah / (i_batt_ship_ua / 1000.0) if i_batt_ship_ua > 0.0 else None

    # Cold 0 degC ohmic voltage sag & UVLO margin
    r_0c_ohm = r_int_0c_mohm / 1000.0
    sag_wc_continuous_mv = i_batt_wc_ma * r_0c_ohm
    # Transient peak burst: worst-case logging + W25N01JW 25 mA QSPI program/erase + 6.5 mA BLE TX peak
    rail_transient_ma = dict(rail_max_ma)
    rail_transient_ma[round(v_sys, 4)] = rail_transient_ma.get(round(v_sys, 4), 0.0) + 25.0 + 6.5
    i_batt_transient_peak_ma = _battery_current_from_rails(rail_transient_ma, v_batt, buck_eta)
    sag_transient_peak_mv = i_batt_transient_peak_ma * r_0c_ohm

    v_loaded_wc_10pct_mv = soc_10pct_ocv_cold_mv - sag_wc_continuous_mv
    v_loaded_transient_10pct_mv = soc_10pct_ocv_cold_mv - sag_transient_peak_mv
    uvlo_margin_wc_mv = v_loaded_wc_10pct_mv - uvlo_mv
    uvlo_margin_transient_mv = v_loaded_transient_10pct_mv - uvlo_mv

    # NAND Flash Storage Budget (dynamically computed from contract binary_flash_format)
    flash_cfg = hw["flash"]
    sensors_cfg = hw["sensors"]
    page_bytes = int(flash_cfg["page_bytes"])
    block_bytes = int(flash_cfg["block_bytes"])
    usable_blocks = int(flash_cfg["usable_ring_blocks"])
    usable_ring_bytes = usable_blocks * block_bytes

    imu_odr = float(sensors_cfg["primary_imu"]["odr_hz"])
    mag_odr = float(sensors_cfg["magnetometer"]["odr_hz"])
    imu_batch_n = 20
    imu_frame_bytes = _frame_bytes_from_contract(bff, "IMU_BATCH", imu_batch_n)
    imu_rate_bps = imu_frame_bytes * (imu_odr / imu_batch_n)

    aux_frame_bytes = _frame_bytes_from_contract(bff, "AUX_ENV")
    aux_rate_bps = aux_frame_bytes * mag_odr

    event_frame_bytes = _frame_bytes_from_contract(bff, "EVENT_MARKER")
    event_rate_bps = event_frame_bytes * 0.1

    high_g_burst_n = 512
    high_g_burst_frame_bytes = _frame_bytes_from_contract(bff, "HIGH_G_BATCH", high_g_burst_n)
    high_g_burst_rate_bps = high_g_burst_frame_bytes * (60.0 / 3600.0)

    nominal_total_bps = imu_rate_bps + aux_rate_bps + high_g_burst_rate_bps + event_rate_bps
    nominal_mb_per_hr = nominal_total_bps * 3600.0 / 1e6
    nominal_2h_bytes = nominal_total_bps * 7200.0
    nominal_2h_mb = nominal_2h_bytes / 1e6
    nand_nominal_h = usable_ring_bytes / (nominal_total_bps * 3600.0)

    # Continuous 400 Hz High-g fallback modes
    hg_400_n40_frame_bytes = _frame_bytes_from_contract(bff, "HIGH_G_BATCH", 40)
    hg_400_n40_bps = hg_400_n40_frame_bytes * 10.0
    total_400_n40_bps = imu_rate_bps + aux_rate_bps + hg_400_n40_bps + event_rate_bps
    nand_400_n40_h = usable_ring_bytes / (total_400_n40_bps * 3600.0)

    hg_400_n50_frame_bytes = _frame_bytes_from_contract(bff, "HIGH_G_BATCH", 50)
    hg_400_n50_bps = hg_400_n50_frame_bytes * 8.0
    total_400_n50_bps = imu_rate_bps + aux_rate_bps + hg_400_n50_bps + event_rate_bps
    nand_400_n50_h = usable_ring_bytes / (total_400_n50_bps * 3600.0)

    # Offload energy and durations for a 2h nominal session
    ble_offload_duration_s = nominal_2h_bytes / ble_offload_throughput_bps
    ble_offload_energy_mah = i_batt_ble_offload_ma * (ble_offload_duration_s / 3600.0)
    ble_offload_pct_usable = 100.0 * ble_offload_energy_mah / cap_add_mah if cap_add_mah > 0.0 else 0.0
    usb_offload_duration_s = nominal_2h_bytes / usb_offload_throughput_bps

    # Mass reconciliation & mechanical envelope checks
    ee_non_battery_mass_g = sum(float(r["total_mass_g"]) for r in ee_rows if r["subsystem_key"] != "battery")
    ee_battery_mass_g = sum(float(r["total_mass_g"]) for r in ee_rows if r["subsystem_key"] == "battery")

    env = me_dims["envelope_mm"]
    mass_cfg = me_dims["mass_budget_g"]
    z_cfg = me_dims["tolerance_stackup_z_mm"]
    z_nom_calc = sum(float(layer["nominal_mm"]) for layer in z_cfg["layers"])
    z_wc_calc = sum(float(layer["nominal_mm"]) + float(layer["plus_tol_mm"]) for layer in z_cfg["layers"])
    dims_mass_sum_g = round(sum(float(v) for v in mass_cfg["breakdown_g"].values()), 4)
    dims_with_strap_mass_g = round(dims_mass_sum_g + float(mass_cfg["optional_strap_mass_g"]), 4)

    me_row_failures: list[str] = []
    me_mass_summary: dict[str, float] = {}
    if me_rows is not None:
        for row in me_rows:
            item_id = row["item_id"] or "<unknown>"
            qty = int(row["qty"])
            unit_g = float(row["unit_mass_g"])
            total_g = float(row["total_mass_g"])
            if qty < 0:
                me_row_failures.append(f"ME BOM row {item_id} has negative qty={qty}")
            if unit_g < 0.0 or total_g < 0.0 or abs(qty * unit_g - total_g) > 1e-4:
                me_row_failures.append(
                    f"ME BOM row {item_id} mass mismatch: qty={qty}, unit={unit_g}, total={total_g}"
                )
        for scope in ("pod_battery", "pod_ee", "pod_me", "accessory_strap"):
            me_mass_summary[scope] = round(
                sum(float(r["total_mass_g"]) for r in me_rows if r["assembly_scope"] == scope), 4
            )
        pod_mass_g = round(
            me_mass_summary["pod_battery"] + me_mass_summary["pod_ee"] + me_mass_summary["pod_me"], 4
        )
        pod_with_strap_mass_g = round(pod_mass_g + me_mass_summary["accessory_strap"], 4)
    else:
        pod_mass_g = dims_mass_sum_g
        pod_with_strap_mass_g = dims_with_strap_mass_g

    # Gate checks
    forbidden_rf_hits = _detect_forbidden_rf_parts(ee_rows, me_rows)
    gate_failures: list[str] = list(batt_param_failures) + list(ee_row_failures) + list(me_row_failures)

    if constants.get("gps_enabled") is not False or hw.get("uses_gps") is not False:
        gate_failures.append("GPS must be disabled in contract")
    if constants.get("external_anchors_enabled") is not False or hw.get("uses_external_anchors") is not False:
        gate_failures.append("External anchors must be disabled in contract")
    if u_and_t.get("gps_or_anchors_used") is not False:
        gate_failures.append("units_and_tolerances.gps_or_anchors_used must be false")
    if forbidden_rf_hits:
        gate_failures.append(f"Forbidden GPS/UWB/anchor parts in BOM: {forbidden_rf_hits}")

    if t_nom_add_h is None:
        gate_failures.append(f"Nominal battery current must be > 0 (got {i_batt_nom_ma:.4f} mA)")
    elif t_nom_add_h < min_life_h:
        gate_failures.append(f"Nominal runtime {t_nom_add_h:.2f} h < {min_life_h:.2f} h")
    if t_wc_add_h is None:
        gate_failures.append(f"Worst-case battery current must be > 0 (got {i_batt_wc_ma:.4f} mA)")
    elif t_wc_add_h < min_life_h:
        gate_failures.append(f"Worst-case runtime {t_wc_add_h:.2f} h < {min_life_h:.2f} h")
    if uvlo_margin_transient_mv <= 0.0:
        gate_failures.append(f"Cold UVLO margin violated: {uvlo_margin_transient_mv:.2f} mV <= 0")

    if nand_nominal_h < min_life_h:
        gate_failures.append(f"NAND nominal retention {nand_nominal_h:.2f} h < {min_life_h:.2f} h")
    if nand_400_n40_h < min_life_h:
        gate_failures.append(f"NAND 400Hz N=40 retention {nand_400_n40_h:.2f} h < {min_life_h:.2f} h")
    if nand_400_n50_h < min_life_h:
        gate_failures.append(f"NAND 400Hz N=50 retention {nand_400_n50_h:.2f} h < {min_life_h:.2f} h")

    hw_env = hw["enclosure_envelope_mm"]
    ei = me_dims.get("environmental_and_impact", {})
    length_mm = float(env["length_mm"])
    width_mm = float(env["width_mm"])
    thickness_mm = float(env["thickness_mm"])
    if (
        length_mm > float(env["max_allowed_length_mm"])
        or length_mm > 55.0
        or abs(length_mm - float(hw_env["length_mm"])) > 1e-6
    ):
        gate_failures.append(
            f"Enclosure length_mm={length_mm} violates ceiling (<=55.0) or contract ({hw_env['length_mm']})"
        )
    if (
        width_mm > float(env["max_allowed_width_mm"])
        or width_mm > 40.0
        or abs(width_mm - float(hw_env["width_mm"])) > 1e-6
    ):
        gate_failures.append(
            f"Enclosure width_mm={width_mm} violates ceiling (<=40.0) or contract ({hw_env['width_mm']})"
        )
    if (
        thickness_mm > float(env["max_allowed_thickness_mm"])
        or thickness_mm > 12.0
        or abs(thickness_mm - float(hw_env["thickness_mm"])) > 1e-6
        or z_wc_calc > float(env["max_allowed_thickness_mm"])
        or z_wc_calc > 12.0
    ):
        gate_failures.append(
            f"Enclosure thickness_mm={thickness_mm} / z_wc={z_wc_calc:.2f} violates ceiling (<=12.0) or contract ({hw_env['thickness_mm']})"
        )
    if ei.get("ip_rating") != hw_env.get("ip_rating"):
        gate_failures.append(
            f"Enclosure ip_rating={ei.get('ip_rating')!r} != contract {hw_env.get('ip_rating')!r}"
        )

    if dims_mass_sum_g > float(mass_cfg["max_pod_mass_g"]) or dims_mass_sum_g > 28.0:
        gate_failures.append(
            f"dimensions.json breakdown_g sum {dims_mass_sum_g:.2f} g > pod ceiling (<=28.0 g)"
        )
    if dims_with_strap_mass_g > float(mass_cfg["max_contract_mass_g"]) or dims_with_strap_mass_g > 40.0:
        gate_failures.append(
            f"dimensions.json pod+strap mass {dims_with_strap_mass_g:.2f} g > contract ceiling (<=40.0 g)"
        )
    if pod_mass_g > float(mass_cfg["max_pod_mass_g"]) or pod_mass_g > 28.0:
        gate_failures.append(f"Pod mass {pod_mass_g:.2f} g > pod ceiling (<=28.0 g)")
    if pod_with_strap_mass_g > float(mass_cfg["max_contract_mass_g"]) or pod_with_strap_mass_g > 40.0:
        gate_failures.append(
            f"Pod+strap mass {pod_with_strap_mass_g:.2f} g > contract ceiling (<=40.0 g)"
        )
    if me_rows is not None:
        if abs(dims_mass_sum_g - pod_mass_g) > 0.05:
            gate_failures.append(
                f"dimensions.json breakdown_g sum ({dims_mass_sum_g:.3f} g) != ME BOM pod mass ({pod_mass_g:.3f} g)"
            )
        if abs(me_mass_summary["pod_ee"] - ee_non_battery_mass_g) > 0.05:
            gate_failures.append(
                f"ME pod_ee ({me_mass_summary['pod_ee']:.3f} g) != EE non-battery ({ee_non_battery_mass_g:.3f} g)"
            )
        if abs(me_mass_summary["pod_battery"] - ee_battery_mass_g) > 0.05:
            gate_failures.append(
                f"ME pod_battery ({me_mass_summary['pod_battery']:.3f} g) != EE battery ({ee_battery_mass_g:.3f} g)"
            )

    # Coordinate frame R_BS parity, orthogonality, and det == +1
    canonical_r_bs = contract["coordinate_frames"]["frames"]["S"]["canonical_R_BS"]
    me_r_bs = me_dims.get("coordinate_frame_markings", {}).get("R_BS")
    gate_failures.extend(_validate_r_bs(me_r_bs, canonical_r_bs))

    # Battery compartment swollen-cell fit (all 3 axes: swollen < cavity)
    batt_comp = me_dims.get("battery_compartment_mm", {})
    swollen_mm = batt_comp.get("cell_max_swollen_mm")
    cavity_mm = batt_comp.get("cavity_inner_mm")
    if (
        not isinstance(swollen_mm, list)
        or not isinstance(cavity_mm, list)
        or len(swollen_mm) != 3
        or len(cavity_mm) != 3
    ):
        gate_failures.append("battery_compartment_mm must define 3-element cell_max_swollen_mm and cavity_inner_mm")
    else:
        for axis_name, s_val, c_val in zip(("X", "Y", "Z"), swollen_mm, cavity_mm):
            if float(s_val) >= float(c_val):
                gate_failures.append(
                    f"Battery compartment {axis_name} clearance violated: "
                    f"cell_max_swollen_mm={s_val} >= cavity_inner_mm={c_val}"
                )

    return {
        "invariants": {
            "gps_enabled": constants["gps_enabled"],
            "external_anchors_enabled": constants["external_anchors_enabled"],
            "uses_gps": hw["uses_gps"],
            "uses_external_anchors": hw["uses_external_anchors"],
            "forbidden_rf_parts": forbidden_rf_hits,
            "min_battery_life_hours": min_life_h,
        },
        "battery": {
            "form_factor": batt_cfg["form_factor"],
            "nominal_v": v_batt,
            "nominal_capacity_mah": cap_nom_mah,
            "cold_derating_frac": cold_derating_frac,
            "aging_derating_frac": aging_derating_frac,
            "usable_capacity_add_mah": round(cap_add_mah, 4),
            "usable_capacity_mul_mah": round(cap_mul_mah, 4),
            "buck_efficiency": buck_eta,
            "r_int_25c_mohm": r_int_25c_mohm,
            "r_int_0c_mohm": r_int_0c_mohm,
            "uvlo_mv": uvlo_mv,
        },
        "rails_ma": {
            "nominal": {str(k): round(v, 4) for k, v in sorted(rail_nom_ma.items())},
            "worst_case": {str(k): round(v, 4) for k, v in sorted(rail_max_ma.items())},
            "standby": {str(k): round(v, 6) for k, v in sorted(rail_standby_ma.items())},
        },
        "operating_modes": {
            "nominal_play": {
                "i_batt_ma": round(i_batt_nom_ma, 4),
                "power_mw": round(p_batt_nom_mw, 3),
                "runtime_add_h": _round_opt(t_nom_add_h, 2),
                "runtime_mul_h": _round_opt(t_nom_mul_h, 2),
            },
            "worst_case_logging": {
                "i_batt_ma": round(i_batt_wc_ma, 4),
                "power_mw": round(p_batt_wc_mw, 3),
                "runtime_add_h": _round_opt(t_wc_add_h, 2),
                "runtime_mul_h": _round_opt(t_wc_mul_h, 2),
            },
            "play_plus_1hz_ble_beacon": {
                "i_batt_ma": round(i_batt_beacon_ma, 4),
                "power_mw": round(i_batt_beacon_ma * v_batt, 3),
                "runtime_add_h": _round_opt(t_beacon_add_h, 2),
            },
            "ble_2m_bulk_offload": {
                "i_1v8_ma": round(ble_offload_1v8_ma, 3),
                "i_batt_ma": round(i_batt_ble_offload_ma, 4),
                "power_mw": round(p_batt_ble_offload_mw, 3),
                "two_hour_log_mb": round(nominal_2h_mb, 3),
                "offload_duration_s": round(ble_offload_duration_s, 2),
                "offload_energy_mah": round(ble_offload_energy_mah, 4),
                "offload_pct_usable_cap": round(ble_offload_pct_usable, 3),
            },
            "usb_fs_bulk_offload": {
                "net_battery_drain_ma": 0.0,
                "jeita_charge_limit_ma": round(cap_nom_mah * 0.5, 1),
                "two_hour_log_mb": round(nominal_2h_mb, 3),
                "offload_duration_s": round(usb_offload_duration_s, 2),
            },
            "wom_standby": {
                "i_batt_ua": round(i_batt_standby_ma * 1000.0, 2),
                "runtime_add_h": _round_opt(t_standby_add_h, 1),
            },
            "ship_mode": {
                "i_batt_ua": round(i_batt_ship_ua, 3),
                "shelf_life_h": _round_opt(t_ship_h, 1),
            },
        },
        "ohmic_sag_cold_0c": {
            "soc_10pct_ocv_mv": soc_10pct_ocv_cold_mv,
            "continuous_wc_sag_mv": round(sag_wc_continuous_mv, 3),
            "transient_peak_batt_ma": round(i_batt_transient_peak_ma, 3),
            "transient_peak_sag_mv": round(sag_transient_peak_mv, 3),
            "v_loaded_wc_10pct_mv": round(v_loaded_wc_10pct_mv, 2),
            "v_loaded_transient_10pct_mv": round(v_loaded_transient_10pct_mv, 2),
            "uvlo_margin_wc_mv": round(uvlo_margin_wc_mv, 2),
            "uvlo_margin_transient_mv": round(uvlo_margin_transient_mv, 2),
        },
        "nand_storage": {
            "part_number": flash_cfg["part_number"],
            "page_bytes": page_bytes,
            "block_bytes": block_bytes,
            "usable_ring_blocks": usable_blocks,
            "usable_ring_bytes": usable_ring_bytes,
            "frame_sizes_bytes": {
                "imu_batch_n20": imu_frame_bytes,
                "aux_env": aux_frame_bytes,
                "event_marker": event_frame_bytes,
                "high_g_burst_n512": high_g_burst_frame_bytes,
                "high_g_cont_n40": hg_400_n40_frame_bytes,
                "high_g_cont_n50": hg_400_n50_frame_bytes,
            },
            "nominal_burst_3200hz": {
                "bytes_per_sec": round(nominal_total_bps, 4),
                "mb_per_hour": round(nominal_mb_per_hr, 3),
                "two_hour_mb": round(nominal_2h_mb, 3),
                "retention_hours": round(nand_nominal_h, 2),
            },
            "continuous_400hz_n40": {
                "bytes_per_sec": round(total_400_n40_bps, 4),
                "retention_hours": round(nand_400_n40_h, 2),
            },
            "continuous_400hz_n50": {
                "bytes_per_sec": round(total_400_n50_bps, 4),
                "retention_hours": round(nand_400_n50_h, 2),
            },
        },
        "mechanical_and_mass": {
            "envelope_mm": {
                "length_mm": float(env["length_mm"]),
                "width_mm": float(env["width_mm"]),
                "thickness_mm": float(env["thickness_mm"]),
                "z_stackup_nominal_mm": round(z_nom_calc, 2),
                "z_stackup_worst_case_mm": round(z_wc_calc, 2),
            },
            "ee_non_battery_mass_g": round(ee_non_battery_mass_g, 4),
            "ee_battery_mass_g": round(ee_battery_mass_g, 4),
            "me_bom_scopes_g": me_mass_summary,
            "pod_mass_g": pod_mass_g,
            "pod_with_strap_mass_g": pod_with_strap_mass_g,
        },
        "checks": {
            "passed": len(gate_failures) == 0,
            "failures": gate_failures,
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="hockey_sensor dynamic power, sag, NAND & mechanical budget calculator")
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT_PATH)
    parser.add_argument("--ee-bom", type=Path, default=DEFAULT_EE_BOM_PATH)
    parser.add_argument("--me-dims", type=Path, default=DEFAULT_ME_DIMS_PATH)
    parser.add_argument("--me-bom", type=Path, default=DEFAULT_ME_BOM_PATH)
    parser.add_argument("--json", action="store_true", help="Emit strict RFC 8259 JSON report")
    parser.add_argument("--check", action="store_true", help="Exit non-zero if any hardware budget gate fails")
    args = parser.parse_args(argv)

    contract = load_json_strict(args.contract)
    ee_rows = load_ee_bom(args.ee_bom)
    me_dims = load_json_strict(args.me_dims)
    me_rows = load_me_bom(args.me_bom) if args.me_bom is not None and args.me_bom.exists() else None

    report = compute_budgets(contract, ee_rows, me_dims, me_rows)

    if args.json:
        print(json.dumps(report, indent=2, allow_nan=False))
    else:
        b = report["battery"]
        m = report["operating_modes"]
        s = report["ohmic_sag_cold_0c"]
        n = report["nand_storage"]
        me = report["mechanical_and_mass"]
        def _fmt_h(val: float | None) -> str:
            return "N/A" if val is None else f"{val:.2f} h"

        print(f"Battery ({b['form_factor']}): {b['nominal_capacity_mah']} mAh -> {b['usable_capacity_add_mah']} mAh (additive 0C+age)")
        print(f"  Nominal Play:      {m['nominal_play']['i_batt_ma']:.3f} mA batt -> {_fmt_h(m['nominal_play']['runtime_add_h'])}")
        print(f"  Worst-Case Log:    {m['worst_case_logging']['i_batt_ma']:.3f} mA batt -> {_fmt_h(m['worst_case_logging']['runtime_add_h'])}")
        print(f"  Play + 1Hz Beacon: {m['play_plus_1hz_ble_beacon']['i_batt_ma']:.3f} mA batt -> {_fmt_h(m['play_plus_1hz_ble_beacon']['runtime_add_h'])}")
        print(f"  BLE 2M Offload:    {m['ble_2m_bulk_offload']['offload_energy_mah']:.3f} mAh ({m['ble_2m_bulk_offload']['offload_pct_usable_cap']:.2f}%) in {m['ble_2m_bulk_offload']['offload_duration_s']:.1f} s")
        print(f"  USB FS Offload:    0.0 mA net drain ({m['usb_fs_bulk_offload']['offload_duration_s']:.1f} s), charges @ {m['usb_fs_bulk_offload']['jeita_charge_limit_ma']:.0f} mA")
        print(f"  Cold Sag (0C):     {s['continuous_wc_sag_mv']:.2f} mV cont / {s['transient_peak_sag_mv']:.2f} mV peak (UVLO margin {s['uvlo_margin_transient_mv']:.1f} mV)")
        print(f"NAND ({n['part_number']}): {n['usable_ring_bytes']:,} B ({n['usable_ring_blocks']} blocks)")
        print(f"  Nominal 3200Hz Burst: {n['nominal_burst_3200hz']['bytes_per_sec']:.2f} B/s -> {n['nominal_burst_3200hz']['retention_hours']:.2f} h")
        print(f"  Continuous 400Hz N40: {n['continuous_400hz_n40']['bytes_per_sec']:.2f} B/s -> {n['continuous_400hz_n40']['retention_hours']:.2f} h")
        print(f"  Continuous 400Hz N50: {n['continuous_400hz_n50']['bytes_per_sec']:.2f} B/s -> {n['continuous_400hz_n50']['retention_hours']:.2f} h")
        print(f"Mechanical: {me['envelope_mm']['length_mm']}x{me['envelope_mm']['width_mm']}x{me['envelope_mm']['thickness_mm']} mm, pod={me['pod_mass_g']} g, w/strap={me['pod_with_strap_mass_g']} g")
        print(f"Gate Check: {'PASS' if report['checks']['passed'] else 'FAIL'} {report['checks']['failures']}")

    if args.check and not report["checks"]["passed"]:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
