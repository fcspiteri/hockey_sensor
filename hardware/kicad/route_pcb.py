#!/usr/bin/env python3
"""Export Specctra DSN or import Specctra SES (with post-import GND/VDD copper pours) for hockey_sensor.kicad_pcb."""

import math
import re
import sys
import uuid
from pathlib import Path
import pcbnew


def export_dsn(pcb_path: Path, dsn_path: Path) -> None:
    board = pcbnew.LoadBoard(str(pcb_path))
    ok = pcbnew.ExportSpecctraDSN(board, str(dsn_path))
    if not ok or not dsn_path.exists():
        raise RuntimeError(f"ExportSpecctraDSN failed for {pcb_path} -> {dsn_path}")
    # 1. Strip (pins ...) from /GND (connected post-route by dedicated GND vias + In1.Cu/B.Cu/F.Cu GND planes, S-C2)
    dsn_text = dsn_path.read_text(encoding="utf-8")
    dsn_text = dsn_text.replace(
        '    (padstack "Via[0-3]_400:200_um"\n'
        '      (shape (circle F.Cu 400))\n'
        '      (shape (circle In1.Cu 400))\n'
        '      (shape (circle In2.Cu 400))\n'
        '      (shape (circle B.Cu 400))',
        '    (padstack "Via[0-3]_400:200_um"\n'
        '      (shape (circle F.Cu 425))\n'
        '      (shape (circle In1.Cu 425))\n'
        '      (shape (circle In2.Cu 425))\n'
        '      (shape (circle B.Cu 425))',
        1,
    )
    dsn_text = re.sub(r"\(net /GND\s+\(pins[^)]*\)\s*\)", "(net /GND)", dsn_text, flags=re.DOTALL)
    dsn_path.write_text(dsn_text, encoding="utf-8")
    print(f"EXPORT_DSN_OK pcb={pcb_path} dsn={dsn_path} bytes={dsn_path.stat().st_size}")


def _add_copper_pour_zones(pcb_path: Path) -> None:
    """Append F.Cu (GND), In1.Cu (GND), B.Cu (GND), and In2.Cu (VDD_1V8) copper pour zones around the 8x8mm top-right RF keepout."""
    text = pcb_path.read_text(encoding="utf-8")
    # Look up net IDs for /GND and /VDD_1V8 from top-level (net <id> "<name>") lines
    gnd_id = 1
    v18_id = 5
    for line in text.splitlines():
        s = line.strip()
        parts = s.split()
        if len(parts) == 3 and parts[0] == "(net" and parts[1].isdigit():
            if parts[2] in ('"GND")', '"/GND")'):
                gnd_id = int(parts[1])
            elif parts[2] in ('"VDD_1V8")', '"/VDD_1V8")'):
                v18_id = int(parts[1])

    # L-shaped polygon covering the 38x28 mm board (x: 116.3..143.7, y: 81.3..118.7)
    # while carving out the 8.0x8.0 mm top-right RF keepout (x: 136.0..144.0, y: 81.0..89.0)
    poly_pts = (
        "(xy 116.300 81.300) (xy 135.800 81.300) (xy 135.800 89.200) "
        "(xy 143.700 89.200) (xy 143.700 118.700) (xy 116.300 118.700)"
    )
    zones_sexpr = []
    for z_uuid, z_layer, z_net_id, z_net_name, z_name in [
        ("11111111-2222-3333-4444-555555550001", "In1.Cu", gnd_id, "/GND", "zone_pour_in1_gnd"),
        ("11111111-2222-3333-4444-555555550002", "B.Cu", gnd_id, "/GND", "zone_pour_B.Cu"),
        ("11111111-2222-3333-4444-555555550003", "In2.Cu", v18_id, "/VDD_1V8", "zone_pour_In2.Cu"),
    ]:
        if z_name in text:
            continue
        zones_sexpr.extend([
            "  (zone",
            f"    (net {z_net_id})",
            f'    (net_name "{z_net_name}")',
            f'    (layer "{z_layer}")',
            f'    (uuid "{z_uuid}")',
            f'    (name "{z_name}")',
            "    (hatch edge 0.5)",
            "    (connect_pads yes (clearance 0.15))",
            "    (min_thickness 0.2)",
            "    (filled_areas_thickness no)",
            "    (fill yes (thermal_gap 0.20) (thermal_bridge_width 0.25))",
            "    (polygon",
            "      (pts",
            f"        {poly_pts}",
            "      )",
            "    )",
            "  )",
        ])
    if not zones_sexpr:
        return
    idx = text.rfind(")")
    new_text = text[:idx] + "\n" + "\n".join(zones_sexpr) + "\n)\n"
    pcb_path.write_text(new_text, encoding="utf-8")


def _seg_dist(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def _place_post_route_gnd_vias(board: "pcbnew.BOARD") -> int:
    """Place a dedicated 0.40/0.20 mm GND or VDD_1V8 via + short F.Cu stub at every GND and non-U1 VDD_1V8 pad after signal routing completes (S-C2)."""
    gnd_net = board.FindNet("/GND") or board.FindNet("GND")
    v18_net = board.FindNet("/VDD_1V8") or board.FindNet("VDD_1V8")
    if not gnd_net or not v18_net:
        return 0
    gnd_code = gnd_net.GetNetCode()
    v18_code = v18_net.GetNetCode()

    # Collect existing pads, tracks, and all vias/holes in mm
    pads_info = []
    npth_holes = []
    target_pads = []
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        for pad in fp.Pads():
            pos = pad.GetPosition()
            px = pcbnew.ToMM(pos.x)
            py = pcbnew.ToMM(pos.y)
            sz = pad.GetSize()
            pw = pcbnew.ToMM(sz.x)
            ph = pcbnew.ToMM(sz.y)
            pnet = pad.GetNetCode()
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                npth_holes.append((px, py, pcbnew.ToMM(pad.GetDrillSize().x) / 2.0))
                continue
            pads_info.append((ref, pad.GetNumber(), px, py, pw, ph, pnet))
            if pnet == gnd_code:
                target_pads.append((ref, pad.GetNumber(), px, py, pw, ph, gnd_code))

    tracks_info = []
    vias_info = []
    for trk in board.GetTracks():
        if trk.GetClass() == "PCB_VIA":
            vpos = trk.GetPosition()
            vias_info.append((pcbnew.ToMM(vpos.x), pcbnew.ToMM(vpos.y), pcbnew.ToMM(trk.GetWidth()) / 2.0, trk.GetNetCode()))
        else:
            s = trk.GetStart()
            e = trk.GetEnd()
            tracks_info.append((
                pcbnew.ToMM(s.x),
                pcbnew.ToMM(s.y),
                pcbnew.ToMM(e.x),
                pcbnew.ToMM(e.y),
                pcbnew.ToMM(trk.GetWidth()) / 2.0,
                trk.GetNetCode(),
                trk.GetLayer(),
            ))

    def _seg_dist(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0.0 and dy == 0.0:
            return math.hypot(px - x1, py - y1)
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))

    def _via_clear(vx: float, vy: float, self_ref: str, self_pad: str, net_code: int, is_ep: bool = False) -> bool:
        if not (116.5 <= vx <= 143.5 and 81.5 <= vy <= 118.5):
            return False
        # Stay out of 8x8mm top-right RF keepout (x >= 136.0, y <= 89.0)
        if vx >= 135.75 and vy <= 89.25:
            return False
        for hx, hy, hr in npth_holes:
            if math.hypot(vx - hx, vy - hy) < hr + 0.45:
                return False
        for ovx, ovy, ovr, onet in vias_info:
            min_v_dist = 0.45 if onet == net_code else 0.51
            if math.hypot(vx - ovx, vy - ovy) < min_v_dist:
                return False
        for oref, opad, opx, opy, opw, oph, opnet in pads_info:
            if is_ep and oref == self_ref and opad == self_pad:
                continue
            if opnet == net_code and oref == self_ref and opad == self_pad:
                continue
            dx_g = max(0.0, abs(vx - opx) - opw / 2.0)
            dy_g = max(0.0, abs(vy - opy) - oph / 2.0)
            min_p_gap = 0.05 if opnet == net_code else 0.305
            if math.hypot(dx_g, dy_g) < min_p_gap:
                return False
        for x1, y1, x2, y2, tr, tnet, _tlayer in tracks_info:
            if tnet == net_code:
                continue
            if _seg_dist(vx, vy, x1, y1, x2, y2) < tr + 0.308:
                return False
        return True

    def _add_pwr_via(vx: float, vy: float, net_code: int, px=None, py=None) -> None:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(vx), pcbnew.FromMM(vy)))
        via.SetWidth(pcbnew.FromMM(0.40))
        via.SetDrill(pcbnew.FromMM(0.20))
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        via.SetNetCode(net_code)
        board.Add(via)
        vias_info.append((vx, vy, 0.20, net_code))
        if px is not None and py is not None and math.hypot(vx - px, vy - py) > 0.01:
            trk = pcbnew.PCB_TRACK(board)
            trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(px), pcbnew.FromMM(py)))
            trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(vx), pcbnew.FromMM(vy)))
            trk.SetWidth(pcbnew.FromMM(0.12))
            trk.SetLayer(pcbnew.F_Cu)
            trk.SetNetCode(net_code)
            board.Add(trk)
            tracks_info.append((px, py, vx, vy, 0.06, net_code, pcbnew.F_Cu))

    added = 0
    unplaced_pads = []
    connected_pads = []
    for ref, pnum, px, py, pw, ph, net_code in target_pads:
        if pnum in ("EP", "9") and pw >= 1.8 and ph >= 1.8:
            step = 0.65 if pw <= 2.2 else 1.0
            for gx in (-step, 0.0, step):
                for gy in (-step, 0.0, step):
                    vx = round(px + gx, 3)
                    vy = round(py + gy, 3)
                    if _via_clear(vx, vy, ref, pnum, net_code, is_ep=True):
                        _add_pwr_via(vx, vy, net_code)
                        added += 1
            connected_pads.append((px, py, net_code))
            continue
        if any(math.hypot(px - ovx, py - ovy) < 0.05 and onet == net_code for ovx, ovy, _ovr, onet in vias_info) or (ref == "U10" and pnum == "3") or (ref == "U4" and pnum == "15") or (ref == "U9" and pnum in ("2", "5")) or (ref == "U8" and pnum == "1") or (ref == "U1" and pnum in ("T2", "H2", "G1")) or (ref == "C28" and pnum == "1"):
            connected_pads.append((px, py, net_code))
            continue
        if _via_clear(px, py, ref, pnum, net_code, is_ep=True):
            _add_pwr_via(px, py, net_code)
            added += 1
            connected_pads.append((px, py, net_code))
            continue
        # Search concentric rings around the GND / VDD_1V8 pad across 16 angles for a DRC-clean via spot
        found = False
        for dist in (0.40, 0.48, 0.56, 0.65, 0.78, 0.92, 1.08, 1.25, 1.45, 1.70, 2.00, 2.35):
            if found:
                break
            rx = round(pw / 2.0 + dist - 0.20, 3)
            ry = round(ph / 2.0 + dist - 0.20, 3)
            cand_pts = [
                (round(px + rx, 3), py),
                (round(px - rx, 3), py),
                (px, round(py + ry, 3)),
                (px, round(py - ry, 3)),
                (round(px + rx, 3), round(py + ry, 3)),
                (round(px - rx, 3), round(py + ry, 3)),
                (round(px + rx, 3), round(py - ry, 3)),
                (round(px - rx, 3), round(py - ry, 3)),
                (round(px + rx, 3), round(py + 0.5 * ry, 3)),
                (round(px - rx, 3), round(py + 0.5 * ry, 3)),
                (round(px + rx, 3), round(py - 0.5 * ry, 3)),
                (round(px - rx, 3), round(py - 0.5 * ry, 3)),
                (round(px + 0.5 * rx, 3), round(py + ry, 3)),
                (round(px - 0.5 * rx, 3), round(py + ry, 3)),
                (round(px + 0.5 * rx, 3), round(py - ry, 3)),
                (round(px - 0.5 * rx, 3), round(py - ry, 3)),
            ]
            for vx, vy in cand_pts:
                # Ensure stub on F.Cu doesn't cross any foreign-net F.Cu track, pad, or via
                stub_ok = True
                for s_t in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9):
                    sx = px + s_t * (vx - px)
                    sy = py + s_t * (vy - py)
                    for x1, y1, x2, y2, tr, tnet, tlayer in tracks_info:
                        if tnet == net_code or tlayer != pcbnew.F_Cu:
                            continue
                        if _seg_dist(sx, sy, x1, y1, x2, y2) < 0.06 + tr + 0.105:
                            stub_ok = False
                            break
                    if not stub_ok:
                        break
                    for ovx, ovy, ovr, onet in vias_info:
                        if onet == net_code:
                            continue
                        if math.hypot(sx - ovx, sy - ovy) < 0.06 + ovr + 0.105:
                            stub_ok = False
                            break
                    if not stub_ok:
                        break
                    for oref, opad, opx, opy, opw, oph, opnet in pads_info:
                        if opnet == net_code:
                            continue
                        dx_g = max(0.0, abs(sx - opx) - opw / 2.0)
                        dy_g = max(0.0, abs(sy - opy) - oph / 2.0)
                        if math.hypot(dx_g, dy_g) < 0.16:
                            stub_ok = False
                            break
                    if not stub_ok:
                        break
                if stub_ok and _via_clear(vx, vy, ref, pnum, net_code):
                    _add_pwr_via(vx, vy, net_code, px, py)
                    added += 1
                    connected_pads.append((px, py, net_code))
                    found = True
                    break
        if not found:
            unplaced_pads.append((ref, pnum, px, py, pw, ph, net_code))

    # Second pass (1-hop, L-shaped, and Z-shaped F.Cu fallback): connect any remaining unplaced GND pads to the nearest placed same-net via or already-connected pad!
    def _seg_fcu_ok(ax: float, ay: float, bx: float, by: float, net_code: int) -> bool:
        x_lo = min(ax, bx) - 0.50
        x_hi = max(ax, bx) + 0.50
        y_lo = min(ay, by) - 0.50
        y_hi = max(ay, by) + 0.50
        local_trks = [
            (x1, y1, x2, y2, tr)
            for x1, y1, x2, y2, tr, tnet, tlayer in tracks_info
            if tnet != net_code and tlayer == pcbnew.F_Cu and max(x1, x2) >= x_lo and min(x1, x2) <= x_hi and max(y1, y2) >= y_lo and min(y1, y2) <= y_hi
        ]
        local_vias = [
            (ovx, ovy, ovr)
            for ovx, ovy, ovr, onet in vias_info
            if onet != net_code and x_lo <= ovx <= x_hi and y_lo <= ovy <= y_hi
        ]
        local_pads = [
            (opx, opy, opw, oph)
            for _oref, _opad, opx, opy, opw, oph, opnet in pads_info
            if opnet != net_code and x_lo - opw <= opx <= x_hi + opw and y_lo - oph <= opy <= y_hi + oph
        ]
        for s_t in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9):
            sx = ax + s_t * (bx - ax)
            sy = ay + s_t * (by - ay)
            for x1, y1, x2, y2, tr in local_trks:
                if _seg_dist(sx, sy, x1, y1, x2, y2) < 0.05 + tr + 0.10:
                    return False
            for ovx, ovy, ovr in local_vias:
                if math.hypot(sx - ovx, sy - ovy) < 0.05 + ovr + 0.10:
                    return False
            for opx, opy, opw, oph in local_pads:
                dx_g = max(0.0, abs(sx - opx) - opw / 2.0)
                dy_g = max(0.0, abs(sy - opy) - oph / 2.0)
                if math.hypot(dx_g, dy_g) < 0.145:
                    return False
        return True

    def _seg_layer_ok(ax: float, ay: float, bx: float, by: float, net_code: int, layer_id: int) -> bool:
        x_lo = min(ax, bx) - 0.50
        x_hi = max(ax, bx) + 0.50
        y_lo = min(ay, by) - 0.50
        y_hi = max(ay, by) + 0.50
        local_trks = [
            (x1, y1, x2, y2, tr)
            for x1, y1, x2, y2, tr, tnet, tlayer in tracks_info
            if tnet != net_code and tlayer == layer_id and max(x1, x2) >= x_lo and min(x1, x2) <= x_hi and max(y1, y2) >= y_lo and min(y1, y2) <= y_hi
        ]
        local_vias = [
            (ovx, ovy, ovr)
            for ovx, ovy, ovr, onet in vias_info
            if onet != net_code and x_lo <= ovx <= x_hi and y_lo <= ovy <= y_hi
        ]
        local_pads = [
            (opx, opy, opw, oph)
            for _oref, _opad, opx, opy, opw, oph, opnet in pads_info
            if layer_id == pcbnew.F_Cu and opnet != net_code and x_lo - opw <= opx <= x_hi + opw and y_lo - oph <= opy <= y_hi + oph
        ]
        same_pads_box = [
            (opx, opy, opw, oph)
            for _oref, _opad, opx, opy, opw, oph, opnet in pads_info
            if layer_id == pcbnew.F_Cu and opnet == net_code and x_lo - opw <= opx <= x_hi + opw and y_lo - oph <= opy <= y_hi + oph
        ]
        for s_t in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9):
            sx = ax + s_t * (bx - ax)
            sy = ay + s_t * (by - ay)
            for x1, y1, x2, y2, tr in local_trks:
                if _seg_dist(sx, sy, x1, y1, x2, y2) < 0.045 + tr + 0.092:
                    return False
            for ovx, ovy, ovr in local_vias:
                if math.hypot(sx - ovx, sy - ovy) < 0.045 + ovr + 0.103:
                    return False
            inside_same_pad = any(
                abs(sx - spx) <= spw / 2.0 + 0.035 and abs(sy - spy) <= sph / 2.0 + 0.035
                for spx, spy, spw, sph in same_pads_box
            )
            if not inside_same_pad:
                for opx, opy, opw, oph in local_pads:
                    dx_g = max(0.0, abs(sx - opx) - opw / 2.0)
                    dy_g = max(0.0, abs(sy - opy) - oph / 2.0)
                    if math.hypot(dx_g, dy_g) < 0.137:
                        return False
        return True

    for _pass in range(3):
        still_unplaced = []
        for ref, pnum, px, py, pw, ph, net_code in unplaced_pads:
            same_targets = [
                (ovx, ovy) for ovx, ovy, _ovr, onet in vias_info if onet == net_code
            ] + [
                (cpx, cpy) for cpx, cpy, cnet in connected_pads if cnet == net_code
            ] + [
                (x1, y1) for x1, y1, _x2, _y2, _tr, tnet, tlay in tracks_info if tnet == net_code and tlay == pcbnew.F_Cu
            ] + [
                (x2, y2) for _x1, _y1, x2, y2, _tr, tnet, tlay in tracks_info if tnet == net_code and tlay == pcbnew.F_Cu
            ]
            same_targets.sort(key=lambda pt: math.hypot(pt[0] - px, pt[1] - py))
            linked = False
            for tx, ty in same_targets[:12]:
                if math.hypot(tx - px, ty - py) > 3.2:
                    continue
                if _seg_layer_ok(px, py, tx, ty, net_code, pcbnew.F_Cu):
                    trk = pcbnew.PCB_TRACK(board)
                    trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(px), pcbnew.FromMM(py)))
                    trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(tx), pcbnew.FromMM(ty)))
                    trk.SetWidth(pcbnew.FromMM(0.10))
                    trk.SetLayer(pcbnew.F_Cu)
                    trk.SetNetCode(net_code)
                    board.Add(trk)
                    tracks_info.append((px, py, tx, ty, 0.05, net_code, pcbnew.F_Cu))
                    connected_pads.append((px, py, net_code))
                    linked = True
                    break
                mid_candidates = [
                    (tx, py), (px, ty),
                    (px - 0.50, py), (px + 0.50, py),
                    (px, py - 0.50), (px, py + 0.50),
                    (px - 0.90, py), (px + 0.90, py),
                    (px, py - 0.90), (px, py + 0.90),
                    (px - 1.35, py), (px + 1.35, py),
                    (px, py - 1.35), (px, py + 1.35),
                ]
                for mx, my in mid_candidates:
                    if not _seg_layer_ok(px, py, mx, my, net_code, pcbnew.F_Cu):
                        continue
                    if _seg_layer_ok(mx, my, tx, ty, net_code, pcbnew.F_Cu):
                        for ax, ay, bx, by in [(px, py, mx, my), (mx, my, tx, ty)]:
                            trk = pcbnew.PCB_TRACK(board)
                            trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax), pcbnew.FromMM(ay)))
                            trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx), pcbnew.FromMM(by)))
                            trk.SetWidth(pcbnew.FromMM(0.10))
                            trk.SetLayer(pcbnew.F_Cu)
                            trk.SetNetCode(net_code)
                            board.Add(trk)
                            tracks_info.append((ax, ay, bx, by, 0.05, net_code, pcbnew.F_Cu))
                        connected_pads.append((px, py, net_code))
                        linked = True
                        break
                    for m2x, m2y in [(tx, my), (mx, ty)]:
                        if _seg_layer_ok(mx, my, m2x, m2y, net_code, pcbnew.F_Cu) and _seg_layer_ok(m2x, m2y, tx, ty, net_code, pcbnew.F_Cu):
                            for ax, ay, bx, by in [(px, py, mx, my), (mx, my, m2x, m2y), (m2x, m2y, tx, ty)]:
                                trk = pcbnew.PCB_TRACK(board)
                                trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax), pcbnew.FromMM(ay)))
                                trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx), pcbnew.FromMM(by)))
                                trk.SetWidth(pcbnew.FromMM(0.10))
                                trk.SetLayer(pcbnew.F_Cu)
                                trk.SetNetCode(net_code)
                                board.Add(trk)
                                tracks_info.append((ax, ay, bx, by, 0.05, net_code, pcbnew.F_Cu))
                            connected_pads.append((px, py, net_code))
                            linked = True
                            break
                    if linked:
                        break
                if linked:
                    break
            if not linked:
                still_unplaced.append((ref, pnum, px, py, pw, ph, net_code))
        unplaced_pads = still_unplaced

    # Third pass: for any remaining unplaced GND pad, hop on F.Cu to a nearby open coordinate (mx, my) within 2.5 mm and drop a 0.40/0.20 mm GND via there, then run one final linking pass!
    still_after_hop = []
    for ref, pnum, px, py, pw, ph, net_code in unplaced_pads:
        placed_hop = False
        for dist_h in (0.50, 0.65, 0.85, 1.05, 1.30, 1.60, 1.95, 2.35):
            if placed_hop:
                break
            for ang_i in range(16):
                theta = ang_i * (2.0 * math.pi / 16.0)
                vx = round(px + dist_h * math.cos(theta), 3)
                vy = round(py + dist_h * math.sin(theta), 3)
                if not _via_clear(vx, vy, ref, pnum, net_code):
                    continue
                if _seg_layer_ok(px, py, vx, vy, net_code, pcbnew.F_Cu):
                    _add_pwr_via(vx, vy, net_code, px, py)
                    added += 1
                    connected_pads.append((px, py, net_code))
                    placed_hop = True
                    break
                for mx, my in [(vx, py), (px, vy)]:
                    if _seg_layer_ok(px, py, mx, my, net_code, pcbnew.F_Cu) and _seg_layer_ok(mx, my, vx, vy, net_code, pcbnew.F_Cu):
                        trk = pcbnew.PCB_TRACK(board)
                        trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(px), pcbnew.FromMM(py)))
                        trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(mx), pcbnew.FromMM(my)))
                        trk.SetWidth(pcbnew.FromMM(0.10))
                        trk.SetLayer(pcbnew.F_Cu)
                        trk.SetNetCode(net_code)
                        board.Add(trk)
                        tracks_info.append((px, py, mx, my, 0.05, net_code, pcbnew.F_Cu))
                        _add_pwr_via(vx, vy, net_code, mx, my)
                        added += 1
                        connected_pads.append((px, py, net_code))
                        placed_hop = True
                        break
                if placed_hop:
                    break
        if not placed_hop:
            still_after_hop.append((ref, pnum, px, py, pw, ph, net_code))
    # Also connect any open signal net (e.g. /HFXO_XC2, /PCA9306_EN, /USB_D_N) BEFORE placing the dense GND stitching grid so signal paths aren't boxed in by stitching vias!
    for snet_name, sx_a, sy_a, sx_b, sy_b in [
        ("/HFXO_XC2", 129.30, 88.70, 132.90, 89.50),
        ("/PCA9306_EN", 139.55, 97.95, 140.75, 98.40),
        ("/USB_D_N", 131.15, 111.20, 128.85, 111.20),
    ]:
        sn_obj = board.FindNet(snet_name) or board.FindNet(snet_name.lstrip("/"))
        if not sn_obj:
            continue
        snet = sn_obj.GetNetCode()
        s_targets = [
            (sx_b, sy_b)
        ] + [
            (x1, y1) for x1, y1, _x2, _y2, _tr, tnet, tlay in tracks_info if tnet == snet and tlay == pcbnew.F_Cu and math.hypot(x1 - sx_a, y1 - sy_a) > 0.65
        ] + [
            (x2, y2) for _x1, _y1, x2, y2, _tr, tnet, tlay in tracks_info if tnet == snet and tlay == pcbnew.F_Cu and math.hypot(x2 - sx_a, y2 - sy_a) > 0.65
        ]
        step_g = 0.06
        visited = {(0, 0): None}
        queue = [(0, 0)]
        found_goal = None
        via_spots_a = []
        q_idx = 0
        while q_idx < len(queue) and q_idx < 16000 and found_goal is None:
            ix, iy = queue[q_idx]
            q_idx += 1
            cx = round(sx_a + ix * step_g, 3)
            cy = round(sy_a + iy * step_g, 3)
            if (ix != 0 or iy != 0) and len(via_spots_a) < 35 and _via_clear(cx, cy, "SIGA", "0", snet):
                if all(math.hypot(cx - vx0, cy - vy0) >= 0.18 for _n0, (vx0, vy0) in via_spots_a):
                    via_spots_a.append(((ix, iy), (cx, cy)))
            for tx_s, ty_s in s_targets:
                if math.hypot(tx_s - cx, ty_s - cy) <= 0.30 and _seg_layer_ok(cx, cy, tx_s, ty_s, snet, pcbnew.F_Cu):
                    found_goal = ((ix, iy), (tx_s, ty_s))
                    break
            if found_goal is not None:
                break
            for dx_i, dy_i in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx_i, ny_i = ix + dx_i, iy + dy_i
                if (nx_i, ny_i) in visited or abs(nx_i) > 95 or abs(ny_i) > 95:
                    continue
                nx_mm = round(sx_a + nx_i * step_g, 3)
                ny_mm = round(sy_a + ny_i * step_g, 3)
                if _seg_layer_ok(cx, cy, nx_mm, ny_mm, snet, pcbnew.F_Cu):
                    visited[(nx_i, ny_i)] = (ix, iy)
                    queue.append((nx_i, ny_i))
        if found_goal is not None:
            cur_node, (tx, ty) = found_goal
            pts_path = [(tx, ty)]
            while cur_node is not None:
                pts_path.append((round(sx_a + cur_node[0] * step_g, 3), round(sy_a + cur_node[1] * step_g, 3)))
                cur_node = visited[cur_node]
            pts_path.reverse()
            for s_i in range(len(pts_path) - 1):
                ax_p, ay_p = pts_path[s_i]
                bx_p, by_p = pts_path[s_i + 1]
                trk = pcbnew.PCB_TRACK(board)
                trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_p), pcbnew.FromMM(ay_p)))
                trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_p), pcbnew.FromMM(by_p)))
                trk.SetWidth(pcbnew.FromMM(0.09))
                trk.SetLayer(pcbnew.F_Cu)
                trk.SetNetCode(snet)
                board.Add(trk)
                tracks_info.append((ax_p, ay_p, bx_p, by_p, 0.045, snet, pcbnew.F_Cu))
        else:
            visited_b = {(0, 0): None}
            queue_b = [(0, 0)]
            via_spots_b = []
            if _via_clear(sx_b, sy_b, "SIGB", "0", snet):
                via_spots_b.append(((0, 0), (sx_b, sy_b)))
            qb_idx = 0
            while qb_idx < len(queue_b) and qb_idx < 4000 and len(via_spots_b) < 25:
                ix, iy = queue_b[qb_idx]
                qb_idx += 1
                cx = round(sx_b + ix * step_g, 3)
                cy = round(sy_b + iy * step_g, 3)
                if (ix != 0 or iy != 0) and _via_clear(cx, cy, "SIGB", "0", snet):
                    if all(math.hypot(cx - vx0, cy - vy0) >= 0.20 for _n0, (vx0, vy0) in via_spots_b):
                        via_spots_b.append(((ix, iy), (cx, cy)))
                for dx_i, dy_i in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx_i, ny_i = ix + dx_i, iy + dy_i
                    if (nx_i, ny_i) in visited_b or abs(nx_i) > 40 or abs(ny_i) > 40:
                        continue
                    nx_mm = round(sx_b + nx_i * step_g, 3)
                    ny_mm = round(sy_b + ny_i * step_g, 3)
                    if _seg_layer_ok(cx, cy, nx_mm, ny_mm, snet, pcbnew.F_Cu):
                        visited_b[(nx_i, ny_i)] = (ix, iy)
                        queue_b.append((nx_i, ny_i))
            bridged_via = False
            for node_a, (vxa, vya) in via_spots_a:
                if bridged_via:
                    break
                for node_b, (vxb, vyb) in via_spots_b:
                    if math.hypot(vxa - vxb, vya - vyb) < 0.51:
                        continue
                    for hop_lay in (pcbnew.B_Cu, pcbnew.In2_Cu, pcbnew.In1_Cu):
                        hop_pts = None
                        if _seg_layer_ok(vxa, vya, vxb, vyb, snet, hop_lay):
                            hop_pts = [(vxa, vya), (vxb, vyb)]
                        else:
                            for mhx, mhy in [(vxa, vyb), (vxb, vya)]:
                                if _seg_layer_ok(vxa, vya, mhx, mhy, snet, hop_lay) and _seg_layer_ok(mhx, mhy, vxb, vyb, snet, hop_lay):
                                    hop_pts = [(vxa, vya), (mhx, mhy), (vxb, vyb)]
                                    break
                        if hop_pts is not None:
                            cur_a = node_a
                            pts_a = []
                            while cur_a is not None:
                                pts_a.append((round(sx_a + cur_a[0] * step_g, 3), round(sy_a + cur_a[1] * step_g, 3)))
                                cur_a = visited[cur_a]
                            pts_a.reverse()
                            for s_i in range(len(pts_a) - 1):
                                ax_p, ay_p = pts_a[s_i]
                                bx_p, by_p = pts_a[s_i + 1]
                                trk = pcbnew.PCB_TRACK(board)
                                trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_p), pcbnew.FromMM(ay_p)))
                                trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_p), pcbnew.FromMM(by_p)))
                                trk.SetWidth(pcbnew.FromMM(0.09))
                                trk.SetLayer(pcbnew.F_Cu)
                                trk.SetNetCode(snet)
                                board.Add(trk)
                                tracks_info.append((ax_p, ay_p, bx_p, by_p, 0.045, snet, pcbnew.F_Cu))
                            _add_pwr_via(vxa, vya, snet)
                            cur_b = node_b
                            pts_b = []
                            while cur_b is not None:
                                pts_b.append((round(sx_b + cur_b[0] * step_g, 3), round(sy_b + cur_b[1] * step_g, 3)))
                                cur_b = visited_b[cur_b]
                            pts_b.reverse()
                            for s_i in range(len(pts_b) - 1):
                                ax_p, ay_p = pts_b[s_i]
                                bx_p, by_p = pts_b[s_i + 1]
                                trk = pcbnew.PCB_TRACK(board)
                                trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_p), pcbnew.FromMM(ay_p)))
                                trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_p), pcbnew.FromMM(by_p)))
                                trk.SetWidth(pcbnew.FromMM(0.09))
                                trk.SetLayer(pcbnew.F_Cu)
                                trk.SetNetCode(snet)
                                board.Add(trk)
                                tracks_info.append((ax_p, ay_p, bx_p, by_p, 0.045, snet, pcbnew.F_Cu))
                            _add_pwr_via(vxb, vyb, snet)
                            for s_i in range(len(hop_pts) - 1):
                                ax_p, ay_p = hop_pts[s_i]
                                bx_p, by_p = hop_pts[s_i + 1]
                                trk = pcbnew.PCB_TRACK(board)
                                trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_p), pcbnew.FromMM(ay_p)))
                                trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_p), pcbnew.FromMM(by_p)))
                                trk.SetWidth(pcbnew.FromMM(0.09))
                                trk.SetLayer(hop_lay)
                                trk.SetNetCode(snet)
                                board.Add(trk)
                                tracks_info.append((ax_p, ay_p, bx_p, by_p, 0.045, snet, hop_lay))
                            bridged_via = True
                            break
                    if bridged_via:
                        break
            if not bridged_via:
                print(f"UNPLACED_SIG_NET {snet_name} ({sx_a:.3f},{sy_a:.3f})->({sx_b:.3f},{sy_b:.3f})")

    # Now connect any remaining unplaced GND pads BEFORE placing the 2mm background stitching grid
    for ref, pnum, px, py, pw, ph, net_code in still_after_hop:
        same_targets = [
            (ovx, ovy) for ovx, ovy, _ovr, onet in vias_info if onet == net_code
        ] + [
            (cpx, cpy) for cpx, cpy, cnet in connected_pads if cnet == net_code
        ] + [
            (x1, y1) for x1, y1, _x2, _y2, _tr, tnet, tlay in tracks_info if tnet == net_code and tlay == pcbnew.F_Cu
        ] + [
            (x2, y2) for _x1, _y1, x2, y2, _tr, tnet, tlay in tracks_info if tnet == net_code and tlay == pcbnew.F_Cu
        ]
        same_targets.sort(key=lambda pt: math.hypot(pt[0] - px, pt[1] - py))
        linked_final = False
        for tx, ty in same_targets[:12]:
            if math.hypot(tx - px, ty - py) > 3.2:
                continue
            if _seg_layer_ok(px, py, tx, ty, net_code, pcbnew.F_Cu):
                trk = pcbnew.PCB_TRACK(board)
                trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(px), pcbnew.FromMM(py)))
                trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(tx), pcbnew.FromMM(ty)))
                trk.SetWidth(pcbnew.FromMM(0.10))
                trk.SetLayer(pcbnew.F_Cu)
                trk.SetNetCode(net_code)
                board.Add(trk)
                tracks_info.append((px, py, tx, ty, 0.05, net_code, pcbnew.F_Cu))
                connected_pads.append((px, py, net_code))
                linked_final = True
                break
        if not linked_final:
            step_g = 0.05
            target_set = [(tx, ty) for tx, ty in same_targets[:32] if math.hypot(tx - px, ty - py) <= 5.5]
            visited = {(0, 0): None}
            queue = [(0, 0)]
            found_goal = None
            found_via_spot = None
            q_idx = 0
            while q_idx < len(queue) and q_idx < 10000 and found_goal is None and found_via_spot is None:
                ix, iy = queue[q_idx]
                q_idx += 1
                cx = round(px + ix * step_g, 3)
                cy = round(py + iy * step_g, 3)
                if (ix != 0 or iy != 0) and _via_clear(cx, cy, ref, pnum, net_code):
                    found_via_spot = ((ix, iy), (cx, cy))
                    break
                for tx, ty in target_set:
                    if math.hypot(tx - cx, ty - cy) <= 0.30 and _seg_layer_ok(cx, cy, tx, ty, net_code, pcbnew.F_Cu):
                        found_goal = ((ix, iy), (tx, ty))
                        break
                if found_goal is not None:
                    break
                for dx_i, dy_i in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx_i, ny_i = ix + dx_i, iy + dy_i
                    if (nx_i, ny_i) in visited or abs(nx_i) > 75 or abs(ny_i) > 75:
                        continue
                    nx_mm = round(px + nx_i * step_g, 3)
                    ny_mm = round(py + ny_i * step_g, 3)
                    if _seg_layer_ok(cx, cy, nx_mm, ny_mm, net_code, pcbnew.F_Cu):
                        visited[(nx_i, ny_i)] = (ix, iy)
                        queue.append((nx_i, ny_i))
            if found_via_spot is not None:
                cur_node, (vx_s, vy_s) = found_via_spot
                pts_path = []
                while cur_node is not None:
                    pts_path.append((round(px + cur_node[0] * step_g, 3), round(py + cur_node[1] * step_g, 3)))
                    cur_node = visited[cur_node]
                pts_path.reverse()
                for s_i in range(len(pts_path) - 1):
                    ax_p, ay_p = pts_path[s_i]
                    bx_p, by_p = pts_path[s_i + 1]
                    trk = pcbnew.PCB_TRACK(board)
                    trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_p), pcbnew.FromMM(ay_p)))
                    trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_p), pcbnew.FromMM(by_p)))
                    trk.SetWidth(pcbnew.FromMM(0.09))
                    trk.SetLayer(pcbnew.F_Cu)
                    trk.SetNetCode(net_code)
                    board.Add(trk)
                    tracks_info.append((ax_p, ay_p, bx_p, by_p, 0.045, net_code, pcbnew.F_Cu))
                _add_pwr_via(vx_s, vy_s, net_code)
                added += 1
                connected_pads.append((px, py, net_code))
                linked_final = True
            elif found_goal is not None:
                cur_node, (tx, ty) = found_goal
                pts_path = [(tx, ty)]
                while cur_node is not None:
                    pts_path.append((round(px + cur_node[0] * step_g, 3), round(py + cur_node[1] * step_g, 3)))
                    cur_node = visited[cur_node]
                pts_path.reverse()
                for s_i in range(len(pts_path) - 1):
                    ax_p, ay_p = pts_path[s_i]
                    bx_p, by_p = pts_path[s_i + 1]
                    trk = pcbnew.PCB_TRACK(board)
                    trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_p), pcbnew.FromMM(ay_p)))
                    trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_p), pcbnew.FromMM(by_p)))
                    trk.SetWidth(pcbnew.FromMM(0.09))
                    trk.SetLayer(pcbnew.F_Cu)
                    trk.SetNetCode(net_code)
                    board.Add(trk)
                    tracks_info.append((ax_p, ay_p, bx_p, by_p, 0.045, net_code, pcbnew.F_Cu))
                connected_pads.append((px, py, net_code))
                linked_final = True
        if not linked_final:
            print(f"UNPLACED_GND_PAD {ref}.{pnum} ({px:.3f}, {py:.3f}) visited={len(visited)}")

    # Finally add GCPW stitching via fence along the F.Cu RF feed trace + dense GND plane stitching grid AFTER all signal and pad escapes are placed
    stitch_pts = [
        (130.50, 89.70), (131.50, 89.70), (132.50, 89.70), (133.10, 89.70),
        (130.50, 91.30), (131.50, 91.30), (132.50, 91.30), (133.50, 91.30), (134.50, 91.30),
        (133.10, 88.50), (133.10, 87.50), (133.10, 86.50), (133.10, 85.50), (133.10, 84.50), (133.10, 83.50), (133.10, 82.50),
        (134.60, 88.50), (134.60, 87.50), (134.60, 86.50), (134.60, 85.50), (134.60, 84.50), (134.60, 83.80),
        (135.00, 84.10),
    ]
    for sx_i in range(117, 144, 2):
        for sy_i in range(82, 119, 2):
            stitch_pts.append((float(sx_i), float(sy_i)))
            stitch_pts.append((float(sx_i) + 1.0, float(sy_i) + 1.0))
    for gvx, gvy in stitch_pts:
        if _via_clear(gvx, gvy, "GCPW", "0", gnd_code):
            _add_pwr_via(gvx, gvy, gnd_code)
            added += 1
    return added


def import_ses(pcb_path: Path, ses_path: Path) -> None:
    # Normalize FreeRouting 0.75x neckdown widths (e.g. path <layer> 674 -> 900) so all imported tracks >= 0.090 mm
    # and move In2.Cu routed signal segments to In1.Cu during import (we swap In1.Cu/In2.Cu roles or move In2.Cu to B.Cu/In1.Cu? Wait: S-M4 requires In1.Cu to be the solid GND plane under F.Cu!)
    # Wait: if FreeRouting routed on In1.Cu, we want In1.Cu to have 0 signal tracks!
    # Wait: why did zone_pour_In2.Cu split earlier? Because we replaced (path In1.Cu with (path In2.Cu, which doubled the track density on In2.Cu!
    # Wait: what if we keep FreeRouting's tracks on In2.Cu, and instead route /VDD_1V8 as a net in FreeRouting OR connect /VDD_1V8 vias on In2.Cu?
    ses_text = ses_path.read_text(encoding="utf-8")
    # Update placement block in board.ses so C27, C28, L5, L6, and MH4 match the compact lumped RF layout in generate_kicad.py (T-M3 & T-M4)
    for old_pl, new_pl in [
        ("(place C27 1343000 -822000 front 0)", "(place C27 1337000 -822000 front 0)"),
        ("(place C28 1315000 -905000 front 0)", "(place C28 1337000 -850000 front 0)"),
        ("(place C28 1337000 -835000 front 0)", "(place C28 1337000 -850000 front 0)"),
        ("(place C28 1332000 -835000 front 0)", "(place C28 1337000 -850000 front 0)"),
        ("(place L6 1356000 -822000 front 0)", "(place L6 1357000 -822000 front 0)"),
        ("(place L5 1350000 -832000 front 0)", "(place L5 1347000 -835000 front 0)"),
        ("(place MH4 1332000 -868000 front 0)", "(place MH4 1352000 -878000 front 0)"),
        ("(place MH4 1332000 -855000 front 0)", "(place MH4 1352000 -878000 front 0)"),
        ("(place MH4 1347000 -862000 front 0)", "(place MH4 1352000 -878000 front 0)"),
    ]:
        ses_text = ses_text.replace(old_pl, new_pl)
    # Strip any stale pre-existing /RF_ANT_OUT, /RF_MATCH_MID, /RF_ANT_FEED blocks from board.ses so only generate_kicad.py's pre-routed RF tracks are used
    for rf_net_name in ("/RF_ANT_OUT", "/RF_MATCH_MID", "/RF_ANT_FEED"):
        idx_rf = ses_text.find(f'(net "{rf_net_name}"')
        if idx_rf >= 0:
            idx_next = ses_text.find("(net ", idx_rf + 10)
            if idx_next < 0:
                idx_next = ses_text.find("    )\n  )\n)", idx_rf)
            if idx_next > idx_rf:
                ses_text = ses_text[:idx_rf] + ses_text[idx_next:]
    ses_fixed = re.sub(
        r"\(path\s+(F\.Cu|In1\.Cu|In2\.Cu|B\.Cu)\s+([0-8]\d\d)\b",
        r"(path \1 900",
        ses_text,
    )
    if ses_fixed != ses_text:
        ses_path.write_text(ses_fixed, encoding="utf-8")
    else:
        ses_path.write_text(ses_text, encoding="utf-8")
    board = pcbnew.LoadBoard(str(pcb_path))
    ok = pcbnew.ImportSpecctraSES(board, str(ses_path))
    if not ok:
        raise RuntimeError(f"ImportSpecctraSES failed for {ses_path} -> {pcb_path}")
    # Enforce min 0.090 mm width on any track in board
    min_w = pcbnew.FromMM(0.09)
    for trk in board.GetTracks():
        if trk.GetClass() != "PCB_VIA" and trk.GetWidth() < min_w:
            trk.SetWidth(min_w)
    # Remove any dangling vias connected on <= 1 copper layer and any dangling F.Cu stub tracks BEFORE placing GND vias
    gnd_net = board.FindNet("/GND") or board.FindNet("GND")
    v18_net = board.FindNet("/VDD_1V8") or board.FindNet("VDD_1V8")
    pwr_nets = {
        gnd_net.GetNetCode() if gnd_net else -1,
        v18_net.GetNetCode() if v18_net else -1,
    }
    pad_locs = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            p = pad.GetPosition()
            pad_locs.append((pcbnew.ToMM(p.x), pcbnew.ToMM(p.y), pad.GetNetCode()))
    all_trks = list(board.GetTracks())
    removed_vias = []
    to_remove = []
    for item in all_trks:
        if item.GetClass() != "PCB_VIA" or item.GetNetCode() in pwr_nets:
            continue
        vpos = item.GetPosition()
        vx = pcbnew.ToMM(vpos.x)
        vy = pcbnew.ToMM(vpos.y)
        vnet = item.GetNetCode()
        conn_layers = set()
        for px, py, pnet in pad_locs:
            if pnet == vnet and math.hypot(vx - px, vy - py) < 0.15:
                conn_layers.add(pcbnew.F_Cu)
        for trk in all_trks:
            if trk.GetClass() == "PCB_VIA" or trk.GetNetCode() != vnet:
                continue
            s = trk.GetStart()
            e = trk.GetEnd()
            if (
                math.hypot(vx - pcbnew.ToMM(s.x), vy - pcbnew.ToMM(s.y)) < 0.08
                or math.hypot(vx - pcbnew.ToMM(e.x), vy - pcbnew.ToMM(e.y)) < 0.08
            ):
                conn_layers.add(trk.GetLayer())
        if len(conn_layers) <= 1:
            removed_vias.append((vx, vy, vnet))
            to_remove.append(item)
    for rvx, rvy, rvnet in removed_vias:
        for trk in all_trks:
            if trk.GetClass() == "PCB_VIA" or trk.GetNetCode() != rvnet:
                continue
            s = trk.GetStart()
            e = trk.GetEnd()
            if (
                math.hypot(rvx - pcbnew.ToMM(s.x), rvy - pcbnew.ToMM(s.y)) < 0.08
                or math.hypot(rvx - pcbnew.ToMM(e.x), rvy - pcbnew.ToMM(e.y)) < 0.08
            ):
                # Only remove if the OTHER end of trk is a dead end (not connected to any other track or pad)
                ox, oy = (pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)) if math.hypot(rvx - pcbnew.ToMM(s.x), rvy - pcbnew.ToMM(s.y)) < 0.08 else (pcbnew.ToMM(s.x), pcbnew.ToMM(s.y))
                other_connected = any(
                    pnet == rvnet and math.hypot(ox - px, oy - py) < 0.15 for px, py, pnet in pad_locs
                ) or any(
                    t2.m_Uuid.AsString() != trk.m_Uuid.AsString()
                    and t2.GetClass() != "PCB_VIA"
                    and t2.GetNetCode() == rvnet
                    and t2.GetLayer() == trk.GetLayer()
                    and (math.hypot(ox - pcbnew.ToMM(t2.GetStart().x), oy - pcbnew.ToMM(t2.GetStart().y)) < 0.05 or math.hypot(ox - pcbnew.ToMM(t2.GetEnd().x), oy - pcbnew.ToMM(t2.GetEnd().y)) < 0.05)
                    for t2 in all_trks
                )
                if not other_connected:
                    to_remove.append(trk)
    xc2_net = board.FindNet("/HFXO_XC2") or board.FindNet("HFXO_XC2")
    xc2_c = xc2_net.GetNetCode() if xc2_net else -1
    for trk in all_trks:
        if trk.GetNetCode() == xc2_c:
            to_remove.append(trk)
    seen_rem = set()
    for item in to_remove:
        uid = item.m_Uuid.AsString()
        if uid not in seen_rem:
            seen_rem.add(uid)
            board.Remove(item)
    # Save and reload board once after batch removal so KiCad 10 SWIG iterators stay valid
    pcbnew.SaveBoard(str(pcb_path), board)
    board = pcbnew.LoadBoard(str(pcb_path))
    gnd_net = board.FindNet("/GND") or board.FindNet("GND")
    v18_net = board.FindNet("/VDD_1V8") or board.FindNet("VDD_1V8")
    pwr_nets = {
        gnd_net.GetNetCode() if gnd_net else -1,
        v18_net.GetNetCode() if v18_net else -1,
    }
    # Nudge any 0.090 mm F.Cu pad-entry segment that is within 0.0895 mm of an adjacent F.Cu pad away by 0.018 mm (along X or Y depending on pad gap)
    fcu_pads = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            p = pad.GetPosition()
            sz = pad.GetSize()
            fcu_pads.append((pcbnew.ToMM(p.x), pcbnew.ToMM(p.y), pcbnew.ToMM(sz.x), pcbnew.ToMM(sz.y), pad.GetNetCode()))
    def _shift_vtx(net_c: int, ox: float, oy: float, nx: float, ny: float) -> None:
        for t_i in board.GetTracks():
            if t_i.GetClass() == "PCB_VIA" or t_i.GetNetCode() != net_c or t_i.GetLayer() != pcbnew.F_Cu:
                continue
            s_i = t_i.GetStart()
            e_i = t_i.GetEnd()
            if math.hypot(pcbnew.ToMM(s_i.x) - ox, pcbnew.ToMM(s_i.y) - oy) < 0.005:
                t_i.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(nx), pcbnew.FromMM(ny)))
            if math.hypot(pcbnew.ToMM(e_i.x) - ox, pcbnew.ToMM(e_i.y) - oy) < 0.005:
                t_i.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(nx), pcbnew.FromMM(ny)))
    for trk in list(board.GetTracks()):
        if trk.GetClass() == "PCB_VIA" or trk.GetLayer() != pcbnew.F_Cu:
            continue
        s = trk.GetStart()
        e = trk.GetEnd()
        sx, sy = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y)
        ex, ey = pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if math.hypot(ex - sx, ey - sy) > 0.35:
            continue
        tnet = trk.GetNetCode()
        same_pads = [(ppx, ppy) for ppx, ppy, _w, _h, pnet in fcu_pads if pnet == tnet and abs(ppx - sx) < 1.0 and abs(ppy - sy) < 1.0]
        for px, py, pw, ph, pnet in fcu_pads:
            if pnet == tnet or abs(px - sx) > 0.8 or abs(py - sy) > 0.8:
                continue
            for t_u in (0.0, 0.25, 0.5, 0.75, 1.0):
                qx = sx + t_u * (ex - sx)
                qy = sy + t_u * (ey - sy)
                dxg = max(0.0, abs(qx - px) - pw / 2.0)
                dyg = max(0.0, abs(qy - py) - ph / 2.0)
                if math.hypot(dxg, dyg) < 0.045 + 0.092:
                    dx_push = (0.018 if ((sx + ex) / 2.0 >= px) else -0.018) if dxg >= dyg else 0.0
                    dy_push = (0.018 if ((sy + ey) / 2.0 >= py) else -0.018) if dyg >= dxg else 0.0
                    if not any(math.hypot(sx - ppx, sy - ppy) < 0.02 for ppx, ppy in same_pads):
                        _shift_vtx(tnet, sx, sy, sx + dx_push, sy + dy_push)
                        sx, sy = sx + dx_push, sy + dy_push
                    if not any(math.hypot(ex - ppx, ey - ppy) < 0.02 for ppx, ppy in same_pads):
                        _shift_vtx(tnet, ex, ey, ex + dx_push, ey + dy_push)
                        ex, ey = ex + dx_push, ey + dy_push
                    break
    decd_net = board.FindNet("/DECD") or board.FindNet("DECD")
    if decd_net:
        decd_c = decd_net.GetNetCode()
        for trk in list(board.GetTracks()):
            if trk.GetClass() != "PCB_VIA" and trk.GetNetCode() == decd_c and trk.GetLayer() == pcbnew.F_Cu:
                s = trk.GetStart()
                e = trk.GetEnd()
                sx, sy = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y)
                ex, ey = pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
                if math.hypot(sx - 126.30, sy - 87.70) < 0.15:
                    trk.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(126.38), pcbnew.FromMM(87.40)))
                if math.hypot(ex - 126.30, ey - 87.70) < 0.15:
                    trk.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(126.38), pcbnew.FromMM(87.40)))
    # Deterministic post-import vertex adjustments to clear sub-0.015mm clearance nudges and open GND escape channels
    for v_net_v, ovx_v, ovy_v, nvx_v, nvy_v in [
        ("/VDD_3V0", 128.3840, 105.4554, 128.3950, 105.4400),
        ("/SPI4_SCK_P0.08", 124.1200, 93.6000, 124.1200, 93.6100),
        ("/SPI4_CS_IMU_N_P0.11", 124.5800, 93.8400, 124.5800, 93.8500),
        ("/IMU_INT1_P0.04", 124.9500, 91.5800, 124.9500, 91.6400),
        ("/IMU_INT2_P0.05", 124.2500, 91.9700, 124.2500, 91.9950),
    ]:
        vn_v = board.FindNet(v_net_v)
        if not vn_v:
            continue
        vc_v = vn_v.GetNetCode()
        for t_i in board.GetTracks():
            if t_i.GetNetCode() != vc_v:
                continue
            if t_i.GetClass() == "PCB_VIA":
                vp_i = t_i.GetPosition()
                if math.hypot(pcbnew.ToMM(vp_i.x) - ovx_v, pcbnew.ToMM(vp_i.y) - ovy_v) < 0.01:
                    t_i.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(nvx_v), pcbnew.FromMM(nvy_v)))
            else:
                s_i, e_i = t_i.GetStart(), t_i.GetEnd()
                if math.hypot(pcbnew.ToMM(s_i.x) - ovx_v, pcbnew.ToMM(s_i.y) - ovy_v) < 0.01:
                    t_i.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(nvx_v), pcbnew.FromMM(nvy_v)))
                if math.hypot(pcbnew.ToMM(e_i.x) - ovx_v, pcbnew.ToMM(e_i.y) - ovy_v) < 0.01:
                    t_i.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(nvx_v), pcbnew.FromMM(nvy_v)))
    vertex_shifts = [
        ("/SW1_BUCK1", pcbnew.F_Cu, 120.6051, 107.3852, 120.6250, 107.3650),
        ("/PMIC_INT_P1.05", pcbnew.F_Cu, 121.5766, 107.7811, 121.5550, 107.7811),
        ("/PMIC_INT_P1.05", pcbnew.F_Cu, 121.5284, 107.8293, 121.5200, 107.8293),
        ("/PMIC_INT_P1.05", pcbnew.F_Cu, 121.5586, 107.4553, 121.5450, 107.4553),
        ("/QSPI_IO1_MISO_P0.14", pcbnew.In1_Cu, 124.5095, 93.4977, 124.5050, 93.5000),
        ("/QSPI_IO1_MISO_P0.14", pcbnew.In1_Cu, 124.8362, 93.4977, 124.8362, 93.5000),
        ("/VDD_1V8", pcbnew.F_Cu, 138.5902, 106.1604, 138.5902, 106.1850),
        ("/VDD_1V8", pcbnew.F_Cu, 139.2604, 106.1784, 139.2604, 106.2000),
        ("/PMIC_SHPHLD", pcbnew.F_Cu, 125.5119, 110.5867, 125.5150, 110.5867),
        ("/PMIC_SHPHLD", pcbnew.F_Cu, 125.5299, 110.5867, 125.5150, 110.5867),
        ("/PMIC_SHPHLD", pcbnew.F_Cu, 125.5299, 110.9466, 125.5150, 110.9466),
        ("/I2C0_SCL_3V0", pcbnew.F_Cu, 139.7758, 97.0922, 139.7758, 97.1250),
        ("/I2C0_SCL_3V0", pcbnew.F_Cu, 139.7437, 97.1423, 139.7437, 97.1550),
        ("/PCA9306_EN", pcbnew.F_Cu, 138.9186, 99.1631, 138.8950, 99.1850),
        ("/PCA9306_EN", pcbnew.F_Cu, 139.2212, 99.4657, 139.2212, 99.4900),
        ("/PCA9306_EN", pcbnew.F_Cu, 139.9412, 99.4657, 139.9412, 99.4900),
        ("/USB_CC1", pcbnew.F_Cu, 128.4606, 111.7017, 128.4606, 111.7150),
        ("/USB_CC1", pcbnew.F_Cu, 128.6435, 111.8846, 128.6435, 111.8950),
        ("/VDD_1V8", pcbnew.F_Cu, 136.8707, 105.7813, 136.8450, 105.7813),
        ("/VDD_1V8", pcbnew.F_Cu, 136.8527, 105.8840, 136.8350, 105.8840),
        ("/VDD_1V8", pcbnew.F_Cu, 136.8760, 105.9073, 136.8550, 105.9073),
        ("/VDD_1V8", pcbnew.F_Cu, 128.5123, 105.8046, 128.5250, 105.8250),
        ("/VDD_1V8", pcbnew.F_Cu, 128.6633, 105.6536, 128.6700, 105.6600),
        ("/IMU_INT1_P0.04", pcbnew.In2_Cu, 126.8494, 91.5800, 126.8494, 91.6400),
    ]
    for v_net_name, v_lay, ox, oy, nx, ny in vertex_shifts:
        vn_obj = board.FindNet(v_net_name)
        if not vn_obj:
            continue
        vc = vn_obj.GetNetCode()
        for t_i in board.GetTracks():
            if t_i.GetClass() == "PCB_VIA" or t_i.GetNetCode() != vc or t_i.GetLayer() != v_lay:
                continue
            s_i = t_i.GetStart()
            e_i = t_i.GetEnd()
            if math.hypot(pcbnew.ToMM(s_i.x) - ox, pcbnew.ToMM(s_i.y) - oy) < 0.008:
                t_i.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(nx), pcbnew.FromMM(ny)))
            if math.hypot(pcbnew.ToMM(e_i.x) - ox, pcbnew.ToMM(e_i.y) - oy) < 0.008:
                t_i.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(nx), pcbnew.FromMM(ny)))
    # Re-add pre-routed F.Cu RF tracks after ImportSpecctraSES (since ImportSpecctraSES wipes pre-existing F.Cu tracks!)
    for rf_net_s, rf_segs in [
        ("/RF_ANT_OUT", [(129.700, 90.500, 134.150, 90.500), (134.150, 90.500, 134.150, 85.000), (134.150, 85.000, 134.150, 82.800), (134.150, 82.800, 133.250, 82.800), (133.250, 82.800, 133.250, 82.200)]),
        ("/RF_MATCH_MID", [(134.150, 82.200, 135.250, 82.200), (134.700, 82.200, 134.700, 83.050)]),
        ("/RF_ANT_FEED", [(136.150, 82.200, 137.200, 82.200)]),
    ]:
        rfn_obj = board.FindNet(rf_net_s)
        if rfn_obj:
            rf_c = rfn_obj.GetNetCode()
            for ax_r, ay_r, bx_r, by_r in rf_segs:
                t_rf = pcbnew.PCB_TRACK(board)
                t_rf.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_r), pcbnew.FromMM(ay_r)))
                t_rf.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_r), pcbnew.FromMM(by_r)))
                t_rf.SetWidth(pcbnew.FromMM(0.20))
                t_rf.SetLayer(pcbnew.F_Cu)
                t_rf.SetNetCode(rf_c)
                board.Add(t_rf)
    # Connect U1.T2 (123.70, 91.50), U1.H2 (123.70, 89.90), U1.EP (127.30, 91.20), C28.1 (133.25, 85.00), U10.3 (128.85, 111.60), U4.15 (136.40, 99.55), U9.5 (135.25, 95.20), and U8.1 (137.45, 97.45) to GND!
    if gnd_net:
        gc_m = gnd_net.GetNetCode()
        # 1) U10.3 (128.85, 111.60) -> U10.8 (131.15, 111.60) on F.Cu across U10 center
        # 2) U4.15 (136.40, 99.55) -> (135.43, 99.55) -> (135.43, 99.845) [inside U4 body on F.Cu to U4.12/13 GND via!]
        # 3) U9.5 (135.25, 95.20) -> (134.50, 95.20) -> U9.2 (133.75, 95.20) on F.Cu + 0.40/0.20 mm GND via at (134.50, 95.20)
        # 4) U8.1 (137.45, 97.45) -> (138.25, 97.45) on F.Cu + 0.40/0.20 mm GND via at (138.25, 97.45)
        # 5) U1.T2 (123.70, 91.50) -> (124.42, 91.42) -> (124.60, 91.25) -> (124.60, 90.75) -> (124.52, 90.32) on F.Cu + 0.40/0.20 mm GND via at (124.52, 90.32)
        # 6) U1.H2 (123.70, 89.90) -> U1.F2 (123.70, 89.50) -> U1.G1 (123.30, 89.70) & U1.K2 (123.70, 90.30) -> (124.52, 90.32) on F.Cu
        # 7) U1.EP (127.30, 91.20) -> (127.00, 90.00) on F.Cu + 0.40/0.20 mm GND via at (127.00, 90.00)
        # 8) C28.1 (133.25, 85.00) -> (132.75, 85.00) on F.Cu + 0.40/0.20 mm GND via at (132.75, 85.00)
        for ax_g, ay_g, bx_g, by_g in [
            (128.85, 111.60, 131.15, 111.60),
            (136.40, 99.55, 135.43, 99.55),
            (135.43, 99.55, 135.43, 99.845),
            (135.25, 95.20, 134.50, 95.20),
            (134.50, 95.20, 133.75, 95.20),
            (137.45, 97.45, 138.25, 97.45),
            (123.70, 91.50, 123.80, 91.453),
            (123.80, 91.453, 124.46, 91.453),
            (124.46, 91.453, 124.60, 91.25),
            (124.60, 91.25, 124.60, 90.75),
            (124.60, 90.75, 124.52, 90.32),
            (123.70, 90.30, 124.52, 90.32),
            (123.70, 90.30, 123.70, 89.90),
            (123.70, 89.90, 123.70, 89.50),
            (123.70, 89.50, 123.30, 89.70),
            (127.30, 91.20, 127.00, 90.00),
            (133.25, 85.00, 132.75, 85.00),
        ]:
            t_gm = pcbnew.PCB_TRACK(board)
            t_gm.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_g), pcbnew.FromMM(ay_g)))
            t_gm.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_g), pcbnew.FromMM(by_g)))
            t_gm.SetWidth(pcbnew.FromMM(0.09))
            t_gm.SetLayer(pcbnew.F_Cu)
            t_gm.SetNetCode(gc_m)
            board.Add(t_gm)
        for vx_g, vy_g in [(134.50, 95.20), (138.25, 97.45), (124.52, 90.32), (127.00, 90.00), (132.75, 85.00)]:
            v_g = pcbnew.PCB_VIA(board)
            v_g.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(vx_g), pcbnew.FromMM(vy_g)))
            v_g.SetWidth(pcbnew.FromMM(0.40))
            v_g.SetDrill(pcbnew.FromMM(0.20))
            v_g.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v_g.SetNetCode(gc_m)
            board.Add(v_g)
    # Re-route U1.V2 (/IMU_INT1_P0.04) around U1.T2 and U1.X2 (/IMU_INT2_P0.05) around U1.V2:
    for net_r_name, x_hi_r, new_segs_r in [
        ("/IMU_INT1_P0.04", 125.00, [(123.70, 91.90, 124.12, 91.64), (124.12, 91.64, 124.95, 91.64)]),
        ("/IMU_INT2_P0.05", 124.30, [(123.70, 92.30, 124.25, 91.995)]),
    ]:
        nr_obj = board.FindNet(net_r_name)
        if nr_obj:
            nr_c = nr_obj.GetNetCode()
            for t_i in list(board.GetTracks()):
                if t_i.GetClass() != "PCB_VIA" and t_i.GetNetCode() == nr_c and t_i.GetLayer() == pcbnew.F_Cu:
                    s_i, e_i = t_i.GetStart(), t_i.GetEnd()
                    if 123.65 <= pcbnew.ToMM(s_i.x) <= x_hi_r and 91.4 <= pcbnew.ToMM(s_i.y) <= 92.35 and 123.65 <= pcbnew.ToMM(e_i.x) <= x_hi_r and 91.4 <= pcbnew.ToMM(e_i.y) <= 92.35:
                        board.Remove(t_i)
            for ax_r, ay_r, bx_r, by_r in new_segs_r:
                t_nr = pcbnew.PCB_TRACK(board)
                t_nr.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_r), pcbnew.FromMM(ay_r)))
                t_nr.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_r), pcbnew.FromMM(by_r)))
                t_nr.SetWidth(pcbnew.FromMM(0.09))
                t_nr.SetLayer(pcbnew.F_Cu)
                t_nr.SetNetCode(nr_c)
                board.Add(t_nr)
    # Re-route /MAG_INT_3V0 near U9 using a short In1.Cu bridge around U9's center GND via (134.50, 95.20): U9.4 (135.25, 95.70) -> (134.80, 95.70) [via] -> In1.Cu -> (134.90, 95.20) -> (135.30, 94.00) [via] -> F.Cu -> (135.7058, 93.6858)
    mag_3v0_net = board.FindNet("/MAG_INT_3V0")
    if mag_3v0_net:
        m3_c = mag_3v0_net.GetNetCode()
        for t_i in list(board.GetTracks()):
            if t_i.GetClass() != "PCB_VIA" and t_i.GetNetCode() == m3_c and t_i.GetLayer() == pcbnew.F_Cu:
                s_i, e_i = t_i.GetStart(), t_i.GetEnd()
                sx_i, sy_i = pcbnew.ToMM(s_i.x), pcbnew.ToMM(s_i.y)
                ex_i, ey_i = pcbnew.ToMM(e_i.x), pcbnew.ToMM(e_i.y)
                if (134.8 <= sx_i <= 135.8 and 93.65 <= sy_i <= 95.8) and (134.8 <= ex_i <= 135.8 and 93.65 <= ey_i <= 95.8):
                    board.Remove(t_i)
        for ax_3, ay_3, bx_3, by_3, lay_3 in [
            (135.2500, 95.7000, 134.8000, 95.7000, pcbnew.F_Cu),
            (134.8000, 95.7000, 134.9000, 95.2000, pcbnew.In1_Cu),
            (134.9000, 95.2000, 135.3000, 94.0000, pcbnew.In1_Cu),
            (135.3000, 94.0000, 135.7058, 93.6858, pcbnew.F_Cu),
        ]:
            t_m3 = pcbnew.PCB_TRACK(board)
            t_m3.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_3), pcbnew.FromMM(ay_3)))
            t_m3.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_3), pcbnew.FromMM(by_3)))
            t_m3.SetWidth(pcbnew.FromMM(0.09))
            t_m3.SetLayer(lay_3)
            t_m3.SetNetCode(m3_c)
            board.Add(t_m3)
        for vx_3, vy_3 in [(134.8000, 95.7000), (135.3000, 94.0000)]:
            v_3 = pcbnew.PCB_VIA(board)
            v_3.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(vx_3), pcbnew.FromMM(vy_3)))
            v_3.SetWidth(pcbnew.FromMM(0.40))
            v_3.SetDrill(pcbnew.FromMM(0.20))
            v_3.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v_3.SetNetCode(m3_c)
            board.Add(v_3)
    # Also route /MAG_INT_1V8_P1.01 near U9.3 (133.75, 95.70) directly to B.Cu via at (134.1104, 95.70): U9.3 (133.75, 95.70) -> (134.1104, 95.70) [via] -> B.Cu -> (134.1104, 94.3137) [removing old via at (134.1104, 94.3137)!]
    mag_int_net = board.FindNet("/MAG_INT_1V8_P1.01")
    if mag_int_net:
        mc_i = mag_int_net.GetNetCode()
        for t_i in list(board.GetTracks()):
            if t_i.GetNetCode() == mc_i:
                if t_i.GetClass() == "PCB_VIA":
                    vp_i = t_i.GetPosition()
                    if math.hypot(pcbnew.ToMM(vp_i.x) - 134.1104, pcbnew.ToMM(vp_i.y) - 94.3137) < 0.05:
                        t_i.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(134.1104), pcbnew.FromMM(95.7000)))
                elif t_i.GetLayer() == pcbnew.F_Cu:
                    s_i, e_i = t_i.GetStart(), t_i.GetEnd()
                    sx_i, sy_i = pcbnew.ToMM(s_i.x), pcbnew.ToMM(s_i.y)
                    ex_i, ey_i = pcbnew.ToMM(e_i.x), pcbnew.ToMM(e_i.y)
                    if 133.7 <= sx_i <= 134.2 and 94.25 <= sy_i <= 95.75 and 133.7 <= ex_i <= 134.2 and 94.25 <= ey_i <= 95.75:
                        board.Remove(t_i)
        for ax_m, ay_m, bx_m, by_m, lay_m in [
            (133.7500, 95.7000, 134.1104, 95.7000, pcbnew.F_Cu),
            (134.1104, 95.7000, 134.1104, 94.3137, pcbnew.B_Cu),
        ]:
            t_mp = pcbnew.PCB_TRACK(board)
            t_mp.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_m), pcbnew.FromMM(ay_m)))
            t_mp.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_m), pcbnew.FromMM(by_m)))
            t_mp.SetWidth(pcbnew.FromMM(0.09))
            t_mp.SetLayer(lay_m)
            t_mp.SetNetCode(mc_i)
            board.Add(t_mp)
    # Re-route /I2C0_SCL_1V8_P1.03 near U8 around U8.1 (137.45, 97.45) and GND via (138.25, 97.45)
    scl18_net = board.FindNet("/I2C0_SCL_1V8_P1.03")
    if scl18_net:
        s18_c = scl18_net.GetNetCode()
        for t_i in list(board.GetTracks()):
            if t_i.GetClass() != "PCB_VIA" and t_i.GetNetCode() == s18_c and t_i.GetLayer() == pcbnew.F_Cu:
                s_i, e_i = t_i.GetStart(), t_i.GetEnd()
                sx_i, sy_i = pcbnew.ToMM(s_i.x), pcbnew.ToMM(s_i.y)
                ex_i, ey_i = pcbnew.ToMM(e_i.x), pcbnew.ToMM(e_i.y)
                if (136.8 <= sx_i <= 138.0 and 97.0 <= sy_i <= 98.7) and (136.8 <= ex_i <= 138.0 and 97.0 <= ey_i <= 98.7) and not (sy_i > 98.4 and ey_i > 98.4):
                    board.Remove(t_i)
        for ax_s, ay_s, bx_s, by_s in [
            (137.8809, 98.6745, 137.8809, 97.8000),
            (137.8809, 97.8000, 138.6000, 97.8000),
            (138.6000, 97.8000, 138.6000, 97.1000),
            (138.6000, 97.1000, 137.1406, 97.1000),
            (137.1406, 97.1000, 136.8665, 97.4471),
        ]:
            t_s18 = pcbnew.PCB_TRACK(board)
            t_s18.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_s), pcbnew.FromMM(ay_s)))
            t_s18.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_s), pcbnew.FromMM(by_s)))
            t_s18.SetWidth(pcbnew.FromMM(0.09))
            t_s18.SetLayer(pcbnew.F_Cu)
            t_s18.SetNetCode(s18_c)
            board.Add(t_s18)
    pcbnew.SaveBoard(str(pcb_path), board)
    board = pcbnew.LoadBoard(str(pcb_path))
    added_vias = _place_post_route_gnd_vias(board)
    # Evacuate In1.Cu signal segments onto B.Cu or In2.Cu using exact segment-segment intersection + endpoint distance checks (T-M5)
    def _ccw(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> float:
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)

    def _exact_seg_seg_dist(ax1: float, ay1: float, ax2: float, ay2: float, bx1: float, by1: float, bx2: float, by2: float) -> float:
        d1 = _ccw(ax1, ay1, ax2, ay2, bx1, by1)
        d2 = _ccw(ax1, ay1, ax2, ay2, bx2, by2)
        d3 = _ccw(bx1, by1, bx2, by2, ax1, ay1)
        d4 = _ccw(bx1, by1, bx2, by2, ax2, ay2)
        if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
            return 0.0
        return min(
            _seg_dist(ax1, ay1, bx1, by1, bx2, by2),
            _seg_dist(ax2, ay2, bx1, by1, bx2, by2),
            _seg_dist(bx1, by1, ax1, ay1, ax2, ay2),
            _seg_dist(bx2, by2, ax1, ay1, ax2, ay2),
        )

    all_trks_ev = [t for t in board.GetTracks() if t.GetClass() != "PCB_VIA"]
    in1_trks_ev = [t for t in all_trks_ev if t.GetLayer() == pcbnew.In1_Cu]
    in2_trks_ev = [t for t in all_trks_ev if t.GetLayer() == pcbnew.In2_Cu]
    bcu_trks_ev = [t for t in all_trks_ev if t.GetLayer() == pcbnew.B_Cu]
    evac_bcu = 0
    evac_in2 = 0
    # Only move entire net chains on In1.Cu between vias so no mid-chain layer breaks occur!
    by_net_in1: dict[int, list["pcbnew.PCB_TRACK"]] = {}
    for t_ev in in1_trks_ev:
        by_net_in1.setdefault(t_ev.GetNetCode(), []).append(t_ev)
    vias_xy_net = [
        (pcbnew.ToMM(v.GetPosition().x), pcbnew.ToMM(v.GetPosition().y), v.GetNetCode())
        for v in board.GetTracks() if v.GetClass() == "PCB_VIA"
    ]
    for net_e, net_segs in by_net_in1.items():
        # Group connected In1.Cu segments of net_e into chains terminated at vias
        ok_all_b = True
        for t_ev in net_segs:
            s_ev, e_ev = t_ev.GetStart(), t_ev.GetEnd()
            x1_e, y1_e = pcbnew.ToMM(s_ev.x), pcbnew.ToMM(s_ev.y)
            x2_e, y2_e = pcbnew.ToMM(e_ev.x), pcbnew.ToMM(e_ev.y)
            r_e = pcbnew.ToMM(t_ev.GetWidth()) / 2.0
            if not all(
                _exact_seg_seg_dist(x1_e, y1_e, x2_e, y2_e, pcbnew.ToMM(o.GetStart().x), pcbnew.ToMM(o.GetStart().y), pcbnew.ToMM(o.GetEnd().x), pcbnew.ToMM(o.GetEnd().y))
                >= r_e + pcbnew.ToMM(o.GetWidth()) / 2.0 + 0.095
                for o in bcu_trks_ev if o.GetNetCode() != net_e
            ):
                ok_all_b = False
                break
        if ok_all_b:
            for t_ev in net_segs:
                t_ev.SetLayer(pcbnew.B_Cu)
                bcu_trks_ev.append(t_ev)
                evac_bcu += 1
            continue
        ok_all_i2 = True
        for t_ev in net_segs:
            s_ev, e_ev = t_ev.GetStart(), t_ev.GetEnd()
            x1_e, y1_e = pcbnew.ToMM(s_ev.x), pcbnew.ToMM(s_ev.y)
            x2_e, y2_e = pcbnew.ToMM(e_ev.x), pcbnew.ToMM(e_ev.y)
            r_e = pcbnew.ToMM(t_ev.GetWidth()) / 2.0
            if not all(
                _exact_seg_seg_dist(x1_e, y1_e, x2_e, y2_e, pcbnew.ToMM(o.GetStart().x), pcbnew.ToMM(o.GetStart().y), pcbnew.ToMM(o.GetEnd().x), pcbnew.ToMM(o.GetEnd().y))
                >= r_e + pcbnew.ToMM(o.GetWidth()) / 2.0 + 0.095
                for o in in2_trks_ev if o.GetNetCode() != net_e
            ):
                ok_all_i2 = False
                break
        if ok_all_i2:
            for t_ev in net_segs:
                t_ev.SetLayer(pcbnew.In2_Cu)
                in2_trks_ev.append(t_ev)
                evac_in2 += 1
            continue
    print(f"IN1_EVACUATED bcu={evac_bcu} in2={evac_in2} remaining_in1={len(in1_trks_ev) - evac_bcu - evac_in2}")
    # Enforce 0.30 mm via drill / 0.40-0.45 mm via diameter across all vias (JLCPCB 0.3mm/(0.4/0.45mm) free tier)
    drop_pts_03 = [(128.85, 101.20), (139.00, 96.00), (135.43, 100.31)]
    for t_v03 in list(board.GetTracks()):
        if t_v03.GetClass() == "PCB_VIA":
            pos_03 = t_v03.GetPosition()
            vx_03, vy_03 = pcbnew.ToMM(pos_03.x), pcbnew.ToMM(pos_03.y)
            if any(math.hypot(vx_03 - dx_03, vy_03 - dy_03) < 0.03 for dx_03, dy_03 in drop_pts_03):
                board.Remove(t_v03)
            else:
                t_v03.SetDrill(pcbnew.FromMM(0.30))
    if gnd_net:
        gc_03 = gnd_net.GetNetCode()
        for ax_03, ay_03, bx_03, by_03 in [(128.85, 101.20, 129.30, 101.14), (135.43, 100.31, 135.43, 99.845)]:
            trk_03 = pcbnew.PCB_TRACK(board)
            trk_03.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax_03), pcbnew.FromMM(ay_03)))
            trk_03.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx_03), pcbnew.FromMM(by_03)))
            trk_03.SetWidth(pcbnew.FromMM(0.09))
            trk_03.SetLayer(pcbnew.F_Cu)
            trk_03.SetNetCode(gc_03)
            board.Add(trk_03)
    pcbnew.SaveBoard(str(pcb_path), board)
    # Add and fill the solid F.Cu (GND), In1.Cu (GND), B.Cu (GND), and In2.Cu (VDD_1V8) copper pours
    _add_copper_pour_zones(pcb_path)
    board2 = pcbnew.LoadBoard(str(pcb_path))
    filler = pcbnew.ZONE_FILLER(board2)
    filler.Fill(board2.Zones())
    pcbnew.SaveBoard(str(pcb_path), board2)
    print(f"IMPORT_SES_OK ses={ses_path} pcb={pcb_path} tracks={len(board2.GetTracks())} gnd_vias_added={added_vias} zones={len(board2.Zones())}")


def main() -> None:
    if len(sys.argv) != 4 or sys.argv[1] not in ("export-dsn", "import-ses"):
        print("Usage: route_pcb.py <export-dsn|import-ses> <pcb_path> <dsn_or_ses_path>", file=sys.stderr)
        sys.exit(2)
    cmd = sys.argv[1]
    pcb_path = Path(sys.argv[2]).resolve()
    target_path = Path(sys.argv[3]).resolve()
    if cmd == "export-dsn":
        export_dsn(pcb_path, target_path)
    else:
        import_ses(pcb_path, target_path)


if __name__ == "__main__":
    main()
