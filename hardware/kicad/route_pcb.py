#!/usr/bin/env python3
"""Export Specctra DSN or import Specctra SES (with post-import GND/VDD copper pours) for hockey_sensor.kicad_pcb."""

import sys
from pathlib import Path
import pcbnew


def export_dsn(pcb_path: Path, dsn_path: Path) -> None:
    board = pcbnew.LoadBoard(str(pcb_path))
    ok = pcbnew.ExportSpecctraDSN(board, str(dsn_path))
    if not ok or not dsn_path.exists():
        raise RuntimeError(f"ExportSpecctraDSN failed for {pcb_path} -> {dsn_path}")
    print(f"EXPORT_DSN_OK pcb={pcb_path} dsn={dsn_path} bytes={dsn_path.stat().st_size}")


def _add_copper_pour_zones(pcb_path: Path) -> None:
    """Append In1.Cu (GND), B.Cu (GND), and In2.Cu (VDD_1V8) copper pour zones around the 8x8mm top-right RF keepout."""
    text = pcb_path.read_text(encoding="utf-8")
    if "zone_pour_in1_gnd" in text:
        return
    # Look up net IDs for GND and VDD_1V8 from top-level (net <id> "<name>") lines
    gnd_id = 1
    v18_id = 5
    for line in text.splitlines():
        s = line.strip()
        parts = s.split()
        if len(parts) == 3 and parts[0] == "(net" and parts[1].isdigit():
            if parts[2] == '"GND")':
                gnd_id = int(parts[1])
            elif parts[2] == '"VDD_1V8")':
                v18_id = int(parts[1])

    # L-shaped polygon covering the 38x28 mm board (x: 116.3..143.7, y: 81.3..118.7)
    # while carving out the 8.0x8.0 mm top-right RF keepout (x: 136.0..144.0, y: 81.0..89.0)
    poly_pts = (
        "(xy 116.300 81.300) (xy 135.800 81.300) (xy 135.800 89.200) "
        "(xy 143.700 89.200) (xy 143.700 118.700) (xy 116.300 118.700)"
    )
    zones_sexpr = []
    for z_uuid, z_layer, z_net_id, z_net_name in [
        ("11111111-2222-3333-4444-555555550001", "In1.Cu", gnd_id, "GND"),
        ("11111111-2222-3333-4444-555555550002", "B.Cu", gnd_id, "GND"),
        ("11111111-2222-3333-4444-555555550003", "In2.Cu", v18_id, "VDD_1V8"),
    ]:
        zones_sexpr.extend([
            "  (zone",
            f"    (net {z_net_id})",
            f'    (net_name "{z_net_name}")',
            f'    (layer "{z_layer}")',
            f'    (uuid "{z_uuid}")',
            '    (name "zone_pour_in1_gnd")' if z_layer == "In1.Cu" else f'    (name "zone_pour_{z_layer}")',
            "    (hatch edge 0.5)",
            "    (connect_pads (clearance 0.15))",
            "    (min_thickness 0.2)",
            "    (filled_areas_thickness no)",
            "    (fill yes (thermal_gap 0.25) (thermal_bridge_width 0.25))",
            "    (polygon",
            "      (pts",
            f"        {poly_pts}",
            "      )",
            "    )",
            "  )",
        ])
    idx = text.rfind(")")
    new_text = text[:idx] + "\n" + "\n".join(zones_sexpr) + "\n)\n"
    pcb_path.write_text(new_text, encoding="utf-8")


def import_ses(pcb_path: Path, ses_path: Path) -> None:
    board = pcbnew.LoadBoard(str(pcb_path))
    ok = pcbnew.ImportSpecctraSES(board, str(ses_path))
    if not ok:
        raise RuntimeError(f"ImportSpecctraSES failed for {ses_path} -> {pcb_path}")
    pcbnew.SaveBoard(str(pcb_path), board)
    # Add and fill the solid In1.Cu (GND), B.Cu (GND), and In2.Cu (VDD_1V8) copper pours
    _add_copper_pour_zones(pcb_path)
    board2 = pcbnew.LoadBoard(str(pcb_path))
    filler = pcbnew.ZONE_FILLER(board2)
    filler.Fill(board2.Zones())
    pcbnew.SaveBoard(str(pcb_path), board2)
    print(f"IMPORT_SES_OK ses={ses_path} pcb={pcb_path} tracks={len(board2.GetTracks())} zones={len(board2.Zones())}")


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
