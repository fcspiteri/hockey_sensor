#!/usr/bin/env python3
"""Export Specctra DSN or import Specctra SES for hockey_sensor.kicad_pcb using KiCad pcbnew."""

import sys
from pathlib import Path
import pcbnew


def export_dsn(pcb_path: Path, dsn_path: Path) -> None:
    board = pcbnew.LoadBoard(str(pcb_path))
    ok = pcbnew.ExportSpecctraDSN(board, str(dsn_path))
    if not ok or not dsn_path.exists():
        raise RuntimeError(f"ExportSpecctraDSN failed for {pcb_path} -> {dsn_path}")
    print(f"EXPORT_DSN_OK pcb={pcb_path} dsn={dsn_path} bytes={dsn_path.stat().st_size}")


def import_ses(pcb_path: Path, ses_path: Path) -> None:
    board = pcbnew.LoadBoard(str(pcb_path))
    ok = pcbnew.ImportSpecctraSES(board, str(ses_path))
    if not ok:
        raise RuntimeError(f"ImportSpecctraSES failed for {ses_path} -> {pcb_path}")
    # Rebuild connectivity / zones if applicable before saving
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    pcbnew.SaveBoard(str(pcb_path), board)
    print(f"IMPORT_SES_OK ses={ses_path} pcb={pcb_path} tracks={len(board.GetTracks())}")


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
