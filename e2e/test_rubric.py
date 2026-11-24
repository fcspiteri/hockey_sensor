"""Automated 5-Pillar Production-Readiness Rubric Verification (CONTRACT-ONLY and FULL modes)."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from e2e import helpers  # noqa: E402


class TestProductionReadinessRubric(unittest.TestCase):
    def test_five_pillar_production_readiness_rubric(self) -> None:
        contract_only = helpers.is_contract_only_mode()
        report = helpers.evaluate_rubric(contract_only=contract_only)
        table = helpers.format_rubric_table(report)

        if os.environ.get("HKS_E2E_SUPPRESS_RUBRIC_PRINT", "0") != "1":
            print("\n" + table)

        json_out = os.environ.get("HKS_E2E_RUBRIC_JSON", "").strip()
        if json_out:
            out_path = Path(json_out)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")

        self.assertEqual(len(report.pillars), 5)
        self.assertEqual([p.pillar_id for p in report.pillars], [1, 2, 3, 4, 5])
        failed = [
            f"P{p.pillar_id}.{c.name}: {c.detail}"
            for p in report.pillars
            for c in p.checks
            if not c.passed
        ]
        self.assertTrue(
            report.overall_pass,
            msg=f"Production-Readiness Rubric FAILED ({report.mode}): " + "; ".join(failed),
        )


if __name__ == "__main__":
    unittest.main()
