#!/usr/bin/env python3
"""Master dual-mode E2E unittest runner for hockey_sensor.

Modes:
  Default (strict full mode): Runs all Tiers (self, 1, 2, 3, 4, rubric) with zero mocks/skips.
    Exits 1 immediately if any M2/M3/M4 artifact is missing or any test/rubric pillar fails.
  --contract-only: Validates M1 (F1-F3 Tier 1 & Tier 2), e2e/test_self_validators.py,
    and e2e/test_rubric.py in CONTRACT-ONLY mode. Exits 0 iff all pass.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from e2e import helpers  # noqa: E402

M1_TEST_CLASSES = (
    "e2e.test_tier1_features.TestF1SystemSpecification",
    "e2e.test_tier1_features.TestF2InterfaceContract",
    "e2e.test_tier1_features.TestF3ContractValidator",
    "e2e.test_tier2_boundaries.TestF1Boundaries",
    "e2e.test_tier2_boundaries.TestF2Boundaries",
    "e2e.test_tier2_boundaries.TestF3Boundaries",
)


def build_suite(*, contract_only: bool, tier: str) -> unittest.TestSuite:
    loader = unittest.defaultTestLoader
    suite = unittest.TestSuite()

    if contract_only:
        if tier in ("all", "self"):
            suite.addTests(loader.loadTestsFromName("e2e.test_self_validators"))
        if tier in ("all", "m1", "1", "2"):
            for cls_name in M1_TEST_CLASSES:
                if tier == "1" and "test_tier2_boundaries" in cls_name:
                    continue
                if tier == "2" and "test_tier1_features" in cls_name:
                    continue
                suite.addTests(loader.loadTestsFromName(cls_name))
        if tier in ("all", "rubric"):
            suite.addTests(loader.loadTestsFromName("e2e.test_rubric"))
        return suite

    # Strict Full Mode
    if tier == "m1":
        for cls_name in M1_TEST_CLASSES:
            suite.addTests(loader.loadTestsFromName(cls_name))
        return suite

    tier_modules = {
        "self": ["e2e.test_self_validators"],
        "1": ["e2e.test_tier1_features"],
        "2": ["e2e.test_tier2_boundaries"],
        "3": ["e2e.test_tier3_pairwise"],
        "4": ["e2e.test_tier4_scenarios"],
        "rubric": ["e2e.test_rubric"],
        "all": [
            "e2e.test_self_validators",
            "e2e.test_tier1_features",
            "e2e.test_tier2_boundaries",
            "e2e.test_tier3_pairwise",
            "e2e.test_tier4_scenarios",
            "e2e.test_rubric",
        ],
    }
    for mod in tier_modules[tier]:
        suite.addTests(loader.loadTestsFromName(mod))
    return suite


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="hockey_sensor E2E unittest runner")
    parser.add_argument(
        "--contract-only",
        action="store_true",
        help="Validate M1 contract, E2E self-validators, and CONTRACT-ONLY rubric",
    )
    parser.add_argument(
        "--tier",
        choices=["all", "1", "2", "3", "4", "self", "rubric", "m1"],
        default="all",
        help="Select which test tier to execute (default: all)",
    )
    parser.add_argument(
        "-v",
        "--verbosity",
        type=int,
        choices=[1, 2],
        default=1,
        help="Unittest verbosity level (1 or 2)",
    )
    parser.add_argument(
        "--failfast",
        action="store_true",
        help="Stop on first failure or error",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional path to write structured JSON rubric report",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    if args.contract_only and args.tier in ("3", "4"):
        print(
            f"ERROR: --contract-only cannot be combined with --tier {args.tier} "
            "(Tier 3/4 require full M2/M3/M4 downstream deliverables).",
            file=sys.stderr,
        )
        return 2

    os.environ["HKS_E2E_CONTRACT_ONLY"] = "1" if args.contract_only else "0"
    os.environ["HKS_E2E_SUPPRESS_RUBRIC_PRINT"] = "1"
    if args.json_out is not None:
        os.environ["HKS_E2E_RUBRIC_JSON"] = str(args.json_out)
    else:
        os.environ.pop("HKS_E2E_RUBRIC_JSON", None)

    suite = build_suite(contract_only=args.contract_only, tier=args.tier)
    runner = unittest.TextTestRunner(verbosity=args.verbosity, failfast=args.failfast)
    result = runner.run(suite)

    if result.testsRun <= 0:
        print("ERROR: Zero tests were executed (testsRun == 0).", file=sys.stderr)
        return 2

    report = helpers.evaluate_rubric(contract_only=args.contract_only)
    print("\n" + helpers.format_rubric_table(report))

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")

    ok = result.wasSuccessful() and report.overall_pass and result.testsRun > 0
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
