"""Produce inspectable hypotheses; never imply physical incident reproduction."""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

import yaml

from .analysis import Observation, ObservationError


def _ros_topic(name: str) -> str:
    return "/" + name.lstrip("/")


def candidate_scenario(
    observed: Observation,
    *,
    input_topic: str | None = None,
) -> dict:
    if observed.largest_gap is None:
        raise ObservationError("No timing-gap candidate to generate")
    output = _ros_topic(observed.topic)
    source = _ros_topic(input_topic or f"{output}_raw")
    if source == output or source == "/":
        raise ObservationError("Input topic must differ from the output topic")

    start = (observed.largest_gap.start_ns - observed.bag_start_ns) / 1e9
    duration = observed.largest_gap.duration_s
    # drop_probability=1 is a *hypothesis* matching a gap symptom, not an
    # inference about which physical cause produced the original recording.
    return {
        "injectors": [
            {
                "id": "bag2test_topic",
                "type": observed.injector_type,
                "topic": {
                    "input_topic": source,
                    "output_topic": output,
                    "qos_depth": 10,
                },
            }
        ],
        "faults": [
            {
                "id": "candidate_gap",
                "injector_id": "bag2test_topic",
                "active_on_startup": False,
                "start": round(start, 6),
                "duration": round(duration, 6),
                "config": {"drop_probability": 1.0},
            }
        ],
        "assertions": [
            {
                "id": "candidate_fault_activates",
                "type": "fault_event",
                "fault_id": "candidate_gap",
                "state": "active",
                "within": round(start + 5.0, 6),
            }
        ],
    }


def write_results(
    observed: Observation,
    directory: str | Path,
    *,
    input_topic: str | None = None,
) -> list[Path]:
    out = Path(directory)
    if out.exists():
        raise ObservationError(f"Output directory already exists: {out}")
    scenario = (
        candidate_scenario(observed, input_topic=input_topic)
        if observed.largest_gap
        else None
    )

    report = asdict(observed)
    report.update(
        {
            "observation": "MCAP message log_time gaps only",
            "hypothesis": (
                "A temporary full message drop could reproduce this timing symptom"
                if scenario
                else None
            ),
            "hypothesis_status": "UNTESTED" if scenario else "NOT_APPLICABLE",
            "symptom_reproduced": "NOT_VERIFIED",
            "regression_verified": "NOT_VERIFIED",
            "root_cause": "UNKNOWN",
            "limitations": [
                "Recording gaps do not prove message loss at the publisher.",
                "Runner startup time may differ from recording origin; schedule alignment is required.",
                "A short injection window may be missed by real ROS timer scheduling.",
                "The activation assertion does not verify the robot's failure symptom.",
                "Upstream ROS runner execution has not been performed.",
            ],
        }
    )
    out.mkdir(parents=True)
    written: list[Path] = []
    evidence_file = out / "evidence.json"
    evidence_file.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    written.append(evidence_file)
    if scenario:
        scenario_file = out / "scenario.yaml"
        scenario_file.write_text(
            yaml.safe_dump(scenario, sort_keys=False), encoding="utf-8"
        )
        written.append(scenario_file)
    return written
