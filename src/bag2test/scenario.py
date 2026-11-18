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

    if observed.time_basis == "header":
        if observed.timing_origin_ns is None or observed.first_selected_log_ns is None:
            raise ObservationError("Header clock origin is missing")
        # The header clock is independent of the recorder's log clock.
        # Place header-relative elapsed time after the first observed
        # selected-topic message on the bag clock. This is an *assumption*
        # the engineer must align to their actual independent ROS runner.
        start_ns = (
            observed.first_selected_log_ns
            - observed.bag_start_ns
            + observed.largest_gap.start_ns
            - observed.timing_origin_ns
        )
    else:
        start_ns = observed.largest_gap.start_ns - observed.bag_start_ns
    start = start_ns / 1e9
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
    limitations = [
        "A recorded timestamp gap alone cannot prove whether a publisher or receiver lost messages.",
        (
            "A faulted recording cannot supply messages absent from its own gap; "
            "evaluate hypotheses against a separate healthy input stream."
        ),
        "Runner startup and recording time origins may differ; align clocks.",
        "Short injection windows may be missed by ROS timer scheduling.",
        "The fault activation assertion does not verify a robot failure symptom.",
        "Reproduction of this individual incident has not been performed.",
    ]
    if observed.time_basis == "header":
        limitations.append(
            "header.stamp is a sender-provided ROS timestamp, not a verified "
            "synchronized clock. Candidate start is anchored at the first "
            "selected-topic log_time plus elapsed header time; runner clock "
            "alignment remains unverified."
        )
        limitations.append(
            "Trailing coverage, when present, is measured from MCAP log_time "
            "even when the cadence uses header.stamp. It is not physical "
            "proof that the sender was active."
        )
    if observed.trailing_silence_s is not None:
        limitations.append(
            "The selected topic stops before the recording's global endpoint; "
            "other recorded messages provide a continued observation window. "
            "This is a recording-coverage observation, not independent proof "
            "of sensor failure, missing publishes or a restart."
        )
        limitations.append(
            "A trailing-only silence has no observed recovery or known finite "
            "fault duration; no bounded fault-injection YAML can be justified."
        )
    if observed.publish_times_equal_log_times is True:
        limitations.append(
            "All selected MCAP publish_time values equal log_time; this file "
            "provides no independent publish-vs-record clock evidence."
        )
    if (
        observed.largest_gap
        and observed.largest_gap.following_interval_ratio is not None
        and observed.largest_gap.following_interval_ratio < 0.5
    ):
        limitations.append(
            "The message immediately after the largest gap arrived faster "
            "than half the median period; catch-up or batching is plausible."
        )
    report.update(
        {
            "observation": (
                "ROS 2 CDR Header.stamp intervals (sender time)"
                if observed.time_basis == "header"
                else "MCAP message log_time gaps only"
            ),
            "hypothesis": (
                "A temporary full message drop could reproduce this timing symptom"
                if scenario
                else None
            ),
            "hypothesis_status": "UNTESTED" if scenario else "NOT_APPLICABLE",
            "symptom_reproduced": "NOT_VERIFIED",
            "regression_verified": "NOT_VERIFIED",
            "root_cause": "UNKNOWN",
            "limitations": limitations,
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
