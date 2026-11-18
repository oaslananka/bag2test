"""Validate opt-in native ROS 2 MCAP captures, not physical robot incidents.

These files come from a real rclpy node subscribed to an actual upstream
BNO055 ROS 2 publisher connected to a software-emulated serial sensor.
No field recording or hardware fault is claimed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import tempfile
from pathlib import Path

from mcap.reader import make_reader

from bag2test.analysis import inspect
from bag2test.scenario import write_results


def verify_capture(case: str, path: Path) -> dict:
    if path.suffix != ".mcap" or not path.is_file():
        raise ValueError(f"{case}: missing real rosbag2 MCAP: {path}")
    if not (path.parent / "metadata.yaml").is_file():
        raise ValueError(f"{case}: rosbag2 metadata.yaml is missing")
    count = {"/bno055/imu": 0, "/bag2test/heartbeat": 0}
    with path.open("rb") as stream:
        reader = make_reader(stream)
        if reader.get_summary() is None:
            raise ValueError(f"{case}: missing MCAP summary")
        for schema, channel, message in reader.iter_messages():
            if channel.topic not in count:
                raise ValueError(f"{case}: unexpected topic {channel.topic}")
            if (
                schema is None
                or schema.encoding != "ros2msg"
                or channel.message_encoding != "cdr"
                or not message.data
            ):
                raise ValueError(f"{case}: not real ROS2 CDR-format message metadata")
            count[channel.topic] += 1
    if count["/bno055/imu"] < 15 or count["/bag2test/heartbeat"] < 20:
        raise AssertionError(
            f"{case}: insufficient actually recorded ROS messages: {count}"
        )

    observed = inspect(path, "/bno055/imu")
    with tempfile.TemporaryDirectory(prefix="bag2test-capture-check-") as tmp:
        names = {p.name for p in write_results(observed, Path(tmp) / "analysis")}

    if case == "healthy":
        assert observed.status == "NO_CANDIDATE", observed
        assert names == {"evidence.json"}, names
        assert observed.trailing_silence_s is None
    elif case == "bad":
        assert observed.status == "TRAILING_SILENCE", observed
        assert observed.largest_gap is None
        assert observed.trailing_silence_s is not None
        assert observed.trailing_silence_s >= 1.0
        assert names == {"evidence.json"}, names
    elif case == "fixed":
        assert observed.status == "CANDIDATE", observed
        assert observed.largest_gap is not None
        assert observed.largest_gap.duration_s >= 1.0
        assert names == {"evidence.json", "scenario.yaml"}, names
    else:
        raise ValueError(f"Unknown case: {case}")

    return {
        "case": case,
        "status": observed.status,
        "imu_messages": count["/bno055/imu"],
        "heartbeat_messages": count["/bag2test/heartbeat"],
        "trailing_silence_s": observed.trailing_silence_s,
        "largest_closed_gap_s": (
            observed.largest_gap.duration_s if observed.largest_gap else None
        ),
        "outputs": sorted(names),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "ros2_runtime": "ACTUAL",
        "serial_peripheral": "EMULATED",
        "field_incident_verified": False,
        "causal_reproduction_verified": False,
        "engineer_time_saved_verified": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "capture_root",
        type=Path,
        help="Directory with bad/bad_0.mcap, healthy/healthy_0.mcap, "
        "fixed/fixed_0.mcap and rosbag2 metadata.yaml in each subfolder",
    )
    args = parser.parse_args()
    results = [
        verify_capture(case, args.capture_root / case / f"{case}_0.mcap")
        for case in ("healthy", "bad", "fixed")
    ]
    print(json.dumps({"acceptance": "PASS", "cases": results}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
