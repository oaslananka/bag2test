"""CDR Header timestamp is explicitly selectable when recorder time is batched."""

from __future__ import annotations

import json
import struct
import subprocess
import sys
from pathlib import Path

import pytest
from mcap.writer import Writer

from bag2test.analysis import ObservationError, inspect
from bag2test.scenario import candidate_scenario, write_results


def laser_scan_cdr(ns: int, *, big_endian: bool = False) -> bytes:
    """Well-formed empty LaserScan with Header and seven metadata floats."""
    sec, nanos = divmod(ns, 1_000_000_000)
    endian = ">" if big_endian else "<"
    payload = bytearray(b"\x00\x00\x00\x00" if big_endian else b"\x00\x01\x00\x00")
    payload.extend(struct.pack(f"{endian}iI", sec, nanos))
    payload.extend(struct.pack(f"{endian}I", 1))  # Empty frame_id + NUL
    payload.extend(b"\x00")
    payload.extend(b"\x00" * (-len(payload) % 4))
    payload.extend(struct.pack(f"{endian}7f", 0.0, 6.28, 0.1, 0.0, 0.1, 0.5, 20.0))
    payload.extend(struct.pack(f"{endian}II", 0, 0))  # Empty ranges, intensities.
    return bytes(payload)


def bag(
    path: Path,
    *,
    sender_deltas_ns: list[int],
    recorder_deltas_ns: list[int],
    big_endian: bool = False,
    invalid_stamp: bool = False,
) -> Path:
    assert len(sender_deltas_ns) == len(recorder_deltas_ns)
    with path.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema(
            name="sensor_msgs/msg/LaserScan", encoding="ros2msg", data=b""
        )
        channel = writer.register_channel(
            topic="/scan", message_encoding="cdr", schema_id=schema
        )
        timestamp = 1_700_000_000_000_000_000
        sender_stamp = 1_600_000_000_000_000_000
        for idx, (dt_send, dt_record) in enumerate(
            zip(sender_deltas_ns, recorder_deltas_ns, strict=True)
        ):
            timestamp += dt_record
            sender_stamp += dt_send
            payload = laser_scan_cdr(
                0 if invalid_stamp and idx == 4 else sender_stamp,
                big_endian=big_endian,
            )
            writer.add_message(
                channel_id=channel,
                log_time=timestamp,
                publish_time=timestamp,
                data=payload,
            )
        writer.finish()
    return path


def test_header_time_rejects_recorder_batching_false_positive(tmp_path: Path):
    n = 40
    source = bag(
        tmp_path / "mixed.mcap",
        sender_deltas_ns=[100_000_000] * n,
        recorder_deltas_ns=[1_000_000] * 20 + [20_000_000_000] + [1_000_000] * 19,
    )
    assert inspect(source, "/scan", min_gap_factor=4.0).status == "CANDIDATE"
    selected = inspect(source, "/scan", min_gap_factor=4.0, time_basis="header")
    assert selected.status == "NO_CANDIDATE"
    assert selected.time_basis == "header"
    assert selected.median_period_s == pytest.approx(0.1)
    assert {path.name for path in write_results(selected, tmp_path / "out")} == {
        "evidence.json"
    }
    evidence = json.loads((tmp_path / "out" / "evidence.json").read_text())
    assert evidence["time_basis"] == "header"
    assert evidence["hypothesis"] is None


def test_header_time_gap_has_correct_start_and_duration(tmp_path: Path):
    n = 30
    source = bag(
        tmp_path / "gap.mcap",
        sender_deltas_ns=[100_000_000] * 16 + [4_000_000_000] + [100_000_000] * 13,
        recorder_deltas_ns=[1_000_000] * n,
    )
    selected = inspect(source, "/scan", min_gap_factor=4.0, time_basis="header")
    assert selected.status == "CANDIDATE"
    assert selected.largest_gap is not None
    assert selected.largest_gap.duration_s == pytest.approx(4.0)
    assert selected.median_period_s == pytest.approx(0.1)
    scenario = candidate_scenario(selected)
    # Anchor the upstream header's elapsed time to its first observed log arrival.
    assert scenario["faults"][0]["start"] == pytest.approx(1.5)
    assert scenario["faults"][0]["duration"] == pytest.approx(4.0)


def test_header_time_supports_big_endian_and_rejects_unreliable_data(tmp_path: Path):
    good = bag(
        tmp_path / "good.mcap",
        sender_deltas_ns=[100_000_000] * 15,
        recorder_deltas_ns=[100_000_000] * 15,
        big_endian=True,
    )
    assert inspect(good, "/scan", time_basis="header").status == "NO_CANDIDATE"
    bad = bag(
        tmp_path / "bad.mcap",
        sender_deltas_ns=[100_000_000] * 15,
        recorder_deltas_ns=[100_000_000] * 15,
        invalid_stamp=True,
    )
    with pytest.raises(ObservationError, match="header"):
        inspect(bad, "/scan", time_basis="header")
    backwards = bag(
        tmp_path / "backwards.mcap",
        sender_deltas_ns=[100_000_000] * 9 + [-200_000_000] + [100_000_000] * 5,
        recorder_deltas_ns=[100_000_000] * 15,
    )
    with pytest.raises(ObservationError, match="non-monotonic"):
        inspect(backwards, "/scan", time_basis="header")


def test_header_time_cli_explains_sender_clock_and_candidate(tmp_path: Path):
    source = bag(
        tmp_path / "cli.mcap",
        sender_deltas_ns=[100_000_000] * 16 + [4_000_000_000] + [100_000_000] * 13,
        recorder_deltas_ns=[1_000_000] * 30,
    )
    output = tmp_path / "result"
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "bag2test.cli",
            str(source),
            "--topic",
            "/scan",
            "--time-basis",
            "header",
            "--min-gap-factor",
            "4.0",
            "--out",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "time basis: header" in result.stdout
    assert "clock alignment is NOT VERIFIED" in result.stdout
    evidence = json.loads((output / "evidence.json").read_text())
    assert evidence["time_basis"] == "header"
    assert evidence["observation"].startswith("ROS 2 CDR Header.stamp")
    assert any("sender-provided" in msg for msg in evidence["limitations"])
    assert (output / "scenario.yaml").exists()


def test_header_time_rejects_invalid_mode_and_unstamped_type(tmp_path: Path):
    source = bag(
        tmp_path / "mode.mcap",
        sender_deltas_ns=[100_000_000] * 15,
        recorder_deltas_ns=[100_000_000] * 15,
    )
    with pytest.raises(ObservationError, match="time_basis"):
        inspect(source, "/scan", time_basis="unknown")

    # No Header can exist in geometry_msgs/Twist. Do not decode its data
    # merely because its CDR bytes happen to resemble a stamp.
    twist = tmp_path / "twist.mcap"
    with twist.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema(
            name="geometry_msgs/msg/Twist", encoding="ros2msg", data=b""
        )
        channel = writer.register_channel(
            topic="/cmd_vel", message_encoding="cdr", schema_id=schema
        )
        for idx in range(12):
            writer.add_message(
                channel_id=channel,
                log_time=1_000_000_000 + idx * 100_000_000,
                publish_time=1_000_000_000 + idx * 100_000_000,
                data=laser_scan_cdr(1_600_000_000_000_000_000),
            )
        writer.finish()
    with pytest.raises(ObservationError, match="no std_msgs/Header"):
        inspect(twist, "/cmd_vel", time_basis="header")
