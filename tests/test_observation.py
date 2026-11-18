from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml
from mcap.writer import Writer

from bag2test.analysis import ObservationError, inspect
from bag2test.scenario import candidate_scenario, write_results


def recording(
    tmp_path: Path,
    *,
    name: str = "sensor_msgs/msg/JointState",
    topic: str = "/joint_states",
    intervals_ms: tuple[int, ...] = (10, 10, 10, 10, 10, 60, 10, 10, 10, 10, 10),
) -> Path:
    file = tmp_path / "source.mcap"
    t = 1_000_000_000
    with file.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema(name=name, encoding="ros2msg", data=b"")
        channel = writer.register_channel(
            topic=topic, message_encoding="cdr", schema_id=schema
        )
        writer.add_message(
            channel_id=channel,
            log_time=t,
            publish_time=t,
            data=b"not-a-decoded-ros-payload",
        )
        for delta in intervals_ms:
            t += delta * 1_000_000
            writer.add_message(
                channel_id=channel,
                log_time=t,
                publish_time=t,
                data=b"not-a-decoded-ros-payload",
            )
        writer.finish()
    return file


def test_detects_known_gap_and_uses_real_upstream_schema(tmp_path: Path):
    observation = inspect(recording(tmp_path), "/joint_states")
    assert observation.status == "CANDIDATE"
    assert observation.count == 12
    assert observation.message_type == "sensor_msgs/msg/JointState"
    assert observation.median_period_s == pytest.approx(0.01)
    assert observation.largest_gap is not None
    assert observation.largest_gap.duration_s == pytest.approx(0.06)
    assert observation.largest_gap.ratio == pytest.approx(6.0)
    scenario = candidate_scenario(observation)
    assert scenario["injectors"][0]["type"] == "joint_state"
    assert scenario["injectors"][0]["topic"] == {
        "input_topic": "/joint_states_raw",
        "output_topic": "/joint_states",
        "qos_depth": 10,
    }
    assert scenario["faults"][0]["start"] == pytest.approx(0.05)
    assert scenario["faults"][0]["duration"] == pytest.approx(0.06)
    assert scenario["faults"][0]["config"] == {"drop_probability": 1.0}
    assert scenario["assertions"][0]["type"] == "fault_event"


def test_real_yaml_and_evidence_are_explicitly_unverified(tmp_path: Path):
    result = inspect(recording(tmp_path), "/joint_states")
    files = write_results(result, tmp_path / "out")
    assert {p.name for p in files} == {"evidence.json", "scenario.yaml"}
    yaml_data = yaml.safe_load((tmp_path / "out" / "scenario.yaml").read_text())
    assert yaml_data["faults"][0]["injector_id"] == yaml_data["injectors"][0]["id"]
    evidence = json.loads((tmp_path / "out" / "evidence.json").read_text())
    assert evidence["root_cause"] == "UNKNOWN"
    assert evidence["symptom_reproduced"] == "NOT_VERIFIED"
    assert evidence["regression_verified"] == "NOT_VERIFIED"
    assert evidence["hypothesis_status"] == "UNTESTED"


def test_healthy_recording_has_no_scenario(tmp_path: Path):
    data = recording(tmp_path, intervals_ms=(10,) * 11)
    observation = inspect(data, "/joint_states")
    assert observation.status == "NO_CANDIDATE"
    assert observation.largest_gap is None
    assert [p.name for p in write_results(observation, tmp_path / "out")] == [
        "evidence.json"
    ]
    assert not (tmp_path / "out" / "scenario.yaml").exists()


def test_sparse_recording_is_inconclusive(tmp_path: Path):
    data = recording(tmp_path, intervals_ms=(10,) * 3)
    observation = inspect(data, "/joint_states")
    assert observation.status == "INSUFFICIENT_DATA"
    assert observation.largest_gap is None


def test_topic_and_type_must_be_supported(tmp_path: Path):
    file = recording(tmp_path, name="sensor_msgs/msg/CompressedImage")
    with pytest.raises(ObservationError, match="Unsupported"):
        inspect(file, "/joint_states")
    with pytest.raises(ObservationError, match="No messages"):
        inspect(file, "/unknown")


def test_historical_ros2_names_are_canonicalized(tmp_path: Path):
    data = recording(tmp_path, name="sensor_msgs/JointState", topic="relative/joints")
    observation = inspect(data, "relative/joints")
    assert observation.message_type == "sensor_msgs/msg/JointState"
    assert (
        candidate_scenario(observation)["injectors"][0]["topic"]["output_topic"]
        == "/relative/joints"
    )


def test_rejects_invalid_configuration_and_output_collisions(tmp_path: Path):
    data = recording(tmp_path)
    with pytest.raises(ObservationError, match="min_gap_factor"):
        inspect(data, "/joint_states", min_gap_factor=1.0)
    with pytest.raises(ObservationError, match="end_sec"):
        inspect(data, "/joint_states", start_sec=1.0, end_sec=0.9)
    with pytest.raises(ObservationError, match="Input topic"):
        candidate_scenario(inspect(data, "/joint_states"), input_topic="/joint_states")
    existing = tmp_path / "out"
    existing.mkdir()
    with pytest.raises(ObservationError, match="already exists"):
        write_results(inspect(data, "/joint_states"), existing)


def test_window_excludes_gap(tmp_path: Path):
    data = recording(tmp_path)
    observation = inspect(data, "/joint_states", start_sec=0.11)
    assert observation.status == "INSUFFICIENT_DATA"


def test_corrupt_mcap_fails_without_writing_results(tmp_path: Path):
    data = tmp_path / "broken.mcap"
    data.write_bytes(b"not mcap")
    output = tmp_path / "out"
    cli = subprocess.run(
        [
            sys.executable,
            "-m",
            "bag2test.cli",
            str(data),
            "--topic",
            "/joint_states",
            "--out",
            str(output),
        ],
        check=False,
        text=True,
        capture_output=True,
    )
    assert cli.returncode == 2
    assert "Unable to read MCAP" in cli.stderr
    assert not output.exists()


def test_cli_output_and_non_overwrite(tmp_path: Path):
    data = recording(tmp_path)
    out = tmp_path / "out"
    command = [
        sys.executable,
        "-m",
        "bag2test.cli",
        str(data),
        "--topic",
        "/joint_states",
        "--out",
        str(out),
    ]
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "NOT VERIFIED" in result.stdout
    assert (out / "scenario.yaml").exists()
    second = subprocess.run(command, check=False, capture_output=True, text=True)
    assert second.returncode == 2
    assert "already exists" in second.stderr
