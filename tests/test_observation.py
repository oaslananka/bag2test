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
    publish_lag_ms: int = 0,
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
            publish_time=t - publish_lag_ms * 1_000_000,
            data=b"not-a-decoded-ros-payload",
        )
        for delta in intervals_ms:
            t += delta * 1_000_000
            writer.add_message(
                channel_id=channel,
                log_time=t,
                publish_time=t - publish_lag_ms * 1_000_000,
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


def test_reports_timestamp_provenance_and_a_post_gap_burst(tmp_path: Path):
    # The next 2 ms interval can occur when timestamps catch up after a pause.
    file = recording(
        tmp_path,
        intervals_ms=(10, 10, 10, 10, 10, 27, 2, 10, 10, 10, 10),
    )
    result = inspect(file, "/joint_states")
    assert result.status == "CANDIDATE"
    assert result.publish_times_equal_log_times is True
    assert result.largest_gap is not None
    assert result.largest_gap.duration_s == pytest.approx(0.027)
    assert result.largest_gap.following_interval_s == pytest.approx(0.002)
    assert result.largest_gap.following_interval_ratio == pytest.approx(0.2)

    out = tmp_path / "result"
    write_results(result, out)
    evidence = json.loads((out / "evidence.json").read_text())
    assert evidence["publish_times_equal_log_times"] is True
    assert evidence["largest_gap"]["following_interval_s"] == pytest.approx(0.002)
    assert any("catch-up" in item for item in evidence["limitations"])
    assert evidence["root_cause"] == "UNKNOWN"
    assert evidence["symptom_reproduced"] == "NOT_VERIFIED"

    cmd = [
        sys.executable,
        "-m",
        "bag2test.cli",
        str(file),
        "--topic",
        "/joint_states",
        "--out",
        str(tmp_path / "cli"),
    ]
    done = subprocess.run(cmd, check=False, capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    assert "publish_time equals log_time" in done.stdout
    assert "Catch-up/batching is possible" in done.stdout
    assert "independent healthy ROS input stream" in done.stdout


def test_different_mcap_timestamps_dont_trigger_false_clock_warning(tmp_path: Path):
    file = recording(tmp_path, publish_lag_ms=2)
    result = inspect(file, "/joint_states")
    assert result.publish_times_equal_log_times is False
    out = tmp_path / "out"
    write_results(result, out)
    evidence = json.loads((out / "evidence.json").read_text())
    assert evidence["publish_times_equal_log_times"] is False
    assert not any(
        "no independent publish-vs-record" in l for l in evidence["limitations"]
    )


def test_final_gap_has_no_following_interval(tmp_path: Path):
    file = recording(tmp_path, intervals_ms=(10,) * 10 + (60,))
    result = inspect(file, "/joint_states")
    assert result.status == "CANDIDATE"
    assert result.largest_gap is not None
    assert result.largest_gap.following_interval_s is None
    assert result.largest_gap.following_interval_ratio is None


def test_healthy_recording_still_reports_timestamp_provenance(tmp_path: Path):
    file = recording(tmp_path, intervals_ms=(10,) * 10, publish_lag_ms=1)
    result = inspect(file, "/joint_states")
    assert result.status == "NO_CANDIDATE"
    assert result.publish_times_equal_log_times is False


def test_manual_fault_scenario_matches_generated_synthetic_reference(tmp_path: Path):
    """Semantic parity is measurable; human authoring-time savings are not."""
    file = recording(
        tmp_path,
        intervals_ms=(50,) * 50 + (2000,) + (50,) * 55,
    )
    observed = inspect(file, "/joint_states")
    assert observed.status == "CANDIDATE"
    generated = candidate_scenario(observed)
    manual = yaml.safe_load(
        (Path(__file__).parent / "integration" / "manual_joint_state.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert generated == manual


def test_imu_observation_yields_candidate_only_not_sensor_recovery(tmp_path: Path):
    """IMU topic dropout is not the same as a serial driver's failed read."""
    data = recording(
        tmp_path,
        name="sensor_msgs/msg/Imu",
        topic="/imu/data",
        intervals_ms=(10,) * 5 + (60,) + (10,) * 5,
    )
    observed = inspect(data, "/imu/data")
    assert observed.status == "CANDIDATE"
    assert observed.message_type == "sensor_msgs/msg/Imu"
    scenario = candidate_scenario(observed)
    assert scenario["injectors"][0]["type"] == "imu"
    assert scenario["injectors"][0]["topic"]["input_topic"] == "/imu/data_raw"
    assert scenario["injectors"][0]["topic"]["output_topic"] == "/imu/data"
    assert scenario["faults"][0]["config"]["drop_probability"] == 1.0
    assert scenario["assertions"][0]["type"] == "fault_event"
    assert not any(
        assertion["type"] == "node_restart" for assertion in scenario["assertions"]
    )
    out = tmp_path / "result"
    write_results(observed, out)
    evidence = json.loads((out / "evidence.json").read_text())
    assert evidence["root_cause"] == "UNKNOWN"
    assert evidence["symptom_reproduced"] == "NOT_VERIFIED"
    assert evidence["regression_verified"] == "NOT_VERIFIED"


def recording_with_trailing_silence(
    tmp_path: Path,
    *,
    topic_count: int = 16,
    internal_gap_ms: int = 0,
    independent_end_ms: int = 2000,
) -> Path:
    """Synthetic multi-topic MCAP: another real channel survives an IMU outage."""
    file = tmp_path / "terminal.mcap"
    start = 1_000_000_000
    with file.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        imu_schema = writer.register_schema(
            name="sensor_msgs/msg/Imu", encoding="ros2msg", data=b""
        )
        monitor_schema = writer.register_schema(
            name="std_msgs/msg/String", encoding="ros2msg", data=b""
        )
        imu = writer.register_channel(
            topic="/bno055/imu", message_encoding="cdr", schema_id=imu_schema
        )
        monitor = writer.register_channel(
            topic="/independent/heartbeat",
            message_encoding="cdr",
            schema_id=monitor_schema,
        )
        for idx in range(topic_count):
            t = start + idx * 10_000_000
            if internal_gap_ms and idx >= 8:
                t += internal_gap_ms * 1_000_000
            writer.add_message(channel_id=imu, log_time=t, publish_time=t, data=b"imu")
        for offset_ms in (5, independent_end_ms):
            t = start + offset_ms * 1_000_000
            writer.add_message(
                channel_id=monitor, log_time=t, publish_time=t, data=b"heartbeat"
            )
        writer.finish()
    return file


def test_terminal_sensor_silence_is_observable_without_a_closed_gap(tmp_path: Path):
    data = recording_with_trailing_silence(tmp_path)
    obs = inspect(data, "/bno055/imu")
    assert obs.count == 16
    assert obs.status == "TRAILING_SILENCE"
    assert obs.largest_gap is None
    assert obs.median_period_s == pytest.approx(0.01)
    assert obs.trailing_silence_s == pytest.approx(1.85)
    assert obs.trailing_silence_ratio == pytest.approx(185.0)
    assert obs.recording_end_ns == 3_000_000_000
    files = write_results(obs, tmp_path / "trailing")
    assert [f.name for f in files] == ["evidence.json"]
    evidence = json.loads(files[0].read_text())
    assert evidence["status"] == "TRAILING_SILENCE"
    assert evidence["trailing_silence_s"] == pytest.approx(1.85)
    assert evidence["root_cause"] == "UNKNOWN"
    assert evidence["hypothesis"] is None
    assert evidence["regression_verified"] == "NOT_VERIFIED"
    assert any("independent" in sentence for sentence in evidence["limitations"])


def test_single_topic_cannot_establish_a_terminal_outage(tmp_path: Path):
    obs = inspect(
        recording(
            tmp_path, name="sensor_msgs/msg/Imu", topic="/imu", intervals_ms=(10,) * 15
        ),
        "/imu",
    )
    assert obs.status == "NO_CANDIDATE"
    assert obs.trailing_silence_s is None


def test_an_earlier_independent_topic_does_not_establish_silence(tmp_path: Path):
    obs = inspect(
        recording_with_trailing_silence(tmp_path, independent_end_ms=20),
        "/bno055/imu",
    )
    assert obs.status == "NO_CANDIDATE"
    assert obs.trailing_silence_s is None


def test_time_window_disables_unproven_terminal_silence(tmp_path: Path):
    obs = inspect(
        recording_with_trailing_silence(tmp_path),
        "/bno055/imu",
        end_sec=0.14,
    )
    assert obs.status == "NO_CANDIDATE"
    assert obs.trailing_silence_s is None


def test_sparse_data_does_not_assert_terminal_silence(tmp_path: Path):
    obs = inspect(
        recording_with_trailing_silence(tmp_path, topic_count=5),
        "/bno055/imu",
    )
    assert obs.status == "INSUFFICIENT_DATA"
    assert obs.trailing_silence_s is None


def test_internal_gap_keeps_candidate_and_reports_terminal_context(tmp_path: Path):
    obs = inspect(
        recording_with_trailing_silence(tmp_path, internal_gap_ms=100),
        "/bno055/imu",
    )
    assert obs.status == "CANDIDATE"
    assert obs.largest_gap is not None
    assert obs.largest_gap.duration_s == pytest.approx(0.11)
    assert obs.trailing_silence_s == pytest.approx(1.75)
    assert {file.name for file in write_results(obs, tmp_path / "out")} == {
        "evidence.json",
        "scenario.yaml",
    }


def test_cli_reports_terminal_silence_without_inventing_a_recovery(tmp_path: Path):
    data = recording_with_trailing_silence(tmp_path)
    output = tmp_path / "analysis"
    done = subprocess.run(
        [
            sys.executable,
            "-m",
            "bag2test.cli",
            str(data),
            "--topic",
            "/bno055/imu",
            "--out",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert done.returncode == 0, done.stderr
    assert "TRAILING_SILENCE" in done.stdout
    assert "1.850000s" in done.stdout
    assert "No bounded YAML" in done.stdout
    assert (output / "evidence.json").exists()
    assert not (output / "scenario.yaml").exists()


def test_terminal_silence_honors_requested_relative_threshold(tmp_path: Path):
    data = recording_with_trailing_silence(tmp_path, independent_end_ms=190)
    ordinary = inspect(data, "/bno055/imu")
    cautious = inspect(data, "/bno055/imu", min_gap_factor=5.0)
    assert ordinary.status == "TRAILING_SILENCE"
    assert ordinary.trailing_silence_ratio == pytest.approx(4.0)
    assert cautious.status == "NO_CANDIDATE"
    assert cautious.trailing_silence_s is None
