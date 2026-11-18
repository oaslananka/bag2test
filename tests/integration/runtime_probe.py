"""Real ROS 2 injection acceptance for synthetic JointState dropout (no robot)."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import tempfile
import time
from itertools import pairwise
from pathlib import Path

import rclpy
import yaml
from mcap.writer import Writer
from rclpy.node import Node
from rclpy.serialization import serialize_message
from ros2_fault_injection.msg import FaultEvent
from sensor_msgs.msg import Imu, JointState

from bag2test.analysis import inspect
from bag2test.scenario import write_results

DEFAULT_HZ = 20
PUBLISH_S = 7.5
SHORT_GAP_START_S = 2.5


def make_recording(
    path: Path, *, gap_ms: int, hz: int, message_type: str = "joint_state"
) -> None:
    """MCAP with serialized JointState messages and a measured timing gap.

    Long gaps omit messages; short gaps adjust just one timestamp interval,
    keeping the nominal period on either side. This does not simulate a
    physical root cause or identify which packet was lost by the recorder.
    """
    if message_type == "imu":
        msg = Imu()
        msg.header.frame_id = "1"
        schema_name = "sensor_msgs/msg/Imu"
        schema_text = (
            b"std_msgs/Header header\n"
            b"geometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\n"
            b"geometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\n"
            b"geometry_msgs/Vector3 linear_acceleration\n"
            b"float64[9] linear_acceleration_covariance\n"
        )
        topic = "/imu/data"
    else:
        msg = JointState(name=["joint_a"], position=[1.0], velocity=[0.0])
        schema_name = "sensor_msgs/msg/JointState"
        schema_text = (
            b"std_msgs/Header header\nstring[] name\nfloat64[] position\n"
            b"float64[] velocity\nfloat64[] effort\n"
        )
        topic = "/joint_states"
    with path.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema(
            name=schema_name,
            encoding="ros2msg",
            data=schema_text,
        )
        channel = writer.register_channel(
            topic=topic, message_encoding="cdr", schema_id=schema
        )
        period_s = 1.0 / hz
        for step in range(round(PUBLISH_S * hz)):
            seconds = step * period_s
            if gap_ms >= 1000:
                if SHORT_GAP_START_S < seconds < SHORT_GAP_START_S + gap_ms / 1000.0:
                    continue
            elif seconds > SHORT_GAP_START_S:
                # One interval has exactly the chosen gap, rather than
                # rounding an omitted number of full messages.
                seconds += gap_ms / 1000.0 - period_s
            ns = 1_000_000_000 + round(seconds * 1_000_000_000)
            writer.add_message(
                channel_id=channel,
                log_time=ns,
                publish_time=ns,
                data=serialize_message(msg),
            )
        writer.finish()


def run_once(
    file: Path, domain: int, *, hz: int, message_type: str = "joint_state"
) -> dict:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain)
    # A packaged hand-written YAML lives in /opt (read-only to the
    # non-root container user); write runner logs into a secure temp file.
    logfile = Path(tempfile.mkdtemp(prefix=f"bag2test-runner-{domain}-")) / "runner.log"
    out = logfile.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "ros2_fault_injection",
            "fault_injector.launch.py",
            f"scenario_file:={file}",
        ],
        stdout=out,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )
    before = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    node = None
    try:
        rclpy.init()
        node = Node(f"bag2test_probe_{domain}")
        if message_type == "imu":
            msg_class = Imu
            upstream_topic = "/imu/data_raw"
            downstream_topic = "/imu/data"

            def new_message(sequence: int) -> Imu:
                msg = Imu()
                msg.header.frame_id = str(sequence)
                return msg

            def read_sequence(msg: Imu) -> int:
                return int(msg.header.frame_id)

        else:
            msg_class = JointState
            upstream_topic = "/joint_states_raw"
            downstream_topic = "/joint_states"

            def new_message(sequence: int) -> JointState:
                return JointState(name=["joint_a"], position=[float(sequence)])

            def read_sequence(msg: JointState) -> int:
                return int(msg.position[0])

        publisher = node.create_publisher(msg_class, upstream_topic, 10)
        arrivals = []
        received_sequences = []
        warmup_received = []
        events = []

        def on_message(msg: JointState | Imu) -> None:
            sequence = read_sequence(msg)
            if sequence < 0:
                warmup_received.append(time.monotonic())
                return
            arrivals.append(time.monotonic())
            received_sequences.append(sequence)

        _subscriber = node.create_subscription(
            msg_class,
            downstream_topic,
            on_message,
            10,
        )
        node.create_subscription(
            FaultEvent,
            "/fault_injection/events",
            lambda ev: events.append((ev.fault_id, ev.state)),
            10,
        )
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if publisher.get_subscription_count() and _subscriber.get_publisher_count():
                break
            if proc.poll() is not None:
                raise RuntimeError(f"Injector crashed; see {logfile}")
        else:
            raise RuntimeError(f"Injector did not become ready; see {logfile}")

        # DDS graph discovery is asynchronous. Confirm the full relay by
        # receiving a handful of warmup messages before counting sequences;
        # otherwise lost startup samples resemble injected dropouts.
        warmup_limit = time.monotonic() + 2.0
        while len(warmup_received) < 5 and time.monotonic() < warmup_limit:
            publisher.publish(new_message(-1))
            rclpy.spin_once(node, timeout_sec=0.02)
        if len(warmup_received) < 5:
            raise RuntimeError(f"Cannot establish a healthy ROS relay; see {logfile}")

        begin = time.monotonic()
        next_send = begin
        sent = 0
        while time.monotonic() - begin < PUBLISH_S:
            now = time.monotonic()
            if now >= next_send:
                publisher.publish(new_message(sent))
                sent += 1
                next_send += 1.0 / hz
            rclpy.spin_once(node, timeout_sec=0.002)
        # Let the reliable publisher/subscriber pair flush in-flight messages
        # before attributing gaps to the fault injector.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.01)
        gaps = [b - a for a, b in pairwise(arrivals)]
        delivered = set(received_sequences)
        missing = [seq for seq in range(sent) if seq not in delivered]
        return {
            "domain": domain,
            "sent": sent,
            "received": len(arrivals),
            "missing_sequence_count": len(missing),
            "missing_sequence_sample": missing[:12],
            "max_gap_s": round(max(gaps, default=0.0), 3),
            "activated": ("candidate_gap", "active") in events,
            "deactivated": ("candidate_gap", "inactive") in events,
            "log": str(logfile),
        }
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if before is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = before
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=2)
        out.close()


def main(
    repetitions: int,
    *,
    gap_ms: int,
    hz: int,
    compare_manual: bool = False,
    message_type: str = "joint_state",
) -> int:
    if not 1 <= repetitions <= 10:
        raise ValueError("repetitions must be between 1 and 10")
    if not 20 <= hz <= 200:
        raise ValueError("publisher rate must be between 20 and 200 Hz")
    if not 20 <= gap_ms <= 3000:
        raise ValueError("gap must be between 20 and 3000 ms")
    if gap_ms / 1000.0 <= 1.0 / hz:
        raise ValueError("gap must be greater than the nominal message period")
    if message_type not in {"joint_state", "imu"}:
        raise ValueError("message_type must be joint_state or imu")
    if compare_manual and (gap_ms != 2000 or hz != 20 or message_type != "joint_state"):
        raise ValueError("Manual baseline supports only 2000 ms, 20 Hz JointState")
    with tempfile.TemporaryDirectory(prefix="bag2test-runtime-") as folder:
        base = Path(folder)
        recording = base / "known_synthetic_gap.mcap"
        make_recording(recording, gap_ms=gap_ms, hz=hz, message_type=message_type)
        topic = "/imu/data" if message_type == "imu" else "/joint_states"
        observation = inspect(recording, topic)
        assert observation.status == "CANDIDATE"
        write_results(observation, base / "candidate")
        candidate = base / "candidate" / "scenario.yaml"
        control = yaml.safe_load(candidate.read_text())
        control["faults"][0]["config"]["drop_probability"] = 0.0
        healthy = base / "healthy.yaml"
        healthy.write_text(yaml.safe_dump(control, sort_keys=False))
        passes = []
        for iteration in range(repetitions):
            faulty = run_once(
                candidate, 110 + 2 * iteration, hz=hz, message_type=message_type
            )
            normal = run_once(
                healthy, 111 + 2 * iteration, hz=hz, message_type=message_type
            )
            common = (
                faulty["activated"]
                and faulty["deactivated"]
                and faulty["sent"] >= hz * 6
                and normal["activated"]
                and normal["sent"] >= hz * 6
                and normal["missing_sequence_count"] == 0
            )
            if gap_ms >= 1000:
                ok = (
                    common
                    and faulty["max_gap_s"] >= 1.2
                    and faulty["missing_sequence_count"] > 0
                    and normal["max_gap_s"] < 0.4
                )
            else:
                # A fault lifecycle event is not proof that a 27ms window
                # coincided with any published JointState. Count the missing
                # sequence numbers independently instead.
                ok = common and faulty["missing_sequence_count"] > 0
            passes.append(ok)
            print(
                json.dumps(
                    {
                        "trial": iteration + 1,
                        "fault": faulty,
                        "control": normal,
                        "pass": ok,
                    }
                ),
                flush=True,
            )
        result = {
            "kind": "controlled_synthetic_ros2_injection",
            "mode": "short" if gap_ms < 1000 else "long",
            "publisher_hz": hz,
            "message_type": message_type,
            "requested_gap_ms": gap_ms,
            "observed_source_gap_s": observation.largest_gap.duration_s,
            "passed": sum(passes),
            "trials": len(passes),
            "real_incident_reproduced": "NOT_VERIFIED",
            "faulty_vs_fixed_regression": "NOT_VERIFIED",
            "result": "PASS" if all(passes) else "FAIL",
        }
        if compare_manual:
            # Compare against an independently authored static upstream YAML
            # baseline rather than assuming automated output is valuable.
            manual = Path(__file__).with_name("manual_joint_state.yaml")
            authored = yaml.safe_load(manual.read_text(encoding="utf-8"))
            generated = yaml.safe_load(candidate.read_text(encoding="utf-8"))
            if authored != generated:
                raise AssertionError(
                    "Generated scenario diverges from independently authored YAML"
                )
            manual_control = yaml.safe_load(manual.read_text(encoding="utf-8"))
            manual_control["faults"][0]["config"]["drop_probability"] = 0.0
            manual_healthy = base / "manual_control.yaml"
            manual_healthy.write_text(
                yaml.safe_dump(manual_control, sort_keys=False),
                encoding="utf-8",
            )
            baseline_passes = []
            for iteration in range(repetitions):
                faulty_baseline = run_once(manual, 160 + 2 * iteration, hz=hz)
                healthy_baseline = run_once(manual_healthy, 161 + 2 * iteration, hz=hz)
                accepted = (
                    faulty_baseline["activated"]
                    and faulty_baseline["deactivated"]
                    and faulty_baseline["missing_sequence_count"] > 0
                    and faulty_baseline["max_gap_s"] >= 1.2
                    and healthy_baseline["activated"]
                    and healthy_baseline["missing_sequence_count"] == 0
                    and healthy_baseline["max_gap_s"] < 0.4
                    and faulty_baseline["sent"] >= hz * 6
                    and healthy_baseline["sent"] >= hz * 6
                )
                baseline_passes.append(accepted)
                print(
                    json.dumps(
                        {
                            "manual_trial": iteration + 1,
                            "fault": faulty_baseline,
                            "control": healthy_baseline,
                            "pass": accepted,
                        }
                    ),
                    flush=True,
                )
            result["manual_equivalent_yaml"] = True
            result["manual_baseline_passed"] = sum(baseline_passes)
            result["manual_baseline_trials"] = repetitions
            result["human_time_savings"] = "NOT_VERIFIED"
            result["result"] = (
                "PASS" if all(passes) and all(baseline_passes) else "FAIL"
            )
        print(json.dumps(result, indent=2), flush=True)
        return 0 if result["result"] == "PASS" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repetitions",
        type=int,
        default=int(os.environ.get("BAG2TEST_REPETITIONS", "3")),
    )
    parser.add_argument(
        "--gap-ms", type=int, default=int(os.environ.get("BAG2TEST_GAP_MS", "2000"))
    )
    parser.add_argument(
        "--hz", type=int, default=int(os.environ.get("BAG2TEST_HZ", str(DEFAULT_HZ)))
    )
    parser.add_argument(
        "--message-type",
        choices=("joint_state", "imu"),
        default=os.environ.get("BAG2TEST_MESSAGE_TYPE", "joint_state"),
    )
    parser.add_argument(
        "--compare-manual",
        action="store_true",
        default=os.environ.get("BAG2TEST_COMPARE_MANUAL") == "1",
    )
    args = parser.parse_args()
    raise SystemExit(
        main(
            args.repetitions,
            gap_ms=args.gap_ms,
            hz=args.hz,
            compare_manual=args.compare_manual,
            message_type=args.message_type,
        )
    )
