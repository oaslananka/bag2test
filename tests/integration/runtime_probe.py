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
from sensor_msgs.msg import JointState

from bag2test.analysis import inspect
from bag2test.scenario import write_results

HZ = 20
PUBLISH_S = 7.5


def make_recording(path: Path) -> None:
    """MCAP with genuinely serialized ROS 2 CDR messages and known 2s gap."""
    msg = JointState(name=["joint_a"], position=[1.0], velocity=[0.0])
    with path.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema(
            name="sensor_msgs/msg/JointState",
            encoding="ros2msg",
            data=b"std_msgs/Header header\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n",
        )
        channel = writer.register_channel(
            topic="/joint_states", message_encoding="cdr", schema_id=schema
        )
        for step in range(150):
            seconds = step / HZ
            if 2.5 < seconds < 4.5:
                continue
            ns = 1_000_000_000 + round(seconds * 1_000_000_000)
            writer.add_message(
                channel_id=channel,
                log_time=ns,
                publish_time=ns,
                data=serialize_message(msg),
            )
        writer.finish()


def run_once(file: Path, domain: int) -> dict:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain)
    logfile = file.with_name(f"runner-{domain}.log")
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
        publisher = node.create_publisher(JointState, "/joint_states_raw", 10)
        arrivals = []
        events = []
        node.create_subscription(
            JointState,
            "/joint_states",
            lambda _: arrivals.append(time.monotonic()),
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
            if publisher.get_subscription_count() and node.count_publishers(
                "/joint_states"
            ):
                break
            if proc.poll() is not None:
                raise RuntimeError(f"Injector crashed; see {logfile}")
        else:
            raise RuntimeError(f"Injector did not become ready; see {logfile}")

        begin = time.monotonic()
        next_send = begin
        sent = 0
        while time.monotonic() - begin < PUBLISH_S:
            now = time.monotonic()
            if now >= next_send:
                publisher.publish(JointState(name=["joint_a"], position=[float(sent)]))
                sent += 1
                next_send += 1.0 / HZ
            rclpy.spin_once(node, timeout_sec=0.005)
        gaps = [b - a for a, b in pairwise(arrivals)]
        return {
            "domain": domain,
            "sent": sent,
            "received": len(arrivals),
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


def main(repetitions: int) -> int:
    if not 1 <= repetitions <= 10:
        raise ValueError("repetitions must be between 1 and 10")
    with tempfile.TemporaryDirectory(prefix="bag2test-runtime-") as folder:
        base = Path(folder)
        recording = base / "known_synthetic_gap.mcap"
        make_recording(recording)
        observation = inspect(recording, "/joint_states")
        assert observation.status == "CANDIDATE"
        write_results(observation, base / "candidate")
        candidate = base / "candidate" / "scenario.yaml"
        control = yaml.safe_load(candidate.read_text())
        control["faults"][0]["config"]["drop_probability"] = 0.0
        healthy = base / "healthy.yaml"
        healthy.write_text(yaml.safe_dump(control, sort_keys=False))
        passes = []
        for iteration in range(repetitions):
            faulty = run_once(candidate, 110 + 2 * iteration)
            normal = run_once(healthy, 111 + 2 * iteration)
            ok = (
                faulty["activated"]
                and faulty["deactivated"]
                and faulty["max_gap_s"] >= 1.2
                and faulty["sent"] >= 100
                and normal["activated"]
                and normal["max_gap_s"] < 0.4
                and normal["sent"] >= 100
            )
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
            "observed_source_gap_s": observation.largest_gap.duration_s,
            "passed": sum(passes),
            "trials": len(passes),
            "real_incident_reproduced": "NOT_VERIFIED",
            "faulty_vs_fixed_regression": "NOT_VERIFIED",
            "result": "PASS" if all(passes) else "FAIL",
        }
        print(json.dumps(result, indent=2), flush=True)
        return 0 if all(passes) else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--repetitions", type=int, default=3)
    raise SystemExit(main(parser.parse_args().repetitions))
