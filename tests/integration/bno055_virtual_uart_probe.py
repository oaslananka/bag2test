"""Offline ROS 2 + real BNO055 UART driver with an emulated PTY sensor.

The upstream source is not modified or imported via fakes: it uses a real
rclpy node, ROS timers, ROS Imu publishers, pyserial UART and a restartable
process. Only physical BNO055 register responses are simulated.
"""

from __future__ import annotations

import errno
import json
import os
import pty
import select
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

MAX_FAILURES_AT_10HZ = 30
SOURCE_DIRS = {
    "bad": "/opt/bno055-bad",
    "fixed": "/opt/bno055-fixed",
}


class Bno055VirtualUart:
    """Respond to BNO055 UART reads/writes over an actual pseudo-terminal."""

    def __init__(self) -> None:
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.data_reads = 0
        self.silenced_reads = 0
        self.responded_data_reads = 0
        # Start healthy until the independent ROS subscriber confirms
        # actual end-to-end message delivery; avoid DDS discovery races.
        self.silent_after: int | None = None
        self.worker = threading.Thread(target=self._serve, daemon=True)
        self.worker.start()

    def restore(self) -> None:
        with self.lock:
            self.silent_after = None

    def start_failure(self) -> None:
        # Trigger the source-side transport failure only after independent
        # ROS messages prove that the driver is healthy and connected.
        with self.lock:
            assert self.responded_data_reads >= 4
            self.silent_after = self.responded_data_reads

    def stats(self) -> dict[str, int]:
        with self.lock:
            return {
                "data_reads": self.data_reads,
                "silenced_reads": self.silenced_reads,
                "responded_data_reads": self.responded_data_reads,
            }

    def _handle_read(self, reg: int, length: int) -> None:
        with self.lock:
            if reg == 0x08 and length == 45:
                self.data_reads += 1
                if (
                    self.silent_after is not None
                    and self.responded_data_reads >= self.silent_after
                ):
                    self.silenced_reads += 1
                    return
                self.responded_data_reads += 1

        data = bytearray(length)
        if reg == 0x00 and length >= 1:
            data[0] = 0xA0  # Official BNO055 chip identifier.
        if reg == 0x08 and length == 45:
            data[24] = 1  # Valid nonzero quaternion W prevents normalization / 0.
        os.write(self.master, bytes((0xBB, length)) + data)

    def _serve(self) -> None:
        pending = bytearray()
        while not self.stop_event.is_set():
            ready, _, _ = select.select([self.master], [], [], 0.05)
            if not ready:
                continue
            try:
                chunk = os.read(self.master, 512)
            except OSError as exc:
                if exc.errno in {errno.EIO, errno.EBADF}:
                    break
                raise
            if not chunk:
                continue
            pending.extend(chunk)
            while len(pending) >= 4:
                # BNO055 serial prefix; discard spurious bytes, not requests.
                if pending[0] != 0xAA:
                    pending.pop(0)
                    continue
                operation = pending[1]
                register = pending[2]
                length = pending[3]
                needed = 4 + length if operation == 0x00 else 4
                if operation not in {0x00, 0x01}:
                    pending.pop(0)
                    continue
                if len(pending) < needed:
                    break
                del pending[:needed]
                if operation == 0x00:  # write register bytes
                    os.write(self.master, b"\xee\x01")
                else:
                    self._handle_read(register, length)

    def close(self) -> None:
        self.stop_event.set()
        self.worker.join(timeout=1)
        os.close(self.slave)
        os.close(self.master)


class ProbeSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("bag2test_bno055_probe")
        self.received_at: list[float] = []
        self._subscription = self.create_subscription(
            Imu,
            "/bno055/imu",
            lambda _msg: self.received_at.append(time.monotonic()),
            10,
        )


def _advance(node: ProbeSubscriber, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.04)


def _until(node: ProbeSubscriber, predicate, seconds: float, reason: str) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.04)
    raise AssertionError(reason)


def _start_driver(revision: str, port: str, logfile: Path) -> subprocess.Popen:
    env = dict(os.environ)
    # Preserve the ROS 2 Python paths from the sourced Jazzy environment.
    # Replacing them with the upstream package path hides rclpy in the child.
    env["PYTHONPATH"] = SOURCE_DIRS[revision] + os.pathsep + env.get("PYTHONPATH", "")
    # ROS 2 Jazzy discovers only processes sharing this container's loopback.
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    logfile_handle = logfile.open("w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "bno055.bno055",
                "--ros-args",
                "-p",
                f"uart_port:={port}",
                "-p",
                "uart_timeout:=0.04",
                "-p",
                "data_query_frequency:=10",
                "-p",
                "calib_status_frequency:=0.2",
                "-p",
                "ros_topic_prefix:=/bno055/",
            ],
            env=env,
            stdout=logfile_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    finally:
        logfile_handle.close()
    return process


def _stop_driver(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


def run_case(case: str) -> dict:
    uart = Bno055VirtualUart()
    if case == "healthy":
        uart.restore()
    node: ProbeSubscriber | None = None
    proc: subprocess.Popen | None = None
    restart: subprocess.Popen | None = None
    with tempfile.TemporaryDirectory(prefix="bno055-ros-") as folder:
        work = Path(folder)
        logs: list[Path] = []
        try:
            rclpy.init()
            node = ProbeSubscriber()
            revision = "fixed" if case != "bad" else "bad"
            first_log = work / "first.log"
            logs.append(first_log)
            proc = _start_driver(revision, uart.port, first_log)
            _until(
                node,
                lambda: len(node.received_at) >= 4,
                seconds=9,
                reason="No healthy IMU messages from the real ROS publisher before failure",
            )
            baseline_received = len(node.received_at)
            if case != "healthy":
                uart.start_failure()
            if case == "healthy":
                _advance(node, 2)
                assert proc.poll() is None, "Healthy node unexpectedly exited"
                assert len(node.received_at) >= baseline_received + 8
                return {
                    "case": case,
                    "real_ros_node": True,
                    "real_uart_connector": True,
                    "virtual_bno055": True,
                    "messages": len(node.received_at),
                    "driver_exit": "ALIVE",
                    "result": "PASS",
                }

            _until(
                node,
                lambda: uart.stats()["silenced_reads"] >= 5,
                seconds=5,
                reason="Pseudo-terminal did not simulate consecutive serial read failures",
            )
            stopped_at = len(node.received_at)
            if case == "bad":
                _advance(node, 4)
                assert proc.poll() is None, "Faulty source unexpectedly exited"
                assert len(node.received_at) == stopped_at, (
                    "Silent upstream unexpectedly recovered"
                )
                return {
                    "case": case,
                    "real_ros_node": True,
                    "real_uart_connector": True,
                    "virtual_bno055": True,
                    "messages_before_silence": stopped_at,
                    "additional_messages_during_silence": 0,
                    "driver_exit": "STILL_ALIVE",
                    "uart": uart.stats(),
                    "result": "PASS",
                }

            _until(
                node,
                lambda: proc.poll() is not None,
                seconds=7,
                reason="Fixed upstream did not terminate after sustained serial read failure",
            )
            exitcode = proc.wait(timeout=2)
            assert exitcode == 1, f"Expected driver recovery exit 1, got {exitcode}"
            first_stats = uart.stats()
            assert first_stats["silenced_reads"] >= MAX_FAILURES_AT_10HZ
            messages_before_restart = len(node.received_at)

            # A real external supervisor starts a fresh process after the
            # virtual sensor starts responding. The driver does not self-restart.
            uart.restore()
            second_log = work / "restart.log"
            logs.append(second_log)
            restart = _start_driver("fixed", uart.port, second_log)
            _until(
                node,
                lambda: len(node.received_at) >= messages_before_restart + 5,
                seconds=8,
                reason="Supervisor restart did not restore real ROS topic traffic",
            )
            assert restart.poll() is None, "Recovered process died"
            return {
                "case": case,
                "real_ros_node": True,
                "real_uart_connector": True,
                "virtual_bno055": True,
                "pre_restart_messages": messages_before_restart,
                "recovered_messages": len(node.received_at) - messages_before_restart,
                "driver_exit": exitcode,
                "supervisor_restart_count": 1,
                "uart": uart.stats(),
                "result": "PASS",
            }
        except Exception:
            for log in logs:
                if log.is_file():
                    print(f"--- {log.name} tail ---", file=sys.stderr)
                    print(log.read_text(encoding="utf-8")[-3500:], file=sys.stderr)
            raise
        finally:
            if proc is not None:
                _stop_driver(proc)
            if restart is not None:
                _stop_driver(restart)
            if node is not None:
                node.destroy_node()
                rclpy.shutdown()
            uart.close()


def main() -> int:
    case = os.environ.get("BAG2TEST_BNO_CASE", "fixed")
    if case not in {"bad", "fixed", "healthy"}:
        raise ValueError("BAG2TEST_BNO_CASE must be bad, fixed or healthy")
    answer = run_case(case)
    print(json.dumps(answer, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
