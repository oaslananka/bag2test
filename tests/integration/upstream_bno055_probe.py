"""Opt-in, offline source-level differential for a documented ROS 2 IMU bug.

Run two *unchanged* external upstream Python source files, checked by SHA-256.
The ROS node, serial port, sensor and scheduler are deterministic test doubles.
No actual ROS graph, robot, hardware or service-restart behavior is exercised.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import runpy
import sys
import types
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

# Exact source revisions for the upstream BNO055 read-failure fix.
# https://github.com/ssharma0704/bno055/commit/fe31e0862b93414dff88a2e0ab496ca03cbbd5a0
BAD_COMMIT = "d8dd54bd930a9aa104dad6052dfac73ad00fd734"
FIX_COMMIT = "fe31e0862b93414dff88a2e0ab496ca03cbbd5a0"
BAD_SHA256 = "17cfc8011773c1a2aa756b354643a9ca7511dc37ea1bc974ec3e2fcb98cc0c3f"
FIX_SHA256 = "1e4a00adbae13385e25086bc41155684bcc7fb7a9d15a3cce6e4335f6f019767"
TICKS = 80
HZ = 10
INITIAL_HEALTHY_TICKS = 8


@dataclass
class Run:
    revision: str
    case: str
    outcome: str
    attempted_reads: int
    good_reads: int
    failed_reads: int


class Value:
    def __init__(self, value: object) -> None:
        self.value = value


class FakeLogger:
    def info(self, _message: str) -> None:
        pass

    def warn(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


class FakeNode:
    def __init__(self, _name: str) -> None:
        self.timers: list[Callable[[], None]] = []
        self.sensor: FakeSensorService

    def get_logger(self) -> FakeLogger:
        return FakeLogger()

    def create_timer(
        self, _period: float, callback: Callable[[], None]
    ) -> Callable[[], None]:
        self.timers.append(callback)
        return callback

    def destroy_timer(self, _timer: Callable[[], None]) -> None:
        pass

    def destroy_node(self) -> None:
        pass


class FakeParameters:
    def __init__(self, _node: FakeNode) -> None:
        self.connection_type = Value(7)
        self.uart_baudrate = Value(115200)
        self.uart_port = Value("SIMULATED")
        self.uart_timeout = Value(0.02)
        self.data_query_frequency = Value(HZ)
        self.calib_status_frequency = Value(1)


class FakeUART:
    CONNECTIONTYPE_UART = 7

    def __init__(self, *_args: object) -> None:
        pass

    def connect(self) -> None:
        pass


class FakeI2C:
    CONNECTIONTYPE_I2C = 8


# Deliberately mutable only for the active source-level test case.
_active_case = ""
_active_sensor: FakeSensorService | None = None


class FakeSensorService:
    def __init__(
        self, _node: FakeNode, _connector: object, _params: FakeParameters
    ) -> None:
        global _active_sensor
        self.attempted = 0
        self.good = 0
        self.failed = 0
        _active_sensor = self

    def configure(self) -> None:
        pass

    def get_sensor_data(self) -> None:
        self.attempted += 1
        idx = self.attempted
        if _active_case == "permanent_disconnect":
            fails = idx > INITIAL_HEALTHY_TICKS
        elif _active_case == "intermittent_recovery":
            fails = (9 <= idx <= 28) or (30 <= idx <= 49)
        elif _active_case == "healthy":
            fails = False
        else:
            raise AssertionError(f"Unexpected case: {_active_case}")
        if fails:
            self.failed += 1
            raise OSError("Simulated IMU serial read failure")
        self.good += 1

    def get_calib_status(self) -> None:
        pass


def _install_fakes() -> None:
    def fake_module(name: str, **attributes: object) -> None:
        item = types.ModuleType(name)
        item.__dict__.update(attributes)
        sys.modules[name] = item

    fake_module("bno055")
    fake_module("bno055.connectors")
    fake_module("bno055.connectors.i2c", I2C=FakeI2C)
    fake_module("bno055.connectors.uart", UART=FakeUART)
    fake_module("bno055.error_handling")
    fake_module(
        "bno055.error_handling.exceptions",
        BusOverRunException=type("BusOverRunException", (Exception,), {}),
    )
    fake_module("bno055.params")
    fake_module("bno055.params.NodeParameters", NodeParameters=FakeParameters)
    fake_module("bno055.sensor")
    fake_module("bno055.sensor.SensorService", SensorService=FakeSensorService)
    fake_module("rclpy", init=lambda: None, spin=_spin, shutdown=lambda: None)
    fake_module("rclpy.node", Node=FakeNode)


def _spin(node: FakeNode) -> None:
    for _ in range(TICKS):
        node.timers[0]()


def _load_exact_source(path: Path, expected_hash: str, label: str) -> dict:
    raw = path.read_bytes()
    actual = hashlib.sha256(raw).hexdigest()
    if actual != expected_hash:
        raise ValueError(f"{label} upstream source hash mismatch: {actual}")
    # Load independently pinned, SHA-256 verified source in this explicitly
    # opt-in disposable test process (never as part of normal PR CI).
    return runpy.run_path(str(path), run_name=f"upstream_bno055_{label}")


def run(source: dict, revision: str, case: str) -> Run:
    global _active_case, _active_sensor
    _active_case = case
    _active_sensor = None
    outcome = "completed"
    try:
        source["main"]()
    except SystemExit as exc:
        outcome = f"SystemExit({exc.code})"
    sensor = _active_sensor
    if sensor is None:
        raise AssertionError("Upstream source did not construct a sensor service")
    return Run(revision, case, outcome, sensor.attempted, sensor.good, sensor.failed)


def main(bad_file: Path, fixed_file: Path) -> int:
    _install_fakes()
    sources = {
        "bad": _load_exact_source(bad_file, BAD_SHA256, "bad"),
        "fixed": _load_exact_source(fixed_file, FIX_SHA256, "fixed"),
    }
    outcomes: list[Run] = []
    for revision, source in sources.items():
        for case in ("healthy", "intermittent_recovery", "permanent_disconnect"):
            result = run(source, revision, case)
            outcomes.append(result)
            if case in {"healthy", "intermittent_recovery"}:
                assert result.outcome == "completed" and result.attempted_reads == TICKS
            elif revision == "bad":
                assert result.outcome == "completed"
                assert (
                    result.attempted_reads,
                    result.good_reads,
                    result.failed_reads,
                ) == (80, 8, 72)
            else:
                assert result.outcome == "SystemExit(1)"
                assert (
                    result.attempted_reads,
                    result.good_reads,
                    result.failed_reads,
                ) == (38, 8, 30)
    print(
        json.dumps(
            {
                "kind": "external_upstream_source_level_regression",
                "repo": "ssharma0704/bno055",
                "bad_commit": BAD_COMMIT,
                "fixed_commit": FIX_COMMIT,
                "sensor": "SIMULATED",
                "ros_runtime": "MOCKED",
                "hardware_verified": False,
                "systemd_restart_verified": False,
                "bag2test_generated_scenario": "NOT_EXECUTED_IN_THIS_PROBE",
                "cases": [result.__dict__ for result in outcomes],
                "result": "PASS",
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bad-file", type=Path, required=True)
    parser.add_argument("--fixed-file", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(main(args.bad_file, args.fixed_file))
