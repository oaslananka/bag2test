# bag2test

[![GitGem](https://gitgem.org/api/badge/github/oaslananka/bag2test.svg)](https://gitgem.org/github/oaslananka/bag2test)

bag2test examines timing gaps in ROS 2 MCAP recordings and generates **candidate** fault-injection scenarios for selected topics.

## Status

**Experimental.** The current CLI reads MCAP message metadata and writes an observation report and, when a sufficiently large gap is found, a YAML candidate for [ros2_fault_injection](https://github.com/reeceholland/ros2_fault_injection). The output is not an executed regression test. An optional controlled synthetic ROS 2 runner test has passed; no physical robot, field incident, or MCP integration has been validated.

## Install

Python 3.11 or later is required. ROS 2 is not required to inspect an MCAP file.

```bash
python -m pip install -e .
```

## Analyze a recording

```bash
bag2test recording.mcap --topic /joint_states --out results
```

`results/evidence.json` includes the selected topic, sample count, median
log-time period, largest detected timing gap, the following interval, whether
MCAP `publish_time` duplicates `log_time`, and hypothesis/verification
statuses.

If a gap exceeds the default threshold of **2.5 × median observed period**, `results/scenario.yaml` contains a candidate scenario using the upstream injector schema. Otherwise, only the report is written. `results` must not already exist.

For example, with a lower threshold and a selected time window:

```bash
bag2test recording.mcap --topic /joint_states \
  --min-gap-factor 2.0 --start-sec 5 --end-sec 25 \
  --input-topic /joint_states_raw --out window-results
```

Input and output topics must be distinct. The engineer must supply a **separate
healthy ROS input stream**; messages already missing from a faulty recording
cannot be dropped again. The user must arrange the publisher remapping and
runner clock alignment. A generated fault is a candidate mechanism, not proof
of the original fault's cause.

## Interpretation

- **CANDIDATE:** a gap between two selected-topic messages crossed the threshold; the generated injection is a hypothesis.
- **TRAILING_SILENCE:** the topic stops before another recorded channel's last message. This is **recording-coverage evidence**, not proof of a sensor failure. No bounded injection YAML is generated without a recorded recovery.
- **NO_CANDIDATE:** neither an internal gap nor an evidenced trailing silence crossed the selected threshold.
- **INSUFFICIENT_DATA:** too few samples to estimate a useful cadence.
- **NOT_VERIFIED:** the proposed fault was not exercised in ROS 2 and the failure was not reproduced.

[End-of-recording topic silence](docs/terminal-silence.md) is reported
only when the MCAP's last message comes from another recorded channel.
A one-topic recording cannot establish how long its silent sensor
remained unobserved; a clipped analysis window cannot establish a
terminal outage.

The current generator supports timing-gap hypotheses on `JointState`, `Odometry`, `LaserScan`, `Imu`, and `Twist` ROS 2 CDR topics. Only message metadata is inspected, not sensor or command payloads. A gap in recorded timestamps can result from sensor timing, transport, driver
behavior or the recorder. The next interval may be unusually short, consistent
with catch-up or batching. Equal MCAP publish/log timestamps do not provide
an independent clock check. Neither pattern identifies message loss or its
physical cause.

A generated `fault_event` assertion only checks activation. It is **not** an assertion that the robot experienced the same failure. Short proposed windows may not be reliable under ROS timer scheduling. Never connect generated faults to a physical robot without independent controls.

## Validation and limitations

Run local tests:

```bash
python -m pip install -e ".[test]"
python -m pytest -q
```

Unit tests use synthetic MCAP files and check timing, output shape and error handling. The [real-robot recording smoke test](docs/real-data-smoke.md) analyzes a separate, non-committed recording. An [opt-in ROS 2 runtime acceptance test](docs/ros2-runtime-acceptance.md) exercises the generated YAML against a compiled fault injector with synthetic ROS messages and negative controls (3/3 PASS); a separate 27 ms / 100 Hz synthetic short-window acceptance
run measured downstream sequence loss against a zero-drop control (5/5
PASS). An independent upstream BNO055 driver bug/fix has also been
tested with a real ROS 2 executor and a software-emulated UART
(6/6 controlled trials PASS). This is **not** a physical hardware
or labeled field-incident regression; no manual-authoring time
benchmark has been verified.

A [comparative feasibility check](docs/comparative-benchmark.md) confirms
that existing ROS recording tools also find the same timing-gap observation.
An [independently pinned IMU driver regression](docs/upstream-imu-regression.md)
demonstrates the actual ROS 2 driver bug/fix and virtual-UART supervised
recovery, separately from a synthetically injected ROS topic drop.
The CLI's output remains an **unverified candidate** for an
existing fault-injection runner; no human engineering-time advantage is established.

This repository is not a general ROS debugger, a robot controller, or a safety system.

## License

GPL-3.0-or-later. See [LICENSE](LICENSE).
