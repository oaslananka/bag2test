# Comparative feasibility check — 19 November 2026

This check asks whether bag2test does anything useful that existing
ROS recording tools do not already do. It is **not** a ranking, a human
productivity study, or a reproduced field-incident regression.

## Data and setup

- Recording: [IO Intelligence DualArx R5a sample](https://huggingface.co/datasets/io-intelligence/FoldingTShirt_DualArxR5a_Samples), `livingroom_T-shirt_folding_DualArxR5a_251110_173213_0.mcap`.
- Dataset declares Apache-2.0; the 145,137,874-byte file was downloaded separately and **not redistributed**.
- SHA-256: `43ec38b592b95bfa7cbdedf490b379dc8d11030a5911bfb7e07da008e313b69b`.
- Topic: `io_teleop/joint_cmd` (6,076 messages), with a median 10.001136 ms interval and maximum **27.140731 ms** log-time interval.
- Comparison versions: bag2test `0.1.0.dev0`, [ROSBag Doctor](https://github.com/sylvesterkaczmarek/rosbag-doctor) `0.1.1` and [Backplane Tracecheck](https://github.com/backplane-robotics/backplane-tracecheck) `0.4.0`.
- CPU-only local CLI runs. No real robot, remote upload, paid compute or external automation.

To compare similar *observed timing* thresholds, we used:

```yaml
# ROSBag Doctor: doctor25.yaml
version: 1
topics:
  io_teleop/joint_cmd:
    required: true
    max_gap_ms: 25
```

```json
{
  "version": 1,
  "name": "25ms observed log-time threshold",
  "topics": {
    "io_teleop/joint_cmd": {
      "required": true,
      "min_count": 1,
      "time_basis": "log_time",
      "clock_confirmed": true,
      "max_gap_ns": 25000000
    }
  }
}
```

Tracecheck's `clock_confirmed: true` is an **explicit comparison assumption**
that the recorded log-time scale can be checked against 25 ms; it is *not*
independent proof of clock accuracy or actual missing publisher messages.
bag2test's default 2.5-times-median threshold is about 25.003 ms on this
topic, nearly equivalent to the others' 25 ms absolute limit.

## Observed results

| Tool | No explicit 25 ms contract | With the near-equivalent threshold | Extra output |
| --- | --- | --- | --- |
| ROSBag Doctor 0.1.1 | PASS, no recording issues | FAIL: 27.1 ms > 25 ms | Recording-quality report |
| Backplane Tracecheck 0.4.0 | PASS under a minimal inventory contract | FAIL: configured C07 log-time gap rule | JSON / HTML integrity report |
| bag2test 0.1.0.dev0 | CANDIDATE under default 2.5× median threshold | Same 27.140731 ms observation | `scenario.yaml` and `evidence.json` |

These tools have different default purposes and checks. A FAIL in the first
two means a configured *recording contract* failed, not that the robot
physically malfunctioned. A bag2test CANDIDATE is not a regression test pass.
The other tools already detect the observed timing anomaly; **timing-gap
detection is not a differentiator**. Other ROS recording tools also support
additional checks, policies and file formats not provided by bag2test.

bag2test's current distinct output is a tentative `ros2_fault_injection`
scenario. Its correctness, execution and benefit must be evaluated
**separately** from anomaly detection.

## Hand-authored scenario comparison

The reference file `tests/integration/manual_joint_state.yaml` is a
standalone, hand-authored upstream scenario describing a **synthetic**
two-second `JointState` dropout. The test asserts its parsed YAML matches
the generated `bag2test` scenario exactly. This guards against accidental
generator-schema drift but does not measure authoring effort.

Run the opt-in CPU-only ROS 2 acceptance experiment:

```bash
docker build -f tests/integration/Dockerfile \
  -t bag2test-ros2-acceptance .
docker run --rm --network none --memory 4g --cpus 4 \
  -e BAG2TEST_COMPARE_MANUAL=1 \
  -e BAG2TEST_REPETITIONS=2 \
  bag2test-ros2-acceptance
```

The experiment executes the generated candidate and the manually authored
scenario with independent live ROS publishers, downstream subscribers and
zero-drop controls. It inspects missing sequence numbers, not just
fault-activation events. It uses the previously pinned ROS 2 Jazzy
`ros2_fault_injection` runtime.

### Actual ROS 2 results

The compiled upstream runner ran in CPU-only, network-isolated Docker as a
non-root user. Each trial involved a 20 Hz publisher and a synthetic two-second
message-drop window. In each paired control, only `drop_probability` changed
from `1.0` to `0.0`.

| Scenario source | Trial | Sent | Delivered | Missing IDs | Longest gap | Healthy control |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Generated YAML | 1 | 150 | 110 | 40 | 2.049 s | 150/150 |
| Generated YAML | 2 | 150 | 110 | 40 | 2.049 s | 150/150 |
| Hand-authored YAML | 1 | 150 | 110 | 40 | 2.048 s | 150/150 |
| Hand-authored YAML | 2 | 150 | 110 | 40 | 2.051 s | 150/150 |

**Generated 2/2 PASS; manual 2/2 PASS.** Injector activation and
deactivation events were observed in both cases. The healthy controls had
zero missing message IDs. The independent manual YAML parsed identically to
bag2test's generated YAML in the fixed synthetic fixture; a normal pytest
test now guards the equivalence.

No human participants were timed. Comparable success says the generated
scenario behaves like an accurately authored manual scenario, **not** that
it is faster to create or easier to diagnose a real incident.

## Decision boundary

What can be verified: the same observation is detected by mature
recording-quality tools, bag2test emits a compatible scenario, and a
controlled upstream runner can be exercised for both generated and manual
scenarios.

What remains **NOT VERIFIED**: a field incident with independent ground truth,
a real faulty software revision failing and a fixed revision passing, and
measured human effort saved over writing an upstream scenario by hand.
Equivalent YAML and successful synthetic ROS execution **do not** imply
time savings or commercial usefulness. Until those missing gates pass,
building a general ROS diagnostics platform is **NO-GO**; a narrow,
experimental record-to-test connector remains under evaluation.
