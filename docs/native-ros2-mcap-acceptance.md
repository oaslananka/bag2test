# Native ROS 2 MCAP acceptance — virtual BNO055 sensor

**Date:** 19 November 2026. **Status:** opt-in experiment, not a
labeled physical field incident.

This acceptance test connects two previously separate steps: a real
upstream BNO055 ROS 2 driver connected to a virtual serial peripheral,
and bag2test's actual MCAP reader. It checks whether the recorded
symptom is correctly classified, without inventing a fault mechanism
or a recovery time absent from the recording.

## Provenance and execution

- The driver source is the unchanged, SHA-256-checked
  [upstream BNO055 source](https://github.com/ssharma0704/bno055)
  at known-bad parent
  `d8dd54bd930a9aa104dad6052dfac73ad00fd734`
  and fixed
  `fe31e0862b93414dff88a2e0ab496ca03cbbd5a0`.
  Driver source license: BSD-3-Clause. No upstream code or generated
  MCAP was committed to this repository.
- ROS 2 Jazzy `rclpy` executes the actual publisher and a test
  subscriber. The subscriber writes real `sensor_msgs/msg/Imu` CDR
  payloads to a native `rosbag2_py.SequentialWriter` using MCAP
  storage. The same test process publishes and receives a **separate
  ROS 2 heartbeat topic** (`std_msgs/msg/String`) and records those
  subscribed messages, establishing that ROS activity continued.
- The *peripheral* is an in-memory BNO055 UART protocol emulator on
  a Linux pseudo-terminal. A deliberate loss of responses is activated
  only after at least **18 IMU messages** have been independently
  received, avoiding early DDS discovery false positives.
- In the faulty source, the publisher remains alive but silently
  stops sending IMU messages. In the repaired source, the driver
  exits code 1 after 30 consecutive UART read failures and an
  independent test supervisor starts a new process after restoring
  the virtual peripheral. No `systemd` service was involved.
- Run in a rootless, CPU-limited Docker container with
  `--network none`, no host serial devices and no paid services.
  Network access is needed only to build the opt-in image and fetch
  the exact upstream source; the runtime does not access the internet.

## Reproduce

On a machine with Docker and a disposable directory writable by UID
10001 (the container's unprivileged user), from the repository root:

```bash
docker build -f tests/integration/Dockerfile.bno055 \
  -t bag2test-bno055-native .

# On Linux, prepare CAPTURE_DIR with appropriate write permissions
# for container UID 10001. On Docker Desktop use a writable shared folder.
CAPTURE_DIR="/path/to/your/disposable-capture-folder"

for case in healthy bad fixed; do
  docker run --rm --network none --memory 2g --cpus 2 \
    -v "$CAPTURE_DIR:/evidence" \
    -e BAG2TEST_BNO_CASE="$case" \
    -e BAG2TEST_BNO_CAPTURE_DIR="/evidence/$case" \
    bag2test-bno055-native
done

# Requires only the normal Python dependencies: pip install -e .
python tests/integration/check_bno055_captures.py "$CAPTURE_DIR"
```

Each case creates its own `metadata.yaml` and native `.mcap` file.
Every destination must be new; neither recording nor evidence output
overwrites an existing directory. The checker reads the three real
ROS 2 recordings and verifies message encoding, two independently
recorded topics, observed timing classification, and the presence or
absence of `scenario.yaml`. Ordinary PR CI does **not** build this
image, download upstream code, or execute this experiment.

## Observed results

Two separately executed groups of healthy/bad/fixed runs were
recorded and analyzed using the current bag2test CLI and the
independent acceptance checker. All **6/6** classified as expected.

| Run | Upstream case | IMU messages | Recorded heartbeat | bag2test observation | Gap |
| --- | --- | ---: | ---: | --- | --- |
| A | Healthy | 38 | 73 | `NO_CANDIDATE` | — |
| A | Bad; UART silent | 18 | 98 | `TRAILING_SILENCE` | 4.474 s after last IMU |
| A | Fixed; restarted | 23 | 100 | `CANDIDATE` | 4.305 s between IMUs |
| B | Healthy | 38 | 72 | `NO_CANDIDATE` | — |
| B | Bad; UART silent | 18 | 98 | `TRAILING_SILENCE` | 4.483 s after last IMU |
| B | Fixed; restarted | 23 | 100 | `CANDIDATE` | 4.268 s between IMUs |

The bad case produced **`evidence.json` only**, because the sensor
never published again during the recording: there is no observed
finite fault duration. The fixed/restarted case produced
**`evidence.json` and a candidate `scenario.yaml`** because the
return of actual IMU data closes the observed interval. The generated
YAML is *not* evidence that a ROS topic-drop fault induces the
driver's internal UART failure.

The three run-A native MCAP SHA-256 values, ordered healthy/bad/fixed:

- `0483c9e6556d4a1f240ad57f10b82197da9da9d36f35ef1771ee59a0475f821e`
- `8033ef582f856f2b9a18d4dd7d444e4170a5a530b59e324423709b2603eea61a`
- `4055aa55c6b611a38848526a687ebac0318bda7d561add770d94539e4e3660ed`

The three run-B values:

- `7b6e0b77dc9d4c15c22b8e09055e0b8ca5cd68702dd2b5b8f5e7b924db2abab8`
- `e4cc021a6bf2645d9ac3bfc6e3b8a60a41752b7ec4c8fb9c47c494d2aebd6792`
- `3a3b2adff936efa7e64c453e0b3881bdb0b07758dcc47adeaa9c33e2e84c3a85`

The result depends on the independent heartbeat continuing: a
single-topic MCAP ending with the IMU cannot prove any following
silence. These are actual ROS 2 subscriber captures of a
**controlled virtual-sensor experiment**, not manually fabricated
MCAP messages, but also **not physical robot field data**.

## Independent public-data suitability check

Another candidate, the Cartographer backpack real-world ROS 1 data,
has publisher-provided labels for laser scan gaps and a
[3.7 MB excerpt](https://github.com/thrmnn/ros2-localization-triage/blob/main/demo/NOTICE.md).
It is a ROS **1 `.bag`**, not a ROS 2 CDR MCAP; the existing bag2test
reader cannot use it without a correctly specified, verified
conversion of message types and time bases. Those labels cannot be
treated as evidence for this particular ROS 2 code path.
An independently labeled
[localization failure dataset](https://tore.tuhh.de/items/version/419)
contains ROS 2 bags generated in **Isaac Sim** for localization
failure prediction, a different task rather than a physical sensor
dropout matched to a known faulty/fixed driver.

Neither is silently counted as a real-world, directly reproducible
MCAP-based BNO055 timing failure. This suitability screening is not
an exhaustive survey of public ROS datasets.

## Product decision

**Verified:** native real ROS 2 CDR recording → MCAP reading →
appropriate healthy, terminal-silence and recovery-gap outputs,
with no fabricated fault duration for an open-ended silence. The
upstream bug/fix and virtual UART process restart were independently
verified.

**NOT VERIFIED:** original physical robot incident replay, downstream
topic-drop injection causing the BNO055 serial driver failure,
human engineering time saved over existing tools, or commercial
adoption. Next evaluation should obtain an independently labeled,
compatible field failure and time actual users completing the
scenario-authoring task. Do not expand into a general robotics
diagnostics suite on the strength of this controlled experiment.
