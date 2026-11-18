# Pinned IMU driver failure and controlled message-drop test

This evidence addresses two **different** questions. It does not establish
that dropping ROS messages reproduces the physical cause of a failed IMU
serial read.

## 1. A documented faulty and repaired ROS 2 source revision

[Rover Robotics' ROS 2 README](https://github.com/RoverRobotics/roverrobotics_ros2)
describes a BNO055 IMU driver that continued running when sensor reads
repeatedly failed, leaving its IMU topic silent. Its documented repair causes
the process to exit after sustained failed reads so an *external supervisor*
can restart it.

The upstream BSD-3-Clause [BNO055 source repository](https://github.com/ssharma0704/bno055)
contains an exact, directly adjacent code change:

- Faulty (direct parent): [`d8dd54bd930a9aa104dad6052dfac73ad00fd734`](https://github.com/ssharma0704/bno055/commit/d8dd54bd930a9aa104dad6052dfac73ad00fd734)
- Repaired: [`fe31e0862b93414dff88a2e0ab496ca03cbbd5a0`](https://github.com/ssharma0704/bno055/commit/fe31e0862b93414dff88a2e0ab496ca03cbbd5a0)

The repair adds a counter for consecutive failed reads and exits after
`max(5, 3 * configured_read_frequency)` failures. It does **not** itself
perform a restart.

### Reproduce without a physical IMU

This is an *opt-in source-level test*, not part of normal PR CI. Download
the two unchanged files to a **disposable test environment**; the script
checks their exact SHA-256 hashes before it imports either source file.
No external source files or hardware dumps are committed to bag2test.

```bash
mkdir -p /tmp/bno055-source-regression
curl -fsSL \
  https://raw.githubusercontent.com/ssharma0704/bno055/d8dd54bd930a9aa104dad6052dfac73ad00fd734/bno055/bno055.py \
  -o /tmp/bno055-source-regression/bad.py
curl -fsSL \
  https://raw.githubusercontent.com/ssharma0704/bno055/fe31e0862b93414dff88a2e0ab496ca03cbbd5a0/bno055/bno055.py \
  -o /tmp/bno055-source-regression/fixed.py
python tests/integration/upstream_bno055_probe.py \
  --bad-file /tmp/bno055-source-regression/bad.py \
  --fixed-file /tmp/bno055-source-regression/fixed.py
```

Expected hashes are embedded in the tester. A modified source fails
closed with a hash-mismatch error, as tested. Although the files come from
a pinned public commit, the tester **executes their Python source**:
run it only in a disposable, least-privileged environment.

The test replaces `rclpy` node/scheduler APIs and serial/sensor interfaces
with deterministic, in-memory substitutes. It invokes the unchanged upstream
`main()` entry point and its actual nested `read_data()` callback 80 times.
At 10 Hz, it first supplies eight successful reads, then permanent failures.

| Source | Good reads | Failed reads | Outcome |
| --- | ---: | ---: | --- |
| Direct parent, known bad | 8 | 72 | Still running after 80 read callbacks |
| Fix commit | 8 | 30 | Exits with code 1 on callback 38 |

Controls at both commits: 80/80 healthy reads complete normally, and
two interrupted runs of 20 failures separated by recovery do not cause
an exit. The exact 6 outcomes are printed in machine-readable JSON.
Results are deterministic **source-level regression evidence** for a
documented software fault, not a sensor, ROS executor, or systemd acceptance.

## 2. Controlled ROS 2 `sensor_msgs/Imu` message dropout

Separately, bag2test already supports the upstream `imu` injector type.
The opt-in ROS 2 Jazzy acceptance harness now constructs genuinely
serialized `sensor_msgs/msg/Imu` CDR messages and verifies actual downstream
sequence numbers through the compiled upstream `ros2_fault_injection`
runner. The messages and their source gap are **synthetic**, not collected
from a BNO055 robot.

```bash
docker build -f tests/integration/Dockerfile \
  -t bag2test-ros2-acceptance .
docker run --rm --network none --memory 4g --cpus 4 \
  -e BAG2TEST_MESSAGE_TYPE=imu \
  -e BAG2TEST_REPETITIONS=2 \
  bag2test-ros2-acceptance
```

| Trial | Fault sent/received | Missing IDs | Healthy sent/received | Outcome |
| --- | --- | ---: | --- | --- |
| 1 | 150/110 | 40 | 150/150 | PASS |
| 2 | 150/110 | 40 | 150/150 | PASS |

Fault activation and deactivation events were independently observed.
The source-to-YAML `imu` mapping is also checked in normal pytest.
The original `JointState` runtime test was rerun after the harness
change and remained PASS (1/1), with 150/110 fault and 150/150 control.

## Interpretation and remaining product gate

- **VERIFIED:** a real upstream source-level faulty/fixed behavior
  difference under mocked hardware and scheduler; and real ROS 2 runtime
  execution of a *different*, synthetically generated IMU topic-drop
  candidate.
- **NOT VERIFIED:** that this topic-drop fault mechanism reproduces
  the internal UART/I²C failure that triggered the upstream bug.
  Injecting a drop *after* a functioning IMU publisher does not cause
  `SensorService.get_sensor_data()` to fail, and therefore cannot prove
  that this particular driver fix resolves such an injected outage.
- **NOT VERIFIED:** actual hardware failure and subsequent recovery,
  supervisor restarting the node, a labeled field-incident MCAP, an
  end-to-end real faulty/fixed system regression, or human time savings.

The case illustrates a crucial product constraint: **recreating a visible
topic silence is not necessarily reproducing the original failure
mechanism or validating the software fix**. bag2test must not call that
a reproduced field incident.
