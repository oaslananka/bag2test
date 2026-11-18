# ROS 2 runtime acceptance — 18 November 2026

This experiment tests a generated fault scenario against the actual ROS 2
fault-injection runner. It uses controlled synthetic ROS messages and does not
claim to reproduce a field incident or verify a software fix.

## Run it

Requires a local Docker engine, several GB of temporary disk space, and a CPU.
Neither a physical robot nor an NVIDIA GPU is needed.

~~~bash
docker build --progress=plain \
  -f tests/integration/Dockerfile \
  -t bag2test-ros2-acceptance .
docker run --rm --network none --memory 4g --cpus 4 bag2test-ros2-acceptance
~~~

The Dockerfile pins the ROS 2 Jazzy base image by digest and checks out
[ros2_fault_injection](https://github.com/reeceholland/ros2_fault_injection)
at commit `62a1e4bd9916eeb5ab270b21bdb5b10306cb203c`.
The upstream source is compiled inside the disposable image, not vendored
into this repository. The experiment has no external ROS network access.

## What is exercised

1. Generate a valid MCAP containing serialized `sensor_msgs/msg/JointState`
   messages, with a known two-second message-log-time gap.
2. Run the actual bag2test analysis and scenario generator, producing
   `scenario.yaml` with the upstream injector schema.
3. Launch the real `ros2_fault_injection` C++ runtime, publish JointState
   messages to `/joint_states_raw` at 20 Hz, and independently subscribe
   to `/joint_states` while collecting typed fault events.
4. Check that the 100% drop scenario creates a downstream gap and produces
   activation/deactivation events.
5. Repeat the same run with only `drop_probability` changed to `0.0`.
   This control still produces fault activation events but must deliver
   messages without a comparable gap.
6. Run three separate pairs in isolated ROS domains.

The independent downstream observation matters: fault activation alone
would not demonstrate a loss of robot data.

## Recorded results

Execution: ROS 2 Jazzy base image `sha256:00b8d47a6e6384a2834e3d01fa87ae767876543baa574f2d64a75c04ea33c698`
and pinned upstream source, CPU-only Docker.

| Trial | Fault producer / downstream | Fault maximum downstream gap | Control producer / downstream | Control maximum gap | Result |
| --- | --- | --- | --- | --- | --- |
| 1 | 150 / 110 | 2.050 s | 150 / 150 | 0.056 s | PASS |
| 2 | 150 / 110 | 2.048 s | 150 / 150 | 0.055 s | PASS |
| 3 | 150 / 110 | 2.049 s | 150 / 150 | 0.055 s | PASS |

Both the fault and control runs emitted the expected active and inactive
fault events. The controlled fault gap exceeded 1.2 seconds in every run;
the control maximum gap stayed below 0.4 seconds. All **3/3** pairs passed.

### Scope of this evidence

**VERIFIED:** the generated YAML was accepted by the compiled upstream
runner and caused the expected downstream message-loss *symptom* in this
controlled ROS 2 experiment; a zero-drop negative control did not.

**NOT VERIFIED:** a real-world robot failure, its physical root cause,
replay of a recorded robot's complete behavior, faulty-versus-fixed software
regression, performance or manual-authoring time savings, or reliable
reproduction of brief gaps such as the previously observed 27 ms gap.
The MCAP stimulus in this experiment was synthetically constructed,
even though it contains genuine ROS 2 serialized JointState messages.
