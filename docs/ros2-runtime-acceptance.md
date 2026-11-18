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
would not demonstrate a loss of robot data. For short windows, the harness
also tracks actual integer sequence numbers encoded in JointState positions.
It waits for a fully connected ROS relay and discards warmup messages before
counting missing data.

### Short-gap experiment

Use the same pinned container with a 27 ms input gap and a 100 Hz publisher:

~~~bash
docker run --rm --network none --memory 4g --cpus 4   -e BAG2TEST_GAP_MS=27 -e BAG2TEST_HZ=100   -e BAG2TEST_REPETITIONS=5 bag2test-ros2-acceptance
~~~

The candidate schedule is derived from a newly constructed synthetic MCAP.
It is **not** the 145 MB real-robot demonstration bag and does not replay
its CDR payloads.

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
the control maximum gap stayed below 0.4 seconds. All **3/3** pairs passed
in the original experiment. After the sequence-counting and ROS
warmup refinements, the original 20 Hz / 2-second experiment was rerun:
another **3/3** positive/negative pairs passed, each with 150 sent /
110 downstream fault messages and 150/150 healthy controls.

### Short 27 ms window / 100 Hz ROS 2 publishing

A separate experiment was run on a rootless, isolated CPU Docker container,
with a genuinely serialized JointState stream and a *synthetically inserted*
27 ms MCAP log-time interval. The generator emitted the unmodified short
injection candidate. The healthy control changed only drop probability to
zero. Sequence numbers, not merely wall-clock gaps, were checked.

| Trial | Fault sent/received | Missing sequences | Control sent/received | Result |
| --- | --- | ---: | --- | --- |
| 1 | 750/748 | 2 | 750/750 | PASS |
| 2 | 750/747 | 3 | 750/750 | PASS |
| 3 | 750/747 | 3 | 750/750 | PASS |
| 4 | 750/747 | 3 | 750/750 | PASS |
| 5 | 750/747 | 3 | 750/750 | PASS |

Fault and control both emitted activation/deactivation events, proving why
the events alone would have been insufficient. Five of five pairs showed
real downstream sequence loss exclusively in the injected stream.

The control maximum observed inter-arrival time occasionally reached
70 ms even though **no messages were missing**. This demonstrates that
an arrival-time gap is not itself proof of packet loss or its cause.
Five passes are a narrow functional measurement, not a population-wide
reliability guarantee.

### Scope of this evidence

**VERIFIED:** the generated YAML was accepted by the compiled upstream
runner and caused the expected downstream message-loss *symptom* in this
controlled ROS 2 experiment; a zero-drop negative control did not.

**NOT VERIFIED:** a real-world robot failure, its physical root cause,
replay of a recorded robot's complete behavior, faulty-versus-fixed software
regression, performance or manual-authoring time savings, or reliable
reproduction of *arbitrary real-world* 27 ms gaps in different scheduling
conditions. The MCAP stimuli in both experiments were synthetically
constructed, even though they contain genuine ROS 2 serialized JointState
messages. The real demonstration's timing-gap *cause* remains unknown.
