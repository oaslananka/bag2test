# Engineer task study: manual versus assisted fault-scenario authoring

**Status:** an executable, controlled study procedure. **No independent human
participants have completed it yet.** This is not a result, productivity claim
or new diagnostic feature.

## Question and setup

Does bag2test help an engineer get from an MCAP recording to a **correct,
working** upstream `ros2_fault_injection` scenario with less total effort
and fewer corrections than writing the same scenario manually?

The existing [manual parity experiment](comparative-benchmark.md) already
showed 2/2 generated and 2/2 hand-written ROS 2 scenarios worked in a
synthetic JointState experiment. That proves functional compatibility, **not
human time savings**. This study measures a different outcome.

An evaluator, not a participant, prepares the task with the existing optional
`tests/integration/runtime_probe.py` ROS 2 Jazzy Docker image. The source
MCAP is **synthetic**, uses actual serialized ROS 2 CDR messages, and
contains a 20 Hz stream with a two-second observed gap after 2.5 seconds.
It is **not** a physical robot incident. No physical devices are connected.
No human recordings, personal names or response data are committed.

## Prepare two independent, comparable tasks

**Evaluator-only instructions.** Do **not** share this evaluator document,
the repository source tree, the static reference YAML, historical pilot
results, the target duration or the scoring implementation with participants.
Give each person only the relevant MCAP task recording, its required
input/output topic names, normal upstream documentation and the
[participant brief](study-participant-brief.md). Preinstall the same ROS 2
dependencies for both groups; provide bag2test as an additional installed
tool only to the assisted group. Because this is a public repository,
ask about prior exposure to the study solution and record that limitation.

Build the image once, on a Linux Docker host or Docker Desktop:

```bash
docker build -f tests/integration/Dockerfile \
  -t bag2test-ros2-study .
```

Create a disposable host folder writable by the unprivileged container
user (UID 10001), then export a fresh task recording. The named
`study-data/` directory is Git-ignored to prevent ordinary accidental
staging of participant YAML, notes and evaluation results. The evaluator
must still avoid copying private participant data into Git by other means:

```bash
mkdir -p ./study-data
# On Linux, grant UID 10001 access to ./study-data before running.
docker run --rm --network none --cpus 4 --memory 4g \
  -v "$PWD/study-data:/study" \
  -e BAG2TEST_EXPORT_RECORDING=/study/joint.mcap \
  -e BAG2TEST_MESSAGE_TYPE=joint_state \
  bag2test-ros2-study
```

For a distinct second task, use
`BAG2TEST_MESSAGE_TYPE=imu` and
`BAG2TEST_EXPORT_RECORDING=/study/imu.mcap`. Use separate folders for
different study sessions, never overwrite a participant's work.

For both conditions, use the short
[participant brief](study-participant-brief.md) rather than copying the
timing parameters or result table from this evaluator-only document.
Participants must infer the gap interval from the MCAP; do not tell them
the known answer from the synthetic fixture.

A **manual** condition may inspect recording data, upstream injector docs
and ROS tools, but not invoke bag2test's generator or copy its already
committed `manual_joint_state.yaml` example. An **assisted** condition
may use the same tools plus bag2test CLI; all participants receive identical
time, machine access and upstream docs. Record help requested and exposure
to prior examples as potential confounds.

## Independent scoring of a submitted YAML

The evaluator, **not the participant**, saves the participant-authored
scenario in `./study-data/submission.yaml` and runs:

```bash
docker run --rm --network none --cpus 4 --memory 4g \
  -v "$PWD/study-data:/study:ro" \
  -e BAG2TEST_STUDY_SUBMISSION=/study/submission.yaml \
  -e BAG2TEST_MESSAGE_TYPE=joint_state \
  -e BAG2TEST_REPETITIONS=1 \
  bag2test-ros2-study
```

Use `BAG2TEST_MESSAGE_TYPE=imu` for the IMU task. The evaluator accepts
only a **single bounded**, correctly routed topic drop around 2.5s for
two seconds and the corresponding activation assertion; wrong topic,
invalid type, multiple faults, incorrect duration or unsupported YAML
must be rejected. The test runs the participant's **actual submitted
YAML** through the compiled ROS 2 upstream injector, observes message
sequence IDs from an independent subscriber, and compares against an
otherwise identical **0% drop control**. The target gap should be about
two seconds and should lose approximately 40 of 150 sent messages.
Activation events alone **cannot** pass the test.

The test returns machine-readable `result: PASS` or `FAIL` and
`scenario_source: external_study_submission`, along with sent, received,
missing IDs, longest gap and negative-control results. An automated PASS
says nothing about the time a human needed.

## Verified automated pilot (not human evidence)

On 19 November 2026 the evaluator generated separate JointState and IMU
synthetic ROS 2 CDR MCAP files (3,277 and 3,554 bytes), then executed the
**actual** upstream Jazzy fault injector in a non-root container with no
network or host devices.

| Submitted scenario | Fault received / sent | Missing IDs | Healthy received / sent | Verdict |
| --- | --- | --- | --- | --- |
| Independent, hand-authored JointState YAML | 110 / 150 | 40 | 150 / 150 | PASS |
| bag2test-generated JointState YAML | 110 / 150 | 40 | 150 / 150 | PASS |
| bag2test-generated IMU YAML | 110 / 150 | 40 | 150 / 150 | PASS |
| Wrong output topic | Not launched | — | — | REJECTED |
| Wrong fault duration (0.25 s instead of 2 s) | Not launched | — | — | REJECTED |

The original, non-study `--compare-manual` integration mode was also
rechecked at one trial per arm: both generated and hand-authored
JointState scenarios passed real ROS fault and zero-drop controls.
The values above are machine-verified **functional acceptance**,
**not measured human effort, task usability or time savings**.

## Recording an actual usability outcome

Prefer at least four independent participants and balance both task types
and manual/assisted conditions. A participant should not solve the same
recording in both conditions, because the first attempt teaches the
answer. Randomly assign task/condition order and use an independent
evaluator who does not coach the participant. Include failures and timeouts.

For each anonymous session record **outside Git**:

- Condition (`manual` or `assisted`), task type, participant familiarity
  with ROS 2, and any prior exposure to example scenarios.
- Start at first sight of the MCAP and stop when the first *correct,
  executable* YAML passes the independent fault/control check.
- Active human elapsed time, number of validation attempts and edits,
  completion/failure, whether the fault mechanism was accurately described,
  and any observed points of confusion.
- Test runtime as a **separate** machine-time field so Docker startup is
  not confused with authoring time.

Compare completion rate and time-to-valid-scenario **only among genuinely
observed participants**, retaining failures. A preliminary GO to further
user testing requires no loss of correctness and a meaningful reduction
in median human task effort across balanced cases. Small samples support
descriptive observations, **not statistically proven productivity or
commercial demand**.

**Stop rule:** If no independent engineer is available, report
`HUMAN_STUDY_NOT_VERIFIED`; do not fill in invented times or substitute
an LLM/automated execution for a human participant. No external messages
or recruitment may be sent without the repository owner's separate
explicit permission.
