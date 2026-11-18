# Independent MCAP smoke test — 18 November 2026

This is a *read-only recording analysis*, not a reproduced robot incident.

## Source and provenance

- Dataset: [IO Intelligence — FoldingTShirt_DualArxR5a_Samples](https://huggingface.co/datasets/io-intelligence/FoldingTShirt_DualArxR5a_Samples).
- Recording: `data/livingroom_T-shirt_folding_DualArxR5a_251110_173213_0.mcap`.
- Declared dataset license: Apache-2.0. The source recording is **not redistributed** in this repository.
- Downloaded size: 145,137,874 bytes.
- SHA-256: `43ec38b592b95bfa7cbdedf490b379dc8d11030a5911bfb7e07da008e313b69b`.

The recording contains ROS 2 JointState and other teleoperation topics from a robot demonstration. The dataset does **not** label the timing gap below as an actual robot failure.

## Reproduce the local analysis

Download the recording separately from its source under the dataset terms, then:

```bash
bag2test livingroom_T-shirt_folding_DualArxR5a_251110_173213_0.mcap \
  --topic io_teleop/joint_cmd --out command-report

bag2test livingroom_T-shirt_folding_DualArxR5a_251110_173213_0.mcap \
  --topic io_teleop/joint_states --out state-report
```

Observed with MCAP Python 1.6.0 and the default 2.5x median-period threshold:

| Topic | Messages | Median observed interval | Largest interval | Result |
| --- | ---: | ---: | ---: | --- |
| `io_teleop/joint_cmd` | 6,076 | 10.001136 ms | 27.140731 ms | CANDIDATE (2.714×) |
| `io_teleop/joint_states` | 6,088 | 10.000260 ms | 17.218928 ms | NO_CANDIDATE (1.722×) |

The command-topic candidate generated a YAML structure with injector `type: joint_state`, `config.drop_probability: 1.0`, a relative start around 17.199 seconds and a 27.141 ms duration. The different topic served as a *threshold-based negative control*, not a proof of sensor health.

## What this does not establish

- No ground-truth failure was provided. A 27 ms logging-time gap might be scheduler or recorder jitter.
- The upstream ROS 2 fault-injection runner was **not** executed against this scenario.
- ROS process startup and recorded timestamps have not been aligned.
- The short window might not inject any messages when executed.
- No failure reproduction, fixed-version pass, repeatability rate, or time savings against manual authoring has been established.

Until these tests are performed, claims of root cause and reproduced incidents are not supported.
