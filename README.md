# bag2test

bag2test is an experimental project investigating whether incidents recorded in ROS 2 bag files can be turned into reproducible regression tests.

## Status

**Research / prototype stage.** The repository does not yet provide a usable command-line tool, an MCP server, or a validated end-to-end ROS 2 workflow. No real-robot incident reproduction has been demonstrated.

## Proposed workflow

1. Read an MCAP/ROS 2 recording and select a time window containing a suspected failure.
2. Identify observable timing anomalies, such as message gaps, reduced publish rates, or stale samples.
3. Generate *candidate* fault-injection scenarios and assertions for an existing ROS 2 test runner.
4. Execute the candidates in a controlled test environment and compare observed behavior with the recorded incident.
5. Record test evidence, including inconclusive results and failed reproduction attempts.

The first experiments will focus on timing and message-delivery symptoms, rather than arbitrary robot failures.

## Evidence and limitations

A gap in a recorded topic does not establish its cause. For example, a missing LiDAR message may originate in the sensor, driver, network, or recording process.

The project will distinguish:

- **Observed:** an anomaly is present in the input data.
- **Hypothesized:** a proposed mechanism could explain the observation.
- **Symptom reproduced:** a controlled test produces the specified observable failure.
- **Regression verified:** the same test fails against a known faulty baseline and passes against a changed version.
- **Inconclusive:** evidence is missing, contradictory, or insufficient.

Reproducing a symptom is not proof that the original physical root cause has been identified. No generated scenario should be described as verified without execution evidence.

## Validation before expanding scope

The initial feasibility gate is one independently sourced robot recording, a generated executable scenario, repeated reproduction attempts, a negative control, and a comparison against a manually written equivalent test. Results and limitations should be published with the test data's license and provenance.

The intended integrations are [MCAP](https://mcap.dev/), [ros2_fault_injection](https://github.com/reeceholland/ros2_fault_injection), and [replay_testing](https://github.com/polymathrobotics/replay_testing). Integration compatibility has not yet been established.

This project is not a general ROS debugger, a robot controller, or a safety system. It is not intended to run commands on physical robots.

## License

GPL-3.0-or-later. See [LICENSE](LICENSE).
