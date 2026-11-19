# ROS 2 fault-scenario task — participant instructions

You are given a ROS 2 MCAP recording of one selected topic, a
description of the intended input/output topic names and access to the
upstream `ros2_fault_injection` documentation.

Your task is to examine the recording and create a **valid YAML scenario**
that reproduces an observed downstream topic-delivery gap using an
independent, healthy ROS 2 publisher. Include a relevant fault-activation
assertion. Do **not** assume the recording proves the original cause of the
gap or that the recording itself can restore missing messages.

The evaluator will execute your submitted YAML in an isolated ROS 2
environment and independently check the observed message sequence,
the fault window and a zero-drop healthy control. You may revise the
scenario until it passes or the session ends.

**Your available tools will be specified before the task.** In the
manual condition you may use ROS 2 and the upstream injector documentation,
but not bag2test. In the assisted condition you may use those same
resources plus bag2test. Do not look up prewritten study solution YAMLs or
an evaluator's scoring instructions in either condition.

Tell the evaluator when you start working. At submission, describe what
the recorded gap shows, what fault mechanism you chose to simulate, and
what you **cannot** infer about the original robot or publisher.

This task uses a **controlled synthetic recording**, not a physical
robot incident. No real robot connection or hardware interaction is needed.
