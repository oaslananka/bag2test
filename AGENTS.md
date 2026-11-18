# Repository guidance

This repository is an early-stage research prototype for deriving candidate ROS 2 regression tests from recorded incidents.

- Read README.md before making changes. Do not claim functionality exists until a working implementation and repeatable tests demonstrate it.
- Keep changes focused on reproducible experiments and evidence. Prefer existing MCAP, ROS 2 and fault-injection tools over writing replacements.
- Separate observable symptoms, candidate causes, successful symptom reproduction and regression verification in code and reporting.
- Treat real-world robot safety as out of scope. Do not add autonomous physical robot actuation.
- Document data origin, license and limitations for any added test recordings; avoid committing unnecessarily large bag files.
- Until a test runner is introduced, report validation as not run rather than inventing test commands or results.
