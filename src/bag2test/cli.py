"""Command-line entry point."""

from __future__ import annotations

import argparse
import sys

from .analysis import ObservationError, inspect
from .scenario import write_results


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="bag2test",
        description="Derive an unverified fault-injection candidate from MCAP message timing.",
    )
    parser.add_argument("mcap", help="Path to a local MCAP recording")
    parser.add_argument(
        "--topic", required=True, help="Exact channel topic name in the MCAP"
    )
    parser.add_argument(
        "--out", required=True, help="New output directory (must not exist)"
    )
    parser.add_argument("--min-gap-factor", type=float, default=2.5)
    parser.add_argument("--start-sec", type=float, default=None)
    parser.add_argument("--end-sec", type=float, default=None)
    parser.add_argument(
        "--input-topic", default=None, help="Explicit upstream raw ROS 2 topic"
    )
    args = parser.parse_args(argv)
    try:
        observation = inspect(
            args.mcap,
            args.topic,
            min_gap_factor=args.min_gap_factor,
            start_sec=args.start_sec,
            end_sec=args.end_sec,
        )
        files = write_results(observation, args.out, input_topic=args.input_topic)
    except ObservationError as exc:
        print(f"bag2test: {exc}", file=sys.stderr)
        return 2
    print(f"Observation: {observation.status} ({observation.count} messages)")
    for path in files:
        print(f"Wrote: {path}")
    if observation.largest_gap:
        print("Candidate only: symptom reproduction and regression NOT VERIFIED")
    else:
        print("No test scenario generated")
    return 0


if __name__ == "__main__":
    sys.exit(main())
