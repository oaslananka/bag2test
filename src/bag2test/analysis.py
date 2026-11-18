"""Conservative timing-gap observations from an MCAP message channel."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
from statistics import median

from mcap.exceptions import McapError
from mcap.reader import make_reader

# The upstream injector's ROS message contract is type-specific. Do not
# manufacture scenarios for types that have not been checked against it.
SUPPORTED_TYPES = {
    "sensor_msgs/msg/JointState": "joint_state",
    "nav_msgs/msg/Odometry": "odom",
    "sensor_msgs/msg/LaserScan": "scan",
    "sensor_msgs/msg/Imu": "imu",
    "geometry_msgs/msg/Twist": "twist",
}
MAX_TOPIC_MESSAGES = 250_000
MIN_TOPIC_MESSAGES = 10


class ObservationError(ValueError):
    """Invalid input or unsafe/incompatible message metadata."""


def canonical_type(name: str) -> str:
    """Accept the ROS 2 names seen in both historical and recent MCAP files."""
    parts = name.split("/")
    if len(parts) == 2 and parts[0] in {"sensor_msgs", "nav_msgs", "geometry_msgs"}:
        return f"{parts[0]}/msg/{parts[1]}"
    return name


@dataclass(frozen=True)
class Gap:
    start_ns: int
    end_ns: int
    duration_s: float
    ratio: float
    # A rapid next message is compatible with jitter/batching, not proof.
    following_interval_s: float | None = None
    following_interval_ratio: float | None = None


@dataclass(frozen=True)
class Observation:
    topic: str
    message_type: str
    injector_type: str
    count: int
    bag_start_ns: int
    median_period_s: float | None
    largest_gap: Gap | None
    status: str
    threshold_ratio: float
    start_sec: float | None
    end_sec: float | None
    # True means there is no separate publish-time signal in this MCAP.
    publish_times_equal_log_times: bool | None = None
    # Coverage after the final selected-topic message is observable only
    # when other MCAP messages extend the recording's global endpoint.
    trailing_silence_s: float | None = None
    trailing_silence_ratio: float | None = None
    recording_end_ns: int | None = None


def inspect(
    filename: str | Path,
    topic: str,
    *,
    min_gap_factor: float = 2.5,
    start_sec: float | None = None,
    end_sec: float | None = None,
) -> Observation:
    if not topic:
        raise ObservationError("Topic must not be empty")
    if not (1.0 < min_gap_factor <= 100.0):
        raise ObservationError("min_gap_factor must be > 1 and <= 100")
    if start_sec is not None and start_sec < 0:
        raise ObservationError("start_sec must be non-negative")
    if end_sec is not None and end_sec < 0:
        raise ObservationError("end_sec must be non-negative")
    if start_sec is not None and end_sec is not None and end_sec <= start_sec:
        raise ObservationError("end_sec must be after start_sec")

    path = Path(filename)
    if not path.is_file():
        raise ObservationError(f"MCAP file not found: {path}")

    try:
        with path.open("rb") as stream:
            reader = make_reader(stream)
            summary = reader.get_summary()
            # If the file has no summary, use the first message on the
            # selected channel as the relative timeline origin.
            origin_ns = (
                summary.statistics.message_start_time
                if summary and summary.statistics
                else None
            )
            recording_end_ns = (
                summary.statistics.message_end_time
                if summary and summary.statistics
                else None
            )
            first_topic_ns: int | None = None
            times: list[int] = []
            publish_times_equal_log_times = True
            kind: str | None = None

            for schema, channel, message in reader.iter_messages(
                topics=[topic], log_time_order=True
            ):
                if (
                    schema is None
                    or schema.encoding != "ros2msg"
                    or channel.message_encoding != "cdr"
                ):
                    raise ObservationError("Selected topic is not a ROS 2 CDR channel")
                this_type = canonical_type(schema.name)
                if this_type not in SUPPORTED_TYPES:
                    raise ObservationError(
                        f"Unsupported ROS 2 type {this_type!r}; supported types: "
                        + ", ".join(sorted(SUPPORTED_TYPES))
                    )
                if kind is None:
                    kind = this_type
                elif kind != this_type:
                    raise ObservationError(
                        "Topic changes ROS 2 type inside the recording"
                    )

                if first_topic_ns is None:
                    first_topic_ns = message.log_time
                    if origin_ns is None:
                        origin_ns = first_topic_ns

                if start_sec is not None and message.log_time < origin_ns + int(
                    start_sec * 1e9
                ):
                    continue
                if end_sec is not None and message.log_time > origin_ns + int(
                    end_sec * 1e9
                ):
                    continue
                times.append(message.log_time)
                if message.publish_time != message.log_time:
                    publish_times_equal_log_times = False
                if len(times) > MAX_TOPIC_MESSAGES:
                    raise ObservationError(
                        f"Topic exceeds the first-spike limit of {MAX_TOPIC_MESSAGES} messages"
                    )
    except ObservationError:
        raise
    except (McapError, OSError, EOFError, ValueError, RuntimeError) as exc:
        raise ObservationError(f"Unable to read MCAP: {exc}") from exc

    if kind is None or origin_ns is None:
        raise ObservationError(f"No messages found for topic {topic!r}")

    if len(times) < MIN_TOPIC_MESSAGES:
        return Observation(
            topic,
            kind,
            SUPPORTED_TYPES[kind],
            len(times),
            origin_ns,
            None,
            None,
            "INSUFFICIENT_DATA",
            min_gap_factor,
            start_sec,
            end_sec,
            publish_times_equal_log_times,
        )

    positive = [right - left for left, right in pairwise(times) if right > left]
    if len(positive) < MIN_TOPIC_MESSAGES - 1:
        return Observation(
            topic,
            kind,
            SUPPORTED_TYPES[kind],
            len(times),
            origin_ns,
            None,
            None,
            "INSUFFICIENT_DATA",
            min_gap_factor,
            start_sec,
            end_sec,
            publish_times_equal_log_times,
        )

    period_ns = median(positive)
    largest_idx = max(
        range(len(times) - 1), key=lambda idx: times[idx + 1] - times[idx]
    )
    delta = times[largest_idx + 1] - times[largest_idx]
    ratio = delta / period_ns
    following_ns = (
        times[largest_idx + 2] - times[largest_idx + 1]
        if largest_idx + 2 < len(times)
        else None
    )
    # This is a timestamp symptom only; never assert that messages were lost.
    gap = (
        Gap(
            times[largest_idx],
            times[largest_idx + 1],
            delta / 1e9,
            ratio,
            following_ns / 1e9 if following_ns is not None else None,
            following_ns / period_ns if following_ns is not None else None,
        )
        if ratio >= min_gap_factor
        else None
    )

    trailing_silence_s = None
    trailing_silence_ratio = None
    # A clipped selection is not the entire observation interval. Without
    # a separate external clock, infer neither a trailing outage nor a
    # finite recovery when the selected topic is the last recorded stream.
    if end_sec is None and recording_end_ns is not None:
        remaining_ns = recording_end_ns - times[-1]
        if remaining_ns / period_ns >= min_gap_factor:
            trailing_silence_s = remaining_ns / 1e9
            trailing_silence_ratio = remaining_ns / period_ns

    return Observation(
        topic,
        kind,
        SUPPORTED_TYPES[kind],
        len(times),
        origin_ns,
        period_ns / 1e9,
        gap,
        (
            "CANDIDATE"
            if gap
            else "TRAILING_SILENCE"
            if trailing_silence_s is not None
            else "NO_CANDIDATE"
        ),
        min_gap_factor,
        start_sec,
        end_sec,
        publish_times_equal_log_times,
        trailing_silence_s,
        trailing_silence_ratio,
        recording_end_ns,
    )
