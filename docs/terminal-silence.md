# End-of-recording topic silence

When an IMU stops publishing and never recovers during a recording, the
last recorded IMU message has no later same-topic message to form a
closed time interval. This can occur in the documented BNO055 failure
scenario (see [the upstream regression](upstream-imu-regression.md)).

## Evidence and behavior

On an MCAP with a valid summary, bag2test compares the final selected-topic
`log_time` against the **global last recorded message**. A later last
message proves that *some other recorded channel* continued to supply
events, but does not prove the robot or sensor was operational.

For a topic with at least 10 timestamp observations and a reliable
positive median cadence, a relative terminal interval above
`--min-gap-factor` produces:

- `TRAILING_SILENCE` status when there was no qualifying internal gap
- `trailing_silence_s`, `trailing_silence_ratio`, and `recording_end_ns`
  fields in `evidence.json`
- A CLI warning about observed coverage and an explicit statement
  that physical cause remains `NOT VERIFIED`

A trailing-only observation **never produces `scenario.yaml`**, because
there is no observed recovery or known bounded fault duration. If
the same recording contains a qualifying *internal* gap, the prior
`CANDIDATE` status and corresponding YAML are preserved; terminal
coverage is supplementary evidence.

Insufficient selected-topic messages, absence of a later message on
another channel, or a clipped `--end-sec` window do not establish a
terminal event. The reported duration is measured only to the final
recorded MCAP timestamp; the actual length of the real-world fault
may be shorter or longer.

## Reproducible demonstration

`tests/test_observation.py` constructs a small synthetic **two-topic
MCAP**: a 100 Hz `/bno055/imu` stream ending at 150 ms, and a separate
heartbeat channel with a final message at 2,000 ms. Bag2test reports a
1.850-second terminal interval (185× the selected topic's 10 ms median
period), `TRAILING_SILENCE`, and only `evidence.json`.

```bash
python -m pytest tests/test_observation.py -q
```

Other deterministic tests check that single-topic recordings and earlier
independent messages cannot create a false terminal report; sparse data
and clipped windows remain inconclusive; an internal gap keeps its
original bounded YAML; and a higher threshold suppresses a smaller
terminal interval.

## Limits

This is observation of **recording coverage**, not diagnosis. A missing
publisher may be intentional; message recording may have ceased for
unrelated reasons; and MCAP log timestamps alone do not verify the
publisher's clock. A stopped sensor in a single-topic file is
fundamentally unobservable beyond that file's last timestamp.

The real ROS 2 BNO055 source regression under a virtual UART is
**separate** from this synthetic MCAP fixture; these tests do not
prove that bag2test's generated message-drop YAML triggers the internal
driver failure. ROSBag Doctor and Backplane Tracecheck already
provide coverage checks, so this closes a false negative rather
than claiming a new class of anomaly detection.

A labeled field incident, actual erroneous-vs-fixed behavior driven
by its recorded evidence, and human time savings remain unverified.
