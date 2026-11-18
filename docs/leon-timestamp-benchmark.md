# Recorder-time versus sender-time: paired ROS 2 counterexample

**Source:** Universidad de León, [*Simulated attacks Rosbags against mobile robot in ROS 2*](https://zenodo.org/records/17649537), Experiment 1 (2025-11-19). Author: David Sobrín Hidalgo. License: CC BY 4.0 (Zenodo record metadata). The original labels are **E1-1 clean** and **E1-2 cmd_vel-flood attack**. These are real ROS 2 recordings of an attack experiment, **not evidence that a physical laser sensor broke**.

## Failure mode

A ROS 2 bag recorder may stall and then batch messages. Its receive or `log_time` is not necessarily the sensor's own sampling timeline. An [independent investigation](https://github.com/thrmnn/ros2-localization-triage/blob/main/docs/finding-recorder-artifacts.md) previously found this defect in its laser-gap detector and corrected it using `header.stamp`. The issue and remedy are **not novel to bag2test**.

Zenodo `Experiment_1.zip` is 1,644,570 bytes, MD5 `e8adc1796821264f39580f6aa97c6388`, with paired **native ROS 2 SQLite3** recordings. We converted outside this repository using `rosbags==0.11.9` to ROS 2 CDR/MCAP. We compared each original `/scan` SQLite3 message's timestamp and serialized CDR bytes with its converted MCAP counterpart in order; **all 484 clean and 208 attack messages matched exactly**. No payloads or timestamps were edited.

## Results

The independently frozen source policy uses a **4× median** scan-gap threshold. We used that identical setting on both paired recordings and both clock bases. It is not a universally safe threshold; bag2test's preexisting default is 2.5×.

| Source | Scan samples | Recorder median / max gap | Header median / max gap | `log` at 4× | `header` at 4× |
| --- | ---: | --- | --- | --- | --- |
| E1-1, clean | 484 | 0.000981 / **27.131 s** | 0.066724 / **0.200 s** | **CANDIDATE, false alarm** | **NO_CANDIDATE** |
| E1-2, attack | 208 | 0.000957 / **13.712 s** | 0.066720 / **22.667 s** | CANDIDATE, clock ambiguous | **CANDIDATE** |

With header time at 4×, the clean case emits only `evidence.json`. The attack case emits `evidence.json` plus a **hypothetical, unverified** `scenario.yaml`. The 22.667-second sensor-stamped interval means recorded scan data is absent in that interval, **not** that a specific physical sensor failed or that downstream topic-drop injection can reproduce the original network flooding mechanism. At bag2test's 2.5× default, even header time can flag a 0.200-second gap; **clock selection alone does not replace a calibrated threshold**.

## Reproduce (optional, data not committed)

```bash
mkdir -p /tmp/bag2test-leon && cd /tmp/bag2test-leon
curl -fsSL https://zenodo.org/api/records/17649537/files/Experiment_1.zip/content -o Experiment_1.zip
echo 'e8adc1796821264f39580f6aa97c6388  Experiment_1.zip' | md5sum -c -
unzip -q Experiment_1.zip
python3 -m venv convert-env
convert-env/bin/pip install 'rosbags==0.11.9'
for case in E1-1 E1-2; do
  convert-env/bin/rosbags-convert --src "$case" --dst "$case-mcap" --dst-storage mcap
done
# With bag2test installed or run from your bag2test project venv:
bag2test E1-1-mcap/E1-1-mcap.mcap --topic /scan \
  --time-basis header --min-gap-factor 4 --out clean-report
bag2test E1-2-mcap/E1-2-mcap.mcap --topic /scan \
  --time-basis header --min-gap-factor 4 --out attack-report
```

Converted MCAP SHA-256 (rosbags 0.11.9):

- Clean: `313f5b8d46e950259bf8e29fceef41c9f069211db30aae611524b5b5363eb81e`
- Attack: `fa9151e3f7d60850125955bc387965ec89159f81c66a1ccee3c922a07de07c77`

These hashes identify precisely what was analyzed. If another converter version produces different files, recheck its timestamps and raw CDR bytes before claiming the same result.

## Interpretation and scope

`--time-basis log` remains the default and means MCAP recorder `log_time`. `--time-basis header` extracts only the `std_msgs/Header.stamp` from supported ROS 2 CDR message types (JointState, Odometry, LaserScan, Imu). Unstamped types (Twist), invalid/zero headers, unsupported CDR encodings and non-monotonic sender stamps are **rejected**, not silently substituted. Window bounds and terminal observation coverage still use MCAP log-time.

For a header-based candidate, the proposed YAML start maps elapsed sender time relative to the first selected message's observed recorder arrival; **inter-clock alignment is not independently verified**, so the engineer must align against a separate ROS runtime. No physical robot incident was reproduced, no original cause or false-negative rate inferred, and no measured human engineering-time advantage was demonstrated. This is **a bounded negative-control timing correctness check**, not validation of the whole product.
