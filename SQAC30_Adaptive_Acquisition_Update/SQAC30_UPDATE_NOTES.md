# SQAC30 and adaptive acquisition update

This bundle updates Tilt Robot V2.0 with the SQAC30 model, nested 26/34/42/50
fit-pose choices, an adaptive electrolytic-sensor homing gate, and a TP3
stability/retry gate.

## Replace these files

- `correctors.py` and `poses.py` in the `accel_cal` package
- `hardware.py`, `leveling.py`, and `sequencer.py` in the `station` package
- `config.py`, `gui.py`, and `station_config.toml` in the application root

The bundle also contains the unchanged supporting source files so it can be
used as a complete source snapshot.

## Homing and electrolytic-sensor phase

The station still performs this order:

1. Mechanically home both rotary stages.
2. Coarse-level with the configured DUT/TP3.
3. If either 005 electrolytic channel is railed, use the existing bounded stage
   sweep to bring both channels into range.
4. Begin the adaptive 005 fine-level loop.

After every fine-level movement, all earlier 005 samples are discarded. The
software requires two consecutive, non-overlapping 10-sample windows. Both
windows must meet the 0.001-degree per-axis SD limit, adjacent window means
must differ by no more than 0.001 degree per axis, and both final window means
must be within +/-0.002 degree on both axes. Each stable-reading request has a
20-second timeout. Failure to settle or converge aborts the campaign.

Once accepted, the resulting rotary coordinates become the plate zero. The 005
sensors are not used at calibration or verification poses.

## TP3 pose acquisition phase

For the configured `dut_slot`, every fit and verification pose now follows this
sequence:

1. Move and confirm both stages are within 0.5 degree of command.
2. Continuously evaluate a rolling window of 25 fresh TP3 samples.
3. Qualify only when X, Y, and Z SD are each at or below 3 mg.
4. Collect 25 entirely new samples for the actual data point.
5. Apply the same 3 mg per-axis SD gate to that final measurement set.

The Teensy `units_seq` counter is used to prevent a cached frame from being
counted as a new sample. A pose that does not qualify within 10 seconds, cannot
collect its final samples within 10 seconds, or fails the final SD/magnitude
check is deferred. The station completes the current sweep before revisiting
deferred poses. Three total attempts are allowed. If any required pose still
fails, the campaign reports `SYSTEM_FAIL`; no partial fit is performed.

Successful and failed attempts log raw samples, timestamps, sequence metadata,
window statistics, attempt number, elapsed time, and failure reason. A failed
campaign writes a `_failed.json` diagnostic file.

## Configuration

All starting thresholds can be tuned in `station_config.toml`:

```toml
[campaign]
n_samples = 25
tp3_sample_period_s = 0.03
tp3_stability_sd_mg = 3.0
tp3_stability_timeout_s = 10.0
tp3_max_attempts = 3

[leveling]
tolerance_deg = 0.002
coarse_tol_g = 0.0022
window_samples = 10
stable_windows = 2
sd_threshold_deg = 0.001
drift_threshold_deg = 0.001
timeout_s = 20.0
sample_period_s = 0.1
```

## SQAC30 and pose sets

The SQAC30 basis remains:

```text
1, own-axis linear, own-axis cubic, two cross-axis linear terms,
x^2-z^2, y^2-z^2, xy, xz, yz
```

The GUI offers nested 26, 34, 42, and 50 fit-pose sets. The independent
12-pose verification set is unchanged.

## Validation performed

- All Python files compile and all configuration values load without warnings.
- Stable, transiently noisy, permanently noisy, and final-window-noisy TP3
  simulations produce the required accept/defer/retry/fail behavior.
- A simulated homing run exercises DUT coarse leveling, both-channel 005 rail
  recovery, axis identification, and stable fine nulling.
- Full mock campaigns pass for 26, 34, 42, and 50 fit poses.
- Freshness testing confirms repeated Teensy sequence numbers are not counted
  twice.
- The recorded 2026-08-07 run still reproduces SQAC30 verification MAE of
  0.835 mg, vector RMSE of 3.479 mg, and maximum fit condition number of 8.04.

Physical hardware validation is still required. The 0.001-degree 005 SD/drift
limits are initial engineering values because the existing run files did not
preserve raw 005 homing samples. The new logs are designed to support later
threshold tuning from actual station data.
