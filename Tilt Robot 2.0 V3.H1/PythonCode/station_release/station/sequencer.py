from collections import deque
import math
import time

import numpy as np

from accel_cal.simulator import a_true_from_angles
from accel_cal.correctors import Affine12
from accel_cal.geometry import solve_axes, truth_axes
from accel_cal.metrics import (
    DEFAULT_VERIFICATION_THRESHOLDS,
    evaluate_verification,
    verification_metrics,
)


POSITION_TOLERANCE_DEG = 0.5
TP3_STABILITY_SD_G = 0.003
TP3_STABILITY_TIMEOUT_S = 10.0
TP3_MAX_ATTEMPTS = 3
TP3_SAMPLE_PERIOD_S = 0.03
TP3_MIN_SETTLE_S = 0.5
TP3_STABLE_WINDOWS = 2
TP3_MEAN_DRIFT_G = 0.001


class PoseDeferred(RuntimeError):
    """A pose measurement was unstable and should be retried after the sweep."""

    def __init__(self, message, entry):
        super().__init__(message)
        self.entry = entry


class AcquisitionFailure(RuntimeError):
    """One or more required poses failed every permitted attempt."""

    def __init__(self, phase, failed_poses, attempts):
        details = ", ".join(
            f"#{index + 1} ({outer:.3f}, {inner:.3f})"
            for index, (outer, inner) in failed_poses
        )
        super().__init__(f"{phase} acquisition failed after {attempts} attempts: {details}")
        self.phase = phase
        self.failed_poses = list(failed_poses)
        self.attempts = attempts


def _wrap180(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


def _check_abort(abort):
    if abort is not None and abort():
        raise RuntimeError("aborted by operator")


def _sleep_until(target, abort):
    while True:
        _check_abort(abort)
        remaining = target - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(remaining, 0.05))


def _fresh_sample(sensor):
    """Normalize real and simulated sensors to a timestamped sample record."""
    if hasattr(sensor, "read_sample"):
        raw = sensor.read_sample()
    else:
        raw = sensor.read()

    if isinstance(raw, dict) and "values" in raw:
        values = raw["values"]
        metadata = {key: raw.get(key) for key in
                    ("sequence", "sample_us", "age_ms", "timestamp_s")}
        pitch_roll_deg = raw.get("pitch_roll_deg")
    else:
        values = raw
        metadata = {"sequence": None, "sample_us": None, "age_ms": None,
                    "timestamp_s": time.time()}
        pitch_roll_deg = None

    try:
        values = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid TP3 sample {values!r}") from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"invalid TP3 vector {values!r}")

    if pitch_roll_deg is not None:
        try:
            pitch_roll_deg = [float(value) for value in pitch_roll_deg]
        except (TypeError, ValueError):
            pitch_roll_deg = None
        if (pitch_roll_deg is not None
                and (len(pitch_roll_deg) != 2
                     or not all(math.isfinite(value)
                                for value in pitch_roll_deg))):
            pitch_roll_deg = None

    return {
        "values": values,
        "pitch_roll_deg": pitch_roll_deg,
        **metadata,
    }


def _accepted_window_pitch_roll_deg(stream):
    """Circular-mean native [pitch, roll] over the accepted final window."""
    sample_range = stream.get("window_sample_range")
    if sample_range is None:
        return None
    first, last = sample_range
    selected = stream["samples"][first:last + 1]
    values = []
    for sample in selected:
        pitch_roll = sample.get("pitch_roll_deg")
        if (pitch_roll is None or len(pitch_roll) != 2
                or not all(math.isfinite(float(value))
                           for value in pitch_roll)):
            return None
        values.append([float(value) for value in pitch_roll])
    if not values:
        return None

    radians = np.radians(np.asarray(values, dtype=float))
    sine = np.mean(np.sin(radians), axis=0)
    cosine = np.mean(np.cos(radians), axis=0)
    if np.any(np.hypot(sine, cosine) < 1e-12):
        return None
    return [float(value) for value in np.degrees(np.arctan2(sine, cosine))]


def _window_stats(samples):
    """Return per-axis statistics for one sample window."""
    values = np.asarray([sample["values"] for sample in samples], dtype=float)
    mean = np.mean(values, axis=0)
    std = np.std(values, axis=0)
    peak_to_peak = np.ptp(values, axis=0)

    elapsed = np.asarray([sample["elapsed_s"] for sample in samples], dtype=float)
    centered_time = elapsed - np.mean(elapsed)
    denominator = float(np.dot(centered_time, centered_time))
    if denominator > 0.0:
        slope = np.sum(
            centered_time[:, None] * (values - mean), axis=0
        ) / denominator
    else:
        slope = np.zeros(3, dtype=float)

    return {
        "values": values,
        "mean": mean,
        "std": std,
        "peak_to_peak": peak_to_peak,
        "slope": slope,
    }


def _sample_stream(sensor, count, timeout_s, sample_period_s, abort,
                   accept_window=None):
    """Collect fresh samples until count is reached or a window is accepted."""
    started = time.monotonic()
    deadline = started + timeout_s
    next_sample = started
    samples = []
    errors = []
    window = deque(maxlen=count)
    last_stats = None
    last_window_range = None

    while time.monotonic() < deadline:
        _check_abort(abort)
        _sleep_until(min(next_sample, deadline), abort)
        if time.monotonic() >= deadline:
            break
        next_sample = max(next_sample + sample_period_s, time.monotonic())
        try:
            sample = _fresh_sample(sensor)
        except Exception as exc:
            errors.append({"elapsed_s": time.monotonic() - started,
                           "error": str(exc)})
            continue

        sample["elapsed_s"] = time.monotonic() - started
        samples.append(sample)
        window.append(sample)
        if len(window) < count:
            continue

        last_stats = _window_stats(window)
        last_window_range = [len(samples) - count, len(samples) - 1]
        if (accept_window is None
                or accept_window(last_stats["mean"], last_stats["std"])):
            return {
                "accepted": True,
                "elapsed_s": time.monotonic() - started,
                "samples": samples,
                "errors": errors,
                "window_values": last_stats["values"],
                "window_sample_range": last_window_range,
                "mean": last_stats["mean"],
                "std": last_stats["std"],
                "peak_to_peak": last_stats["peak_to_peak"],
                "slope": last_stats["slope"],
            }

    return {
        "accepted": False,
        "elapsed_s": time.monotonic() - started,
        "samples": samples,
        "errors": errors,
        "window_values": None if last_stats is None else last_stats["values"],
        "window_sample_range": last_window_range,
        "mean": None if last_stats is None else last_stats["mean"],
        "std": None if last_stats is None else last_stats["std"],
        "peak_to_peak": (None if last_stats is None
                         else last_stats["peak_to_peak"]),
        "slope": None if last_stats is None else last_stats["slope"],
    }


def _qualification_stream(sensor, count, required_windows, timeout_s,
                          sample_period_s, stability_sd_g, mean_drift_g,
                          abort):
    """Require consecutive, non-overlapping stable windows before acquisition."""
    if required_windows < 2:
        raise ValueError("required_windows must be at least 2")

    started = time.monotonic()
    deadline = started + timeout_s
    next_sample = started
    samples = []
    errors = []
    current_window = []
    windows = []
    stable_chain = []
    last_stats = None

    while time.monotonic() < deadline:
        _check_abort(abort)
        _sleep_until(min(next_sample, deadline), abort)
        if time.monotonic() >= deadline:
            break
        next_sample = max(next_sample + sample_period_s, time.monotonic())
        try:
            sample = _fresh_sample(sensor)
        except Exception as exc:
            errors.append({"elapsed_s": time.monotonic() - started,
                           "error": str(exc)})
            continue

        sample["elapsed_s"] = time.monotonic() - started
        samples.append(sample)
        current_window.append(sample)
        if len(current_window) < count:
            continue

        last_stats = _window_stats(current_window)
        window_index = len(windows)
        sd_pass = bool(np.all(last_stats["std"] <= stability_sd_g))
        if stable_chain:
            previous_mean = windows[stable_chain[-1]]["_mean"]
            mean_change = np.abs(last_stats["mean"] - previous_mean)
            drift_pass = bool(np.all(mean_change <= mean_drift_g))
        else:
            mean_change = None
            drift_pass = True

        if sd_pass and drift_pass:
            stable_chain.append(window_index)
        elif sd_pass:
            # A low-noise window that moved too far becomes the start of a
            # new candidate chain.
            stable_chain = [window_index]
        else:
            stable_chain = []

        windows.append({
            "index": window_index,
            "sample_range": [len(samples) - count, len(samples) - 1],
            "started_elapsed_s": float(current_window[0]["elapsed_s"]),
            "ended_elapsed_s": float(current_window[-1]["elapsed_s"]),
            "mean": [float(value) for value in last_stats["mean"]],
            "std": [float(value) for value in last_stats["std"]],
            "peak_to_peak_mg": [
                float(value * 1000.0) for value in last_stats["peak_to_peak"]
            ],
            "slope_mg_per_s": [
                float(value * 1000.0) for value in last_stats["slope"]
            ],
            "sd_pass": sd_pass,
            "mean_change_mg": (None if mean_change is None else [
                float(value * 1000.0) for value in mean_change
            ]),
            "drift_pass": drift_pass,
            "consecutive_pass_count": len(stable_chain),
            "_mean": last_stats["mean"],
        })
        current_window = []

        if len(stable_chain) >= required_windows:
            for window_record in windows:
                window_record.pop("_mean", None)
            return {
                "accepted": True,
                "elapsed_s": time.monotonic() - started,
                "samples": samples,
                "errors": errors,
                "windows": windows,
                "stable_window_indices": stable_chain[-required_windows:],
                "window_values": last_stats["values"],
                "window_sample_range": windows[-1]["sample_range"],
                "mean": last_stats["mean"],
                "std": last_stats["std"],
                "peak_to_peak": last_stats["peak_to_peak"],
                "slope": last_stats["slope"],
            }

    for window_record in windows:
        window_record.pop("_mean", None)
    return {
        "accepted": False,
        "elapsed_s": time.monotonic() - started,
        "samples": samples,
        "errors": errors,
        "windows": windows,
        "stable_window_indices": [],
        "window_values": None if last_stats is None else last_stats["values"],
        "window_sample_range": (None if not windows
                                else windows[-1]["sample_range"]),
        "mean": None if last_stats is None else last_stats["mean"],
        "std": None if last_stats is None else last_stats["std"],
        "peak_to_peak": (None if last_stats is None
                         else last_stats["peak_to_peak"]),
        "slope": None if last_stats is None else last_stats["slope"],
    }


def _json_stream(stream, include_all_samples=True):
    result = {
        "accepted": bool(stream["accepted"]),
        "elapsed_s": float(stream["elapsed_s"]),
        "sample_count": len(stream["samples"]),
        "samples": stream["samples"] if include_all_samples else [],
        "errors": stream["errors"],
        "window_sample_range": stream.get("window_sample_range"),
        "mean": (None if stream["mean"] is None
                 else [float(value) for value in stream["mean"]]),
        "std": (None if stream["std"] is None
                else [float(value) for value in stream["std"]]),
        "peak_to_peak_mg": (
            None if stream.get("peak_to_peak") is None else
            [float(value * 1000.0) for value in stream["peak_to_peak"]]
        ),
        "slope_mg_per_s": (
            None if stream.get("slope") is None else
            [float(value * 1000.0) for value in stream["slope"]]
        ),
    }
    if "windows" in stream:
        result["windows"] = stream["windows"]
        result["stable_window_indices"] = stream["stable_window_indices"]
    return result


def _acquire_pose(motion, sensor, pose, pose_index, attempt, phase,
                  n_samples, stability_sd_g, stability_timeout_s,
                  sample_period_s, skew_x_deg, abort, min_settle_s,
                  stable_windows, mean_drift_g):
    outer, inner = pose
    entry = {
        "phase": phase,
        "pose_index": pose_index,
        "attempt": attempt,
        "accepted": False,
        "commanded": [outer, inner],
        "started_timestamp_s": time.time(),
    }

    motion.move("OUTER", outer)
    motion.move("INNER", inner)
    st = motion.status()
    act_o = st.get("outer", outer)
    act_i = st.get("inner", inner)
    entry["measured"] = [act_o, act_i]
    if (abs(_wrap180(act_o - outer)) > POSITION_TOLERANCE_DEG
            or abs(_wrap180(act_i - inner)) > POSITION_TOLERANCE_DEG):
        raise RuntimeError(f"pose mismatch: commanded ({outer},{inner}) "
                           f"but stage at ({act_o:.2f},{act_i:.2f})")

    # The final blocking move has reported IDLE. Give mechanical vibration a
    # fixed minimum time to decay before any qualification sample is used.
    entry["stage_idle_timestamp_s"] = time.time()
    settle_started = time.monotonic()
    _sleep_until(settle_started + min_settle_s, abort)
    entry["post_idle_settle"] = {
        "minimum_s": float(min_settle_s),
        "elapsed_s": float(time.monotonic() - settle_started),
    }

    qualification = _qualification_stream(
        sensor, n_samples, stable_windows, stability_timeout_s,
        sample_period_s, stability_sd_g, mean_drift_g, abort,
    )
    entry["qualification"] = _json_stream(qualification)
    if not qualification["accepted"]:
        entry["failure_reason"] = (
            f"qualification did not reach {stable_windows} consecutive "
            f"{n_samples}-sample windows at <= "
            f"{stability_sd_g * 1000:g} mg SD and <= "
            f"{mean_drift_g * 1000:g} mg mean change within "
            f"{stability_timeout_s:g} s"
        )
        raise PoseDeferred(entry["failure_reason"], entry)

    # These samples are entirely new; none of the qualification samples are
    # reused in the fitted/verification data point.
    qualification_mean = qualification["mean"]
    measurement = _sample_stream(
        sensor, n_samples, stability_timeout_s, sample_period_s, abort,
        accept_window=lambda mean, std: bool(
            np.all(std <= stability_sd_g)
            and np.all(np.abs(mean - qualification_mean) <= mean_drift_g)
        ),
    )
    entry["measurement"] = _json_stream(measurement)
    native_pitch_roll_deg = _accepted_window_pitch_roll_deg(measurement)
    entry["measurement"]["native_pitch_roll_deg"] = native_pitch_roll_deg
    entry["native_pitch_roll_deg"] = native_pitch_roll_deg
    if measurement["mean"] is not None:
        mean_change = np.abs(measurement["mean"] - qualification_mean)
        entry["measurement"]["mean_change_from_qualification_mg"] = [
            float(value * 1000.0) for value in mean_change
        ]
        entry["measurement"]["sd_pass"] = bool(
            np.all(measurement["std"] <= stability_sd_g)
        )
        entry["measurement"]["mean_drift_pass"] = bool(
            np.all(mean_change <= mean_drift_g)
        )
    else:
        entry["measurement"]["mean_change_from_qualification_mg"] = None
        entry["measurement"]["sd_pass"] = False
        entry["measurement"]["mean_drift_pass"] = False
    if not measurement["accepted"]:
        entry["failure_reason"] = (
            f"final {n_samples}-sample window did not reach <= "
            f"{stability_sd_g * 1000:g} mg SD and <= "
            f"{mean_drift_g * 1000:g} mg mean change from qualification "
            f"within {stability_timeout_s:g} s"
        )
        raise PoseDeferred(entry["failure_reason"], entry)

    reading = measurement["mean"]
    noise = measurement["std"]
    if not np.all(noise <= stability_sd_g):
        entry["failure_reason"] = (
            "final sample SD exceeded threshold: "
            + ", ".join(f"{value * 1000:.3f} mg" for value in noise)
        )
        raise PoseDeferred(entry["failure_reason"], entry)

    magnitude = float(np.linalg.norm(reading))
    if not 0.8 < magnitude < 1.2:
        entry["failure_reason"] = f"mean magnitude {magnitude:.3f} g is not physical"
        raise PoseDeferred(entry["failure_reason"], entry)

    entry.update({
        "accepted": True,
        "reading": [float(value) for value in reading],
        "reading_std": [float(value) for value in noise],
        "magnitude_g": magnitude,
        "completed_timestamp_s": time.time(),
    })
    truth = a_true_from_angles(act_o, act_i, skew_x_deg)
    return entry, truth, reading


def run_campaign(motion, sensor, poses, abort=None, record=None, phase="fit",
                 n_samples=25, skew_x_deg=0.0,
                 stability_sd_g=TP3_STABILITY_SD_G,
                 stability_timeout_s=TP3_STABILITY_TIMEOUT_S,
                 max_attempts=TP3_MAX_ATTEMPTS,
                 sample_period_s=TP3_SAMPLE_PERIOD_S, log=print,
                 min_settle_s=TP3_MIN_SETTLE_S,
                 stable_windows=TP3_STABLE_WINDOWS,
                 mean_drift_g=TP3_MEAN_DRIFT_G):
    """Acquire all required poses, retrying unstable poses only after a sweep."""
    pending = list(enumerate(poses))
    accepted = []

    for attempt in range(1, max_attempts + 1):
        if attempt > 1:
            log(f"{phase}: retry sweep {attempt}/{max_attempts} for "
                f"{len(pending)} deferred pose(s)")
        deferred = []
        for pose_index, pose in pending:
            _check_abort(abort)
            try:
                entry, truth, reading = _acquire_pose(
                    motion, sensor, pose, pose_index, attempt, phase,
                    n_samples, stability_sd_g, stability_timeout_s,
                    sample_period_s, skew_x_deg, abort, min_settle_s,
                    stable_windows, mean_drift_g,
                )
            except PoseDeferred as exc:
                if record is not None:
                    record.append(exc.entry)
                deferred.append((pose_index, pose))
                log(f"{phase} pose {pose_index + 1}/{len(poses)} deferred "
                    f"(attempt {attempt}/{max_attempts}): {exc}")
                continue

            if record is not None:
                record.append(entry)
            accepted.append((pose_index, truth, reading))
            final_shift = entry["measurement"][
                "mean_change_from_qualification_mg"
            ]
            log(f"{phase} pose {pose_index + 1}/{len(poses)} accepted: SD "
                + ", ".join(f"{value * 1000:.2f} mg"
                            for value in entry["reading_std"])
                + "; mean shift "
                + ", ".join(f"{value:.2f} mg" for value in final_shift))

        pending = deferred
        if not pending:
            break

    if pending:
        raise AcquisitionFailure(phase, pending, max_attempts)

    # Restore the caller's pose order even when some poses were acquired later.
    accepted.sort(key=lambda item: item[0])
    return (np.asarray([item[1] for item in accepted]),
            np.asarray([item[2] for item in accepted]))


def calibrate(motion, sensor, fit_poses, verify_poses, model=None, abort=None,
              record=None, n_samples=25, skew_x_deg=0.0, solve_geometry=False,
              stability_sd_g=TP3_STABILITY_SD_G,
              stability_timeout_s=TP3_STABILITY_TIMEOUT_S,
              max_attempts=TP3_MAX_ATTEMPTS,
              sample_period_s=TP3_SAMPLE_PERIOD_S, log=print,
              min_settle_s=TP3_MIN_SETTLE_S,
              stable_windows=TP3_STABLE_WINDOWS,
              mean_drift_g=TP3_MEAN_DRIFT_G,
              verification_thresholds=None,
              roll_main_max_abs_pitch_deg=45.0):
    if model is None:
        model = Affine12()
    tips = None
    campaign_record = [] if record is None else record

    acquisition_args = {
        "abort": abort,
        "record": campaign_record,
        "n_samples": n_samples,
        "stability_sd_g": stability_sd_g,
        "stability_timeout_s": stability_timeout_s,
        "max_attempts": max_attempts,
        "sample_period_s": sample_period_s,
        "min_settle_s": min_settle_s,
        "stable_windows": stable_windows,
        "mean_drift_g": mean_drift_g,
        "log": log,
    }
    a_true_fit, a_meas_fit = run_campaign(
        motion, sensor, fit_poses, phase="fit", skew_x_deg=skew_x_deg,
        **acquisition_args,
    )

    if solve_geometry:
        fit_rows = sorted(
            (entry for entry in campaign_record
             if entry["phase"] == "fit" and entry.get("accepted")),
            key=lambda entry: entry["pose_index"],
        )
        angles = np.asarray([entry["measured"] for entry in fit_rows])
        tips = solve_axes(angles, a_meas_fit)
        # skew_x_deg = float(tips[3]) (we calculate outer and inner axis tipping now instead of just one skew )
        a_true_fit = truth_axes(angles, tips)

    # Fitting occurs only after every required fit pose has been accepted.
    model.fit(a_meas_fit, a_true_fit)

    a_true_verify, a_meas_verify = run_campaign(
        motion, sensor, verify_poses, phase="verify", skew_x_deg=skew_x_deg,
        **acquisition_args,
    )

    verify_rows = sorted(
        (entry for entry in campaign_record
         if entry["phase"] == "verify" and entry.get("accepted")),
        key=lambda entry: entry["pose_index"],
    )

    if solve_geometry:
        a_true_verify = truth_axes(
            np.asarray([entry["measured"] for entry in verify_rows]), tips)

    corrected = model.apply(a_meas_verify)
    raw_error = np.mean(np.abs(a_meas_verify - a_true_verify)) * 1000
    corrected_error = np.mean(np.abs(corrected - a_true_verify)) * 1000
    measured_pitch_roll = np.asarray([
        (entry["native_pitch_roll_deg"]
         if entry.get("native_pitch_roll_deg") is not None
         else [np.nan, np.nan])
        for entry in verify_rows
    ], dtype=float)
    metrics = verification_metrics(
        corrected,
        a_true_verify,
        measured_pitch_roll_deg=measured_pitch_roll,
        roll_main_max_abs_pitch_deg=roll_main_max_abs_pitch_deg,
    )
    limits = (dict(DEFAULT_VERIFICATION_THRESHOLDS)
              if verification_thresholds is None
              else dict(verification_thresholds))
    verification_report = evaluate_verification(metrics, limits)

    # Store the exact reference, measured angle, wrapped error, and roll region
    # beside each accepted verification pose for auditability.
    for entry, pose_report in zip(
            verify_rows, metrics["pitch_roll_by_pose"]):
        entry["verification_pitch_roll"] = dict(pose_report)

    return {
        "model": model,
        "raw_error_mg": raw_error,
        "corrected_error_mg": corrected_error,
        "skew_x_deg": skew_x_deg,
        "metrics": metrics,
        "verification_report": verification_report,

        "axis_geometry_deg": None if tips is None else {
            "xy_nonorthogonality": float(tips[0]),
            "outer_z_tip": float(tips[1]),
            "inner_z_tip": float(tips[3]),
            "yaw_gauge": "inner_x_zero",

            "truth_axis_components": {
                "outer_y": float(tips[0]),
                "outer_z": float(tips[1]),
                "inner_x": float(tips[2]),
                "inner_z": float(tips[3]),
            },
        },

        # Keep this temporarily for compatibility with any older code
        # that expects the original four-element array.
        "axis_tips_deg": None if tips is None else [
            float(tips[0]),
            float(tips[1]),
            float(tips[2]),
            float(tips[3]),
        ],
    }
