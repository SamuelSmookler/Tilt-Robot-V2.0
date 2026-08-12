from collections import deque
import math
import time

import numpy as np

from accel_cal.simulator import a_true_from_angles
from accel_cal.correctors import Affine12
from accel_cal.geometry import solve_axes, truth_axes
from accel_cal.metrics import verification_metrics


POSITION_TOLERANCE_DEG = 0.5
TP3_STABILITY_SD_G = 0.003
TP3_STABILITY_TIMEOUT_S = 10.0
TP3_MAX_ATTEMPTS = 3
TP3_SAMPLE_PERIOD_S = 0.03


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
    else:
        values = raw
        metadata = {"sequence": None, "sample_us": None, "age_ms": None,
                    "timestamp_s": time.time()}

    try:
        values = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid TP3 sample {values!r}") from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"invalid TP3 vector {values!r}")

    return {"values": values, **metadata}


def _sample_stream(sensor, count, timeout_s, sample_period_s, abort,
                   accept_window=None):
    """Collect fresh samples until count is reached or a window is accepted."""
    started = time.monotonic()
    deadline = started + timeout_s
    next_sample = started
    samples = []
    errors = []
    window = deque(maxlen=count)

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
        window.append(sample["values"])
        if len(window) < count:
            continue

        values = np.asarray(window, dtype=float)
        mean = np.mean(values, axis=0)
        std = np.std(values, axis=0)
        if accept_window is None or accept_window(mean, std):
            return {
                "accepted": True,
                "elapsed_s": time.monotonic() - started,
                "samples": samples,
                "errors": errors,
                "window_values": values,
                "mean": mean,
                "std": std,
            }

    return {
        "accepted": False,
        "elapsed_s": time.monotonic() - started,
        "samples": samples,
        "errors": errors,
        "window_values": None,
        "mean": None,
        "std": None,
    }


def _json_stream(stream, include_all_samples=True):
    return {
        "accepted": bool(stream["accepted"]),
        "elapsed_s": float(stream["elapsed_s"]),
        "sample_count": len(stream["samples"]),
        "samples": stream["samples"] if include_all_samples else [],
        "errors": stream["errors"],
        "mean": (None if stream["mean"] is None
                 else [float(value) for value in stream["mean"]]),
        "std": (None if stream["std"] is None
                else [float(value) for value in stream["std"]]),
    }


def _acquire_pose(motion, sensor, pose, pose_index, attempt, phase,
                  n_samples, stability_sd_g, stability_timeout_s,
                  sample_period_s, skew_x_deg, abort):
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

    # Qualification begins immediately after the blocking move reports idle.
    qualification = _sample_stream(
        sensor, n_samples, stability_timeout_s, sample_period_s, abort,
        accept_window=lambda _mean, std: bool(np.all(std <= stability_sd_g)),
    )
    entry["qualification"] = _json_stream(qualification)
    if not qualification["accepted"]:
        entry["failure_reason"] = (
            f"qualification did not reach {stability_sd_g * 1000:g} mg SD "
            f"within {stability_timeout_s:g} s"
        )
        raise PoseDeferred(entry["failure_reason"], entry)

    # These samples are entirely new; none of the qualification samples are
    # reused in the fitted/verification data point.
    measurement = _sample_stream(
        sensor, n_samples, stability_timeout_s, sample_period_s, abort,
        accept_window=lambda _mean, _std: True,
    )
    entry["measurement"] = _json_stream(measurement)
    if not measurement["accepted"]:
        entry["failure_reason"] = (
            f"could not collect {n_samples} final fresh samples within "
            f"{stability_timeout_s:g} s"
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
                 sample_period_s=TP3_SAMPLE_PERIOD_S, log=print):
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
                    sample_period_s, skew_x_deg, abort,
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
            log(f"{phase} pose {pose_index + 1}/{len(poses)} accepted: SD "
                + ", ".join(f"{value * 1000:.2f} mg"
                            for value in entry["reading_std"]))

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
              sample_period_s=TP3_SAMPLE_PERIOD_S, log=print):
    if model is None:
        model = Affine12()
    tips = None
    if solve_geometry and record is None:
        record = []

    acquisition_args = {
        "abort": abort,
        "record": record,
        "n_samples": n_samples,
        "stability_sd_g": stability_sd_g,
        "stability_timeout_s": stability_timeout_s,
        "max_attempts": max_attempts,
        "sample_period_s": sample_period_s,
        "log": log,
    }
    a_true_fit, a_meas_fit = run_campaign(
        motion, sensor, fit_poses, phase="fit", skew_x_deg=skew_x_deg,
        **acquisition_args,
    )

    if solve_geometry:
        fit_rows = sorted(
            (entry for entry in record
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

    if solve_geometry:
        verify_rows = sorted(
            (entry for entry in record
             if entry["phase"] == "verify" and entry.get("accepted")),
            key=lambda entry: entry["pose_index"],
        )
        a_true_verify = truth_axes(
            np.asarray([entry["measured"] for entry in verify_rows]), tips)

    corrected = model.apply(a_meas_verify)
    raw_error = np.mean(np.abs(a_meas_verify - a_true_verify)) * 1000
    corrected_error = np.mean(np.abs(corrected - a_true_verify)) * 1000
    return {
        "model": model,
        "raw_error_mg": raw_error,
        "corrected_error_mg": corrected_error,
        "skew_x_deg": skew_x_deg,
        "metrics": verification_metrics(corrected, a_true_verify),
        
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
    }
