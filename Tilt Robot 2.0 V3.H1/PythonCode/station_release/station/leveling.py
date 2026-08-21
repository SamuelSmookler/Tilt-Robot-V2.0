import json
import math
import os
import time
from pathlib import Path

SENSOR_RANGE_DEG = 0.5
LINEAR_RANGE_DEG = 0.125
IN_RANGE_LIMIT   = 0.45
RAIL_RECOVERY_LIMIT_DEG = 0.35
TOLERANCE_DEG    = 0.002
MAX_STEPS        = 15

STABILITY_WINDOW_SAMPLES = 10
STABLE_WINDOWS           = 2
STABILITY_SD_DEG         = 0.001
STABILITY_DRIFT_DEG      = 0.001
STABILITY_TIMEOUT_S      = 20.0
STABILITY_SAMPLE_PERIOD_S = 0.1

LEVEL_AXIS = {"OUTER": "x", "INNER": "y"}

FINE_NUDGE_DEG   = 0.10
FINE_GAIN_IDEAL  = 1.0
MIN_RESPONSE_G   = 0.002
DEAD_CHANNEL_SPAN_DEG = 0.002

RECOVERY_SWEEP_RADIUS_DEG    = 10.0
RECOVERY_SWEEP_SPEED_DEG_S   = 0.5
RECOVERY_DETECTION_LIMIT_DEG = 0.42
RECOVERY_DETECTION_SAMPLES   = 3
RECOVERY_START_SETTLE_S      = 1.0
RECOVERY_CONFIRM_SETTLE_S    = 1.5
RECOVERY_CONFIRM_SAMPLES     = 3
RECOVERY_SAMPLE_PERIOD_S     = 0.1
RECOVERY_LOCAL_STEP_DEG      = 0.25
RECOVERY_LOCAL_RADIUS_DEG    = 1.0
RECOVERY_SEARCH_LIMIT_DEG    = 130.0
SAVED_NULL_LIMIT_DEG         = (
    RECOVERY_SEARCH_LIMIT_DEG - RECOVERY_SWEEP_RADIUS_DEG
)


class Aborted(RuntimeError):
    pass


class NoResponse(RuntimeError):
    pass


class NotInRange(RuntimeError):
    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


class Unstable(NoResponse):
    """The 005 did not produce enough stable, in-range windows in time."""

    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


def _check(abort):
    if abort is not None and abort():
        raise Aborted("stopped by operator")


def _span(values):
    return (max(values) - min(values)) if values else 0.0


def _finite_float(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_level_state(payload):
    """Validate and normalize a persisted last-good 005 null record."""
    if not isinstance(payload, dict):
        raise ValueError("level state must be a JSON object")
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported level-state schema")
    offsets = payload.get("offsets_deg")
    if not isinstance(offsets, dict):
        raise ValueError("level state has no offsets_deg object")
    normalized_offsets = {
        axis: _finite_float(offsets.get(axis), f"offsets_deg.{axis}")
        for axis in ("outer", "inner")
    }
    if any(abs(value) > SAVED_NULL_LIMIT_DEG
           for value in normalized_offsets.values()):
        raise ValueError(
            f"saved null must remain within +/-{SAVED_NULL_LIMIT_DEG:g} deg "
            "so its full +/-10 deg recovery sweep stays fixture-safe"
        )

    mapping = payload.get("tilt_mapping")
    if mapping is not None:
        if (not isinstance(mapping, dict)
                or set(mapping) != {"OUTER", "INNER"}
                or set(mapping.values()) != {"x", "y"}):
            raise ValueError("saved tilt_mapping must uniquely map OUTER/INNER to x/y")
        mapping = dict(mapping)

    normalized = dict(payload)
    normalized["offsets_deg"] = normalized_offsets
    normalized["tilt_mapping"] = mapping
    return normalized


def load_level_state(path):
    """Load the last successful null; return None when no state exists yet."""
    path = Path(path)
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8") as stream:
        return validate_level_state(json.load(stream))


def save_level_state(path, payload):
    """Atomically replace the last-good null so a partial write is never used."""
    payload = validate_level_state(payload)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def is_saturated(reading):
    return reading is None or abs(reading) >= IN_RANGE_LIMIT


def is_recovered(reading):
    """True when a 005 channel has enough margin for the +0.10 deg ID move."""
    return reading is not None and abs(reading) <= RAIL_RECOVERY_LIMIT_DEG


def _mean_std(values):
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return mean, math.sqrt(variance)


def wait_for_stable_tilt(read_once, window_samples=STABILITY_WINDOW_SAMPLES,
                         stable_windows=STABLE_WINDOWS,
                         sd_threshold_deg=STABILITY_SD_DEG,
                         drift_threshold_deg=STABILITY_DRIFT_DEG,
                         timeout_s=STABILITY_TIMEOUT_S,
                         sample_period_s=STABILITY_SAMPLE_PERIOD_S,
                         abort=None):
    """Wait for consecutive non-overlapping, stable 005 windows.

    The history is local to one call, so callers naturally discard all old
    samples after every stage movement.  The latest window mean is returned for
    feedback; all samples and window statistics are retained for the run log.
    """
    if window_samples < 2 or stable_windows < 2:
        raise ValueError("005 stability requires at least two samples and two windows")

    started = time.monotonic()
    deadline = started + timeout_s
    next_sample = started
    samples = []
    windows = []
    current = []
    passing = []

    def diagnostics(accepted=False):
        return {
            "accepted": accepted,
            "elapsed_s": time.monotonic() - started,
            "window_samples": window_samples,
            "required_stable_windows": stable_windows,
            "sd_threshold_deg": sd_threshold_deg,
            "drift_threshold_deg": drift_threshold_deg,
            "samples": samples,
            "windows": windows,
        }

    while time.monotonic() < deadline:
        _check(abort)
        now = time.monotonic()
        if now < next_sample:
            time.sleep(min(next_sample - now, 0.05))
            continue
        next_sample = max(next_sample + sample_period_s, now)

        reading = read_once()
        elapsed = time.monotonic() - started
        if (reading is None or not reading.get("ack_ok", True)
                or "x" not in reading or "y" not in reading):
            samples.append({"elapsed_s": elapsed, "valid": False,
                            "timestamp_s": (time.time() if reading is None
                                            else reading.get("timestamp_s", time.time())),
                            "reason": "invalid_or_unacknowledged"})
            current = []
            passing = []
            continue

        x, y = float(reading["x"]), float(reading["y"])
        if not math.isfinite(x) or not math.isfinite(y):
            samples.append({"elapsed_s": elapsed, "valid": False,
                            "timestamp_s": reading.get("timestamp_s", time.time()),
                            "reason": "non_finite", "x": x, "y": y})
            current = []
            passing = []
            continue
        if is_saturated(x) or is_saturated(y):
            samples.append({"elapsed_s": elapsed, "valid": False,
                            "timestamp_s": reading.get("timestamp_s", time.time()),
                            "reason": "out_of_range", "x": x, "y": y})
            current = []
            passing = []
            continue

        sample = {"elapsed_s": elapsed,
                  "timestamp_s": reading.get("timestamp_s", time.time()),
                  "valid": True, "x": x, "y": y}
        samples.append(sample)
        current.append(sample)
        if len(current) < window_samples:
            continue

        x_mean, x_sd = _mean_std([item["x"] for item in current])
        y_mean, y_sd = _mean_std([item["y"] for item in current])
        window = {
            "sample_start": len(samples) - len(current),
            "sample_end": len(samples) - 1,
            "mean": {"x": x_mean, "y": y_mean},
            "std": {"x": x_sd, "y": y_sd},
            "sd_pass": x_sd <= sd_threshold_deg and y_sd <= sd_threshold_deg,
            "drift": None,
            "drift_pass": None,
            "accepted": False,
        }
        current = []

        if window["sd_pass"]:
            if passing:
                previous = passing[-1]["mean"]
                drift = {"x": abs(x_mean - previous["x"]),
                         "y": abs(y_mean - previous["y"])}
                window["drift"] = drift
                window["drift_pass"] = (drift["x"] <= drift_threshold_deg
                                         and drift["y"] <= drift_threshold_deg)
                if window["drift_pass"]:
                    passing.append(window)
                else:
                    passing = [window]
            else:
                passing = [window]
        else:
            passing = []

        windows.append(window)
        if len(passing) >= stable_windows:
            for accepted_window in passing[-stable_windows:]:
                accepted_window["accepted"] = True
            out = diagnostics(accepted=True)
            out.update({"mean": dict(window["mean"]),
                        "std": dict(window["std"]),
                        "stable_windows": passing[-stable_windows:]})
            return out

    info = diagnostics(accepted=False)
    raise Unstable(
        f"005 did not produce {stable_windows} stable windows within {timeout_s:g} s",
        diagnostics=info,
    )


def parse_tilt(reply):
    if reply is None or " OK " not in f" {reply}":
        return None
    head, _, raw = reply.partition("|")
    _, _, body = head.partition("OK ")
    out = {"ack_ok": "ack=2A" in raw}
    for tok in body.split():
        key, sep, val = tok.partition("=")
        if not sep:
            continue
        try:
            out[key] = float(val)
        except ValueError:
            pass
    return out if "x" in out and "y" in out else None


def identify_tilt_axes(motion, tilt_all, nudge_deg=FINE_NUDGE_DEG,
                       live=("x", "y"), log=print, abort=None):
    mapping, gains = {}, {}
    for axis in ("OUTER", "INNER"):
        _check(abort)
        before = tilt_all()
        pos = motion.status()[axis.lower()]
        motion.move(axis, pos + nudge_deg)
        try:
            after = tilt_all()
        finally:
            motion.move(axis, pos)
        if before is None or after is None:
            raise NoResponse(f"{axis}: lost the 005 during axis identification")

        deltas = {ch: after[ch] - before[ch] for ch in live}
        ch = max(deltas, key=lambda c: abs(deltas[c]))
        log(f"  {axis}: {nudge_deg:+g} deg -> "
            + "  ".join(f"{c} {deltas[c]:+.5f}" for c in live))
        if abs(deltas[ch]) < MIN_RESPONSE_G:
            log(f"  {axis}: no live 005 channel responds -- keeping DUT level")
            continue

        mapping[axis] = ch
        gains[axis] = nudge_deg / deltas[ch]
        others = [c for c in live if c != ch]
        if others and abs(deltas[others[0]]) > 0.3 * abs(deltas[ch]):
            log(f"  NOTE {axis}: '{others[0]}' also moved "
                f"{abs(deltas[others[0]]) / abs(deltas[ch]) * 100:.0f}% as much"
                " -- the 005 is rotated relative to the stage axes")

    if any(LEVEL_AXIS.get(ax) != ch for ax, ch in mapping.items()):
        log(f"  *** MEASURED MAPPING {mapping} DISAGREES WITH LEVEL_AXIS "
            f"{LEVEL_AXIS} -- using the measured one.")
    if len(mapping) == 2 and mapping["OUTER"] == mapping["INNER"]:
        raise NoResponse(f"both stages drive 005 '{mapping['OUTER']}'")
    if not mapping:
        raise NoResponse("no stage drives any live 005 channel")
    return mapping, gains


def _fine_null_stable(motion, tilt_all, mapping, gains, tol, log=print,
                      abort=None, max_steps=MAX_STEPS):
    """Null both 005 channels using a fresh stable reading after every move."""
    axes = [axis for axis in ("OUTER", "INNER") if axis in mapping]
    if len(axes) != 2:
        raise NoResponse(f"both stage axes must map to live 005 channels; got {mapping}")

    for step_index in range(max_steps + 1):
        _check(abort)
        values = tilt_all()
        if values is None:
            raise NoResponse("005 did not return a stable fine-level reading")
        errors = {axis: values[mapping[axis]] for axis in axes}
        window_means = values.get("_stable_window_means", [values])
        log("  005 stable: " + "  ".join(
            f"{mapping[axis]} {errors[axis]:+.5f}" for axis in axes))
        if all(abs(window[mapping[axis]]) <= tol
               for window in window_means for axis in axes):
            return errors, {channel: values[channel] for channel in ("x", "y")}
        if step_index >= max_steps:
            break

        for axis in axes:
            error = errors[axis]
            if abs(error) <= tol:
                continue
            correction = -error * gains[axis]
            position = motion.status()[axis.lower()]
            motion.move(axis, position + correction)
            log(f"  {axis} (005 {mapping[axis]}): {error:+.5f} -> "
                f"moved {correction:+.4f} deg")

    raise NoResponse(f"005 fine leveling did not converge within {max_steps} corrections")


def _sleep_with_abort(duration_s, abort=None):
    deadline = time.monotonic() + max(0.0, duration_s)
    while time.monotonic() < deadline:
        _check(abort)
        time.sleep(min(0.05, deadline - time.monotonic()))


def _clean_tilt(reading):
    if (reading is None or not reading.get("ack_ok", True)
            or "x" not in reading or "y" not in reading):
        return None
    try:
        x, y = float(reading["x"]), float(reading["y"])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return {
        "x": x,
        "y": y,
        "timestamp_s": reading.get("timestamp_s", time.time()),
    }


def _collect_live_samples(read_live, count, sample_period_s, settle_s=0.0,
                          abort=None):
    _sleep_with_abort(settle_s, abort)
    samples = []
    for sample_index in range(count):
        _check(abort)
        sample = _clean_tilt(read_live())
        samples.append(sample)
        if sample_index + 1 < count:
            _sleep_with_abort(sample_period_s, abort)
    return samples


def _channels_recovered(samples, limit=RAIL_RECOVERY_LIMIT_DEG):
    return {
        channel for channel in ("x", "y")
        if samples and all(
            sample is not None and abs(sample[channel]) <= limit
            for sample in samples
        )
    }


def _scan_position(axis, center_deg, offset_deg):
    target = _finite_float(center_deg, f"{axis} scan center") + offset_deg
    if abs(target) > RECOVERY_SEARCH_LIMIT_DEG:
        raise NotInRange(
            f"{axis} recovery target {target:+.3f} deg exceeds the fixture's "
            f"+/-{RECOVERY_SEARCH_LIMIT_DEG:g} deg safe search window",
            diagnostics={"axis": axis, "target_deg": target,
                         "safe_limit_deg": RECOVERY_SEARCH_LIMIT_DEG},
        )
    return target


def _confirm_channel(read_live, channel, settle_s, sample_count,
                     sample_period_s, abort=None):
    samples = _collect_live_samples(
        read_live, sample_count, sample_period_s, settle_s=settle_s,
        abort=abort,
    )
    accepted = channel in _channels_recovered(samples)
    return {"accepted": accepted, "channel": channel, "samples": samples}


def _local_recovery_after_stop(motion, read_live, axis, channel, step_deg,
                               radius_deg, settle_s, confirm_samples,
                               sample_period_s, log=print, abort=None):
    """Recover a scan overshoot with small, stopped moves around stop position."""
    base = motion.status()[axis.lower()]
    events = []
    step_count = max(1, int(math.ceil(radius_deg / step_deg)))
    for multiplier in range(1, step_count + 1):
        for direction in (+1, -1):  # reverse the negative sweep first
            _check(abort)
            offset = direction * multiplier * step_deg
            if abs(offset) > radius_deg + 1e-9:
                continue
            try:
                target = _scan_position(axis, base, offset)
            except NotInRange:
                continue
            motion.move(axis, target)
            confirmation = _confirm_channel(
                read_live, channel, settle_s, confirm_samples,
                sample_period_s, abort=abort,
            )
            event = {"offset_from_stop_deg": offset,
                     "target_deg": target, **confirmation}
            events.append(event)
            if confirmation["accepted"]:
                last = confirmation["samples"][-1]
                log(f"  {axis}: stopped-step recovery found '{channel}' at "
                    f"{target:+.3f} deg ({last[channel]:+.4f})")
                return {"accepted": True, "events": events,
                        "position_deg": target, "last": last}
    return {"accepted": False, "events": events,
            "position_deg": motion.status()[axis.lower()], "last": None}


def sweep_for_recovery(motion, read_live, axis, center_deg,
                       targets=("x", "y"),
                       radius_deg=RECOVERY_SWEEP_RADIUS_DEG,
                       speed_deg_s=RECOVERY_SWEEP_SPEED_DEG_S,
                       detection_limit_deg=RECOVERY_DETECTION_LIMIT_DEG,
                       detection_samples=RECOVERY_DETECTION_SAMPLES,
                       start_settle_s=RECOVERY_START_SETTLE_S,
                       confirm_settle_s=RECOVERY_CONFIRM_SETTLE_S,
                       confirm_samples=RECOVERY_CONFIRM_SAMPLES,
                       sample_period_s=RECOVERY_SAMPLE_PERIOD_S,
                       local_step_deg=RECOVERY_LOCAL_STEP_DEG,
                       local_radius_deg=RECOVERY_LOCAL_RADIUS_DEG,
                       log=print, abort=None):
    """Sweep one axis from saved-null +radius to -radius and stop on capture."""
    targets = tuple(channel for channel in targets if channel in ("x", "y"))
    if not targets:
        raise ValueError("recovery sweep needs at least one x/y target")
    start = _scan_position(axis, center_deg, abs(radius_deg))
    end = _scan_position(axis, center_deg, -abs(radius_deg))
    diagnostics = {
        "axis": axis,
        "center_deg": center_deg,
        "start_deg": start,
        "end_deg": end,
        "speed_deg_s": speed_deg_s,
        "detection_limit_deg": detection_limit_deg,
        "required_consecutive_detections": detection_samples,
        "targets": list(targets),
        "samples": [],
    }
    log(f"  {axis}: staging at {start:+.3f} deg, then sweeping to "
        f"{end:+.3f} deg at {speed_deg_s:g} deg/s; targets {list(targets)}")
    motion.move(axis, start)
    _sleep_with_abort(start_settle_s, abort)

    consecutive = {channel: 0 for channel in targets}
    values_seen = {"x": [], "y": []}
    captured = None
    scan_started = time.monotonic()
    deadline = scan_started + (abs(start - end) / speed_deg_s) + 15.0
    scan_active = False
    try:
        motion.start_scan(axis, end, speed_deg_s)
        scan_active = True
        next_sample = time.monotonic()
        while time.monotonic() < deadline:
            _check(abort)
            now = time.monotonic()
            if now < next_sample:
                time.sleep(min(0.02, next_sample - now))
                continue
            next_sample = max(next_sample + sample_period_s, now)

            reading = _clean_tilt(read_live())
            status = motion.status()
            position = status.get(axis.lower())
            sample = {
                "elapsed_s": time.monotonic() - scan_started,
                "position_deg": position,
                "valid": reading is not None,
            }
            if reading is not None:
                sample.update(reading)
                for channel in ("x", "y"):
                    values_seen[channel].append(reading[channel])
                for channel in targets:
                    if abs(reading[channel]) <= detection_limit_deg:
                        consecutive[channel] += 1
                    else:
                        consecutive[channel] = 0
                qualified = [
                    channel for channel in targets
                    if consecutive[channel] >= detection_samples
                ]
                if qualified:
                    captured = min(qualified,
                                   key=lambda channel: abs(reading[channel]))
            else:
                for channel in targets:
                    consecutive[channel] = 0
            diagnostics["samples"].append(sample)

            moving = status.get("state") in ("MOVING", "HOMING")
            if captured is not None:
                if moving:
                    motion.stop(wait=True)
                scan_active = False
                diagnostics["detected_channel"] = captured
                diagnostics["detected_position_deg"] = position
                break
            if not moving:
                scan_active = False
                break
        else:
            diagnostics["timed_out"] = True
    except Exception:
        if scan_active:
            try:
                motion.stop(wait=True)
            except Exception:
                pass
        raise

    if scan_active:
        motion.stop(wait=True)
        scan_active = False

    diagnostics["span_deg"] = {
        channel: _span(values_seen[channel]) for channel in ("x", "y")
    }
    diagnostics["sample_count"] = len(diagnostics["samples"])

    if captured is not None:
        confirmation = _confirm_channel(
            read_live, captured, confirm_settle_s, confirm_samples,
            sample_period_s, abort=abort,
        )
        diagnostics["stop_confirmation"] = confirmation
        if confirmation["accepted"]:
            last = confirmation["samples"][-1]
            position = motion.status()[axis.lower()]
            log(f"  {axis}: '{captured}' confirmed inside +/-"
                f"{RAIL_RECOVERY_LIMIT_DEG:g} deg at {position:+.3f} deg "
                f"({last[captured]:+.4f})")
            return {"freed": captured, "position_deg": position,
                    "diagnostics": diagnostics}

        log(f"  {axis}: dynamic detection of '{captured}' did not remain in "
            "range after stopping; checking the +/-"
            f"{local_radius_deg:g} deg stop neighborhood")
        local = _local_recovery_after_stop(
            motion, read_live, axis, captured, local_step_deg,
            local_radius_deg, confirm_settle_s, confirm_samples,
            sample_period_s, log=log, abort=abort,
        )
        diagnostics["local_recovery"] = local
        if local["accepted"]:
            return {"freed": captured, "position_deg": local["position_deg"],
                    "diagnostics": diagnostics}

        # A transient crossing on one channel must not starve the other
        # channel.  The first scan stopped as soon as ``captured`` qualified,
        # so it did not necessarily traverse the rest of the +radius to
        # -radius interval.  Re-stage and repeat the same bounded sweep while
        # monitoring only the still-untried channel(s).  ``targets`` contains
        # at most x and y, so recursion is strictly bounded to one retry.
        remaining_targets = tuple(
            channel for channel in targets if channel != captured
        )
        if remaining_targets:
            log(f"  {axis}: '{captured}' was a false capture; repeating the "
                "bounded sweep for remaining targets "
                f"{list(remaining_targets)}")
            retry = sweep_for_recovery(
                motion, read_live, axis, center_deg,
                targets=remaining_targets,
                radius_deg=radius_deg,
                speed_deg_s=speed_deg_s,
                detection_limit_deg=detection_limit_deg,
                detection_samples=detection_samples,
                start_settle_s=start_settle_s,
                confirm_settle_s=confirm_settle_s,
                confirm_samples=confirm_samples,
                sample_period_s=sample_period_s,
                local_step_deg=local_step_deg,
                local_radius_deg=local_radius_deg,
                log=log, abort=abort,
            )
            diagnostics["remaining_target_sweep"] = retry["diagnostics"]
            retry_spans = retry["diagnostics"].get("span_deg", {})
            for channel in ("x", "y"):
                diagnostics["span_deg"][channel] = max(
                    diagnostics["span_deg"].get(channel, 0.0),
                    retry_spans.get(channel, 0.0),
                )
            diagnostics["sample_count"] += retry["diagnostics"].get(
                "sample_count", 0
            )
            if retry["freed"] is not None:
                diagnostics["freed_on_remaining_target_sweep"] = retry["freed"]
                return {
                    "freed": retry["freed"],
                    "position_deg": retry["position_deg"],
                    "diagnostics": diagnostics,
                }

    # Put a failed axis back at its saved center before trying the other stage.
    motion.move(axis, center_deg)
    diagnostics["final_position_deg"] = motion.status()[axis.lower()]
    log(f"  {axis}: no target channel was confirmed during the bounded sweep")
    return {"freed": None, "position_deg": diagnostics["final_position_deg"],
            "diagnostics": diagnostics}


def _mapping_hint(mapping):
    if (isinstance(mapping, dict)
            and set(mapping) == {"OUTER", "INNER"}
            and set(mapping.values()) == {"x", "y"}):
        return dict(mapping)
    return {}


def auto_level(motion, tilt_all, tilt_live, seed_null_deg,
               saved_mapping=None, log=print, abort=None,
               tolerance_deg=TOLERANCE_DEG, max_steps=MAX_STEPS,
               recovery_sweep_radius_deg=RECOVERY_SWEEP_RADIUS_DEG,
               recovery_sweep_speed_deg_s=RECOVERY_SWEEP_SPEED_DEG_S,
               recovery_detection_limit_deg=RECOVERY_DETECTION_LIMIT_DEG,
               recovery_detection_samples=RECOVERY_DETECTION_SAMPLES,
               recovery_start_settle_s=RECOVERY_START_SETTLE_S,
               recovery_confirm_settle_s=RECOVERY_CONFIRM_SETTLE_S,
               recovery_confirm_samples=RECOVERY_CONFIRM_SAMPLES,
               recovery_sample_period_s=RECOVERY_SAMPLE_PERIOD_S,
               recovery_local_step_deg=RECOVERY_LOCAL_STEP_DEG,
               recovery_local_radius_deg=RECOVERY_LOCAL_RADIUS_DEG):
    """Find and fine-null the 005 without reading the DUT accelerometer."""
    seed = {
        axis: _finite_float(seed_null_deg[axis], f"seed_null_deg.{axis}")
        for axis in ("outer", "inner")
    }
    if not 0 < recovery_sweep_radius_deg <= 10.0:
        raise ValueError("recovery sweep radius must be in (0, 10]")
    if recovery_sweep_speed_deg_s <= 0:
        raise ValueError("recovery sweep speed must be > 0")
    if recovery_detection_samples < 2 or recovery_confirm_samples < 2:
        raise ValueError("recovery detection and confirmation need >= 2 samples")
    if recovery_sample_period_s < 0:
        raise ValueError("recovery sample period must be >= 0")
    result = {
        "method": "005_only_saved_null_bounded_sweep",
        "dut_coarse_used": False,
        "seed_null_deg": dict(seed),
        "range_search": [],
        "fine": {},
    }

    log("phase 1: 005-only startup leveling (DUT accelerometer not used)")
    log(f"  moving to null seed: OUTER {seed['outer']:+.3f}  "
        f"INNER {seed['inner']:+.3f} deg")
    if hasattr(motion, "move_both"):
        motion.move_both(seed["outer"], seed["inner"])
    else:
        motion.move("OUTER", seed["outer"])
        motion.move("INNER", seed["inner"])

    initial_samples = _collect_live_samples(
        tilt_live, recovery_confirm_samples, recovery_sample_period_s,
        settle_s=recovery_confirm_settle_s, abort=abort,
    )
    if not any(sample is not None for sample in initial_samples):
        raise NoResponse("005 did not report valid data at the saved null seed")
    recovered = _channels_recovered(initial_samples)
    result["initial_seed_samples"] = initial_samples
    log(f"  seed recovery: {sorted(recovered) if recovered else 'none'}")

    hint = _mapping_hint(saved_mapping)
    axes = ["OUTER", "INNER"]
    missing = {"x", "y"} - recovered
    if len(missing) == 1 and hint:
        needed_channel = next(iter(missing))
        axes.sort(key=lambda axis: hint.get(axis) != needed_channel)

    provisional_mapping = {}
    for axis in axes:
        if not missing:
            break
        # The saved mapping only prioritizes the likely stage.  Monitor every
        # still-missing channel so a changed cable/mount mapping cannot make a
        # perfectly good sweep look like a dead sensor.
        targets = sorted(missing)
        found = sweep_for_recovery(
            motion, tilt_live, axis, seed[axis.lower()], targets=targets,
            radius_deg=recovery_sweep_radius_deg,
            speed_deg_s=recovery_sweep_speed_deg_s,
            detection_limit_deg=recovery_detection_limit_deg,
            detection_samples=recovery_detection_samples,
            start_settle_s=recovery_start_settle_s,
            confirm_settle_s=recovery_confirm_settle_s,
            confirm_samples=recovery_confirm_samples,
            sample_period_s=recovery_sample_period_s,
            local_step_deg=recovery_local_step_deg,
            local_radius_deg=recovery_local_radius_deg,
            log=log, abort=abort,
        )
        result["range_search"].append(found["diagnostics"])
        if found["freed"] is not None:
            provisional_mapping[axis] = found["freed"]

        check_samples = _collect_live_samples(
            tilt_live, recovery_confirm_samples, recovery_sample_period_s,
            settle_s=0.0, abort=abort,
        )
        recovered = _channels_recovered(check_samples)
        missing = {"x", "y"} - recovered

    result["provisional_mapping"] = provisional_mapping
    result["recovered_channels"] = sorted(recovered)
    if missing:
        spans = {channel: max(
            (event.get("span_deg", {}).get(channel, 0.0)
             for event in result["range_search"]), default=0.0)
            for channel in ("x", "y")
        }
        result["dead_channels"] = [
            channel for channel in missing
            if spans[channel] < DEAD_CHANNEL_SPAN_DEG
        ]
        raise NotInRange(
            "005 operating range not found: "
            f"missing {sorted(missing)} after one-axis-at-a-time "
            f"+{recovery_sweep_radius_deg:g} to "
            f"-{recovery_sweep_radius_deg:g} deg sweeps at "
            f"{recovery_sweep_speed_deg_s:g} deg/s",
            diagnostics=result,
        )

    log("\nphase 2: measured mapping and adaptive 005 fine null")
    mapping, fine_gains = identify_tilt_axes(
        motion, tilt_all, FINE_NUDGE_DEG, live=("x", "y"),
        log=log, abort=abort,
    )
    for axis, gain in list(fine_gains.items()):
        if not 0.5 <= abs(gain) <= 2.0:
            sign = +1 if gain > 0 else -1
            log(f"  {axis}: measured gain {gain:+.2f} is off the physical 1.0 "
                f"-- using {sign * FINE_GAIN_IDEAL:+.1f}")
            fine_gains[axis] = sign * FINE_GAIN_IDEAL

    result["tilt_mapping"] = mapping
    result["gains_fine"] = fine_gains
    result["fine"], result["final_tilt"] = _fine_null_stable(
        motion, tilt_all, mapping, fine_gains, tolerance_deg,
        log=log, abort=abort, max_steps=max_steps,
    )
    result["signs"] = {
        axis: (+1 if gain > 0 else -1)
        for axis, gain in fine_gains.items()
    }
    log(f"\nLEVEL: OUTER {result['fine']['OUTER']:+.5f}   "
        f"INNER {result['fine']['INNER']:+.5f} deg")
    log(f"measured LEVEL_SIGN = {result['signs']}")
    return result
