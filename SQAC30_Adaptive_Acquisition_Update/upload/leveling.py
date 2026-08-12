import time
import math

SENSOR_RANGE_DEG = 0.5
LINEAR_RANGE_DEG = 0.15
IN_RANGE_LIMIT   = 0.45
TOLERANCE_DEG    = 0.002
MAX_STEPS        = 15

STABILITY_WINDOW_SAMPLES = 10
STABLE_WINDOWS           = 2
STABILITY_SD_DEG         = 0.001
STABILITY_DRIFT_DEG      = 0.001
STABILITY_TIMEOUT_S      = 20.0
STABILITY_SAMPLE_PERIOD_S = 0.1

ACCEL_AXIS = {"OUTER": 1, "INNER": 0}
LEVEL_AXIS = {"OUTER": "y", "INNER": "x"}

COARSE_TOL_G     = 0.0022
COARSE_NUDGE_DEG = 1.0
FINE_NUDGE_DEG   = 0.10
COARSE_SETTLE_S  = 0.4
GAIN_DEG_PER_G   = 57.2958
FINE_GAIN_IDEAL  = 1.0
GAIN_TOLERANCE   = 4.0
MIN_RESPONSE_G   = 0.002

RAIL_SEARCH_STEP_DEG  = 0.4
RAIL_SEARCH_MAX_DEG   = 8.0
RAIL_SEARCH_SETTLE_S  = 0.15
DEAD_CHANNEL_SPAN_DEG = 0.002


class Aborted(RuntimeError):
    pass


class NoResponse(RuntimeError):
    pass


class NotInRange(RuntimeError):
    pass


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


def is_saturated(reading):
    return reading is None or abs(reading) >= IN_RANGE_LIMIT


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


def measure_response(motion, sense, axis, nudge_deg, log=print, abort=None):
    _check(abort)
    before = sense(axis)
    if before is None:
        raise NoResponse(f"{axis}: no sensor reading before the test move")

    pos = motion.status()[axis.lower()]
    motion.move(axis, pos + nudge_deg)
    try:
        after = sense(axis)
    finally:
        motion.move(axis, pos)

    if after is None:
        raise NoResponse(f"{axis}: lost the sensor during the test move")
    delta = after - before
    if abs(delta) < MIN_RESPONSE_G:
        raise NoResponse(f"{axis}: moved {nudge_deg:+g} deg but the channel only "
                         f"changed {delta:+.5f} -- check the axis mapping")
    gain = nudge_deg / delta
    log(f"  {axis}: {nudge_deg:+g} deg -> channel {delta:+.5f}  "
        f"=> {gain:+.1f} deg per unit")
    return gain


def check_gain(gain, axis, log=print):
    ratio = abs(gain) / GAIN_DEG_PER_G
    if ratio > GAIN_TOLERANCE or ratio < 1.0 / GAIN_TOLERANCE:
        log(f"  WARNING {axis}: gain {gain:+.1f} is {ratio:.1f}x the ideal "
            f"{GAIN_DEG_PER_G:.1f} -- axis mapping or units may be wrong")
        return False
    return True


def null_axis(motion, sense, axis, gain, tol, log=print, abort=None,
              max_steps=MAX_STEPS, name=""):
    reading = sense(axis)
    if reading is None:
        raise NoResponse(f"{axis}: sensor not reporting")

    worse = 0
    for _ in range(max_steps):
        _check(abort)
        if abs(reading) <= tol:
            log(f"  {axis} {name}: {reading:+.5f} within {tol}")
            return reading
        step = -reading * gain
        pos = motion.status()[axis.lower()]
        motion.move(axis, pos + step)
        new = sense(axis)
        if new is None:
            raise NoResponse(f"{axis}: sensor dropped out mid-loop")
        log(f"  {axis} {name}: {reading:+.5f} -> moved {step:+.4f} deg -> {new:+.5f}")
        worse = worse + 1 if abs(new) > abs(reading) else 0
        if worse >= 2:
            raise NoResponse(f"{axis}: error grew twice running -- gain "
                             f"{gain:+.1f} looks wrong")
        reading = new

    log(f"  {axis} {name}: did not converge in {max_steps} steps, last {reading:+.5f}")
    return reading


def sweep_for_window(motion, tilt_fast, axis, targets=("x", "y"),
                     log=print, abort=None):
    start = motion.status()[axis.lower()]
    seen = {"x": [], "y": []}

    for direction in (+1, -1):
        motion.move(axis, start)
        walked = 0.0
        while walked < RAIL_SEARCH_MAX_DEG:
            _check(abort)
            walked += RAIL_SEARCH_STEP_DEG
            motion.move(axis, start + direction * walked)
            now = tilt_fast()
            if now is None:
                raise NoResponse(f"{axis}: lost the 005 during the sweep")
            for ch in ("x", "y"):
                seen[ch].append(now[ch])
            for ch in targets:
                if not is_saturated(now[ch]):
                    log(f"  {axis}: '{ch}' entered range at "
                        f"{direction * walked:+.2f} deg ({now[ch]:+.4f})")
                    return {"freed": ch, "offset": direction * walked,
                            "span": {c: _span(seen[c]) for c in ("x", "y")}}

            if walked >= 2.0:
                moved = max(_span(seen[c]) for c in ("x", "y"))
                if moved > 20 * DEAD_CHANNEL_SPAN_DEG and all(
                        _span(seen[c]) < DEAD_CHANNEL_SPAN_DEG for c in targets):
                    log(f"  {axis}: {list(targets)} flat over {walked:.1f} deg "
                        f"while another moved {moved:.3f} -- dead")
                    motion.move(axis, start)
                    return {"freed": None, "offset": 0.0,
                            "span": {c: _span(seen[c]) for c in ("x", "y")}}

    motion.move(axis, start)
    span = {c: _span(seen[c]) for c in ("x", "y")}
    log(f"  {axis}: swept +/-{RAIL_SEARCH_MAX_DEG:g} deg, nothing entered range"
        f"   (x moved {span['x']:.4f}, y moved {span['y']:.4f})")
    return {"freed": None, "offset": 0.0, "span": span}


def dead_channels(spans):
    return [ch for ch, span in spans.items() if span < DEAD_CHANNEL_SPAN_DEG]


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


def auto_level(motion, accel_sense, tilt_all, tilt_fast=None, log=print,
               abort=None, tolerance_deg=TOLERANCE_DEG,
               coarse_tol_g=COARSE_TOL_G, max_steps=MAX_STEPS):
    result = {"gains": {}, "coarse": {}, "fine": {}}
    tilt_fast = tilt_fast or tilt_all

    log("phase 1: coarse approach on the DUT (never saturates)")
    for axis in ("OUTER", "INNER"):
        gain = measure_response(motion, accel_sense, axis, COARSE_NUDGE_DEG,
                                log=log, abort=abort)
        check_gain(gain, axis, log=log)
        result["gains"][axis] = gain

    for _ in range(2):
        for axis in ("OUTER", "INNER"):
            result["coarse"][axis] = null_axis(
                motion, accel_sense, axis, result["gains"][axis],
                coarse_tol_g, log=log, abort=abort, name="(DUT, g)")

    log("\nphase 2: fine null on the 005")
    # Range discovery deliberately uses the fast single-reading path.  The
    # adaptive two-window gate starts only after both 005 channels are live.
    now = tilt_fast()
    if now is None:
        raise NoResponse("005 not reporting")

    result["mount_offset"] = {}
    full_spans = None
    for axis in ("OUTER", "INNER"):
        railed = [ch for ch in ("x", "y") if is_saturated(now[ch])]
        if not railed:
            break
        log(f"  005 {railed} railed (x {now['x']:+.4f}, y {now['y']:+.4f}) -- "
            f"sweeping {axis}")
        found = sweep_for_window(motion, tilt_fast, axis, targets=railed,
                                 log=log, abort=abort)
        if found["freed"]:
            result["mount_offset"][f"{found['freed']}<-{axis}"] = found["offset"]
            now = tilt_fast()
        else:
            full_spans = {ch: max(found["span"][ch],
                                  (full_spans or {}).get(ch, 0.0))
                          for ch in ("x", "y")}

    dead = [ch for ch in (dead_channels(full_spans) if full_spans else [])
            if is_saturated(now[ch])]
    if dead:
        log(f"  *** 005 channel(s) {dead} never moved across a full sweep -- "
            "dead, not railed")
    result["dead_channels"] = dead

    live = [ch for ch in ("x", "y")
            if ch not in dead and not is_saturated(now[ch])]
    if len(live) != 2:
        raise NotInRange(f"both 005 channels must be usable (x {now['x']:+.4f}, "
                         f"y {now['y']:+.4f}, live {live}, dead {dead})")

    if result["mount_offset"]:
        log(f"  005 mounting offset from DUT-level: {result['mount_offset']} deg")
        worst = max(abs(v) for v in result["mount_offset"].values())
        if worst > SENSOR_RANGE_DEG:
            log(f"  *** WARNING: finishing on the 005 leaves the fixture "
                f"{worst:.2f} deg off level by the DUT's reckoning -- shim it")

    mapping, fine_gains = identify_tilt_axes(motion, tilt_all, FINE_NUDGE_DEG,
                                             live=live, log=log, abort=abort)

    for axis, gain in list(fine_gains.items()):
        if not 0.5 <= abs(gain) <= 2.0:
            sign = +1 if gain > 0 else -1
            log(f"  {axis}: measured gain {gain:+.2f} is off the physical 1.0 "
                f"-- using {sign * FINE_GAIN_IDEAL:+.1f}")
            fine_gains[axis] = sign * FINE_GAIN_IDEAL

    result["tilt_mapping"] = mapping
    result["gains_fine"] = fine_gains

    # This final loop evaluates both channels together.  A home is accepted only
    # when the same post-movement stable acquisition is within tolerance on both.
    result["fine"], result["final_tilt"] = _fine_null_stable(
        motion, tilt_all, mapping, fine_gains, tolerance_deg,
        log=log, abort=abort, max_steps=max_steps)

    result["signs"] = {ax: (+1 if g > 0 else -1) for ax, g in fine_gains.items()}
    log(f"\nLEVEL: OUTER {result['fine']['OUTER']:+.5f}   "
        f"INNER {result['fine']['INNER']:+.5f} deg")
    log(f"measured LEVEL_SIGN = {result['signs']}")
    return result
