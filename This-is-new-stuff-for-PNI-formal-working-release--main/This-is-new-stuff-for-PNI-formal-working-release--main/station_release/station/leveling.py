import time

SENSOR_RANGE_DEG = 0.5
LINEAR_RANGE_DEG = 0.15
IN_RANGE_LIMIT   = 0.45
TOLERANCE_DEG    = 0.002
SETTLE_S         = 2.0
N_AVG            = 5
MAX_STEPS        = 15

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


def _check(abort):
    if abort is not None and abort():
        raise Aborted("stopped by operator")


def _span(values):
    return (max(values) - min(values)) if values else 0.0


def is_saturated(reading):
    return reading is None or abs(reading) >= IN_RANGE_LIMIT


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


def auto_level(motion, accel_sense, tilt_all, tilt_fast=None, log=print,
               abort=None):
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
                COARSE_TOL_G, log=log, abort=abort, name="(DUT, g)")

    log("\nphase 2: fine null on the 005")
    now = tilt_all()
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
            now = tilt_all()
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
    if not live:
        raise NotInRange(f"no usable 005 channel (x {now['x']:+.4f}, "
                         f"y {now['y']:+.4f}, dead {dead}); phase 1's "
                         "DUT-level result stands")

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

    def sense_measured(axis):
        vals = tilt_all()
        return None if vals is None else vals[mapping[axis]]

    drivable = [ax for ax in ("OUTER", "INNER") if ax in mapping]
    if len(drivable) < 2:
        log(f"  only {drivable} can be nulled on the 005")

    for _ in range(2):
        for axis in drivable:
            result["fine"][axis] = null_axis(
                motion, sense_measured, axis, fine_gains[axis], TOLERANCE_DEG,
                log=log, abort=abort, name=f"(005 {mapping[axis]})")
        if all(abs(v) <= TOLERANCE_DEG for v in result["fine"].values()):
            break

    for axis in ("OUTER", "INNER"):
        result["fine"].setdefault(axis, result["coarse"][axis])

    result["signs"] = {ax: (+1 if g > 0 else -1) for ax, g in fine_gains.items()}
    log(f"\nLEVEL: OUTER {result['fine']['OUTER']:+.5f}   "
        f"INNER {result['fine']['INNER']:+.5f} deg")
    log(f"measured LEVEL_SIGN = {result['signs']}")
    return result