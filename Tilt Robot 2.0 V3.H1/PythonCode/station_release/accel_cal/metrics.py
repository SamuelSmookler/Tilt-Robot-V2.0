import math

import numpy as np


DEFAULT_VERIFICATION_THRESHOLDS = {
    "rms_pitch_error_deg": 0.2,
    "rms_roll_main_error_deg": 0.2,
    "rms_roll_edge_error_deg": 0.5,
    "max_abs_pitch_error_deg": 3.0,
    "max_abs_roll_error_deg": 3.0,
    "rms_x_acceleration_error_mg": 2.0,
    "rms_y_acceleration_error_mg": 2.0,
    "rms_z_acceleration_error_mg": 2.5,
}


# The order here is also the order used by the GUI and JSON report.
VERIFICATION_STATISTICS = (
    ("rms_pitch_error_deg", "RMS pitch error", "deg"),
    ("rms_roll_main_error_deg", "RMS roll error over main region", "deg"),
    ("rms_roll_edge_error_deg", "RMS roll error over edge region", "deg"),
    ("max_abs_pitch_error_deg", "Absolute pitch error", "deg"),
    ("max_abs_roll_error_deg", "Absolute roll error", "deg"),
    ("rms_x_acceleration_error_mg", "RMS X acceleration error", "mg"),
    ("rms_y_acceleration_error_mg", "RMS Y acceleration error", "mg"),
    ("rms_z_acceleration_error_mg", "RMS Z acceleration error", "mg"),
)


def wrap180(degrees):
    """Wrap scalar or array angles to [-180, 180)."""
    degrees = np.asarray(degrees, dtype=float)
    return (degrees + 180.0) % 360.0 - 180.0


def angular_error_deg(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    dot = np.sum(a * b, axis=1)
    norms = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        cos = np.clip(dot / norms, -1.0, 1.0)
        return np.degrees(np.arccos(cos))


def pitch_roll_from_vectors(vectors):
    """Return [pitch, roll] in degrees using the fixture gravity convention.

    The inverse matches:
        gx = -sin(pitch)
        gy = cos(pitch) * sin(roll)
        gz = cos(pitch) * cos(roll)

    atan2 is used for pitch so vector magnitude does not affect the result.
    """
    vectors = np.asarray(vectors, dtype=float)
    if vectors.ndim != 2 or vectors.shape[1] != 3:
        raise ValueError("vectors must have shape (N, 3)")
    pitch = np.degrees(
        np.arctan2(-vectors[:, 0], np.hypot(vectors[:, 1], vectors[:, 2]))
    )
    roll = np.degrees(np.arctan2(vectors[:, 1], vectors[:, 2]))
    return np.column_stack([pitch, roll])


def _finite_stat(value):
    value = float(value)
    return value if math.isfinite(value) else None


def _rmse(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0 or not np.all(np.isfinite(values)):
        return None
    return _finite_stat(np.sqrt(np.mean(values**2)))


def _max_abs(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0 or not np.all(np.isfinite(values)):
        return None
    return _finite_stat(np.max(np.abs(values)))


def _json_float_list(values):
    return [None if not math.isfinite(float(value)) else float(value)
            for value in values]


def verification_metrics(corrected, truth, measured_pitch_roll_deg=None,
                         roll_main_max_abs_pitch_deg=45.0):
    """Calculate acceleration and legacy pitch/roll verification statistics.

    Pitch and roll used for PASS/FAIL are calculated from ``corrected`` and
    ``truth`` using the same gravity-vector equations. This evaluates the
    selected host correction model without changing axis or sign conventions.

    ``measured_pitch_roll_deg`` is the DUT-native [pitch, roll] result retained
    for backward call compatibility and JSON diagnostics only. It never affects
    the correction-model verdict because the host model has not been applied to
    those native DUT angles.

    The roll regions are selected from reference pitch. The default definition
    is main when ``abs(reference pitch) <= 45 deg`` and edge otherwise.
    """
    corrected = np.asarray(corrected, dtype=float)
    truth = np.asarray(truth, dtype=float)
    if (corrected.ndim != 2 or corrected.shape[1] != 3
            or truth.shape != corrected.shape):
        raise ValueError("corrected and truth must have the same (N, 3) shape")

    n_poses = corrected.shape[0]
    if measured_pitch_roll_deg is None:
        native_pitch_roll = np.full((n_poses, 2), np.nan, dtype=float)
    else:
        native_pitch_roll = np.asarray(measured_pitch_roll_deg, dtype=float)
        if native_pitch_roll.shape != (n_poses, 2):
            raise ValueError(
                "measured_pitch_roll_deg must have shape (N, 2) matching "
                "the verification vectors"
            )

    acceleration_inputs_finite = bool(
        n_poses > 0
        and np.all(np.isfinite(corrected))
        and np.all(np.isfinite(truth))
    )
    if acceleration_inputs_finite:
        corrected_pitch_roll = pitch_roll_from_vectors(corrected)
        reference_pitch_roll = pitch_roll_from_vectors(truth)
    else:
        corrected_pitch_roll = np.full((n_poses, 2), np.nan, dtype=float)
        reference_pitch_roll = np.full((n_poses, 2), np.nan, dtype=float)

    pitch_roll_inputs_finite = bool(
        n_poses > 0
        and np.all(np.isfinite(corrected_pitch_roll))
        and np.all(np.isfinite(reference_pitch_roll))
    )
    native_pitch_roll_inputs_finite = bool(
        n_poses > 0 and np.all(np.isfinite(native_pitch_roll))
    )

    reference_pitch = reference_pitch_roll[:, 0]
    main_mask = np.isfinite(reference_pitch) & (
        np.abs(reference_pitch) <= roll_main_max_abs_pitch_deg
    )
    edge_mask = np.isfinite(reference_pitch) & ~main_mask

    pitch_error = wrap180(
        corrected_pitch_roll[:, 0] - reference_pitch_roll[:, 0]
    )
    roll_error = wrap180(
        corrected_pitch_roll[:, 1] - reference_pitch_roll[:, 1]
    )

    if acceleration_inputs_finite:
        err = corrected - truth
        vec = np.linalg.norm(err, axis=1)
        ang = angular_error_deg(corrected, truth)
        per_axis_rmse_mg = np.sqrt(np.mean(err**2, axis=0)) * 1000.0
        vector_rmse_mg = _finite_stat(np.sqrt(np.mean(vec**2)) * 1000.0)
        max_vector_mg = _finite_stat(np.max(vec) * 1000.0)
        p95_vector_mg = _finite_stat(np.percentile(vec, 95) * 1000.0)
        rms_angular_deg = _finite_stat(np.sqrt(np.mean(ang**2)))
        max_angular_deg = _finite_stat(np.max(ang))
    else:
        per_axis_rmse_mg = np.full(3, np.nan, dtype=float)
        vector_rmse_mg = None
        max_vector_mg = None
        p95_vector_mg = None
        rms_angular_deg = None
        max_angular_deg = None

    per_axis_rmse_json = _json_float_list(per_axis_rmse_mg)
    required_regions_present = bool(np.any(main_mask) and np.any(edge_mask))
    pitch_roll_by_pose = []
    for index in range(n_poses):
        if main_mask[index]:
            region = "main"
        elif edge_mask[index]:
            region = "edge"
        else:
            region = "unclassified"
        pitch_roll_by_pose.append({
            "pose_index": index,
            "reference_deg": {
                "pitch": _finite_stat(reference_pitch_roll[index, 0]),
                "roll": _finite_stat(reference_pitch_roll[index, 1]),
            },
            "measured_deg": {
                "pitch": _finite_stat(corrected_pitch_roll[index, 0]),
                "roll": _finite_stat(corrected_pitch_roll[index, 1]),
            },
            "corrected_deg": {
                "pitch": _finite_stat(corrected_pitch_roll[index, 0]),
                "roll": _finite_stat(corrected_pitch_roll[index, 1]),
            },
            "native_dut_deg": {
                "pitch": _finite_stat(native_pitch_roll[index, 0]),
                "roll": _finite_stat(native_pitch_roll[index, 1]),
            },
            "error_deg": {
                "pitch": _finite_stat(pitch_error[index]),
                "roll": _finite_stat(roll_error[index]),
            },
            "roll_region": region,
        })

    return {
        # Existing diagnostic fields retained for report compatibility.
        "per_axis_rmse_mg": per_axis_rmse_json,
        "vector_rmse_mg": vector_rmse_mg,
        "max_vector_mg": max_vector_mg,
        "p95_vector_mg": p95_vector_mg,
        "rms_angular_deg": rms_angular_deg,
        "max_angular_deg": max_angular_deg,
        "n_poses": n_poses,

        # Legacy-style pass/fail statistics requested for the final report.
        "rms_pitch_error_deg": (
            _rmse(pitch_error) if pitch_roll_inputs_finite else None
        ),
        "rms_roll_main_error_deg": (
            _rmse(roll_error[main_mask]) if pitch_roll_inputs_finite else None
        ),
        "rms_roll_edge_error_deg": (
            _rmse(roll_error[edge_mask]) if pitch_roll_inputs_finite else None
        ),
        "max_abs_pitch_error_deg": (
            _max_abs(pitch_error) if pitch_roll_inputs_finite else None
        ),
        "max_abs_roll_error_deg": (
            _max_abs(roll_error) if pitch_roll_inputs_finite else None
        ),
        "rms_x_acceleration_error_mg": per_axis_rmse_json[0],
        "rms_y_acceleration_error_mg": per_axis_rmse_json[1],
        "rms_z_acceleration_error_mg": per_axis_rmse_json[2],
        "all_required_values_finite": bool(
            acceleration_inputs_finite
            and pitch_roll_inputs_finite
            and required_regions_present
        ),
        "roll_region_definition": {
            "basis": "absolute reference pitch",
            "main": f"abs(reference_pitch_deg) <= {roll_main_max_abs_pitch_deg:g}",
            "edge": f"abs(reference_pitch_deg) > {roll_main_max_abs_pitch_deg:g}",
            "main_max_abs_pitch_deg": float(roll_main_max_abs_pitch_deg),
            "main_pose_count": int(np.count_nonzero(main_mask)),
            "edge_pose_count": int(np.count_nonzero(edge_mask)),
            "required_regions_present": required_regions_present,
        },
        "pitch_roll_source": (
            "selected host correction model output converted from corrected XYZ"
        ),
        "native_pitch_roll_source": (
            "DUT native units_pr [component 0, component 1]; diagnostic only"
        ),
        "native_pitch_roll_all_finite": native_pitch_roll_inputs_finite,
        "acceleration_source": "selected host correction model output",
        "pitch_roll_by_pose": pitch_roll_by_pose,
    }


def evaluate_verification(metrics, thresholds=None):
    """Apply inclusive limits and return per-statistic plus overall outcomes."""
    limits = dict(DEFAULT_VERIFICATION_THRESHOLDS)
    if thresholds is not None:
        limits.update(thresholds)

    statistics = {}
    failed = []
    for key, label, unit in VERIFICATION_STATISTICS:
        value = metrics.get(key)
        limit = limits.get(key)
        valid_value = (
            value is not None
            and isinstance(value, (int, float, np.integer, np.floating))
            and math.isfinite(float(value))
        )
        valid_limit = (
            limit is not None
            and isinstance(limit, (int, float, np.integer, np.floating))
            and math.isfinite(float(limit))
        )
        passed = bool(valid_value and valid_limit and float(value) <= float(limit))
        outcome = "PASS" if passed else "FAIL"
        statistics[key] = {
            "label": label,
            "value": None if not valid_value else float(value),
            "limit": None if not valid_limit else float(limit),
            "comparison": "<=",
            "unit": unit,
            "outcome": outcome,
        }
        if not passed:
            failed.append(key)

    all_finite = bool(metrics.get("all_required_values_finite", False))
    if not all_finite and "missing_or_nonfinite_required_value" not in failed:
        failed.append("missing_or_nonfinite_required_value")

    return {
        "outcome": "PASS" if not failed else "FAIL",
        "all_required_values_finite": all_finite,
        "failed_statistics": failed,
        "statistics": statistics,
        "roll_region_definition": metrics.get("roll_region_definition"),
        "pitch_roll_source": metrics.get("pitch_roll_source"),
        "native_pitch_roll_source": metrics.get("native_pitch_roll_source"),
        "acceleration_source": metrics.get("acceleration_source"),
    }
