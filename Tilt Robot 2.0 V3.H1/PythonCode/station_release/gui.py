import tkinter as tk
from tkinter import ttk

#added to prevent bloat and "mailbox"
import math
import threading, queue
from station.protocol import Link

#remenant from testing
# from station.mock_devices import Rig, MockMotion, MockSensor
# from station.clients import MotionClient, SensorClient

from station.hardware import RealMotion, RealDUT



from station.sequencer import calibrate, AcquisitionFailure
import accel_cal.correctors as correction_models
from accel_cal.correctors import Affine12, Bodega24, BodegaR21, SQAC24, SQAC30

MODELS = {
    "Affine12": Affine12,
    "Bodega24": Bodega24,
    "BodegaR21": BodegaR21,
    "SQAC24": SQAC24,
    "SQAC30": SQAC30,
}
# Keep this list literal so every selectable GUI model is obvious in this file.
MODEL_NAMES = ("Affine12", "Bodega24", "BodegaR21", "SQAC24", "SQAC30")
if set(MODEL_NAMES) != set(MODELS) or MODELS.get("SQAC24") is not SQAC24:
    raise RuntimeError("GUI model registry is missing the SQAC24 implementation")
GUI_BUILD = "full-revolution cold-start homing + 005 leveling (2026-08-13 build 6)"

import json
from datetime import datetime
from pathlib import Path

#eehhh addtions for the congif file
import config as station_config
from accel_cal.poses import build_pose_set, order_poses, tour_length, wrap180

import time

from station import leveling
from station.protocol import Link, CommandError



# replaced by computed values



# FIT = [(0,0), (90,0), (180,0), (-90,0), (0,90), (0,-90), (45,60), (135,-90), (-135,110), (-45,-60)]
# VERIFY = [(60,115), (-160,-110), (120,30), (-60,-75)]





# #ADDED LATE FOR WRITING



# record = []
# result = calibrate(motion, sensor, FIT, VERIFY, model=..., record=record)

# artifact = {
#     "timestamp": datetime.now().isoformat(),
#     "model": model_name,
#     "raw_error_mg": result["raw_error_mg"],
#     "corrected_error_mg": result["corrected_error_mg"],
#     "M": result["model"].M.tolist() if hasattr(result["model"], "M") else None,
#     "b": result["model"].b.tolist() if hasattr(result["model"], "b") else None,
#     "poses": record,                       # <- every raw sample + measured angle
# }
# with open(f"run_{datetime.now():%Y%m%d_%H%M%S}.json", "w") as f:
#     json.dump(artifact, f, indent=2)





msgs = queue.Queue()
stop_event = threading.Event()
hw = {"motion": None}


def _verification_settings(cfg):
    return {
        "thresholds": cfg.verification_thresholds(),
        "roll_main_max_abs_pitch_deg": cfg.roll_main_max_abs_pitch_deg,
        "roll_region_rule": (
            "main when abs(reference pitch) <= boundary; edge otherwise"
        ),
    }


def _verification_line(statistic):
    value = statistic["value"]
    limit = statistic["limit"]
    unit = statistic["unit"]
    precision = 3 if unit == "deg" else 2
    value_text = "missing/non-finite" if value is None else f"{value:.{precision}f}"
    limit_text = "missing" if limit is None else f"{limit:.{precision}f}"
    return (f"{statistic['label']} : {value_text} {unit} <= "
            f"{limit_text} {unit}  {statistic['outcome']}")


def _number_text(value, precision):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return "missing/non-finite"
    if not math.isfinite(value):
        return "missing/non-finite"
    return f"{value:.{precision}f}"


def _json_number(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _coefficient_columns(model_parameters):
    """Return exact [(term, coefficient), ...] lists for X, Y, and Z."""
    axes = ("x", "y", "z")
    feature_names = model_parameters.get("feature_names_by_output")
    coefficients = model_parameters.get("coefficients_by_output")

    if feature_names is not None and coefficients is not None:
        columns = {}
        for axis in axes:
            try:
                terms = list(feature_names[axis])
                values = [float(value) for value in coefficients[axis]]
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"invalid coefficient data for {axis.upper()} output"
                ) from exc
            if len(terms) != len(values):
                raise ValueError(
                    f"{axis.upper()} output has {len(terms)} terms but "
                    f"{len(values)} coefficients"
                )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(
                    f"{axis.upper()} output contains a non-finite coefficient"
                )
            columns[axis] = [
                (str(term), value) for term, value in zip(terms, values)
            ]
    else:
        # Affine12 stores corrected = M @ measured + b. Each output therefore
        # has coefficients for x, y, z, and the constant term 1.
        try:
            matrix = model_parameters["M"]
            bias = model_parameters["b"]
            if (len(matrix) != 3 or any(len(row) != 3 for row in matrix)
                    or len(bias) != 3):
                raise ValueError("Affine12 M/b dimensions must be 3x3 and 3")
            columns = {
                axis: [
                    (term, float(value))
                    for term, value in zip(
                        ("x", "y", "z", "1"),
                        [*matrix[index], bias[index]],
                    )
                ]
                for index, axis in enumerate(axes)
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                "model parameters do not expose labeled per-axis coefficients"
            ) from exc
        if not all(
                math.isfinite(value)
                for column in columns.values()
                for _, value in column):
            raise ValueError("Affine12 contains a non-finite coefficient")

    actual_count = sum(len(columns[axis]) for axis in axes)
    declared_count = model_parameters.get("coefficient_count")
    if declared_count is not None and int(declared_count) != actual_count:
        raise ValueError(
            f"model declares {declared_count} coefficients but exposes "
            f"{actual_count}"
        )
    return columns


def _coefficient_report_text(model_name, fit_pose_count, outcome,
                             model_parameters, run_timestamp):
    """Build a fixed-width, human-readable per-axis coefficient report."""
    axes = ("x", "y", "z")
    columns = _coefficient_columns(model_parameters)
    row_count = max(len(columns[axis]) for axis in axes)
    cells_by_axis = {
        axis: [f"{term} = {value:.16e}" for term, value in columns[axis]]
        for axis in axes
    }
    headers = {
        axis: f"{axis.upper()} axis (term = coefficient)" for axis in axes
    }
    widths = {
        axis: max(len(headers[axis]), *(len(cell)
                                        for cell in cells_by_axis[axis]))
        for axis in axes
    }

    lines = [
        "PolyFitIncl",
        f"Correction model: {model_name}",
        f"Fit pose count: {int(fit_pose_count)}",
        f"Calibration outcome: {outcome}",
        f"Run timestamp: {run_timestamp.isoformat(timespec='seconds')}",
        f"Total coefficient count: {sum(len(columns[a]) for a in axes)}",
        "Coefficients per axis: " + ", ".join(
            f"{axis.upper()}={len(columns[axis])}" for axis in axes
        ),
    ]
    if model_parameters.get("basis"):
        lines.append(f"Model basis: {model_parameters['basis']}")
    if model_parameters.get("equation"):
        lines.append(f"Model equation: {model_parameters['equation']}")
    lines.extend([
        "Coefficient order: exact order used by the correction model",
        "",
        f"{'#':>3} | " + " | ".join(
            headers[axis].ljust(widths[axis]) for axis in axes
        ),
        "----+-" + "-+-".join("-" * widths[axis] for axis in axes),
    ])

    for row_index in range(row_count):
        row_cells = []
        for axis in axes:
            cells = cells_by_axis[axis]
            cell = cells[row_index] if row_index < len(cells) else ""
            row_cells.append(cell.ljust(widths[axis]))
        lines.append(f"{row_index + 1:>3} | " + " | ".join(row_cells))

    lines.extend(["", "PolyFitIncl End"])
    return "\n".join(lines) + "\n"

def worker(model_name, do_level=True, fit_pose_count=None, announce_done=True,
           out_paths=None):
    """Run on background thread. Must not touch widgets (Tk is single-threaded)."""
    # #added for actual motion
    motion = sensor = level_sense = None
    last_level_state = None
    park_target = None


    cfg = station_config.load()
    stop_event.clear()
    if fit_pose_count is None:
        fit_pose_count = cfg.fit_pose_count
    for w in cfg.warnings:
        msgs.put(f"CONFIG WARNING: {w}")

    config_dir = Path(station_config.__file__).resolve().parent
    runs_dir = Path(cfg.runs_dir)
    if not runs_dir.is_absolute():
        runs_dir = config_dir / runs_dir
    runs_dir.mkdir(parents=True, exist_ok=True)
    level_state_path = Path(cfg.leveling_state_file)
    if not level_state_path.is_absolute():
        level_state_path = config_dir / level_state_path
    try:
        last_level_state = leveling.load_level_state(level_state_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        msgs.put(f"LEVEL STATE WARNING: {exc} -- using configured initial null")
        last_level_state = None

    if last_level_state is not None:
        level_seed = dict(last_level_state["offsets_deg"])
        seed_mapping = last_level_state.get("tilt_mapping")
        seed_source = "last successful 005 null"
    else:
        level_seed = {
            "outer": cfg.leveling_initial_outer_null_deg,
            "inner": cfg.leveling_initial_inner_null_deg,
        }
        seed_mapping = None
        seed_source = "configured commissioning seed"
    park_target = dict(level_seed)
    run_timestamp = datetime.now()
    tag = "leveled" if do_level else "unleveled"
    run_base = runs_dir / f"run_{run_timestamp:%Y%m%d_%H%M%S}_{tag}"
    fname = str(run_base.with_suffix(".json"))
    coefficients_fname = str(run_base.with_name(
        run_base.name + "_coefficients.txt"
    ))
    failed_fname = str(run_base.with_name(run_base.name + "_failed.json"))
    record = []
    level_info = {
        "enabled": bool(do_level),
        "method": (
            "005_only_saved_null_bounded_sweep" if do_level else "disabled"
        ),
        "dut_coarse_used": False,
        "state_file": str(level_state_path),
        "seed_source": seed_source,
        "seed_null_deg": dict(level_seed),
    }

    FIT, VERIFY, DROPPED = build_pose_set(
        inner_limit=cfg.inner_limit_deg,
        fit_pose_count=fit_pose_count,
    )
    summary_title = f"{model_name} {len(FIT)}-fit pose calibration"
    verification_settings = _verification_settings(cfg)
    msgs.put(f"pose set: {len(FIT)} fit + {len(VERIFY)} verification poses")
    msgs.put(f"TP3 gate: {cfg.tp3_min_settle_s:g} s minimum settle, "
             f"{cfg.tp3_stable_windows} x {cfg.n_samples}-sample "
             f"qualification windows, <= {cfg.tp3_stability_sd_mg:g} mg SD and <= "
             f"{cfg.tp3_mean_drift_mg:g} mg mean change; "
             f"{cfg.n_samples} final samples, "
             f"{cfg.tp3_stability_timeout_s:g} s, "
             f"{cfg.tp3_max_attempts} attempts")
    if DROPPED:
        msgs.put(f"WARNING: {len(DROPPED)} poses unreachable: {DROPPED}")

    # #stuff added for list (attempt)

    # record = []
    # r = calibrate(motion, sensor, FIT, VERIFY, model=MODELS[model_name](), abort=stop_event.is_set, record=record)

    # from datetime import datetime
    # import json
    # artifact = {
    #     "timestamp": datetime.now().isoformat(),
    #     "model": model_name,
    #     "raw_error_mg": r["raw_error_mg"],
    #     "corrected_error_mg": r["corrected_error_mg"],
    #     "M": r["model"].M.tolist() if hasattr(r["model"], "M") else None,
    #         "b": r["model"].b.tolist() if hasattr(r["model"], "b") else None,
    #         "poses": record,


    #     "poses": record,


    # }
    # fname = f"run_{datetime.now():%Y%m%d_%H%M%S}.json"
    # with open(fname, "w") as f:
    #     json.dump(artifact, f, indent=2)
    # msgs.put(f"saved -> {fname}")







    # try:
    #     rig = Rig()
    #     motion = MotionClient(Link(MockMotion(rig)))
    #     sensor = SensorClient(Link(MockSensor(rig)))
    #     msgs.put(f"running {model_name} ...")
    #     stop_event = threading.Event()
    #     r = calibrate(motion, sensor, FIT, VERIFY, model=MODELS[model_name](), abort=stop_event.is_set)
    #     msgs.put(f"raw error : {r['raw_error_mg']:.2f} mg")
    #     msgs.put(f"corrected error : {r['corrected_error_mg']:.2f} ")
    #     msgs.put(f"verdict : {'PASS' if r['corrected_error_mg'] < 5.0 else 'FAIL'} ")
    # except Exception as e:
    #     msgs.put(f"ERROR: {e}")
    # msgs.put("__done__")



    try:
        motion = RealMotion(cfg.stage_port, timeout_s=cfg.motion_timeout_s)
        hw['motion'] = motion
        sensor = RealDUT(cfg.dut_port, slot=cfg.dut_slot)





        st = motion.status()
        if st.get("outer_ok") != 1.0 or st.get("inner_ok") != 1.0:
            raise RuntimeError(
                "both rotary stages must be connected "
                f"(outer_ok={st.get('outer_ok')}, inner_ok={st.get('inner_ok')})"
            )
        try:
            motion_fw = int(st.get("fw"))
        except (TypeError, ValueError, OverflowError):
            motion_fw = None
        if motion_fw is None or motion_fw < 5:
            raise RuntimeError(
                "full-revolution cold-start homing requires the supplied "
                "motion_arduino.ino firmware (STATUS fw=5); "
                f"controller reports fw={st.get('fw', 'missing')}"
            )
        if st.get("homed") == 1.0:
            msgs.put("stage reference is still valid; startup homing skipped")
        else:
            unreferenced = [
                axis for axis in ("OUTER", "INNER")
                if st.get(f"{axis.lower()}_homed") != 1.0
            ]
            # Old firmware may omit the per-axis flags. In that case a global
            # unreferenced state conservatively re-homes both stages.
            if not unreferenced:
                unreferenced = ["OUTER", "INNER"]
            msgs.put(
                "stage reference missing for " + ", ".join(unreferenced)
                + "; sweeping each through its built-in optical home index "
                  "(up to one full revolution)..."
            )
            for axis in unreferenced:
                msgs.put(f"  {axis}: searching for optical home index...")
                motion.home(axis)
                msgs.put(f"  {axis}: reference established")
        st = motion.status()
        if st.get("homed") != 1.0:
            raise RuntimeError(
                f"homing did not complete (homed={st.get('homed')}, "
                f"outer_homed={st.get('outer_homed')} "
                f"inner_homed={st.get('inner_homed')})")



        o0 = i0 = 0.0

        if do_level:
            msgs.put("auto-leveling: saved-null 005 acquisition; DUT coarse disabled")
            msgs.put(f"005 seed ({seed_source}): OUTER "
                     f"{level_seed['outer']:+.3f}, INNER "
                     f"{level_seed['inner']:+.3f} deg")
            msgs.put(f"005 recovery: one axis at a time, +"
                     f"{cfg.leveling_recovery_sweep_radius_deg:g} to -"
                     f"{cfg.leveling_recovery_sweep_radius_deg:g} deg from seed "
                     f"at {cfg.leveling_recovery_sweep_speed_deg_s:g} deg/s; "
                     f"{cfg.leveling_recovery_detection_samples} consecutive "
                     f"samples inside +/-"
                     f"{cfg.leveling_recovery_detection_limit_deg:g} deg")
            msgs.put(f"005 gate: {cfg.leveling_stable_windows} x "
                     f"{cfg.leveling_window_samples} samples, <= "
                     f"{cfg.leveling_sd_threshold_deg:g} deg SD, <= "
                     f"{cfg.leveling_drift_threshold_deg:g} deg drift")
            level_sense = LevelSense(
                sensor,
                window_samples=cfg.leveling_window_samples,
                stable_windows=cfg.leveling_stable_windows,
                sd_threshold_deg=cfg.leveling_sd_threshold_deg,
                drift_threshold_deg=cfg.leveling_drift_threshold_deg,
                timeout_s=cfg.leveling_timeout_s,
                sample_period_s=cfg.leveling_sample_period_s,
                abort=stop_event.is_set,
            )
            out = leveling.auto_level(
                motion, level_sense.tilt_all, level_sense.tilt_live,
                seed_null_deg=level_seed,
                saved_mapping=seed_mapping,
                log=msgs.put, abort=stop_event.is_set,
                tolerance_deg=(cfg.leveling_tolerance_deg
                               or leveling.TOLERANCE_DEG),
                recovery_sweep_radius_deg=(
                    cfg.leveling_recovery_sweep_radius_deg
                ),
                recovery_sweep_speed_deg_s=(
                    cfg.leveling_recovery_sweep_speed_deg_s
                ),
                recovery_detection_limit_deg=(
                    cfg.leveling_recovery_detection_limit_deg
                ),
                recovery_detection_samples=(
                    cfg.leveling_recovery_detection_samples
                ),
                recovery_start_settle_s=(
                    cfg.leveling_recovery_start_settle_s
                ),
                recovery_confirm_settle_s=(
                    cfg.leveling_recovery_confirm_settle_s
                ),
                recovery_confirm_samples=(
                    cfg.leveling_recovery_confirm_samples
                ),
                recovery_sample_period_s=(
                    cfg.leveling_recovery_sample_period_s
                ),
                recovery_local_step_deg=(
                    cfg.leveling_recovery_local_step_deg
                ),
                recovery_local_radius_deg=(
                    cfg.leveling_recovery_local_radius_deg
                ),
            )
            st = motion.status()
            o0, i0 = st["outer"], st["inner"]
            park_target = {"outer": o0, "inner": i0}
            level_info.update({
                "offsets_deg": dict(park_target),
                "fine_deg": out["fine"],
                "final_tilt_deg": out.get("final_tilt"),
                "tilt_mapping": out.get("tilt_mapping"),
                "gains_fine": out.get("gains_fine"),
                "range_search": out.get("range_search", []),
                "initial_seed_samples": out.get("initial_seed_samples"),
                "stability_events": level_sense.stability_events,
                "stability_settings": level_sense.settings,
            })
            new_level_state = {
                "schema_version": 1,
                "saved_at": datetime.now().isoformat(timespec="seconds"),
                "offsets_deg": dict(park_target),
                "tilt_mapping": out.get("tilt_mapping"),
                "gains_fine": out.get("gains_fine"),
                "signs": out.get("signs"),
                "final_tilt_deg": out.get("final_tilt"),
                "motion_firmware": st.get("fw"),
            }
            try:
                leveling.save_level_state(level_state_path, new_level_state)
                level_info["state_saved"] = True
                msgs.put(f"last-good 005 null saved -> {level_state_path}")
            except (OSError, ValueError) as state_exc:
                level_info["state_saved"] = False
                level_info["state_save_error"] = str(state_exc)
                msgs.put(f"LEVEL STATE WARNING: could not save null: {state_exc}")
            msgs.put(f"leveled zero: OUTER {o0:+.4f}  INNER {i0:+.4f}")
        else:
            msgs.put("auto-level SKIPPED -- running from the mechanical zero")

        wrapped = OffsetMotion(motion, o0, i0, pair_moves=cfg.simultaneous_moves)

        fit_l, ver_l = FIT, VERIFY
        if cfg.pose_order == "optimized":
            sim, win = cfg.simultaneous_moves, cfg.outer_window_deg
            before = (tour_length(FIT, (0., 0.), sim, win, o0)
                      + tour_length(VERIFY, FIT[-1], sim, win, o0))
            fit_l = order_poses(FIT, (0., 0.), sim, win, o0)
            ver_l = order_poses(VERIFY, fit_l[-1], sim, win, o0)
            after = (tour_length(fit_l, (0., 0.), sim, win, o0)
                     + tour_length(ver_l, fit_l[-1], sim, win, o0))
            msgs.put(f"pose order optimized for leveled zero {o0:+.3f}: "
                     f"{before:.0f} -> {after:.0f} deg of real slew "
                     f"({100 * (1 - after / before):.0f}% less)")


        msgs.put(f"running {model_name} on HARDWARE ...")
        r = calibrate(wrapped, sensor, fit_l, ver_l, model=MODELS[model_name](),
                      abort=stop_event.is_set, record=record,
                      n_samples=cfg.n_samples,
                      stability_sd_g=cfg.tp3_stability_sd_mg / 1000.0,
                      stability_timeout_s=cfg.tp3_stability_timeout_s,
                      max_attempts=cfg.tp3_max_attempts,
                      sample_period_s=cfg.tp3_sample_period_s,
                      min_settle_s=cfg.tp3_min_settle_s,
                      stable_windows=cfg.tp3_stable_windows,
                      mean_drift_g=cfg.tp3_mean_drift_mg / 1000.0,
                      verification_thresholds=(
                          verification_settings["thresholds"]
                      ),
                      roll_main_max_abs_pitch_deg=(
                          cfg.roll_main_max_abs_pitch_deg
                      ),
                      solve_geometry=True, log=msgs.put)



        report = r["verification_report"]
        verdict = report["outcome"]
        msgs.put("")
        msgs.put(summary_title)
        for statistic in report["statistics"].values():
            msgs.put(_verification_line(statistic))
        msgs.put(f"Overall outcome : {verdict}")
        msgs.put("")
        msgs.put(f"raw error : {_number_text(r['raw_error_mg'], 2)} mg")
        msgs.put(
            f"corrected error : {_number_text(r['corrected_error_mg'], 2)} mg"
        )

        m = r['metrics']

        msgs.put(f"vector RMSE : {_number_text(m['vector_rmse_mg'], 2)} mg")
        msgs.put(f"worst pose : {_number_text(m['max_vector_mg'], 2)} mg")
        msgs.put(f"angular RMS : {_number_text(m['rms_angular_deg'], 3)} deg")

        geometry = r.get("axis_geometry_deg")

        if geometry is not None:
            msgs.put(
                f"X/Y axis non-orthogonality : "
                f"{geometry['xy_nonorthogonality']:+.3f} deg"
            )
            msgs.put(
                f"outer-axis Z tip : "
                f"{geometry['outer_z_tip']:+.3f} deg"
            )
            msgs.put(
                f"inner-axis Z tip : "
                f"{geometry['inner_z_tip']:+.3f} deg"
            )
        else:
            msgs.put("axis geometry : unavailable")

        model_parameters = r["model"].parameter_dict()
        coefficient_report = _coefficient_report_text(
            model_name=model_name,
            fit_pose_count=len(FIT),
            outcome=verdict,
            model_parameters=model_parameters,
            run_timestamp=run_timestamp,
        )

        artifact = {
            "timestamp": datetime.now().isoformat(),
            "campaign_status": verdict,
            "calibration_summary_title": summary_title,
            "model": model_name,
            "raw_error_mg": _json_number(r["raw_error_mg"]),
            "corrected_error_mg": _json_number(r["corrected_error_mg"]),
            "M": (
                r["model"].M.tolist()
                if hasattr(r["model"], "M")
                else None
            ),
            "b": (
                r["model"].b.tolist()
                if hasattr(r["model"], "b")
                else None
            ),
            "model_parameters": model_parameters,
            "coefficient_report_file": Path(coefficients_fname).name,
            "fit_pose_count": len(FIT),
            "requested_fit_pose_count": fit_pose_count,
            "verification_pose_count": len(VERIFY),

            "pose_set": {
                "name": f"sqac_g_optimal_{fit_pose_count}",
                "version": 1,
                "nested_counts": list(station_config.FIT_POSE_COUNTS),
            },

            "poses": record,

            # Legacy field; not the new 3-DOF geometry.
            "skew_x_deg": r["skew_x_deg"],

            "metrics": r["metrics"],
            "verification_report": report,
            "verification_settings": verification_settings,
            "leveling": level_info,

            "acquisition_settings": {
                "qualification_samples": cfg.n_samples,
                "qualification_windows": cfg.tp3_stable_windows,
                "measurement_samples": cfg.n_samples,
                "minimum_post_idle_settle_s": cfg.tp3_min_settle_s,
                "tp3_sd_threshold_mg": cfg.tp3_stability_sd_mg,
                "mean_drift_threshold_mg": cfg.tp3_mean_drift_mg,
                "timeout_s": cfg.tp3_stability_timeout_s,
                "max_attempts": cfg.tp3_max_attempts,
                "sample_period_s": cfg.tp3_sample_period_s,
            },

            "axis_geometry_deg": r.get("axis_geometry_deg"),
        }

        with open(fname, "w", encoding="utf-8") as f:
            json.dump(artifact, f, indent=2)

        with open(
                coefficients_fname, "w", encoding="utf-8", newline="\n") as f:
            f.write(coefficient_report)

        msgs.put(f"saved -> {fname}")
        msgs.put(f"coefficients saved -> {coefficients_fname}")

        if out_paths is not None:
            out_paths.append(fname)

    except Exception as e:
        if level_sense is not None:
            level_info["stability_events"] = level_sense.stability_events
            level_info["stability_settings"] = level_sense.settings
        if isinstance(e, leveling.NotInRange):
            level_info["range_search_failure"] = e.diagnostics

        operator_aborted = stop_event.is_set() or isinstance(e, leveling.Aborted)
        if operator_aborted:
            failure_kind = "operator_abort"
        elif isinstance(e, AcquisitionFailure):
            failure_kind = "acquisition"
        else:
            failure_kind = "system"

        if operator_aborted:
            msgs.put(f"RUN ABORTED: {e}")
        else:
            msgs.put(f"SYSTEM FAIL ({failure_kind}): {e}")

        failure_artifact = {
            "timestamp": datetime.now().isoformat(),
            "campaign_status": (
                "ABORTED" if operator_aborted else "SYSTEM_FAIL"
            ),
            "failure_kind": failure_kind,
            "failure_reason": str(e),
            "calibration_summary_title": summary_title,
            "model": model_name,
            "requested_fit_pose_count": fit_pose_count,
            "poses": record,
            "leveling": level_info,
            "verification_settings": verification_settings,

            "acquisition_settings": {
                "qualification_samples": cfg.n_samples,
                "qualification_windows": cfg.tp3_stable_windows,
                "measurement_samples": cfg.n_samples,
                "minimum_post_idle_settle_s": cfg.tp3_min_settle_s,
                "tp3_sd_threshold_mg": cfg.tp3_stability_sd_mg,
                "mean_drift_threshold_mg": cfg.tp3_mean_drift_mg,
                "timeout_s": cfg.tp3_stability_timeout_s,
                "max_attempts": cfg.tp3_max_attempts,
                "sample_period_s": cfg.tp3_sample_period_s,
            },
        }

        try:
            with open(failed_fname, "w") as f:
                json.dump(failure_artifact, f, indent=2)

            msgs.put(f"failure log saved -> {failed_fname}")

        except Exception as save_exc:
            msgs.put(f"could not save failure log: {save_exc}")

    finally:
        if motion:
            if park_target is not None and not stop_event.is_set():
                try:
                    msgs.put(
                        "returning to 005 null park: OUTER "
                        f"{park_target['outer']:+.3f}, INNER "
                        f"{park_target['inner']:+.3f} deg"
                    )
                    if not motion.park_at(
                            park_target["outer"], park_target["inner"]):
                        msgs.put("park skipped: stage reference is not valid")
                except Exception as park_exc:
                    msgs.put(f"PARK WARNING: {park_exc}")
            elif stop_event.is_set():
                msgs.put("park skipped after operator STOP")

            motion.close()

        if sensor:
            sensor.close()

        hw["motion"] = None

        for obj in (motion, sensor):
            ser = getattr(
                getattr(
                    getattr(obj, "_link", None),
                    "_transport",
                    None
                ),
                "_ser",
                None
            )

            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

        if announce_done:
            msgs.put("__done__")


def repeat_worker(run_count, model_name, do_level, fit_pose_count):
    """Run complete campaigns sequentially without blocking the Tk thread."""
    completed_paths = []
    try:
        for run_index in range(1, run_count + 1):
            if stop_event.is_set():
                msgs.put("repeatability stopped by operator")
                break
            msgs.put(f"repeatability run {run_index}/{run_count}")
            completed_before = len(completed_paths)
            worker(
                model_name,
                do_level,
                fit_pose_count,
                announce_done=False,
                out_paths=completed_paths,
            )
            if len(completed_paths) == completed_before:
                msgs.put(
                    f"repeatability stopped after run {run_index}: "
                    "campaign did not complete"
                )
                break
        msgs.put(
            f"repeatability complete: {len(completed_paths)}/{run_count} "
            "campaigns saved"
        )
    finally:
        msgs.put("__done__")

def pump():
    """Runs on the GUI thread. """
    while not msgs.empty():
        m = msgs.get()
        if m == "__done__":
            status.config(text="idle")
            run_btn.config(state="normal")
            rep_btn.config(state="normal")
        else:
            log(m)
    root.after(100, pump)


def on_stop():
    stop_event.set()
    m = hw.get("motion")
    if m:
        try:
            m.stop_now()
        except Exception:
            pass
    status.config(text="STOPPING")

    status.config(text="stopping...")







class OffsetMotion:
    """Add info here"""

    def __init__(self, motion, outer0=0.0, inner0=0.0, pair_moves=False):
        self._m = motion
        self._off = {"outer": float(outer0), "inner": float(inner0)}


        self._pair = pair_moves
        self._pending = None
        self._moveb_ok = True






    def move(self, axis, deg):
        target = deg + self._off[axis.lower()]

        if axis.upper() == "OUTER":
            target = wrap180(target)
            if self._pair:
                self._pending = target
                return
        if self._pair and axis.upper() == "INNER":
            return self._move_pair(target)
        return self._m.move(axis, target)
    def _move_pair(self, inner_t):
        outer_t, self._pending = self._pending, None
        if outer_t is not None and self._moveb_ok:
            try:
                self._m._link.command(f"MOVEB {outer_t} {inner_t}")
                self._m._wait_idle()
                return
            except CommandError:
                # Firmware without MOVEB, or it refused. Sequential still
                # works, so drop to it for the rest of the run.
                self._moveb_ok = False
        if outer_t is not None:
            self._m.move("OUTER", outer_t)
        self._m.move("INNER", inner_t)

    def status(self):
        if self._pending is not None:
            t, self._pending = self._pending, None
            self._m.move("OUTER", t)
        st = dict(self._m.status())
        for k, off in self._off.items():
            if isinstance(st.get(k), float):
                st[k] = st[k] - off
        return st

    def __getattr__(self, name):
        return getattr(self._m, name)

class LevelSense:
    """Live and stability-gated 005 reads on the campaign sensor link."""

    def __init__(self, dut, window_samples=10, stable_windows=2,
                 sd_threshold_deg=0.001, drift_threshold_deg=0.001,
                 timeout_s=20.0, sample_period_s=0.1, abort=None):
        self._dut = dut
        self._abort = abort
        self.stability_events = []
        self.settings = {
            "window_samples": window_samples,
            "stable_windows": stable_windows,
            "sd_threshold_deg": sd_threshold_deg,
            "drift_threshold_deg": drift_threshold_deg,
            "timeout_s": timeout_s,
            "sample_period_s": sample_period_s,
        }

    def _tilt_once(self):
        try:
            reading = leveling.parse_tilt(self._dut._link.command("TILT"))
            if reading is not None:
                reading["timestamp_s"] = time.time()
            return reading
        except Exception:
            return None

    def tilt_live(self):
        """Single fresh 005 read used while a bounded recovery scan is moving."""
        return self._tilt_once()

    def tilt_all(self):
        """Return a 005 mean only after two fresh stability windows pass."""
        started = time.time()
        try:
            event = leveling.wait_for_stable_tilt(
                self._tilt_once, abort=self._abort, **self.settings)
        except leveling.Unstable as exc:
            event = dict(exc.diagnostics)
            event["started_timestamp_s"] = started
            event["failure_reason"] = str(exc)
            self.stability_events.append(event)
            raise
        event["started_timestamp_s"] = started
        self.stability_events.append(event)
        mean = dict(event["mean"])
        mean["_stable_window_means"] = [
            dict(window["mean"]) for window in event["stable_windows"]
        ]
        return mean

    # Recovery uses tilt_live() while the bounded stage scan is moving.










#building the actual GUI
root = tk.Tk()
root.title(f"Accelerometer Calibration Station — {GUI_BUILD}")
root.geometry("850x500")

top = ttk.Frame(root, padding= 10)
top.pack(fill="x")

ui_cfg = station_config.load()
initial_model = (ui_cfg.default_model
                 if ui_cfg.default_model in MODELS else "SQAC24")

ttk.Label(top, text = "Model:").pack(side="left")
model_var = tk.StringVar(value=initial_model)
model_box = ttk.Combobox(top, textvariable=model_var,
                         values=MODEL_NAMES,
                         state="readonly", width=12)


model_box.pack(side="left", padx=8)

ttk.Label(top, text="Fit poses:").pack(side="left")
pose_count_var = tk.StringVar(value=str(ui_cfg.fit_pose_count))
pose_count_box = ttk.Combobox(
    top,
    textvariable=pose_count_var,
    values=[str(value) for value in station_config.FIT_POSE_COUNTS],
    state="readonly",
    width=5,
)
pose_count_box.pack(side="left", padx=8)

level_var = tk.BooleanVar(value=ui_cfg.auto_level_default)
ttk.Checkbutton(top, text="Auto-level first", variable=level_var).pack(side="left", padx=8)

run_btn = ttk.Button(top, text="Run Calibration")
run_btn.pack(side="left", padx=8)

rep_btn = ttk.Button(top, text="Repeatability")
rep_btn.pack(side="left", padx=8)

def on_repeat():
    stop_event.clear()
    run_btn.config(state="disabled")
    rep_btn.config(state="disabled")
    n = station_config.load().repeatability_runs
    pose_count = int(pose_count_var.get())
    status.config(text=f"repeatability x{n}...")
    log(f"--- REPEATABILITY x{n}: {model_var.get()}, {pose_count} fit poses ---")
    threading.Thread(target=repeat_worker,
                     args=(n, model_var.get(), level_var.get(), pose_count),
                     daemon=True).start()

rep_btn.config(command=on_repeat)

stop_btn = ttk.Button(top, text="Stop", command=on_stop)
stop_btn.pack(side="left", padx=8)

status = ttk.Label(root, text="idle", padding=(10,0))
status.pack(fill="x")

out = tk.Text(root, height=20, wrap="none")
out.pack(fill = "both", expand=True, padx=10, pady = 10)

def log(msg):
    out.insert("end", msg + "\n")
    out.see("end")

log(f"build: {GUI_BUILD}")
log("models loaded: " + ", ".join(MODEL_NAMES))
log(f"GUI source: {Path(__file__).resolve()}")
log(f"config source: {Path(station_config.__file__).resolve()}")
log(f"correctors source: {Path(correction_models.__file__).resolve()}")
log("correctors build: " + getattr(
    correction_models,
    "CORRECTORS_BUILD",
    "older file -- replace accel_cal/correctors.py",
))
if ui_cfg.default_model not in MODELS:
    log(f"CONFIG WARNING: default model {ui_cfg.default_model!r} is unavailable; "
        "using SQAC24")

def on_run():
    stop_event.clear()
    run_btn.config(state="disabled")
    rep_btn.config(state="disabled")
    status.config(text="running...")
    pose_count = int(pose_count_var.get())
    log(f"--- {model_var.get()}, {pose_count} fit poses ---")
    threading.Thread(target=worker,
                     args=(model_var.get(), level_var.get(), pose_count),
                     daemon=True).start()

run_btn.config(command=on_run)



root.after(100,pump)

root.mainloop()
