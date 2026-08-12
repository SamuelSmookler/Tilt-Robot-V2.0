import tkinter as tk
from tkinter import ttk

#added to prevent bloat and "mailbox"
import threading, queue
from station.protocol import Link

#remenant from testing
# from station.mock_devices import Rig, MockMotion, MockSensor
# from station.clients import MotionClient, SensorClient

from station.hardware import RealMotion, RealDUT



from station.sequencer import calibrate, AcquisitionFailure
from accel_cal.correctors import Affine12, Bodega24, BodegaR21, SQAC30

MODELS = {
    "Affine12": Affine12,
    "Bodega24": Bodega24,
    "BodegaR21": BodegaR21,
    "SQAC30": SQAC30,
}

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

def worker(model_name, do_level=True, fit_pose_count=None, announce_done=True,
           out_paths=None):
    """Run on background thread. Must not touch widgets (Tk is single-threaded)."""
    # #added for actual motion
    motion = sensor = level_sense = None
    
    
    cfg = station_config.load()
    if fit_pose_count is None:
        fit_pose_count = cfg.fit_pose_count
    for w in cfg.warnings:
        msgs.put(f"CONFIG WARNING: {w}")

    Path(cfg.runs_dir).mkdir(parents=True, exist_ok=True)
    run_timestamp = datetime.now()
    tag = "leveled" if do_level else "unleveled"
    fname = f"{cfg.runs_dir}/run_{run_timestamp:%Y%m%d_%H%M%S}_{tag}.json"
    failed_fname = fname[:-5] + "_failed.json"
    record = []
    level_info = {"enabled": bool(do_level)}
    
    FIT, VERIFY, DROPPED = build_pose_set(
        inner_limit=cfg.inner_limit_deg,
        fit_pose_count=fit_pose_count,
    )
    msgs.put(f"pose set: {len(FIT)} fit + {len(VERIFY)} verification poses")
    msgs.put(f"TP3 gate: {cfg.n_samples} samples, <= "
             f"{cfg.tp3_stability_sd_mg:g} mg/axis, "
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
        if st.get("homed") == 1.0:
            msgs.put(f"parking at +{cfg.parking_angle_deg:g} before homing...")
            motion.park(cfg.parking_angle_deg)
        motion.home("OUTER", "NEG")
        motion.home("INNER", "NEG")
        st = motion.status()
        if st.get("homed") != 1.0:
            raise RuntimeError(
                f"homing did not complete (homed={st.get('homed')}, "
                f"outer_homed={st.get('outer_homed')} "
                f"inner_homed={st.get('inner_homed')})")
        
        

        o0 = i0 = 0.0
        
        if do_level:
        # sense = LevelSense(sensor)
        
            msgs.put("auto-leveling: DUT coarse, then adaptive 005 stability gate...")
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
                motion, level_sense.accel, level_sense.tilt_all,
                tilt_fast=level_sense.tilt_fast,
                log=msgs.put, abort=stop_event.is_set,
                tolerance_deg=(cfg.leveling_tolerance_deg
                               or leveling.TOLERANCE_DEG),
                coarse_tol_g=(cfg.leveling_coarse_tol_g
                              or leveling.COARSE_TOL_G),
            )
            st = motion.status()
            o0, i0 = st["outer"], st["inner"]
            level_info.update({"offsets_deg": {"outer": o0, "inner": i0},
                            "fine_deg": out["fine"],
                            "final_tilt_deg": out.get("final_tilt"),
                            "tilt_mapping": out.get("tilt_mapping"),
                            "stability_events": level_sense.stability_events,
                            "stability_settings": level_sense.settings})
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
                      solve_geometry=True, log=msgs.put)


        
        
        msgs.put(f"raw error : {r['raw_error_mg']:.2f} mg")
        msgs.put(f"corrected error : {r['corrected_error_mg']:.2f} mg")
        msgs.put(f"verdict : {'PASS' if r['corrected_error_mg'] < 25 else 'FAIL'}")
        
        m = r['metrics']
        per_axis_rmse = m['per_axis_rmse_mg']
        msgs.put(f"vector RMSE : {m['vector_rmse_mg']:.2f} mg")
        msgs.put(f"X-axis RMSE : {per_axis_rmse[0]:.2f} mg")
        msgs.put(f"Y-axis RMSE : {per_axis_rmse[1]:.2f} mg")
        msgs.put(f"Z-axis RMSE : {per_axis_rmse[2]:.2f} mg")
        msgs.put(f"worst pose : {m['max_vector_mg']:.2f} mg")
        msgs.put(f"angular RMS : {m['rms_angular_deg']:.3f} deg")
        axis_tips = r.get("axis_tips_deg")
        if axis_tips is not None and len(axis_tips) >= 4:
            msgs.put(f"outer-axis Z tip : {axis_tips[1]:+.3f} deg")
            msgs.put(f"inner-axis Z tip : {axis_tips[3]:+.3f} deg")
        else:
            msgs.put("axis tipping errors : unavailable")
        
        verdict = "PASS" if r["corrected_error_mg"] < 25 else "FAIL"
        artifact = {
            "timestamp"         : datetime.now().isoformat(),
            "campaign_status"   : verdict,
            "model"             : model_name,
            "raw_error_mg"      : r["raw_error_mg"],
            "corrected_error_mg": r["corrected_error_mg"],
            "M"                 : r["model"].M.tolist() if hasattr(r["model"], "M") else None,
            "b"                 : r["model"].b.tolist() if hasattr(r["model"], "b") else None,
            "model_parameters"  : r["model"].parameter_dict(),
            "fit_pose_count"    : len(FIT),
            "requested_fit_pose_count": fit_pose_count,
            "verification_pose_count": len(VERIFY),
            "pose_set"          : {
                "name": f"sqac_g_optimal_{fit_pose_count}",
                "version": 1,
                "nested_counts": list(station_config.FIT_POSE_COUNTS),
            },
            "poses"             : record,
            "skew_x_deg"        : r["skew_x_deg"],
            "metrics"           : r['metrics'],
            "leveling"          : level_info,
            "acquisition_settings": {
                "qualification_samples": cfg.n_samples,
                "measurement_samples": cfg.n_samples,
                "tp3_sd_threshold_mg": cfg.tp3_stability_sd_mg,
                "timeout_s": cfg.tp3_stability_timeout_s,
                "max_attempts": cfg.tp3_max_attempts,
                "sample_period_s": cfg.tp3_sample_period_s,
            },
            "axis_tips_deg"     : r.get("axis_tips_deg"),
        }
        with open(fname, "w") as f:
            json.dump(artifact, f, indent=2)
        msgs.put(f"saved -> {fname}")
        if out_paths is not None:
            out_paths.append(fname)
        
            
        
        
        
        
    except Exception as e:
        if level_sense is not None:
            level_info["stability_events"] = level_sense.stability_events
            level_info["stability_settings"] = level_sense.settings
        failure_kind = "acquisition" if isinstance(e, AcquisitionFailure) else "system"
        msgs.put(f"SYSTEM FAIL ({failure_kind}): {e}")
        failure_artifact = {
            "timestamp": datetime.now().isoformat(),
            "campaign_status": "SYSTEM_FAIL",
            "failure_kind": failure_kind,
            "failure_reason": str(e),
            "model": model_name,
            "requested_fit_pose_count": fit_pose_count,
            "poses": record,
            "leveling": level_info,
            "acquisition_settings": {
                "qualification_samples": cfg.n_samples,
                "measurement_samples": cfg.n_samples,
                "tp3_sd_threshold_mg": cfg.tp3_stability_sd_mg,
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
            try:
                motion.park()
            except Exception:
                pass
            motion.close()
        if sensor:
            sensor.close()
        hw["motion"] = None
        
        for obj in (motion, sensor):
            ser = getattr(getattr(getattr(obj, "_link", None), "_transport", None), "_ser", None)
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
        if announce_done:
            msgs.put("__done__")

    
def repeat_worker(n, model_name, do_level, fit_pose_count):
    """repeats for repeatability and reliability test
    """
    from ai_assisted import repeatability
    paths = []
    for i in range(n):
        if stop_event.is_set():
            msgs.put(f"stopped after {i} of {n} runs")
            break
        msgs.put(f"\n--- repeatability run {i + 1}/{n} ---")
        worker(model_name, do_level=do_level, fit_pose_count=fit_pose_count,
               announce_done=False, out_paths=paths)
    if len(paths) >= 2:
        try:
            repeatability.analyze(paths, log=msgs.put)
        except Exception as exc:
            msgs.put(f"analysis failed: {exc}")
            msgs.put("artifacts are safe; rerun offline with:")
            msgs.put("  python ai_assisted/repeatability.py " + " ".join(paths))
    else:
        msgs.put(f"only {len(paths)} run(s) completed -- need 2 or more")
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
    """The three callables auto_level needs, on the campaign's own DUT link."""

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

    def accel(self, axis):                      # coarse serch for the DUT itself values
        time.sleep(leveling.COARSE_SETTLE_S)
        ch = leveling.ACCEL_AXIS[axis]
        vals = []
        for _ in range(3):
            try:
                vals.append(self._dut.read()[ch])
            except (RuntimeError, KeyError, IndexError):
                return None
            time.sleep(0.05)
        return sum(vals) / len(vals)

    def _tilt_once(self):
        try:
            reading = leveling.parse_tilt(self._dut._link.command("TILT"))
            if reading is not None:
                reading["timestamp_s"] = time.time()
            return reading
        except Exception:
            return None

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

    def tilt_fast(self):                        # rail search: might be unecissary
        time.sleep(leveling.RAIL_SEARCH_SETTLE_S)
        t = self._tilt_once()
        return None if t is None or not t["ack_ok"] else {"x": t["x"], "y": t["y"]}










#building the actual GUI
root = tk.Tk()
root.title("Accelerometer Calibration Station")
root.geometry("850x500")

top = ttk.Frame(root, padding= 10)
top.pack(fill="x")

ui_cfg = station_config.load()

ttk.Label(top, text = "Model:").pack(side="left")
model_var = tk.StringVar(value=ui_cfg.default_model)
model_box = ttk.Combobox(top, textvariable=model_var,
                         values=station_config.MODELS,
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

