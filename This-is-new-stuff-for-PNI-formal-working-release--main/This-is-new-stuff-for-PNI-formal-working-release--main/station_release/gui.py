import tkinter as tk
from tkinter import ttk

#added to prevent bloat and "mailbox"
import threading, queue
from station.protocol import Link

#remenant from testing
# from station.mock_devices import Rig, MockMotion, MockSensor
# from station.clients import MotionClient, SensorClient

from station.hardware import RealMotion, RealDUT



from station.sequencer import calibrate
from accel_cal.correctors import Affine12, Bodega24, BodegaR21

MODELS = {"Affine12": Affine12, "Bodega24": Bodega24, "BodegaR21": BodegaR21}

import json
from datetime import datetime

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

def worker(model_name, do_level=True, announce_done=True, out_paths=None):
    """Run on background thread. Must not touch widgets (Tk is single-threaded)."""
    # #added for actual motion
    motion = sensor = None
    
    
    cfg = station_config.load()
    for w in cfg.warnings:
        msgs.put(f"CONFIG WARNING: {w}")
    
    FIT, VERIFY, DROPPED = build_pose_set(inner_limit=cfg.inner_limit_deg)     
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
        level_info = {"enabled": True}
        
        if do_level:
        # sense = LevelSense(sensor)
        
            msgs.put("auto-leveling: DUT coarse, then 005 fine...")
            out = leveling.auto_level(motion, LevelSense(sensor).accel,
                                    LevelSense(sensor).tilt_all,
                                    tilt_fast=LevelSense(sensor).tilt_fast,
                                    log=msgs.put, abort=stop_event.is_set)
            st = motion.status()
            o0, i0 = st["outer"], st["inner"]
            level_info.update({"offsets_deg": {"outer": o0, "inner": i0},
                            "fine_deg": out["fine"],
                            "tilt_mapping": out.get("tilt_mapping")})
            msgs.put(f"leveled zero: OUTER {o0:+.4f}  INNER {i0:+.4f}")
        else:
            msgs.put("auto-level SKIPPED -- running from the mechanical zero")

        tag = "leveled" if do_level else "unleveled"
        fname = f"{cfg.runs_dir}/run_{datetime.now():%Y%m%d_%H%M%S}_{tag}.json"
        
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
        record = []
        r = calibrate(wrapped, sensor, fit_l, ver_l, model=MODELS[model_name](),
                      abort=stop_event.is_set, record=record,
                      settle_s=cfg.settle_s, n_samples=cfg.n_samples,
                      solve_geometry=True)


        
        
        msgs.put(f"raw error : {r['raw_error_mg']:.2f} mg")
        msgs.put(f"corrected error : {r['corrected_error_mg']:.2f} mg")
        msgs.put(f"verdict : {'PASS' if r['corrected_error_mg'] < 25 else 'FAIL'}")
        
        m = r['metrics']
        msgs.put(f"vector RMSE :{m['vector_rmse_mg']:.2f} mg")
        msgs.put(f"worst pose :{m['max_vector_mg']:.2f} mg")
        msgs.put(f"angular RMS : {m['rms_angular_deg']:.3f} deg")
        msgs.put(f"fixture skew : {r['skew_x_deg']:+.2f} deg"
                 
                 
        )
        
        artifact = {
            "timestamp"         : datetime.now().isoformat(),
            "model"             : model_name,
            "raw_error_mg"      : r["raw_error_mg"],
            "corrected_error_mg": r["corrected_error_mg"],
            "M"                 : r["model"].M.tolist() if hasattr(r["model"], "M") else None,
            "b"                 : r["model"].b.tolist() if hasattr(r["model"], "b") else None,
            "poses"             : record,
            "skew_x_deg"        : r["skew_x_deg"],
            "metrics"           : r['metrics'],
            "leveling"          : level_info,
            "axis_tips_deg"     : r.get("axis_tips_deg"),
        }
        with open(fname, "w") as f:
            json.dump(artifact, f, indent=2)
        msgs.put(f"saved -> {fname}")
        if out_paths is not None:
            out_paths.append(fname)
        
            
        
        
        
        
    except Exception as e:
        msgs.put(f"ERROR: {e}")
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

    
def repeat_worker(n, model_name, do_level):
    """repeats for repeatability and reliability test
    """
    from ai_assisted import repeatability               # we don't have this 
    paths = []
    for i in range(n):
        if stop_event.is_set():
            msgs.put(f"stopped after {i} of {n} runs")
            break
        msgs.put(f"\n--- repeatability run {i + 1}/{n} ---")
        worker(model_name, do_level, announce_done=False, out_paths=paths)
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

    def __init__(self, dut):
        self._dut = dut

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
            return leveling.parse_tilt(self._dut._link.command("TILT"))
        except Exception:
            return None

    def tilt_all(self):                         # fine: the electrolytic stuff
        time.sleep(leveling.SETTLE_S)
        xs, ys = [], []
        for _ in range(leveling.N_AVG):
            t = self._tilt_once()
            if t is None or not t["ack_ok"]:
                return None
            xs.append(t["x"]); ys.append(t["y"])
        return {"x": sum(xs) / len(xs), "y": sum(ys) / len(ys)}

    def tilt_fast(self):                        # rail search: might be unecissary
        time.sleep(leveling.RAIL_SEARCH_SETTLE_S)
        t = self._tilt_once()
        return None if t is None or not t["ack_ok"] else {"x": t["x"], "y": t["y"]}










#building the actual GUI
root = tk.Tk()
root.title("Acceleromter Calibration Station")
root.geometry("700x500")

top = ttk.Frame(root, padding= 10)
top.pack(fill="x")

ttk.Label(top, text = "Model:").pack(side="left")
model_var = tk.StringVar(value=station_config.load().default_model)
model_box = ttk.Combobox(top, textvariable=model_var, values=["Affine12", "Bodega24", "BodegaR21"], state="readonly", width=12)


model_box.pack(side="left", padx=8)

level_var = tk.BooleanVar(value=True)
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
    status.config(text=f"repeatability x{n}...")
    log(f"--- REPEATABILITY x{n}: {model_var.get()} ---")
    threading.Thread(target=repeat_worker,
                     args=(n, model_var.get(), level_var.get()),
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
    status.config(text="running...")
    log(f"--- {model_var.get()} ---")
    threading.Thread(target=worker, args=(model_var.get(), level_var.get()), daemon=True).start()

run_btn.config(command=on_run)



root.after(100,pump)

root.mainloop()

