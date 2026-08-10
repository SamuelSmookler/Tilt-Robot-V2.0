import numpy as np
import time
from accel_cal.simulator import a_true_from_angles, rot
from accel_cal.solver import solve_linear
from accel_cal.correctors import Affine12
from accel_cal.geometry import solve_skew, truth_block, solve_axes, truth_axes
from accel_cal.metrics import verification_metrics



#fix zaber bullshit
def _wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def run_campaign(motion, sensor, poses, abort=None, record=None, phase="fit",
                 settle_s=1.0, n_samples=25, skew_x_deg=0.0):
    a_true_list = []
    a_meas_list = []
    for outer, inner in poses:
        if abort is not None and abort():
            raise RuntimeError("aborted by operator")

        motion.move("OUTER", outer)
        motion.move("INNER", inner)

        st = motion.status()
        act_o = st.get("outer", outer)
        act_i = st.get("inner", inner)

        # GATE 1: the stage must be check to see if it goes where it was requested, if larger that 0.5 diff, raises error. 
        # Difference was wrapped to get proper vlaues ie 180 and -180 be sme place
        
        if abs(_wrap180(act_o - outer)) > 0.5 or abs(_wrap180(act_i - inner)) > 0.5:
            raise RuntimeError(f"pose mismatch: commanded ({outer},{inner}) "
                               f"but stage at ({act_o:.2f},{act_i:.2f})")

        time.sleep(settle_s)              # let the fixture stop ringing

        reads = []
        for _ in range(n_samples):
            reads.append(sensor.read())
            time.sleep(0.03)              # so each sample is a fresh one
        reads = np.array(reads)
        reading = np.mean(reads, axis=0)  # one value per axis
        noise = np.std(reads, axis=0)     # per-pose noise, essentially free

        # GATE 2: gravity is always ~1 g; anything else is a dead/false sensor
        mag = np.linalg.norm(reading)
        if not (0.8 < mag < 1.2):
            raise RuntimeError(f"sensor reading {mag:.3f} g at pose "
                               f"({outer},{inner}) -- not physical")

        #CHANGED FOR TESTING
        a_true_list.append(a_true_from_angles(act_o, act_i, skew_x_deg))
        a_meas_list.append(reading)

        if record is not None:
            record.append({
                "phase": phase,
                "commanded": [outer, inner],
                "measured": [act_o, act_i],
                "reading": list(reading),
                "reading_std": list(noise),
            })

    return np.array(a_true_list), np.array(a_meas_list)


def calibrate(motion, sensor, fit_poses, verify_poses, model=None, abort=None,
              record=None, settle_s=1.0, n_samples=25, skew_x_deg=0.0, solve_geometry=False):
    if model is None:
        #change to change default
        model = Affine12()
    tips = None
    if solve_geometry and record is None:
        record = []    
        
    a_true_fit, a_meas_fit = run_campaign(motion, sensor, fit_poses, abort=abort,
                                          record=record, phase="fit",
                                          settle_s=settle_s, n_samples=n_samples, skew_x_deg=skew_x_deg)    
    
    if solve_geometry:
        fit_rows = [e for e in record if e ['phase'] == 'fit']
        ang = np.array([e["measured"] for e in fit_rows])
        tips = solve_axes(ang, a_meas_fit)
        skew_x_deg = float(tips[3])
        a_true_fit = truth_axes(ang, tips)
    
    
    model.fit(a_meas_fit, a_true_fit)                 


    a_true_v, a_meas_v = run_campaign(motion, sensor, verify_poses, abort=abort,
                                      record=record, phase="verify",
                                      settle_s=settle_s, n_samples=n_samples, skew_x_deg=skew_x_deg)
    
    
    if solve_geometry:
        ver_rows = [e for e in record if e['phase'] == 'verify']
        a_true_v = truth_axes(np.array([e['measured'] for e in ver_rows]), tips)
    
    
    raw_err = np.mean(np.abs(a_meas_v - a_true_v)) * 1000
    cor_err = np.mean(np.abs(model.apply(a_meas_v) - a_true_v)) * 1000

    return {"model": model, "raw_error_mg": raw_err, "corrected_error_mg": cor_err, 
            "skew_x_deg": skew_x_deg, "metrics": verification_metrics(model.apply(a_meas_v), a_true_v), "axis_tips_deg": None if tips is None else list(tips)}





