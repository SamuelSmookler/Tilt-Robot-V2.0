import math, pathlib, tomllib

from collections import namedtuple

CONFIG_PATH = pathlib.Path(__file__).resolve().parent / "station_config.toml"
INNER_HW_WINDOW_DEG = 130.4
MODELS = ("Affine12", "Bodega24", "BodegaR21", "SQAC30")
FIT_POSE_COUNTS = (26, 34, 42, 50)
ORDERS = ("optimized", "designed")

def _as_bool(v):
    if not isinstance(v, bool):
        raise ValueError("must be an unquoted TOML boolean (true/false)")
    return v

def _as_int(v):
    if isinstance(v, bool):
        raise ValueError("must be an integer, not a boolean")
    return int(v)
def _as_float(v):
    f = float(v)
    if not math.isfinite(f):
        raise ValueError("must be finite")
    return f

def _as_window(v):
    lo, hi = (_as_float(x) for x in v)
    if not lo < hi <= lo + 360.0:
        raise ValueError("must be [lo, hi] with lo < hi and hi - lo <= 360")
    return (lo, hi)

def _optional(cast):
    return lambda v: None if v is None else cast(v)





Setting = namedtuple("Setting", "attr section key default cast rule why")

SPEC = [
    Setting("stage_port", "ports", "stage_port", "COM8", str, None, ""),
    Setting("dut_port", "ports", "dut_port", "COM5", str, None, ""),
    Setting("dut_slot", "ports", "dut_slot", 0, _as_int, lambda v: 0 <= v <= 7, "must be 0..7"),
    Setting("inner_limit_deg", "poses", "inner_limit_deg", 120.0, _as_float,
            lambda v: 0 < v <= INNER_HW_WINDOW_DEG,
            f"outside the (0, {INNER_HW_WINDOW_DEG}] hardware window"), 
    Setting("pose_order", "poses", "order", "optimized", str, 
            lambda v: v in ORDERS, f"must be one of {ORDERS}"),
    Setting("fit_pose_count", "poses", "fit_pose_count", 26, _as_int,
            lambda v: v in FIT_POSE_COUNTS,
            f"must be one of {FIT_POSE_COUNTS}"),
    
    
    
    Setting("parking_angle_deg", "motion", "parking_angle_deg", 30.0, _as_float, 
            lambda v: 0 < v <= 90,
            "must be in (0, 90] -- the NEG home search must start positive"),
    
    Setting("outer_window_deg", "poses", "outer_window_deg", (0.0, 360.0),
            _as_window, None, ""),
    Setting("default_model", "campaign", "default_model", "Affine12", str,
            lambda v: v in MODELS, f"must be one of {MODELS}"),
    Setting("auto_level_default", "campaign", "auto_level_default", True,
            _as_bool, None, ""),
    Setting("n_samples", "campaign", "n_samples", 25, _as_int,
            lambda v: v >= 1, "must be >= 1"),
    Setting("tp3_sample_period_s", "campaign", "tp3_sample_period_s", 0.03,
            _as_float, lambda v: v >= 0, "must be >= 0"),
    Setting("tp3_min_settle_s", "campaign", "tp3_min_settle_s", 0.5,
            _as_float, lambda v: v >= 0, "must be >= 0"),
    Setting("tp3_stable_windows", "campaign", "tp3_stable_windows", 2,
            _as_int, lambda v: 2 <= v <= 5, "must be 2..5"),
    Setting("tp3_stability_sd_mg", "campaign", "tp3_stability_sd_mg", 3.0,
            _as_float, lambda v: v > 0, "must be > 0"),
    Setting("tp3_mean_drift_mg", "campaign", "tp3_mean_drift_mg", 1.0,
            _as_float, lambda v: v > 0, "must be > 0"),
    Setting("tp3_stability_timeout_s", "campaign", "tp3_stability_timeout_s", 10.0,
            _as_float, lambda v: v > 0, "must be > 0"),
    Setting("tp3_max_attempts", "campaign", "tp3_max_attempts", 3,
            _as_int, lambda v: 1 <= v <= 10, "must be 1..10"),
    Setting("simultaneous_moves", "campaign", "simultaneous_moves", True,
            _as_bool, None, ""),
    Setting("motion_timeout_s", "motion", "timeout_s", 120.0, _as_float,
            lambda v: v > 0, "must be > 0"),
    Setting("leveling_tolerance_deg", "leveling", "tolerance_deg", None,
            _optional(_as_float), lambda v: v is None or v > 0, "must be > 0"),
    Setting("leveling_coarse_tol_g", "leveling", "coarse_tol_g", None,
            _optional(_as_float), lambda v: v is None or v > 0, "must be > 0"),
    Setting("leveling_window_samples", "leveling", "window_samples", 10,
            _as_int, lambda v: v >= 2, "must be >= 2"),
    Setting("leveling_stable_windows", "leveling", "stable_windows", 2,
            _as_int, lambda v: 2 <= v <= 5, "must be 2..5"),
    Setting("leveling_sd_threshold_deg", "leveling", "sd_threshold_deg", 0.001,
            _as_float, lambda v: v > 0, "must be > 0"),
    Setting("leveling_drift_threshold_deg", "leveling", "drift_threshold_deg", 0.001,
            _as_float, lambda v: v > 0, "must be > 0"),
    Setting("leveling_timeout_s", "leveling", "timeout_s", 20.0,
            _as_float, lambda v: v > 0, "must be > 0"),
    Setting("leveling_sample_period_s", "leveling", "sample_period_s", 0.1,
            _as_float, lambda v: v >= 0, "must be >= 0"),
    Setting("repeatability_runs", "repeatability", "runs", 6, _as_int,
            lambda v: 2 <= v <= 50, "must be 2..50"),
    Setting("runs_dir", "output", "runs_dir", "runs", str, None, ""),
    Setting("verbose", "output", "verbose", False, _as_bool, None, ""),
]

    
    
_MISSING = object()

def load(path=CONFIG_PATH):
    raw, warnings = _read_toml(path)
    _warn_unknown(raw, warnings)
    
    values = {}
    for s in SPEC:
        v = _get(raw, s.section, s.key, _MISSING)
        if v is _MISSING:
            values[s.attr] = s.default
            continue
        try:
            v = s.cast(v)
            if s.rule and not s.rule(v):
                raise ValueError(s.why)
            values[s.attr] = v
        except (TypeError, ValueError, OverflowError) as exc:
            warnings.append(f"config {s.key}={v!r} {exc} -- using {s.default!r}")
            values[s.attr] = s.default
    return Config(values, warnings)

def _warn_unknown(raw, warnings):
    known = {}
    for s in SPEC:
        known.setdefault(s.section, set()).add(s.key)
    for sec, body in raw.items():
        if sec not in known:
            warnings.append(f"config: unknown section [{sec}] -- ignored")
        elif isinstance(body, dict):
            for k in body:
                if k not in known[sec]:
                    warnings.append(f"config: unknown key '{k}' in [{sec}] "
                                    "-- ignored (typo, or wrong section?)")
    
    
    


def _get(raw, section, key, default):
    try:
        return raw[section][key]
    except (KeyError, TypeError):
        return default
    
def _read_toml(path):
    warnings = []
    if not pathlib.Path(path).exists():
        return {}, [f"no config at {path} -- using defaults"]
    try:
        with open(path, "rb") as f:
            return tomllib.load(f), warnings
    except tomllib.TOMLDecodeError as exc:
        return {}, [f"config unreadable ({exc}) -- using defaults"]
    
class Config:
    def __init__(self, values, warnings):
        self.__dict__.update(values)
        self.warnings = list(warnings)
        
    def snapshot(self):
        return {k: v for k, v in self.__dict__.items() if k != "warnings"}
