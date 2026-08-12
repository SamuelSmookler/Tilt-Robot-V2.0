import numpy as np

def angular_error_deg(a, b):
    
    a = np.asarray(a, float); b = np.asarray(b, float)
    dot = np.sum(a * b, axis=1)
    norms = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
    cos = np.clip(dot / norms, -1.0, 1.0)
    return np.degrees(np.arccos(cos))

    
def verification_metrics(corrected, truth):
    """Full spec report for set of verify poses"""
    corrected = np.asarray(corrected, float); truth = np.asarray(truth, float)
    err = corrected - truth
    vec = np.linalg.norm(err, axis=1)
    ang = angular_error_deg(corrected, truth)
    
    return{
        "per_axis_rmse_mg":       (np.sqrt(np.mean(err**2, axis=0))* 1000).round(2).tolist(),
        "vector_rmse_mg":         float(np.sqrt(np.mean(vec**2)) * 1000),
        "max_vector_mg":          float(vec.max() * 1000),
        "p95_vector_mg":          float(np.percentile(vec, 95) * 1000),
        "rms_angular_deg":        float(np.sqrt(np.mean(ang**2))),
        "max_angular_deg":        float(ang.max()),
        "n_poses":                len(vec),
        
        
        
    }