import numpy as np
from scipy.optimize import minimize_scalar, minimize
from scipy.spatial.transform import Rotation as Rotation


from .simulator import a_true_from_angles
from .correctors import Affine12

_EZ = np.array([0.0, 0.0, 1.0])



def truth_block(angles, skew_x_deg=0.0):
    "Truth vectors for a list of (outer, inner) pairs, under a skew hypothesis"
    
    
    
    return np.array([a_true_from_angles(o, i,  skew_x_deg) for o, i in angles])

    
#added for testing/correcting truth vectors with tipped axis (please work)
def truth_axes(angles, tips):
    oy, oz, ix, iz, = tips
    u = np.array([1.0, np.tan(np.radians(oy)), np.tan(np.radians(oz))])
    v = np.array([np.tan(np.radians(ix)), 1.0, np.tan(np.radians(iz))])
    u /= np.linalg.norm(u)
    v /= np.linalg.norm(v)
    return np.array([
        (Rotation.from_rotvec(np.radians(o) * u)
         * Rotation.from_rotvec(np.radians(i) * v)).as_matrix().T @ _EZ
        for o, i in angles])
        
        
        
        
        
        
# def solve_axes(fit_angles, fit_meas, model_cls=Affine12, start_skew=-0.84):
#     """solve_skew with 2 degrees of freedom rather than 1"""
#     def cost(tips):
#         truth = truth_axes(fit_angles, tips)
#         model = model_cls().fit(fit_meas, truth)
#         return float(np.mean(np.abs(model.apply(fit_meas) - truth)))

#     res = minimize(cost, [0.0, 0.0, 0.0, start_skew], method="Nelder-Mead", options={"xatol": 1e-4, "fatol": 1e-9, "maxiter": 2000})          
    
#     return res.x
          
       
def solve_axes(
    fit_angles,
    fit_meas,
    model_cls=Affine12,
    start_outer_y=0.0,
    start_inner_x=0.0,
    rounds=6,
):
    """Solve the two IDENTIFIABLE axis-direction parameters.

    An axis direction has 2 DOF, so 4 for two axes -- but only 2 are
    observable. Tipping outer toward z, or inner toward z, mimics a constant
    DUT rotation, which the affine M absorbs for free; fitting them makes
    them wander +/-0.24 deg between restarts with no accuracy gain. The two
    tips change the shape of the swept sphere and are pinned by the data.

    Nelder-Mead is restarted until the cost stops improving: scipy steps a
    coordinate that starts at exactly 0.0 by only 0.00025, so a single pass
    can stall before reaching a tip near 1 deg (this cost us 2.19 vs 1.28 mg
    on 2026-08-07).
    """
    def cost(tipping):
        truth = truth_axes(fit_angles, (tipping[0], 0.0, tipping[1], 0.0))
        model = model_cls().fit(fit_meas, truth)
        return float(np.mean(np.abs(model.apply(fit_meas) - truth)))

    z, prev = np.array([start_outer_y, start_inner_x]), np.inf
    for _ in range(rounds):
        res = minimize(cost, tipping, method="Nelder-Mead",
                       options={"xatol": 1e-6, "fatol": 1e-11, "maxiter": 4000})
        tipping = res.x
        if prev - res.fun < 1e-9:
            break
        prev = res.fun
    return np.array([tipping[0], 0.0, tipping[1], 0.0])


def  solve_skew(fit_angles, fit_meas, model_cls=Affine12, limit_deg=5.0):

    def cost(skew):
        truth = truth_block(fit_angles, skew)
        model = model_cls().fit(fit_meas, truth)
        return float (np.mean(np.abs(model.apply(fit_meas) - truth)))
        
    res = minimize_scalar(cost, bounds=(-limit_deg, limit_deg), method="bounded")
    return float(res.x)