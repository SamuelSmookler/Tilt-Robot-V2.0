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
    start_inner_z=-0.84,
    rounds=6,
):
    """Fit the three observable rotary-axis geometry DOF.

    Axis representation:
        [outer_y, outer_z, inner_x, inner_z]

    Gravity + an unrestricted affine correction cannot determine the
    common yaw of the OUTER-X / INNER-Y axis pair about Z.

    Gauge choice:
        inner_x = 0

    Therefore the three fitted parameters are:
        p[0] = outer_y  -> relative X/Y-axis non-orthogonality in this gauge
        p[1] = outer_z  -> OUTER axis out-of-plane tip
        p[2] = inner_z  -> INNER axis out-of-plane tip

    outer_y should not be interpreted as the absolute physical Y component
    of the OUTER axis unless the chosen yaw gauge is physically established.
    """

    def cost(p):
        tips = (
            p[0],   # outer_y: relative in-plane misalignment
            p[1],   # outer_z
            0.0,    # inner_x: fixed yaw gauge
            p[2],   # inner_z
        )

        truth = truth_axes(fit_angles, tips)
        model = model_cls().fit(fit_meas, truth)

        return float(
            np.mean(np.abs(model.apply(fit_meas) - truth))
        )

    p = np.array([
        0.0,             # relative XY misalignment
        0.0,             # outer_z
        start_inner_z,   # inner_z
    ], dtype=float)

    prev = np.inf

    for _ in range(rounds):
        res = minimize(
            cost,
            p,
            method="Nelder-Mead",
            options={
                "xatol": 1e-6,
                "fatol": 1e-11,
                "maxiter": 4000,
            },
        )

        p = res.x

        if prev - res.fun < 1e-9:
            break

        prev = res.fun

    # Full four-component representation expected by truth_axes().
    return np.array([
        p[0],   # outer_y
        p[1],   # outer_z
        0.0,    # inner_x -- gauge
        p[2],   # inner_z
    ])

def  solve_skew(fit_angles, fit_meas, model_cls=Affine12, limit_deg=5.0):

    def cost(skew):
        truth = truth_block(fit_angles, skew)
        model = model_cls().fit(fit_meas, truth)
        return float (np.mean(np.abs(model.apply(fit_meas) - truth)))
        
    res = minimize_scalar(cost, bounds=(-limit_deg, limit_deg), method="bounded")
    return float(res.x)
