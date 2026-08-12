import numpy as np
import math
from scipy.spatial.transform import Rotation as Rotation




def rot(axis, degrees):

    if axis == "x":
        rotation = Rotation.from_euler('x', degrees, degrees=True)
        
        
        return rotation.as_matrix()
        
    if axis == "y":
        rotation = Rotation.from_euler('y', degrees, degrees=True)
        
        #doesnt output automatically as matrix, needs .as_matrix for next def
        return rotation.as_matrix()

    raise ValueError(f"unknown axis: {axis!r}")

#ORIGINAL

# def a_true_from_angles(outer_deg, inner_deg): 
    
#     R = rot('x', outer_deg) @ rot('y', inner_deg)
#     return R.T @ np.array([0,0,1])
    

#TESTING 

def a_true_from_angles(outer_deg, inner_deg, skew_x_deg=0.0):
    if skew_x_deg == 0.0:
        R = rot('x', outer_deg) @ rot('y', inner_deg)      # your existing line, untouched
    else:
        axis = rot('x', skew_x_deg) @ np.array([0.0, 1.0, 0.0])   # y-axis, tipped in the y-z plane
        Ri = Rotation.from_rotvec(np.deg2rad(inner_deg) * axis).as_matrix()
        R = rot('x', outer_deg) @ Ri
    return R.T @ np.array([0, 0, 1])