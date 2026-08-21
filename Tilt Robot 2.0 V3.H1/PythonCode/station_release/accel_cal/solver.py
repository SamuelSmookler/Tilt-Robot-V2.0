import numpy as np

from .model import AccelModel

def solve_linear(a_true, a_meas):
    a_true = np.asarray(a_true, dtype=float)
    a_meas = np.asarray(a_meas, dtype=float)
    n = a_true.shape[0]          # number of poses = number of rows
    
    
    
    #append a column of ones
    X = np.hstack([a_true, np.ones((n,1))])

    results = np.linalg.lstsq(X, a_meas, rcond=None)
    solutions = np.linalg.matrix_transpose(results[0])
    
    #stops before 3 needed 2d handling
    M = solutions[:, 0:3]
    b = solutions[:, 3]
    
    
    
    return AccelModel(M,b)
