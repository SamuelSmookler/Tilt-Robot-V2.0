# programing of the affine

from dataclasses import dataclass, field
import numpy as np
import math

@dataclass
class AccelModel:
    
    M: np.ndarray = field(default_factory=lambda: np.eye(3))
    b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    def __post_init__(self):
    
        self.M = np.asarray(self.M, dtype=float).reshape(3,3)
        self.b = np.asarray(self.b, dtype=float).reshape(3)
    
    def forward(self, a_true):
    
        a_true = np.asarray(a_true, dtype=float)
        return a_true @ self.M.T + self.b
    
    
    
    def correct(self, a_meas: np.ndarray):
    
        a_meas = np.asarray(a_meas, dtype=float)
        return (a_meas - self.b) @ np.linalg.inv(self.M).T


    @classmethod
    def ideal(cls):
        
        return cls(np.eye(3), np.zeros(3))
    