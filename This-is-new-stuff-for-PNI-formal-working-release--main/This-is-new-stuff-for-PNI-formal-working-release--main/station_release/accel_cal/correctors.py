import numpy as np
from .solver import solve_linear

class Affine12:
    """12-coefficient affine corrector w/  corrected = A·measured + b."""
    def fit(self, measured, true):
        self._m = solve_linear(measured, true)   # fits  = A·measured + b 
        self.M = self._m.M
        self.b = self._m.b
        return self

    def apply(self, measured):
        return self._m.forward(measured) #review what ._m. does 
        
class Bodega24:
    """24-coefficient polynomial corrector w/ 8 per axis"""

    
    @staticmethod #why did i do this again
    def _feat_x(m):
        x,y,z = m[:,0], m[:,1], m[:,2]
        return np.column_stack([np.ones_like(x), x, x**2, x**3, y, y**2, z, z**2])
    
    @staticmethod
    def _feat_y(m):
        x,y,z = m[:,0], m[:,1], m[:,2]
        return np.column_stack([np.ones_like(x), x, x**2, y, y**2, y**3, z, z**2])
        
    @staticmethod
    def _feat_z(m):
        x,y,z = m[:,0], m[:,1], m[:,2]
        return np.column_stack([np.ones_like(x), x, x**2, y, y**2, z, z**2, z**3]) 
    
    def fit(self, measured, true):
        measured = np.asarray(measured, float)
        true = np.asarray(true, float)
        self._cx = np.linalg.lstsq(self._feat_x(measured), true[:,0], rcond=None)[0]
        self._cy = np.linalg.lstsq(self._feat_y(measured), true[:,1], rcond=None)[0]
        self._cz = np.linalg.lstsq(self._feat_z(measured), true[:,2], rcond=None)[0]
        return self
    
    def apply(self, measured):
        measured = np.asarray(measured, float)
        cx = self._feat_x(measured) @ self._cx
        cy = self._feat_y(measured) @ self._cy
        cz = self._feat_z(measured) @ self._cz
        return np.column_stack([cx, cy, cz])
    
class BodegaR21:
    @staticmethod
    def _feats(m, own):
        x, y, z = m[:, 0], m[:, 1], m[:, 2]
        o = m[:, own]
        others = [c for i, c in enumerate((x,y,z)) if i != own]
        return np.column_stack([np.ones_like(x), o, o**3,
                                others[0], others[1],
                                x**2 - z**2, y**2 - z**2])
        
    def fit(self, measured, true):
        measured = np.asarray(measured, float)
        true = np.asarray(true, float)
        self._c = [np.linalg.lstsq(self._feats(measured, ax), true[:, ax], rcond=None)[0] for ax in range(3)]
        
        return self
    
    
    def apply(self, measured):
        measured = np.asarray(measured, float)
        return np.column_stack([self._feats(measured, ax) @ self._c[ax]
                                for ax in range(3)])