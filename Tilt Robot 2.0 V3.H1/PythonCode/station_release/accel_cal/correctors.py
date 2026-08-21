import numpy as np
from .solver import solve_linear


AXIS_NAMES = ("x", "y", "z")
CORRECTORS_BUILD = "SQAC24 sparse-24 model bundle (2026-08-13)"


def _measurements(values):
    """Return an (N, 3) float array and whether the input was one vector."""
    values = np.asarray(values, dtype=float)
    single = values.ndim == 1
    if single:
        values = values.reshape(1, -1)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("measurements must have shape (N, 3) or (3,)")
    return values, single


def _restore_shape(values, single):
    return values[0] if single else values

class Affine12:
    """12-coefficient affine corrector w/  corrected = A·measured + b."""
    def fit(self, measured, true):
        self._m = solve_linear(measured, true)   # fits  = A·measured + b 
        self.M = self._m.M
        self.b = self._m.b
        return self

    def apply(self, measured):
        return self._m.forward(measured) #review what ._m. does 

    def parameter_dict(self):
        return {
            "coefficient_count": 12,
            "equation": "corrected = M @ measured + b",
            "M": self.M.tolist(),
            "b": self.b.tolist(),
        }
        
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
        measured, single = _measurements(measured)
        cx = self._feat_x(measured) @ self._cx
        cy = self._feat_y(measured) @ self._cy
        cz = self._feat_z(measured) @ self._cz
        return _restore_shape(np.column_stack([cx, cy, cz]), single)

    def parameter_dict(self):
        return {
            "coefficient_count": 24,
            "feature_names_by_output": {
                "x": ["1", "x", "x^2", "x^3", "y", "y^2", "z", "z^2"],
                "y": ["1", "x", "x^2", "y", "y^2", "y^3", "z", "z^2"],
                "z": ["1", "x", "x^2", "y", "y^2", "z", "z^2", "z^3"],
            },
            "coefficients_by_output": {
                "x": self._cx.tolist(),
                "y": self._cy.tolist(),
                "z": self._cz.tolist(),
            },
        }
    
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
        measured, single = _measurements(measured)
        corrected = np.column_stack([self._feats(measured, ax) @ self._c[ax]
                                     for ax in range(3)])
        return _restore_shape(corrected, single)

    def parameter_dict(self):
        names = {}
        for axis, own in enumerate(AXIS_NAMES):
            others = [name for name in AXIS_NAMES if name != own]
            names[own] = ["1", own, f"{own}^3", *others,
                          "x^2-z^2", "y^2-z^2"]
        return {
            "coefficient_count": 21,
            "feature_names_by_output": names,
            "coefficients_by_output": {
                name: self._c[axis].tolist()
                for axis, name in enumerate(AXIS_NAMES)
            },
        }


class SQAC24:
    """Sparse SQAC corrector with exactly eight coefficients per output.

    BodegaR21's trace-free diagonal quadratic terms are retained, avoiding the
    near dependency ``1 ~= x^2 + y^2 + z^2`` on the gravity sphere. One fixed,
    fit-selected SQAC cross-quadratic term is then added per output.
    """

    CROSS_TERM_BY_OUTPUT = ("xy", "yz", "yz")

    @staticmethod
    def _cross_terms(measured):
        x, y, z = measured[:, 0], measured[:, 1], measured[:, 2]
        return {"xy": x * y, "xz": x * z, "yz": y * z}

    @classmethod
    def _feats(cls, measured, own):
        x, y, z = measured[:, 0], measured[:, 1], measured[:, 2]
        own_values = measured[:, own]
        others = [values for axis, values in enumerate((x, y, z))
                  if axis != own]
        selected_cross = cls._cross_terms(measured)[
            cls.CROSS_TERM_BY_OUTPUT[own]
        ]
        return np.column_stack([
            np.ones_like(x),
            own_values,
            own_values**3,
            others[0],
            others[1],
            x**2 - z**2,
            y**2 - z**2,
            selected_cross,
        ])

    @classmethod
    def feature_names(cls, own):
        own_name = AXIS_NAMES[own]
        others = [name for axis, name in enumerate(AXIS_NAMES) if axis != own]
        return ["1", own_name, f"{own_name}^3", *others,
                "x^2-z^2", "y^2-z^2", cls.CROSS_TERM_BY_OUTPUT[own]]

    def fit(self, measured, true):
        measured, _ = _measurements(measured)
        true, _ = _measurements(true)
        if measured.shape != true.shape:
            raise ValueError("measured and true arrays must have the same shape")

        designs = [self._feats(measured, axis) for axis in range(3)]
        for axis, design in enumerate(designs):
            rank = np.linalg.matrix_rank(design)
            if rank < design.shape[1]:
                raise ValueError(
                    f"SQAC24 {AXIS_NAMES[axis]} design is rank deficient "
                    f"({rank}/{design.shape[1]}); use more distinct poses")

        self._c = [
            np.linalg.lstsq(designs[axis], true[:, axis], rcond=None)[0]
            for axis in range(3)
        ]
        self.coefficients = np.vstack(self._c)
        self.condition_numbers = [float(np.linalg.cond(d)) for d in designs]
        return self

    def apply(self, measured):
        measured, single = _measurements(measured)
        if not hasattr(self, "_c"):
            raise RuntimeError("SQAC24 must be fitted before apply()")
        corrected = np.column_stack([
            self._feats(measured, axis) @ self._c[axis]
            for axis in range(3)
        ])
        return _restore_shape(corrected, single)

    def parameter_dict(self):
        return {
            "coefficient_count": 24,
            "coefficients_per_output": 8,
            "basis": "sparse spherical-quadratic axial-cubic",
            "cross_term_by_output": {
                name: self.CROSS_TERM_BY_OUTPUT[axis]
                for axis, name in enumerate(AXIS_NAMES)
            },
            "feature_names_by_output": {
                name: self.feature_names(axis)
                for axis, name in enumerate(AXIS_NAMES)
            },
            "coefficients_by_output": {
                name: self._c[axis].tolist()
                for axis, name in enumerate(AXIS_NAMES)
            },
            "fit_condition_number_by_output": {
                name: self.condition_numbers[axis]
                for axis, name in enumerate(AXIS_NAMES)
            },
        }


class SQAC30:
    """Spherical Quadratic-Axial Cubic corrector, 10 terms per output.

    The quadratic basis contains the five independent degree-two shapes on
    the unit sphere.  Difference-square terms avoid the near dependency
    1 ~= x^2 + y^2 + z^2 present in the original Bodega24 basis.
    """

    @staticmethod
    def _feats(measured, own):
        x, y, z = measured[:, 0], measured[:, 1], measured[:, 2]
        own_values = measured[:, own]
        others = [values for axis, values in enumerate((x, y, z))
                  if axis != own]
        return np.column_stack([
            np.ones_like(x),
            own_values,
            own_values**3,
            others[0],
            others[1],
            x**2 - z**2,
            y**2 - z**2,
            x * y,
            x * z,
            y * z,
        ])

    @classmethod
    def feature_names(cls, own):
        own_name = AXIS_NAMES[own]
        others = [name for axis, name in enumerate(AXIS_NAMES) if axis != own]
        return ["1", own_name, f"{own_name}^3", *others,
                "x^2-z^2", "y^2-z^2", "xy", "xz", "yz"]

    def fit(self, measured, true):
        measured, _ = _measurements(measured)
        true, _ = _measurements(true)
        if measured.shape != true.shape:
            raise ValueError("measured and true arrays must have the same shape")

        designs = [self._feats(measured, axis) for axis in range(3)]
        for axis, design in enumerate(designs):
            rank = np.linalg.matrix_rank(design)
            if rank < design.shape[1]:
                raise ValueError(
                    f"SQAC30 {AXIS_NAMES[axis]} design is rank deficient "
                    f"({rank}/{design.shape[1]}); use more distinct poses")

        self._c = [
            np.linalg.lstsq(designs[axis], true[:, axis], rcond=None)[0]
            for axis in range(3)
        ]
        self.coefficients = np.vstack(self._c)
        self.condition_numbers = [float(np.linalg.cond(d)) for d in designs]
        return self

    def apply(self, measured):
        measured, single = _measurements(measured)
        if not hasattr(self, "_c"):
            raise RuntimeError("SQAC30 must be fitted before apply()")
        corrected = np.column_stack([
            self._feats(measured, axis) @ self._c[axis]
            for axis in range(3)
        ])
        return _restore_shape(corrected, single)

    def parameter_dict(self):
        return {
            "coefficient_count": 30,
            "basis": "spherical-quadratic axial-cubic",
            "feature_names_by_output": {
                name: self.feature_names(axis)
                for axis, name in enumerate(AXIS_NAMES)
            },
            "coefficients_by_output": {
                name: self._c[axis].tolist()
                for axis, name in enumerate(AXIS_NAMES)
            },
            "fit_condition_number_by_output": {
                name: self.condition_numbers[axis]
                for axis, name in enumerate(AXIS_NAMES)
            },
        }
