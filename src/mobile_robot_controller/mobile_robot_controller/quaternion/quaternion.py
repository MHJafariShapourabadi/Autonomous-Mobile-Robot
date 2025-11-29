import math


class Quaternion:
    """
    A mathematically correct, numerically stable quaternion class.
    Stores quaternions as (x, y, z, w), where w is the scalar part.
    """

    __slots__ = ("x", "y", "z", "w")

    # -------------------------------------------------------------
    # Construction
    # -------------------------------------------------------------
    def __init__(self, x: float, y: float, z: float, w: float):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.w = float(w)

    @staticmethod
    def from_euler(roll: float, pitch: float, yaw: float):
        """
        Convert Euler angles (r, p, y) to quaternion.
        Uses the intrinsic XYZ (roll, pitch, yaw) rotation convention.
        """
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy

        return Quaternion(x, y, z, w)

    # -------------------------------------------------------------
    # Basic algebra
    # -------------------------------------------------------------
    def copy(self):
        return Quaternion(self.x, self.y, self.z, self.w)

    def __repr__(self):
        return f"Quaternion(x={self.x}, y={self.y}, z={self.z}, w={self.w})"

    def __mul__(self, other):
        """
        Quaternion multiplication: q = self ⊗ other.
        Corresponds to applying other first, then self.
        """
        if not isinstance(other, Quaternion):
            raise TypeError("Quaternion can only multiply another Quaternion.")

        x1, y1, z1, w1 = self.x, self.y, self.z, self.w
        x2, y2, z2, w2 = other.x, other.y, other.z, other.w

        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

        return Quaternion(x, y, z, w)

    # -------------------------------------------------------------
    # Conjugate, Norm, Normalization, Inverse
    # -------------------------------------------------------------
    def conjugate(self):
        return Quaternion(-self.x, -self.y, -self.z, self.w)

    def norm_squared(self):
        return self.x*self.x + self.y*self.y + self.z*self.z + self.w*self.w

    def norm(self):
        return math.sqrt(self.norm_squared())

    def normalize(self):
        n = self.norm()
        if n == 0:
            raise ZeroDivisionError("Cannot normalize a zero quaternion.")
        inv = 1.0 / n
        self.x *= inv
        self.y *= inv
        self.z *= inv
        self.w *= inv
        return self

    def normalized(self):
        q = self.copy()
        q.normalize()
        return q

    def inverse(self):
        """
        q^{-1} = q* / ||q||^2
        """
        norm_sq = self.norm_squared()
        if norm_sq == 0:
            raise ZeroDivisionError("Cannot invert a zero quaternion.")

        return Quaternion(
            -self.x / norm_sq,
            -self.y / norm_sq,
            -self.z / norm_sq,
             self.w / norm_sq
        )

    # -------------------------------------------------------------
    # Conversions
    # -------------------------------------------------------------
    def to_euler(self):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        using the intrinsic XYZ convention.
        """
        x, y, z, w = self.x, self.y, self.z, self.w

        # Roll (x-axis rotation)
        t0 = 2.0 * (w*x + y*z)
        t1 = 1.0 - 2.0 * (x*x + y*y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = 2.0 * (w*y - z*x)
        t2 = max(-1.0, min(1.0, t2))  # numerical stability
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = 2.0 * (w*z + x*y)
        t4 = 1.0 - 2.0 * (y*y + z*z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    # -------------------------------------------------------------
    # Rotation of a 3D vector
    # -------------------------------------------------------------
    def rotate_vector(self, v):
        """
        Rotate a 3D vector v = (vx, vy, vz) using this quaternion.

        Uses q * v * q^{-1} with v treated as (vx, vy, vz, 0).
        """
        vx, vy, vz = v

        # Convert v into a pure quaternion
        vq = Quaternion(vx, vy, vz, 0.0)

        # Apply rotation
        rotated = self * vq * self.inverse()

        return (rotated.x, rotated.y, rotated.z)
