"""Analytical surfaces used by the geometry simulator."""

from dataclasses import dataclass


@dataclass(frozen=True)
class FlatSurface:
    """Horizontal steel plate at z = height, with outward normal +z."""

    height: float = 0.0

    def signed_distance(self, point: tuple[float, float]) -> float:
        return point[1] - self.height

    def closest_point(self, point: tuple[float, float]) -> tuple[float, float]:
        return point[0], self.height

    def normal(self, _point: tuple[float, float]) -> tuple[float, float]:
        return 0.0, 1.0
