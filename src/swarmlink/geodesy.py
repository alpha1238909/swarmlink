"""Small-area WGS84 latitude/longitude conversion to SwarmLink's local ENU frame."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .core import Position

_EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class GeoPoint:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 0.0

    def __post_init__(self) -> None:
        if not -90 <= self.latitude_deg <= 90 or not -180 <= self.longitude_deg <= 180:
            raise ValueError("latitude/longitude are outside their valid ranges")


@dataclass(frozen=True)
class LocalFrame:
    """Equirectangular local tangent approximation around one project origin.

    It is appropriate for school fields, test sites, and local rover/drone
    missions—not continental-scale navigation or aviation use.
    """

    origin: GeoPoint

    def to_enu(self, point: GeoPoint) -> Position:
        latitude_0 = math.radians(self.origin.latitude_deg)
        delta_latitude = math.radians(point.latitude_deg - self.origin.latitude_deg)
        delta_longitude = math.radians(point.longitude_deg - self.origin.longitude_deg)
        return Position(
            _EARTH_RADIUS_M * delta_longitude * math.cos(latitude_0),
            _EARTH_RADIUS_M * delta_latitude,
            point.altitude_m - self.origin.altitude_m,
        )

    def to_geographic(self, position: Position) -> GeoPoint:
        latitude_0 = math.radians(self.origin.latitude_deg)
        latitude = self.origin.latitude_deg + math.degrees(position.y_m / _EARTH_RADIUS_M)
        longitude = self.origin.longitude_deg + math.degrees(position.x_m / (_EARTH_RADIUS_M * math.cos(latitude_0)))
        return GeoPoint(latitude, longitude, self.origin.altitude_m + position.z_m)
