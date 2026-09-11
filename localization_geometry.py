"""Conservative geometry for the CUMCM 2026 B bearing problem.

All polygons are convex and counter-clockwise.  Circular constraints are
represented by circumscribed regular polygons, so numerical approximation can
enlarge the feasible set but can never remove the true source.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float]
Polygon = List[Point]
DEG = math.pi / 180.0
EPS = 1e-8


@dataclass(frozen=True)
class Bearing:
    point: Point
    angle: float


@dataclass(frozen=True)
class Circle:
    center: Point
    radius: float


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def circumscribed_disk(center: Point, radius: float, sides: int = 180) -> Polygon:
    vertex_radius = radius / math.cos(math.pi / sides)
    return [
        (center[0] + vertex_radius * math.cos(2.0 * math.pi * k / sides),
         center[1] + vertex_radius * math.sin(2.0 * math.pi * k / sides))
        for k in range(sides)
    ]


def clip_halfplane(poly: Polygon, a: float, b: float, c: float) -> Polygon:
    """Clip by a*x+b*y<=c using Sutherland-Hodgman."""
    if not poly:
        return []
    output: Polygon = []
    previous = poly[-1]
    previous_value = a * previous[0] + b * previous[1] - c
    for current in poly:
        current_value = a * current[0] + b * current[1] - c
        current_inside = current_value <= EPS
        previous_inside = previous_value <= EPS
        if current_inside != previous_inside:
            denominator = previous_value - current_value
            t = 0.0 if abs(denominator) < EPS else previous_value / denominator
            output.append((previous[0] + t * (current[0] - previous[0]),
                           previous[1] + t * (current[1] - previous[1])))
        if current_inside:
            output.append(current)
        previous = current
        previous_value = current_value
    return output


def clip_disk(poly: Polygon, center: Point, radius: float, sides: int = 90) -> Polygon:
    """Conservatively intersect with a disk by tangent half-planes."""
    for k in range(sides):
        angle = 2.0 * math.pi * k / sides
        nx, ny = math.cos(angle), math.sin(angle)
        poly = clip_halfplane(poly, nx, ny,
                              radius + nx * center[0] + ny * center[1])
        if not poly:
            break
    return poly


def clip_bearing(poly: Polygon, observation: Bearing, error: float = DEG) -> Polygon:
    """Intersect with the forward wedge theta+-error."""
    x0, y0 = observation.point
    low = observation.angle - error
    high = observation.angle + error
    lx, ly = math.cos(low), math.sin(low)
    hx, hy = math.cos(high), math.sin(high)
    # cross(u_low, q-S)>=0
    poly = clip_halfplane(poly, ly, -lx, ly * x0 - lx * y0)
    # cross(u_high, q-S)<=0
    poly = clip_halfplane(poly, -hy, hx, -hy * x0 + hx * y0)
    return poly


def feasible_polygon(observations: Sequence[Bearing]) -> Polygon:
    poly = circumscribed_disk((0.0, 0.0), 1800.0)
    for observation in observations:
        poly = clip_bearing(poly, observation)
        # A direction response proves distance <= the source radius <= 1500 m.
        poly = clip_disk(poly, observation.point, 1500.0)
        if not poly:
            break
    return poly


def polygon_diameter(poly: Sequence[Point]) -> Tuple[float, Optional[Tuple[Point, Point]]]:
    best = 0.0
    pair: Optional[Tuple[Point, Point]] = None
    for i, a in enumerate(poly):
        for b in poly[i + 1:]:
            value = distance(a, b)
            if value > best:
                best, pair = value, (a, b)
    return best, pair


def _contains(circle: Circle, point: Point) -> bool:
    return distance(circle.center, point) <= circle.radius + 1e-7


def _diameter_circle(a: Point, b: Point) -> Circle:
    center = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return Circle(center, distance(a, b) / 2.0)


def _circumcircle(a: Point, b: Point, c: Point) -> Optional[Circle]:
    determinant = 2.0 * (a[0] * (b[1] - c[1]) +
                         b[0] * (c[1] - a[1]) +
                         c[0] * (a[1] - b[1]))
    if abs(determinant) < EPS:
        return None
    aa = a[0] * a[0] + a[1] * a[1]
    bb = b[0] * b[0] + b[1] * b[1]
    cc = c[0] * c[0] + c[1] * c[1]
    ux = (aa * (b[1] - c[1]) + bb * (c[1] - a[1]) +
          cc * (a[1] - b[1])) / determinant
    uy = (aa * (c[0] - b[0]) + bb * (a[0] - c[0]) +
          cc * (b[0] - a[0])) / determinant
    center = (ux, uy)
    return Circle(center, distance(center, a))


def minimum_enclosing_circle(points: Sequence[Point]) -> Circle:
    """Welzl-style randomized incremental MEC for a finite vertex set."""
    if not points:
        raise ValueError("empty polygon")
    shuffled = list(points)
    random.Random(2026).shuffle(shuffled)
    circle = Circle(shuffled[0], 0.0)
    for i, p in enumerate(shuffled):
        if _contains(circle, p):
            continue
        circle = Circle(p, 0.0)
        for j in range(i):
            q = shuffled[j]
            if _contains(circle, q):
                continue
            circle = _diameter_circle(p, q)
            for k in range(j):
                r = shuffled[k]
                if _contains(circle, r):
                    continue
                candidate = _circumcircle(p, q, r)
                if candidate is None:
                    pairs = [_diameter_circle(p, q), _diameter_circle(p, r),
                             _diameter_circle(q, r)]
                    candidate = min((item for item in pairs
                                     if all(_contains(item, z) for z in (p, q, r))),
                                    key=lambda item: item.radius)
                circle = candidate
    return circle


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    vx, vy = b[0] - a[0], b[1] - a[1]
    denominator = vx * vx + vy * vy
    if denominator < EPS:
        return distance(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / denominator))
    return distance(p, (a[0] + t * vx, a[1] + t * vy))


def point_in_convex(poly: Sequence[Point], p: Point) -> bool:
    for a, b in zip(poly, list(poly[1:]) + [poly[0]]):
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < -EPS:
            return False
    return True


def distance_to_polygon(poly: Sequence[Point], p: Point) -> float:
    if point_in_convex(poly, p):
        return 0.0
    return min(point_segment_distance(p, a, b)
               for a, b in zip(poly, list(poly[1:]) + [poly[0]]))


def covering_clear_points(poly: Sequence[Point], cover_radius: float = 19.0) -> List[Point]:
    """Triangular lattice whose radius-cover contains the complete polygon."""
    if not poly:
        return []
    spacing_x = math.sqrt(3.0) * cover_radius
    spacing_y = 1.5 * cover_radius
    min_x = min(p[0] for p in poly) - spacing_x
    max_x = max(p[0] for p in poly) + spacing_x
    min_y = min(p[1] for p in poly) - spacing_y
    max_y = max(p[1] for p in poly) + spacing_y
    row_min = math.floor(min_y / spacing_y)
    row_max = math.ceil(max_y / spacing_y)
    result: List[Point] = []
    for row in range(row_min, row_max + 1):
        y = row * spacing_y
        offset = 0.5 * spacing_x if row % 2 else 0.0
        col_min = math.floor((min_x - offset) / spacing_x)
        col_max = math.ceil((max_x - offset) / spacing_x)
        for col in range(col_min, col_max + 1):
            q = (col * spacing_x + offset, y)
            if distance_to_polygon(poly, q) <= cover_radius + EPS:
                result.append(q)
    return result


def nearest_neighbour_order(points: Iterable[Point], start: Point) -> List[Point]:
    remaining = list(points)
    ordered: List[Point] = []
    current = start
    while remaining:
        index = min(range(len(remaining)), key=lambda i: distance(current, remaining[i]))
        current = remaining.pop(index)
        ordered.append(current)
    return ordered


def guaranteed_reception(poly: Sequence[Point], observations: Sequence[Bearing],
                          candidate: Point, base_radius: float = 1000.0,
                          margin: float = 0.0) -> bool:
    """Test guaranteed reception using the range implied by earlier detections.

    For a possible source g, every previous direction response proves that its
    unknown reception radius is at least |g-S_i|, in addition to the global
    1000 m lower bound.  Candidate q can therefore fail only where it is at
    least as far from g as every previous station.  That vulnerable set is a
    convex polygon obtained by clipping with perpendicular-bisector
    half-planes; on it, only the base-radius constraint remains.
    """
    if not poly or not observations:
        return False
    qx, qy = candidate
    vulnerable = list(poly)
    for observation in observations:
        sx, sy = observation.point
        # |q-g|^2 >= |S_i-g|^2, written as a*x+b*y<=c.
        vulnerable = clip_halfplane(
            vulnerable,
            2.0 * (qx - sx),
            2.0 * (qy - sy),
            qx * qx + qy * qy - sx * sx - sy * sy,
        )
        if not vulnerable:
            return True
    limit = base_radius - margin
    return max(distance(candidate, vertex) for vertex in vulnerable) <= limit + EPS


def _information_min_eigenvalue(source: Point, stations: Sequence[Point]) -> float:
    """Smallest eigenvalue of the bearing Jacobian Gram matrix."""
    fxx = fxy = fyy = 0.0
    for station in stations:
        dx = source[0] - station[0]
        dy = source[1] - station[1]
        radius_sq = dx * dx + dy * dy
        if radius_sq <= 25.0:
            # A station this close produces `near` and permits immediate clear.
            return math.inf
        inv_radius_sq = 1.0 / radius_sq
        nx = -dy / math.sqrt(radius_sq)
        ny = dx / math.sqrt(radius_sq)
        fxx += nx * nx * inv_radius_sq
        fxy += nx * ny * inv_radius_sq
        fyy += ny * ny * inv_radius_sq
    discriminant = math.hypot(fxx - fyy, 2.0 * fxy)
    return max(0.0, 0.5 * (fxx + fyy - discriminant))


def _localization_samples(poly: Sequence[Point], limit: int = 72) -> List[Point]:
    """Boundary and interior samples for the robust information surrogate."""
    step = max(1, math.ceil(len(poly) / limit))
    boundary = list(poly[::step])
    midpoints = [
        ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        for a, b in zip(boundary, boundary[1:] + boundary[:1])
    ]
    centroid = (sum(p[0] for p in poly) / len(poly),
                sum(p[1] for p in poly) / len(poly))
    return boundary + midpoints + [centroid]


def _worst_sampled_diameter(poly: Sequence[Point], candidate: Point) -> float:
    """Evaluate the bounded-error minimax diameter on a refined scenario set."""
    worst = 0.0
    for source in _localization_samples(poly, limit=18):
        if distance(source, candidate) <= 5.0:
            continue
        true_angle = math.atan2(source[1] - candidate[1],
                                source[0] - candidate[0])
        for error in (-DEG, -0.5 * DEG, 0.0, 0.5 * DEG, DEG):
            updated = clip_bearing(list(poly), Bearing(candidate, true_angle + error))
            updated = clip_disk(updated, candidate, 1500.0)
            if updated:
                worst = max(worst, polygon_diameter(updated)[0])
    return worst


def guaranteed_receiver_station(poly: Sequence[Point], observations: Sequence[Bearing],
                                current: Point) -> Optional[Point]:
    """Choose a guaranteed-reception station with robust bearing geometry."""
    if not poly or not observations:
        return None
    first = observations[0]
    ux, uy = math.cos(first.angle), math.sin(first.angle)
    wx, wy = -uy, ux
    samples = _localization_samples(poly)
    candidates: List[Point] = []
    for along in range(0, 1501, 50):
        for lateral in range(150, 1001, 50):
            for sign in (-1.0, 1.0):
                candidates.append((first.point[0] + along * ux + sign * lateral * wx,
                                   first.point[1] + along * uy + sign * lateral * wy))
    # Fine polar candidates near the boundary of the universal three-disk
    # reception region.  The 33.32-degree direction is the symmetric
    # full-sector minimax-diameter solution at the 998 m safety radius.
    for radial in (900.0, 950.0, 975.0, 998.0):
        offsets = [math.radians(value) for value in range(10, 81, 5)]
        offsets.append(math.radians(33.3204))
        for offset in offsets:
            for sign in (-1.0, 1.0):
                ca = math.cos(sign * offset)
                sa = math.sin(sign * offset)
                candidates.append((first.point[0] + radial * (ca * ux + sa * wx),
                                   first.point[1] + radial * (ca * uy + sa * wy)))
    mec = minimum_enclosing_circle(poly)
    for radius in range(100, 901, 100):
        for k in range(24):
            angle = 2.0 * math.pi * k / 24.0
            candidates.append((mec.center[0] + radius * math.cos(angle),
                               mec.center[1] + radius * math.sin(angle)))

    ranked: List[Tuple[float, float, Point]] = []
    for q in candidates:
        # Repeating at the same place returns the same fixed environmental
        # error and therefore contributes no new information.
        if min(distance(q, item.point) for item in observations) < 1.0:
            continue
        # Previous receptions imply a source-specific radius lower bound; use
        # it instead of forcing every possible source into a 1000 m disk.
        if not guaranteed_reception(poly, observations, q, margin=2.0):
            continue
        worst_information = math.inf
        stations = [item.point for item in observations] + [q]
        for source in samples:
            information = _information_min_eigenvalue(source, stations)
            worst_information = min(worst_information, information)
        # Positioning quality is primary in Problem 2; travel breaks ties.
        ranked.append((worst_information, -distance(current, q), q))

    if not ranked:
        return None
    ranked.sort(reverse=True)
    # The information matrix is only a fast local proxy.  Re-rank the best
    # geometries using the actual intersection diameter under bounded errors.
    finalists = ranked[:12]
    return min(
        (item[2] for item in finalists),
        key=lambda q: (_worst_sampled_diameter(poly, q), distance(current, q)),
    )
