"""Bounded, collision-checked startup routes. No motor transport or angle wrapping."""

from collections import deque

import numpy as np


RECOVERY_CLEARANCE_MM = 2.0
START_TOLERANCE_DEG = 0.3


def recovery_path(geometry, start, goal, settings):
    """Try a direct move, then search orders of single-joint moves to standby.

    Nodes mix the measured start and standby coordinates: at most 64 poses and
    192 directed edges. No fixed wrist-first assumption, unbounded search, or
    escape from an already colliding pose. Every edge and the joined spline are
    checked; failure leaves the operator responsible for supported repositioning.
    """
    from .weld_plan import JointPath, finite_vector, move_path, speed_report

    start, goal = finite_vector(start, "recovery start"), finite_vector(goal, "standby")
    geometry.check(start, clearance_mm=RECOVERY_CLEARANCE_MM)
    geometry.check(goal, clearance_mm=RECOVERY_CLEARANCE_MM)
    direct = move_path(start, goal, settings)
    try:
        geometry.validate_path(direct, clearance_mm=RECOVERY_CLEARANCE_MM)
        return direct, dict(version=1, route="direct checked recovery",
                            waypoint_times_s=[direct.duration], checked_edges=1,
                            clearance_mm=RECOVERY_CLEARANCE_MM, joint_order=None)
    except ValueError as exc:
        direct_error = str(exc)

    axes = [i for i in range(6) if abs(goal[i] - start[i]) > 1e-9]
    complete = (1 << len(axes)) - 1
    queue, parents = deque([0]), {0: None}
    checked = 1

    def pose(mask):
        raw = start.copy()
        for bit, axis in enumerate(axes):
            if mask & (1 << bit):
                raw[axis] = goal[axis]
        return raw

    while queue and complete not in parents:
        mask = queue.popleft()
        for bit in range(len(axes)):
            following = mask | (1 << bit)
            if following == mask or following in parents:
                continue
            edge = move_path(pose(mask), pose(following), settings)
            checked += 1
            try:
                geometry.validate_path(edge, clearance_mm=RECOVERY_CLEARANCE_MM)
            except ValueError:
                continue
            parents[following] = (mask, edge)
            queue.append(following)

    if complete not in parents or complete == 0:
        raise ValueError(
            "No checked recovery route to standby. Keep supported and reposition/replan; "
            "bounded joint-order search exhausted. Direct route: " + direct_error
        )
    edges, mask = [], complete
    while mask:
        mask, edge = parents[mask]
        edges.append(edge)
    edges.reverse()
    times, positions, stops = [0.0], [start], []
    for index, edge in enumerate(edges):
        offset = times[-1]
        times.extend((offset + edge.times[1:]).tolist())
        positions.extend(edge.positions[1:])
        stops.append(times[-1])
        if index != len(edges) - 1:
            # A flat interval forces zero PCHIP velocity at the waypoint.
            times.append(times[-1] + 0.2)
            positions.append(edge.positions[-1])
    joined = JointPath(times, positions)
    scale = max(1.0, speed_report(joined, settings)["additional_time_scale"] * 1.01)
    joined = JointPath(np.asarray(times) * scale, positions)
    stops = [t * scale for t in stops]
    geometry.validate_path(joined, clearance_mm=RECOVERY_CLEARANCE_MM)
    return joined, dict(version=1, route="checked joint-order recovery",
                        waypoint_times_s=stops, checked_edges=checked,
                        clearance_mm=RECOVERY_CLEARANCE_MM,
                        joint_order=[int(np.flatnonzero(abs(e.positions[-1] - e.positions[0]) > 1e-9)[0]) + 1
                                     for e in edges])
