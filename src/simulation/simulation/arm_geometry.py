"""Authored arm dimensions applied during CAD integration (SI units)."""
import numpy as np

D3_ROD_CENTER_DISTANCE_M = .150
D3_ROD_BODY = 'b4_1_arm_rod_d3'
D3_ROD_SITE = 'closing_b4_2_arm_rod_d3_2'


def normalize_d3_rod_centers(root):
    """Remove CAD rounding at the far connection site, retaining its direction.

    The specified 150 mm is between connection centers; the bare mesh is 140 mm.
    This corrects at most 0.01 mm, without stretching meshes or changing inertia.
    Larger differences require a geometry review instead of moving a site alone.
    """
    body = root.find(f".//body[@name='{D3_ROD_BODY}']")
    if body is None:
        raise ValueError('Missing D3 rod body')
    joint = body.find(f"joint[@name='{D3_ROD_BODY}']")
    site = body.find(f"site[@name='{D3_ROD_SITE}']")
    if joint is None or site is None or joint.get('type') != 'ball':
        raise ValueError('Expected D3 ball joint and far connection site on the rod')
    pivot = np.fromstring(joint.get('pos', '0 0 0'), sep=' ')
    point = np.fromstring(site.get('pos', ''), sep=' ')
    if pivot.shape != (3,) or point.shape != (3,) or not np.isfinite([pivot, point]).all():
        raise ValueError('D3 connection coordinates must be finite 3-vectors')
    offset = point-pivot
    length = np.linalg.norm(offset)
    if abs(length-D3_ROD_CENTER_DISTANCE_M) > 1e-5:
        raise ValueError('D3 rod differs from 150 mm by more than CAD rounding; review geometry')
    point = pivot + offset*(D3_ROD_CENTER_DISTANCE_M/length)
    site.set('pos', ' '.join(f'{v:.17g}' for v in point))
    return float(length)
