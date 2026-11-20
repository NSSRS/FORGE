"""Preserve arm collision pairs while skipping unrelated chassis-only pairs."""

import numpy as np


def retain_arm_pairs(model, arm_geoms):
    """Encode the original pair rule AND (arm A OR arm B) in contact masks.

    Keep both directed type/affinity tests, including asymmetric masks. Explicit
    model pairs and mesh stop checks are untouched. If the original masks use
    too many bits, leave the full collision calculation enabled.
    """
    types = np.asarray(model.geom_contype, dtype=np.int64).copy()
    affinity = np.asarray(model.geom_conaffinity, dtype=np.int64).copy()
    if np.any(types < 0) or np.any(affinity < 0):
        return False
    bits = int(np.bitwise_or.reduce(types | affinity, initial=0)).bit_length()
    if bits == 0 or bits * 2 > 30:
        return False
    model.geom_contype[:] = (types << bits) | np.where(arm_geoms, types, 0)
    model.geom_conaffinity[:] = affinity | np.where(arm_geoms, affinity << bits, 0)
    # MuJoCo's body broad phase also uses the aggregate masks.
    model.body_contype[:] = 0
    model.body_conaffinity[:] = 0
    np.bitwise_or.at(model.body_contype, model.geom_bodyid, model.geom_contype)
    np.bitwise_or.at(model.body_conaffinity, model.geom_bodyid, model.geom_conaffinity)
    return True
