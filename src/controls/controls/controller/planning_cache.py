"""Bounded process-local reuse of immutable model and weave inputs."""

from collections import OrderedDict
from copy import copy, deepcopy
from dataclasses import asdict
from functools import lru_cache
import threading

import numpy as np

from motors.arm_calibration import digest


@lru_cache(maxsize=1)
def _scene_template(identity):
    import mujoco
    from controls.simulation.weave_xy_z import floor_scene
    model, data = floor_scene()
    # Keep the settled integration state, not a second large solver arena.
    signature = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(model, signature))
    mujoco.mj_getState(model, data, state, signature)
    return model, state


def scene_copy(identity):
    """Every caller owns a model/data copy; cached template is never mutated."""
    import mujoco
    source_model, state = _scene_template(identity)
    model = copy(source_model)
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model, data)
    return model, data


_local = threading.local()


def cached_weave(geometry, settings, root, scene_identity):
    """Reuse only the start-independent weave, never recovery or arrival checks.

    Full scene content, mapping, settings and effective motor constraints form
    the key. Returned documents are copies so callers cannot alter a later plan.
    Scene freshness is checked again by build_plan before returning any result.
    """
    from .limits import motor_constraints
    from .weld_plan import JointPath, solve_weave
    key = digest(dict(scene=scene_identity, mapping=geometry.mapping.document,
                      settings=asdict(settings),
                      constraints=motor_constraints(root, settings.max_speed_deg_s,
                                                    profile=settings.limit_profile)))
    if not hasattr(_local, "weaves"):
        _local.weaves = OrderedDict()
    cache = _local.weaves
    hit = key in cache
    if not hit:
        path, report = solve_weave(geometry, settings, root)
        cache[key] = (path.document(), deepcopy(report))
        while len(cache) > 4:
            cache.popitem(last=False)
    cache.move_to_end(key)
    path, report = cache[key]
    return JointPath.from_document(path), deepcopy(report), hit


def clear_planning_caches():
    _scene_template.cache_clear()
    if hasattr(_local, "weaves"):
        _local.weaves.clear()
