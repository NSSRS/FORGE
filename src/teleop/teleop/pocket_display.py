"""Read-only dashboard snapshots. No serial access or command refresh."""
from dataclasses import asdict


def dashboard_snapshot(policy, worker, now, *, hardware_arm=False):
    output = policy.output(now)
    channels = policy.last_channels
    axes = {}
    if channels:
        for name, axis in policy.mapping.axes.items():
            raw = channels[axis.channel-1]
            span = axis.maximum-axis.center if raw >= axis.center else axis.center-axis.minimum
            axes[name] = dict(channel=axis.channel, raw=raw, center=axis.center,
                              deadzone=axis.deadzone, value=axis.value(channels),
                              offset=max(-1., min(1., (raw-axis.center)/span))*axis.sign)
    switches = {}
    for name, switch in policy.mapping.switches.items():
        try:
            position = switch.position(channels) if channels else None
        except ValueError:
            position = None
        switches[name] = dict(channel=switch.channel,
                              raw=channels[switch.channel-1] if channels else None,
                              position=position)
    fresh = bool(channels and now < policy.deadline)
    return dict(version=1, hardware_arm=hardware_arm, radio_fresh=fresh,
                remaining_ms=max(0., (policy.deadline-now)*1000) if fresh else 0.,
                channels=channels, axes=axes, switches=switches,
                selected=output.arm.joint, direction=output.arm.direction,
                authorized=policy.authorized, ready=policy.ready,
                status=output.status, chassis=list(output.chassis), arm=worker)


def position_snapshot(port, now):
    """Called ONLY by the arm owner; cached_positions never reads USB/CAN."""
    return dict(at=now, moving=port.moving, armed=port.armed,
                selected=port.selected, rows=[asdict(row) for row in port.cached_positions()])
