"""Presentation derived from existing controller state; never authorizes motion."""


PAUSE_LABELS = {
    "operator_input": "Operator Pause / Hold",
    "focus_loss": "Window focus lost",
    "ui_heartbeat": "UI heartbeat lost",
    "motion_guard": "Motion guard",
    "output_selection": "Output selection changed",
    "recalculate": "Path recalculation",
    "disable": "Torque disable requested",
    "reset": "Motor reset requested",
    "queue_full": "Input queue full",
    "intent_cancelled": "Motion authorization cancelled",
    "close": "Window closed",
}

STAGES = {
    "idle": ("Idle", "Select outputs and calculate a path."),
    "planned": ("Path checked", "Review the preview, then Start recovery."),
    "verifying_hold": ("Verifying hold", "Wait for stationary hold verification."),
    "endpoint_recovery": ("A3 inward recovery", "Checked inward retreat in progress."),
    "initial": ("Recovering", "Moving through checked recovery waypoints."),
    "approach": ("Approaching weld start", "Wait for arrival and stationary feedback."),
    "ready": ("Ready · holding", "Start welding motion when ready."),
    "welding": ("Welding motion", "Pause / Hold stops the current run."),
    "paused": ("Paused", "Recalculate the path before restarting."),
    "complete": ("Pass complete", "Holding the endpoint. Recalculate for another run."),
    "fault": ("Fault · latched", "Inspect first-fault evidence; reset remains explicit."),
    "resetting": ("Resetting motor fault", "Wait for disabled, stationary verification."),
}


def controller_presentation(controller, now):
    """Timing is observational and uses the same monotonic clock as the owner."""
    c = controller
    recovery = next((e for e in reversed(c.events) if e["event"] == "recovery_requested"), None)
    ready = next((e for e in reversed(c.events) if e["event"] == "stage_arrived" and e["state"] == "ready"), None)
    run = None if recovery is None else recovery["time_s"]
    prep = None
    if run is not None and recovery["state"] != "ready":
        end = ready["time_s"] if ready and ready["time_s"] >= run else now
        stop = next((e for e in c.events if e["event"] in ("pause", "fault") and e["time_s"] >= run), None)
        if stop:
            end = min(end, stop["time_s"])
        prep = max(0, end - run)
    key = {"initial": "home", "approach": "approach", "welding": "weld", "endpoint_recovery": "endpoint_recovery"}.get(c.state)
    path = c.paths.get(key)
    trace = c.trace[-1] if c.trace and c.trace[-1]["stage"] == c.state else None
    progress = None
    if path is not None and trace:
        elapsed = trace["path_elapsed_s"]
        # Home traces restart the clock at each stopped waypoint.
        if c.state == "initial" and c.recovery_index:
            elapsed += c.recovery_stops[c.recovery_index - 1]
        if c.state == "welding" and c.continuous_weld:
            elapsed %= path.duration
        progress = min(1.0, max(0.0, elapsed / path.duration))
    nominal = None if path is None else path.duration
    if c.state == "initial" and path is not None:
        nominal = c.recovery_stops[c.recovery_index] - (c.recovery_stops[c.recovery_index - 1] if c.recovery_index else 0)
    return dict(
        run_id=run, preparation_elapsed_s=prep, stage_progress=progress,
        stage_elapsed_s=None if c.started is None else max(0, now - c.started),
        stage_nominal_s=nominal,
        pause_cause=c.pause_cause,
    )


def feedback_live_expected(value):
    """Output selection alone does not start motor feedback or holding."""
    return bool(value.get("holding")) or value.get("state") in (
        "verifying_hold", "endpoint_recovery", "initial", "approach",
        "ready", "welding", "complete",
    )


def state_summary(value):
    state = value["state"]
    title, action = STAGES.get(state, (state, ""))
    severity = "normal"
    if value.get("pending"):
        title, action = "Checking path", "Calculation in progress; motion starts only on request."
        severity = "planning"
    elif state == "fault":
        severity = "fault"
    elif value.get("message", "").startswith("Blocked:"):
        title, action, severity = "Action blocked", value["message"], "blocked"
    elif state == "paused" or value.get("pause_cause") in ("focus_loss", "ui_heartbeat", "operator_input") and state == "ready":
        cause = PAUSE_LABELS.get(value.get("pause_cause"), "Paused")
        title = f"{title} · {cause}"
        severity = "paused"
    hold = value.get("hold_observation") or {}
    age = value.get("feedback_age_ms")
    if severity == "normal" and value.get("arm_on") and feedback_live_expected(value) and age is not None and age >= 80:
        title, action, severity = title + " · Feedback stale", "Inspect feedback freshness before starting motion.", "warning"
    if severity == "normal" and hold.get("tracking_error_deg") and max(abs(x) for x in hold["tracking_error_deg"]) > hold.get("arrival_tolerance_deg", .25):
        title, severity = title + " · HOLD DRIFT", "warning"
    return title, action, severity
