from mtt_avoidance.policy import AvoidancePolicy, Inputs, State


def _inputs(now, **overrides):
    values = {
        "now_s": now,
        "enabled": True,
        "replaying": True,
        "obstacle_fresh": True,
        "obstacle_stop": False,
        "plan_ready": False,
        "plan_failed": False,
        "execution_finished": False,
        "recovery_finished": False,
    }
    values.update(overrides)
    return Inputs(**values)


def test_persistent_obstacle_requests_plan_once():
    policy = AvoidancePolicy(persistent_stop_s=3.0)
    assert policy.update(_inputs(0.0, obstacle_stop=True)).state == State.HOLDING
    decision = policy.update(_inputs(3.1, obstacle_stop=True))
    assert decision.state == State.PLANNING
    assert decision.request_plan
    assert not policy.update(_inputs(3.2, obstacle_stop=True)).request_plan


def test_shadow_mode_never_requests_execution():
    policy = AvoidancePolicy(execution_enabled=False)
    decision = policy.update(_inputs(1.0, obstacle_stop=True, plan_ready=True))
    assert decision.state == State.SHADOW_READY
    assert not decision.request_execute


def test_no_plan_does_not_reverse_when_recovery_is_locked():
    policy = AvoidancePolicy(
        execution_enabled=True,
        reverse_recovery_enabled=False,
    )
    decision = policy.update(_inputs(1.0, obstacle_stop=True, plan_failed=True))
    assert decision.state == State.BLOCKED
    assert not decision.request_recovery


def test_stale_obstacle_input_is_fail_closed():
    policy = AvoidancePolicy()
    decision = policy.update(_inputs(1.0, obstacle_fresh=False))
    assert decision.state == State.SENSOR_FAULT
    assert decision.cancel_motion


def test_obstacle_clear_cancels_override_and_returns_to_following():
    policy = AvoidancePolicy(execution_enabled=True)
    assert policy.update(
        _inputs(1.0, obstacle_stop=True, plan_ready=True)
    ).state == State.EXECUTING
    decision = policy.update(
        _inputs(2.0, obstacle_stop=True, execution_finished=True)
    )
    assert decision.state == State.FOLLOWING
    assert decision.cancel_motion
