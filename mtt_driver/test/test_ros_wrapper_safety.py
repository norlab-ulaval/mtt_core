import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

from mtt_driver.mtt_driver import SecuritySwitchState
from mtt_driver.mtt_ros_wrapper import MTTRosWrapper


def _make_wrapper():
    wrapper = MTTRosWrapper.__new__(MTTRosWrapper)
    wrapper.driver_lock = threading.RLock()
    wrapper.driver = MagicMock()
    wrapper.driver.get_security_switch_state.return_value = None
    wrapper.active_safety_locks = set()
    wrapper.teleop_estop_active = False
    wrapper.teleop_estop_seen = False
    return wrapper


def test_estop_callback_locks_then_unlocks_safety():
    wrapper = _make_wrapper()

    wrapper.estop_callback(SimpleNamespace(data=True))

    assert wrapper.teleop_estop_seen is True
    assert wrapper.teleop_estop_active is True
    assert "teleop_estop" in wrapper.active_safety_locks
    wrapper.driver._set_security_switch.assert_called_with(SecuritySwitchState.SafetyLocked)

    wrapper.driver.get_security_switch_state.return_value = SecuritySwitchState.SafetyLocked
    wrapper.driver._set_security_switch.reset_mock()

    wrapper.estop_callback(SimpleNamespace(data=False))

    assert wrapper.teleop_estop_active is False
    assert "teleop_estop" not in wrapper.active_safety_locks
    wrapper.driver._set_security_switch.assert_called_with(SecuritySwitchState.SafetyUnlocked)


def test_describe_safety_state_includes_active_lock_reasons():
    wrapper = _make_wrapper()
    wrapper.active_safety_locks = {"teleop_estop", "startup"}

    assert wrapper._describe_safety_state("SafetyLocked") == "SafetyLocked:startup,teleop_estop"
