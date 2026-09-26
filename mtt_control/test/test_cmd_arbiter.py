"""Exercise the real arbiter process. Run only in the isolated test container."""

import math
import os
from pathlib import Path
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TwistStamped
import pytest
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Bool, String


def executable():
    return str(Path(get_package_prefix('mtt_control')) / 'lib/mtt_control/mtt_cmd_arbiter_node')


@pytest.fixture
def arbiter():
    # The runner supplies network isolation. Refuse accidental use on a live graph.
    if os.environ.get('MTT_OFFLINE_TEST') != '1':
        pytest.skip('Use scripts/test_offline in mtt_workspace (isolated container)')
    namespace = '/test_arbiter_' + uuid.uuid4().hex
    rclpy.init()
    node = rclpy.create_node('probe', namespace=namespace)
    proc = subprocess.Popen([
        executable(), '--ros-args', '-r', '__ns:=' + namespace,
        '-p', 'manual_timeout_s:=0.2', '-p', 'auto_timeout_s:=0.2',
        '-p', 'mode_switch_hold_s:=0.05', '-p', 'max_auto_speed_ms:=0.8',
    ])
    pubs = {
        'manual': node.create_publisher(TwistStamped, 'cmd_vel/manual', 10),
        'auto': node.create_publisher(TwistStamped, 'controller/cmd_vel', 10),
        'mode': node.create_publisher(String, 'mtt_control/selected_mode', 10),
        'enabled': node.create_publisher(Bool, 'mtt_control/auto_mode_enabled', 10),
        'deadman': node.create_publisher(Bool, 'mtt_control/teleop_deadman', 10),
        'estop': node.create_publisher(Bool, 'mtt_control/teleop_estop', 10),
    }
    outputs = []
    subscription = node.create_subscription(TwistStamped, 'cmd_vel', outputs.append, 20)

    def pump(duration, send=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert proc.poll() is None, 'Arbiter exited unexpectedly'
            if send:
                send()
            rclpy.spin_once(node, timeout_sec=0.01)

    def send(mode='AUTO', enabled=True, deadman=True, estop=False,
             velocity=0.4, angular=0.1, age=0.0, zero_stamp=False, negative_stamp=False):
        pubs['mode'].publish(String(data=mode))
        pubs['enabled'].publish(Bool(data=enabled))
        pubs['deadman'].publish(Bool(data=deadman))
        pubs['estop'].publish(Bool(data=estop))
        msg = TwistStamped()
        if not zero_stamp:
            msg.header.stamp = (node.get_clock().now() - Duration(seconds=age)).to_msg()
        if negative_stamp:
            msg.header.stamp.sec = -1
            msg.header.stamp.nanosec = 0
        msg.twist.linear.x = velocity
        msg.twist.angular.z = angular
        pubs['manual' if mode == 'MANUAL' else 'auto'].publish(msg)

    try:
        deadline = time.monotonic() + 5.0
        while not all(pub.get_subscription_count() for pub in pubs.values()):
            assert time.monotonic() < deadline, 'DDS discovery timed out'
            pump(0.05)
        pump(0.1)
        yield outputs, pump, send
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def assert_stopped(outputs):
    assert len(outputs) >= 3, 'No sustained output to verify'
    for msg in outputs[-3:]:
        for vector in (msg.twist.linear, msg.twist.angular):
            assert all(math.isfinite(v) and v == 0.0 for v in (vector.x, vector.y, vector.z))


def test_startup_is_stopped(arbiter):
    outputs, pump, _ = arbiter
    pump(0.15)
    assert_stopped(outputs)


@pytest.mark.parametrize('mode', ['MANUAL', 'AUTO'])
def test_valid_command_and_source_timeout(arbiter, mode):
    outputs, pump, send = arbiter
    pump(0.3, lambda: send(mode=mode))
    assert outputs[-1].twist.linear.x == pytest.approx(0.4)
    assert outputs[-1].twist.angular.z == pytest.approx(0.1)
    pump(0.4)
    assert_stopped(outputs)


@pytest.mark.parametrize('mode', ['MANUAL', 'AUTO'])
@pytest.mark.parametrize('invalid', [
    {'age': -60.0}, {'age': 1.0}, {'zero_stamp': True}, {'negative_stamp': True},
    {'velocity': math.nan}, {'velocity': math.inf}, {'angular': math.nan},
])
def test_invalid_command_stops_even_after_valid_input(arbiter, mode, invalid):
    outputs, pump, send = arbiter
    pump(0.3, lambda: send(mode=mode))
    assert outputs[-1].twist.linear.x > 0.0
    pump(0.3, lambda: send(mode=mode, **invalid))
    assert_stopped(outputs)


@pytest.mark.parametrize('gates', [
    {'mode': 'STOP'}, {'mode': 'MANUAL', 'deadman': False},
    {'mode': 'AUTO', 'enabled': False}, {'mode': 'MANUAL', 'estop': True},
    {'mode': 'AUTO', 'estop': True},
])
def test_stop_gates_override_active_commands(arbiter, gates):
    outputs, pump, send = arbiter
    pump(0.3, lambda: send(mode=gates['mode'] if gates['mode'] != 'STOP' else 'AUTO'))
    assert outputs[-1].twist.linear.x > 0.0
    pump(0.3, lambda: send(**gates))
    assert_stopped(outputs)


@pytest.mark.parametrize('velocity', [-2.0, 2.0])
def test_auto_limit_preserves_direction(arbiter, velocity):
    outputs, pump, send = arbiter
    pump(0.3, lambda: send(velocity=velocity))
    assert outputs[-1].twist.linear.x == pytest.approx(math.copysign(0.8, velocity))


@pytest.mark.parametrize('mode', ['MANUAL', 'AUTO'])
def test_rejected_future_command_cannot_become_active_later(arbiter, mode):
    outputs, pump, send = arbiter
    pump(0.15, lambda: send(mode=mode, age=-0.15))
    pump(0.22)
    assert_stopped(outputs)


@pytest.mark.parametrize('parameter', [
    'max_auto_speed_ms:=-1.0', 'max_auto_speed_ms:=6.0',
    'manual_timeout_s:=-0.1', 'auto_timeout_s:=0.0',
    'publish_rate_hz:=0.0', 'mode_switch_hold_s:=-0.1',
])
def test_invalid_startup_parameter_is_rejected(parameter):
    if os.environ.get('MTT_OFFLINE_TEST') != '1':
        pytest.skip('Use scripts/test_offline in mtt_workspace')
    proc = subprocess.Popen([executable(), '--ros-args', '-p', parameter])
    try:
        assert proc.wait(timeout=3) != 0
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=5)
