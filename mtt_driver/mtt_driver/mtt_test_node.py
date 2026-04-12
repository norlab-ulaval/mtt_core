#!/usr/bin/env python3

"""Standalone driver initialization test runnable through ros2 run."""

from __future__ import annotations

import logging
import sys
import time

from .mtt_driver import DirectionState, MTTCanDriver, interface_exists


LOG = logging.getLogger("mtt_test_node")


def test_driver_initialization(can_interface: str = "vcan0") -> bool:
    LOG.info("=" * 60)
    LOG.info("MTT DRIVER INITIALIZATION TEST")
    LOG.info("=" * 60)

    if not interface_exists(can_interface):
        LOG.error("CAN interface '%s' not found", can_interface)
        LOG.info("Create a virtual CAN interface with:")
        LOG.info("sudo modprobe vcan")
        LOG.info("sudo ip link add dev %s type vcan", can_interface)
        LOG.info("sudo ip link set up %s", can_interface)
        return False

    try:
        LOG.info("Test 1: Basic driver initialization on %s", can_interface)
        driver = MTTCanDriver(can_interface=can_interface)
        driver.set_direction(DirectionState.Reverse)
        # This test checks the raw frame layout, so it uses the raw helper on purpose.
        driver._set_steer(128)
        LOG.info("Driver initialized successfully")

        LOG.info("Test 2: Check initial CAN frame")
        LOG.info("Initial frame: %s", driver._get_current_frame_hex())

        LOG.info("Test 3: Verify initial state values")
        LOG.info("Vehicle type: %s", driver.vehicle_type)
        LOG.info("Direction state: %s", driver.direction_state)
        LOG.info("Direction mode: %s", driver.steering_mode)
        LOG.info("Security switch: %s", driver.security_switch_state)
        LOG.info("Light state: %s", driver.light_state)
        LOG.info("Winch state: %s", driver.winch_state)
        LOG.info("Steer value: %s", driver.steer_value)
        LOG.info("Throttle value: %s", driver.throttle_value)
        LOG.info("Brake value: %s", driver.brake_value)

        LOG.info("Test 4: Monitor frames being sent (3 seconds)")
        for index in range(6):
            time.sleep(0.5)
            LOG.info("Frame %s: %s", index + 1, driver._get_current_frame_hex())

        driver.cleanup()
        LOG.info("All tests passed")
        LOG.info("=" * 60)
        return True
    except Exception as exc:  # pragma: no cover - exercised against live CAN
        LOG.exception("Driver initialization test failed: %s", exc)
        return False


def main(args=None) -> None:
    del args
    logging.basicConfig(level=logging.INFO)
    can_interface = sys.argv[1] if len(sys.argv) > 1 else "vcan0"
    success = test_driver_initialization(can_interface)
    raise SystemExit(0 if success else 1)
