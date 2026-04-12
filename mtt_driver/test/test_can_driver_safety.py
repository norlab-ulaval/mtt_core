#!/usr/bin/env python3
"""Focused unit tests for the low-level CAN driver safety logic."""

from __future__ import annotations

import time

import can

from mtt_driver.mtt_driver import MTTCanDriver


class _DummyBus:
    """Small in-memory stand-in for python-can bus objects."""

    def __init__(self):
        self.sent_messages = []
        self.shutdown_calls = 0

    def recv(self, timeout=0.1):
        time.sleep(min(timeout, 0.001))
        return None

    def send(self, message):
        self.sent_messages.append(message)

    def shutdown(self):
        self.shutdown_calls += 1


class _FailingSendBus(_DummyBus):
    def send(self, message):
        raise can.CanOperationError("forced send failure")


def _make_driver(monkeypatch, bus_sequence, telemetry_timeout_seconds=0.05):
    monkeypatch.setattr("mtt_driver.mtt_driver.interface_exists", lambda _: True)
    bus_iter = iter(bus_sequence)
    monkeypatch.setattr(MTTCanDriver, "_open_can_bus", lambda self: next(bus_iter))
    driver = MTTCanDriver(
        can_interface="vcan0",
        telemetry_timeout_seconds=telemetry_timeout_seconds,
    )
    return driver


def test_stale_tachometer_data_is_marked_invalid(monkeypatch):
    driver = _make_driver(monkeypatch, [_DummyBus()])
    try:
        driver._process_tachometer_data(bytes([1, 2, 0, 100, 0, 0, 3, 232]))

        fresh_snapshot = driver.get_tachometer_snapshot()
        assert fresh_snapshot.new_data_available is True

        with driver.frame_lock:
            driver.tachometer_data.monotonic_timestamp = time.monotonic() - 1.0

        stale_snapshot = driver.get_tachometer_snapshot()
        odom_snapshot = driver.get_odometry_snapshot()

        assert stale_snapshot.new_data_available is False
        assert odom_snapshot["telemetry_is_fresh"] is False
        assert odom_snapshot["speed_ms"] == 0.0
        assert odom_snapshot["speed_kmh"] == 0.0
        assert odom_snapshot["cumulative_ticks"] == 1000
    finally:
        driver.cleanup()


def test_send_failure_reopens_can_bus(monkeypatch):
    failing_bus = _FailingSendBus()
    recovered_bus = _DummyBus()
    driver = _make_driver(monkeypatch, [failing_bus, recovered_bus])
    try:
        driver.send_can_frame()
        assert driver.bus is recovered_bus
        assert failing_bus.shutdown_calls == 1
    finally:
        driver.cleanup()
