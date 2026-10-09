"""Public API for the MaixCam to STM32 cyclic link."""

from .protocol import (
    ENABLE_BATTERY,
    ENABLE_LINE,
    ENABLE_MOTOR_CURRENT,
    ENABLE_MOTORS,
    ENABLE_DISTANCE_SENSOR,
    ENABLE,
    ENABLE_FLAGS,
    CommandState,
    EnableFlags,
    TelemetryState,
    TelemetrySnapshot,
)


def __getattr__(name):
    # Keep protocol-only host tests importable without the MaixPy runtime.
    if name in ("LinkStats", "MaixcamLink"):
        from .link import LinkStats, MaixcamLink
        return {"LinkStats": LinkStats, "MaixcamLink": MaixcamLink}[name]
    raise AttributeError(name)

__all__ = [
    "CommandState",
    "EnableFlags",
    "TelemetryState",
    "TelemetrySnapshot",
    "ENABLE",
    "ENABLE_FLAGS",
    "LinkStats",
    "MaixcamLink",
    "ENABLE_MOTORS",
    "ENABLE_LINE",
    "ENABLE_MOTOR_CURRENT",
    "ENABLE_BATTERY",
    "ENABLE_DISTANCE_SENSOR",
]
