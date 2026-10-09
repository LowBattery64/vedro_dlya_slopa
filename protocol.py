"""Wire codec and thread-safe application snapshots for protocol v1."""

import struct
import threading


PROTOCOL_VERSION = 1
COMMAND_SIZE = 30
TELEMETRY_SIZE = 80
TRANSACTION_SIZE = 80

ENABLE_MOTORS = 1 << 0
ENABLE_LINE = 1 << 1
ENABLE_MOTOR_CURRENT = 1 << 2
ENABLE_BATTERY = 1 << 4
ENABLE_DISTANCE_SENSOR = 1 << 5

ENABLE_FLAGS = {
    "motors": ENABLE_MOTORS,
    "line": ENABLE_LINE,
    "motor_current": ENABLE_MOTOR_CURRENT,
    "battery": ENABLE_BATTERY,
    "distance_sensor": ENABLE_DISTANCE_SENSOR,
}


class EnableFlags:
    """Named enable bits for readable user code."""

    MOTORS = ENABLE_MOTORS
    LINE = ENABLE_LINE
    MOTOR_CURRENT = ENABLE_MOTOR_CURRENT
    BATTERY = ENABLE_BATTERY
    DISTANCE_SENSOR = ENABLE_DISTANCE_SENSOR


ENABLE = EnableFlags()

ERROR_NAMES = (
    "ERR_CRC_FRAME", "ERR_FRAME_SIZE", "ERR_FRAME_COUNTER",
    "ERR_LINK_TIMEOUT", "ERR_LINE_SENSOR", "ERR_MOTOR_CURRENT",
    "ERR_BATTERY_ADC", "ERR_DISTANCE_SENSOR", None, None,
    None, None, "ERR_WS2812", "ERR_CONFIGURATION",
    "ERR_SPI_OVERRUN", "ERR_STATE_PUBLISH",
)


def _build_crc_table():
    table = []
    for value in range(256):
        crc = value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC16_TABLE = _build_crc_table()


def crc16_ccitt_false(data):
    """Return CRC-16/CCITT-FALSE used by both endpoints."""
    crc = 0xFFFF
    for value in data:
        crc = ((crc << 8) ^ _CRC16_TABLE[((crc >> 8) ^ value) & 0xFF]) & 0xFFFF
    return crc


def error_names(error_flags):
    return [name for bit, name in enumerate(ERROR_NAMES)
            if name is not None and error_flags & (1 << bit)]


class CommandState:
    """Mutable command snapshot owned by application code.

    Use the setters instead of mutating a speed list. The link takes an
    atomic snapshot before every SPI transaction.
    """

    __slots__ = ("_lock", "_frame_flags", "_frame_counter",
                 "_enable_mask", "_motor_speed")

    def __init__(self):
        self._lock = threading.Lock()
        self._frame_flags = 0
        self._frame_counter = 0
        self._enable_mask = 0
        self._motor_speed = [0, 0, 0, 0]

    @property
    def frame_flags(self):
        with self._lock:
            return self._frame_flags

    @frame_flags.setter
    def frame_flags(self, value):
        with self._lock:
            self._frame_flags = int(value) & 0xFF

    @property
    def frame_counter(self):
        with self._lock:
            return self._frame_counter

    @property
    def enable_mask(self):
        with self._lock:
            return self._enable_mask

    @enable_mask.setter
    def enable_mask(self, value):
        self.set_enable_mask(value)

    @property
    def motor_speed(self):
        with self._lock:
            return tuple(self._motor_speed)

    def set_enable_mask(self, value):
        with self._lock:
            self._enable_mask = int(value) & 0xFFFF

    def set_enable_bit(self, bit, enabled):
        if bit < 0 or bit > 15:
            raise ValueError("enable bit must be in range 0..15")
        with self._lock:
            if enabled:
                self._enable_mask |= 1 << bit
            else:
                self._enable_mask &= ~(1 << bit)

    def set_enable_group(self, group, enabled=True):
        """Set a sensor/actuator enable by its documented name."""
        name = str(group).strip().lower()
        if name not in ENABLE_FLAGS:
            raise ValueError("unknown enable group: {}".format(group))
        self.set_enable_bit(ENABLE_FLAGS[name].bit_length() - 1, enabled)

    def set_enabled(self, *groups):
        """Replace the mask using names such as ``motors`` and ``line``."""
        mask = 0
        for group in groups:
            name = str(group).strip().lower()
            if name not in ENABLE_FLAGS:
                raise ValueError("unknown enable group: {}".format(group))
            mask |= ENABLE_FLAGS[name]
        self.set_enable_mask(mask)

    def set_speed(self, motor, speed):
        if motor < 0 or motor > 3:
            raise ValueError("motor must be in range 0..3")
        if speed < -1000 or speed > 1000:
            raise ValueError("speed must be in range -1000..1000")
        with self._lock:
            self._motor_speed[motor] = int(speed)

    def set_all_speeds(self, speed):
        if speed < -1000 or speed > 1000:
            raise ValueError("speed must be in range -1000..1000")
        with self._lock:
            value = int(speed)
            self._motor_speed[:] = (value, value, value, value)

    def set_safe(self):
        with self._lock:
            self._enable_mask = 0
            self._motor_speed[:] = (0, 0, 0, 0)

    def _set_frame_counter(self, value):
        with self._lock:
            self._frame_counter = value & 0xFFFFFFFF

    def snapshot(self):
        with self._lock:
            return (self._frame_flags, self._frame_counter,
                    self._enable_mask, tuple(self._motor_speed))

    def copy(self):
        result = CommandState()
        flags, counter, mask, speeds = self.snapshot()
        result._frame_flags = flags
        result._frame_counter = counter
        result._enable_mask = mask
        result._motor_speed[:] = speeds
        return result

    def _restore_application_state(self, snapshot):
        """Restore user fields after an automatic startup reset."""
        flags, _, mask, speeds = snapshot
        with self._lock:
            self._frame_flags = flags
            self._enable_mask = mask
            self._motor_speed[:] = speeds

    def pack(self, frame_counter=None):
        """Pack one complete 80-byte command transaction."""
        with self._lock:
            flags = self._frame_flags
            counter = self._frame_counter if frame_counter is None else frame_counter
            mask = self._enable_mask
            speeds = tuple(self._motor_speed)

        frame = bytearray(TRANSACTION_SIZE)
        struct.pack_into(
            "<BBHIH4h10s",
            frame,
            0,
            PROTOCOL_VERSION,
            flags,
            0,
            counter & 0xFFFFFFFF,
            mask,
            speeds[0], speeds[1], speeds[2], speeds[3],
            b"\x00" * 10,
        )
        struct.pack_into("<H", frame, 28, crc16_ccitt_false(frame[:28]))
        return bytes(frame)


class TelemetrySnapshot:
    """Immutable-style copy of one decoded STM32 telemetry frame."""

    __slots__ = ("protocol_version", "telemetry_flags",
                  "command_frame_counter_echo", "active_enable_mask",
                  "error_flags", "line", "motor_current", "battery",
                  "distance_sensor", "raw")

    def __init__(self, protocol_version=PROTOCOL_VERSION, telemetry_flags=0,
                 command_frame_counter_echo=0, active_enable_mask=0,
                  error_flags=0, line=None, motor_current=None, battery=None,
                  distance_sensor=0,
                  raw=b""):
        self.protocol_version = protocol_version
        self.telemetry_flags = telemetry_flags
        self.command_frame_counter_echo = command_frame_counter_echo
        self.active_enable_mask = active_enable_mask
        self.error_flags = error_flags
        self.line = tuple(line or (0, 0, 0, 0))
        self.motor_current = tuple(motor_current or (0, 0, 0, 0))
        self.battery = tuple(battery or (0, 0))
        self.distance_sensor = distance_sensor
        self.raw = bytes(raw)


class TelemetryState:
    """Thread-safe latest telemetry storage exposed by the link."""

    __slots__ = ("_lock", "_value")

    def __init__(self):
        self._lock = threading.Lock()
        self._value = TelemetrySnapshot()

    def publish(self, value):
        with self._lock:
            self._value = value

    def snapshot(self):
        with self._lock:
            value = self._value
            return TelemetrySnapshot(
                value.protocol_version,
                value.telemetry_flags,
                value.command_frame_counter_echo,
                value.active_enable_mask,
                value.error_flags,
                value.line,
                value.motor_current,
                value.battery,
                value.distance_sensor,
                value.raw,
            )

    def clear(self):
        with self._lock:
            self._value = TelemetrySnapshot()


def unpack_telemetry(frame):
    if len(frame) != TELEMETRY_SIZE:
        raise ValueError("telemetry frame must contain {} bytes".format(TELEMETRY_SIZE))
    received_crc = struct.unpack_from("<H", frame, 78)[0]
    if received_crc != crc16_ccitt_false(frame[:78]):
        raise ValueError("telemetry CRC mismatch")
    if frame[0] != PROTOCOL_VERSION:
        raise ValueError("unsupported telemetry protocol version {}".format(frame[0]))

    return TelemetrySnapshot(
        protocol_version=frame[0],
        telemetry_flags=frame[1],
        command_frame_counter_echo=struct.unpack_from("<I", frame, 2)[0],
        active_enable_mask=struct.unpack_from("<H", frame, 6)[0],
        error_flags=struct.unpack_from("<I", frame, 8)[0],
        line=struct.unpack_from("<4H", frame, 12),
        motor_current=struct.unpack_from("<4H", frame, 20),
        battery=struct.unpack_from("<2H", frame, 28),
        distance_sensor=struct.unpack_from("<H", frame, 32)[0],
        raw=bytes(frame),
    )
