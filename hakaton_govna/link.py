"""Continuous SPI worker and public application-facing link API."""

import threading
import time

from . import config
from .protocol import CommandState, TelemetryState, unpack_telemetry
from .spi_transport import SpiTransport
from .stm32_reset import Stm32Reset


class LinkStats:
    __slots__ = ("cycles", "valid_frames", "transfer_errors", "last_error")

    def __init__(self, cycles=0, valid_frames=0, transfer_errors=0,
                 last_error=None):
        self.cycles = cycles
        self.valid_frames = valid_frames
        self.transfer_errors = transfer_errors
        self.last_error = last_error


class MaixcamLink:
    """Own the only SPI/CS worker and expose command/telemetry snapshots.

    ``command`` is written by user code through its setters. ``telemetry`` is
    read with ``read_telemetry()`` or ``telemetry.snapshot()``. The worker is
    the only code that calls SpiTransport.transfer().
    """

    def __init__(self, auto_reset=config.RESET_ON_START):
        self.command = CommandState()
        self.telemetry = TelemetryState()
        # Open SPI only after an optional startup reset has completed. Creating
        # the SPI object can change CS/pin state while STM32 is booting.
        self.transport = None
        self.resetter = Stm32Reset()
        self._state_lock = threading.Lock()
        self._transfer_lock = threading.Lock()
        self._reset_lock = threading.Lock()
        self._running = False
        self._paused = False
        self._thread = None
        self._frame_counter = 0
        self._stats = LinkStats()
        self._auto_reset = bool(auto_reset)
        self._button_input = None
        self._button_raw_pressed = False
        self._button_stable_pressed = False
        self._button_raw_changed_at = 0.0

    @property
    def running(self):
        with self._state_lock:
            return self._running

    def start(self):
        with self._state_lock:
            if self._running:
                return
        if self._auto_reset:
            requested_command = self.command.snapshot()
            self.reset()
            # The reset itself is safe, but preserve the command configured by
            # the application before start() for the first post-reset frame.
            self.command._restore_application_state(requested_command)
        with self._reset_lock:
            if self.transport is None:
                self.transport = SpiTransport()
        with self._state_lock:
            self._running = True
            self._paused = False
        self._thread = threading.Thread(target=self._run, name="maixcam-spi")
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        with self._reset_lock:
            with self._state_lock:
                self._running = False
                self._paused = True
            thread = self._thread
            if thread is not None:
                thread.join(0.2)
            self._thread = None

    def reset(self):
        """Pause new transfers, finish the current one, reset, then resume."""
        with self._reset_lock:
            with self._state_lock:
                self._paused = True
            # A transfer already holding this lock is allowed to finish before
            # the reset pin changes. No transfer can start during pulse/settle.
            with self._transfer_lock:
                self.command.set_safe()
                self.resetter.pulse()
                # This is deliberately after reset and its settle delay.
                if self.transport is None:
                    self.transport = SpiTransport()
                with self._state_lock:
                    self._frame_counter = 0
                    self._paused = False
                self.command._set_frame_counter(0)

    def read_telemetry(self):
        return self.telemetry.snapshot()

    def button_get_state(self):
        """Return the debounced state of the MaixCAM button on GPIOA14."""
        if self._button_input is None:
            from maix import gpio

            self._button_input = gpio.GPIO(config.BUTTON_PIN, gpio.Mode.IN)
            initial = self._button_input.value() == 0
            self._button_raw_pressed = initial
            self._button_stable_pressed = initial
            self._button_raw_changed_at = time.monotonic()

        now = time.monotonic()
        raw_pressed = self._button_input.value() == 0
        if raw_pressed != self._button_raw_pressed:
            self._button_raw_pressed = raw_pressed
            self._button_raw_changed_at = now

        if (raw_pressed != self._button_stable_pressed and
                now - self._button_raw_changed_at >= config.BUTTON_DEBOUNCE_S):
            self._button_stable_pressed = raw_pressed
        return self._button_stable_pressed

    def stats(self):
        with self._state_lock:
            value = self._stats
            return LinkStats(value.cycles, value.valid_frames,
                             value.transfer_errors, value.last_error)

    def _is_paused_or_stopped(self):
        with self._state_lock:
            return (not self._running) or self._paused

    def _record_error(self, error):
        with self._state_lock:
            self._stats.transfer_errors += 1
            self._stats.last_error = str(error)

    def _record_frame(self):
        with self._state_lock:
            self._stats.cycles += 1
            self._stats.valid_frames += 1
            self._stats.last_error = None

    def _run(self):
        deadline = time.monotonic()
        while not self._is_paused_or_stopped():
            if self._is_paused_or_stopped():
                time.sleep(config.PAUSE_POLL_S)
                deadline = time.monotonic()
                continue

            deadline += config.CYCLIC_PERIOD_S
            try:
                with self._transfer_lock:
                    if self._is_paused_or_stopped():
                        continue
                    self._frame_counter = (self._frame_counter + 1) & 0xFFFFFFFF
                    self.command._set_frame_counter(self._frame_counter)
                    rx_frame = self.transport.transfer(
                        self.command.pack(self._frame_counter))
                    # Keep decode/publication under the same lock so reset
                    # cannot publish a stale frame after clearing telemetry.
                    self.telemetry.publish(unpack_telemetry(rx_frame))
                    self._record_frame()
            except Exception as error:
                self._record_error(error)

            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()
