"""Hardware reset control for the STM32 connected to MaixCam."""

import time

from maix import gpio

from . import config


class Stm32Reset:
    def __init__(self):
        self.pin = gpio.GPIO(config.STM32_RESET_PIN, gpio.Mode.OUT)
        self.pin.value(0)

    def pulse(self):
        self.pin.value(0)
        time.sleep(config.STM32_RESET_PRE_DELAY_S)
        self.pin.value(1)
        time.sleep(config.STM32_RESET_PULSE_S)
        self.pin.value(0)
        time.sleep(config.STM32_RESET_SETTLE_S)
