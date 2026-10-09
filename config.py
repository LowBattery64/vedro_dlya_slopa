"""MaixCam hardware and cyclic link configuration."""

SPI_BUS_ID = 4
SPI_FREQUENCY_HZ = 250_000
SPI_PIN_FUNCTIONS = (
    ("A24", "SPI4_CS"),
    ("A23", "SPI4_MISO"),
    ("A25", "SPI4_MOSI"),
    ("A22", "SPI4_SCK"),
)
SPI_MODE_POLARITY = 0
SPI_MODE_PHASE = 0
SPI_BITS = 8
SPI_FRAME_SIZE = 80
CYCLIC_PERIOD_S = 0.010

BUTTON_PIN = "GPIOA29"
BUTTON_DEBOUNCE_S = 0.050

STM32_RESET_PIN = "GPIOA15"
STM32_RESET_PRE_DELAY_S = 1.000
STM32_RESET_PULSE_S = 0.400
STM32_RESET_SETTLE_S = 2.000

# The bridge is a reusable communication library. Reset is an explicit user
# operation via link.reset(), not an implicit side effect of start().
RESET_ON_START = False
# Polling is only used while reset pauses the worker. Normal traffic is timed
# by CYCLIC_PERIOD_S and never sleeps for this interval between transactions.
PAUSE_POLL_S = 0.001
