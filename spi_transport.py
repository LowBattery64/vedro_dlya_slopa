"""MaixPy SPI4 master transport with driver-controlled active-low CS."""

from maix import err, pinmap, spi

from . import config


def _reverse_u32_words(data):
    """Compensate MaixCam SPI4 byte order without intermediate joins."""
    if len(data) % 4 != 0:
        raise ValueError("SPI buffer length must be divisible by 4")
    result = bytearray(data)
    for offset in range(0, len(result), 4):
        result[offset], result[offset + 3] = result[offset + 3], result[offset]
        result[offset + 1], result[offset + 2] = result[offset + 2], result[offset + 1]
    return bytes(result)


class SpiTransport:
    def __init__(self):
        for pin, function in config.SPI_PIN_FUNCTIONS:
            err.check_raise(
                pinmap.set_pin_function(pin, function),
                "Failed to configure {} as {}".format(pin, function),
            )

        self.spi = spi.SPI(
            config.SPI_BUS_ID,
            spi.Mode.MASTER,
            config.SPI_FREQUENCY_HZ,
            polarity=config.SPI_MODE_POLARITY,
            phase=config.SPI_MODE_PHASE,
            bits=config.SPI_BITS,
            hw_cs=-1,
            soft_cs="",
        )

    def transfer(self, tx_frame):
        if len(tx_frame) != config.SPI_FRAME_SIZE:
            raise ValueError("SPI frame must contain {} bytes".format(config.SPI_FRAME_SIZE))
        wire_tx = _reverse_u32_words(tx_frame)
        wire_rx = self.spi.write_read(wire_tx, config.SPI_FRAME_SIZE)
        return _reverse_u32_words(bytes(wire_rx))
