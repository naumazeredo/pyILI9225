# The MIT License (MIT)
#
# Copyright (c) 2017 Ballarat Hackerspace Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import numbers
import time
from libbcm2835._bcm2835 import *


# ILI9225 screen size
TFTWIDTH = 176
TFTHEIGHT = 220

# ILI9225 LCD Registers
DRIVER_OUTPUT_CTRL = 0x01  # Driver Output Control
LCD_AC_DRIVING_CTRL = 0x02  # LCD AC Driving Control
ENTRY_MODE = 0x03  # Entry Mode
DISP_CTRL1 = 0x07  # Display Control 1
BLANK_PERIOD_CTRL1 = 0x08  # Blank Period Control
FRAME_CYCLE_CTRL = 0x0B  # Frame Cycle Control
INTERFACE_CTRL = 0x0C  # Interface Control
OSC_CTRL = 0x0F  # Osc Control
POWER_CTRL1 = 0x10  # Power Control 1
POWER_CTRL2 = 0x11  # Power Control 2
POWER_CTRL3 = 0x12  # Power Control 3
POWER_CTRL4 = 0x13  # Power Control 4
POWER_CTRL5 = 0x14  # Power Control 5
VCI_RECYCLING = 0x15  # VCI Recycling
RAM_ADDR_SET1 = 0x20  # Horizontal GRAM Address Set
RAM_ADDR_SET2 = 0x21  # Vertical GRAM Address Set
GRAM_DATA_REG = 0x22  # GRAM Data Register
GATE_SCAN_CTRL = 0x30  # Gate Scan Control Register
VERTICAL_SCROLL_CTRL1 = 0x31  # Vertical Scroll Control 1 Register
VERTICAL_SCROLL_CTRL2 = 0x32  # Vertical Scroll Control 2 Register
VERTICAL_SCROLL_CTRL3 = 0x33  # Vertical Scroll Control 3 Register
PARTIAL_DRIVING_POS1 = 0x34  # Partial Driving Position 1 Register
PARTIAL_DRIVING_POS2 = 0x35  # Partial Driving Position 2 Register
HORIZONTAL_WINDOW_ADDR1 = 0x36  # Horizontal Address Start Position
HORIZONTAL_WINDOW_ADDR2 = 0x37  # Horizontal Address End Position
VERTICAL_WINDOW_ADDR1 = 0x38  # Vertical Address Start Position
VERTICAL_WINDOW_ADDR2 = 0x39  # Vertical Address End Position
GAMMA_CTRL1 = 0x50  # Gamma Control 1
GAMMA_CTRL2 = 0x51  # Gamma Control 2
GAMMA_CTRL3 = 0x52  # Gamma Control 3
GAMMA_CTRL4 = 0x53  # Gamma Control 4
GAMMA_CTRL5 = 0x54  # Gamma Control 5
GAMMA_CTRL6 = 0x55  # Gamma Control 6
GAMMA_CTRL7 = 0x56  # Gamma Control 7
GAMMA_CTRL8 = 0x57  # Gamma Control 8
GAMMA_CTRL9 = 0x58  # Gamma Control 9
GAMMA_CTRL10 = 0x59  # Gamma Control 10

C_INVOFF = 0x20
C_INVON = 0x21

BLACK = 0x0000
BLUE = 0x001F
RED = 0xF800
GREEN = 0x07E0
CYAN = 0x07FF
MAGENTA = 0xF81F
YELLOW = 0xFFE0
WHITE = 0xFFFF


def color565(r, g, b):
    """Convert red, green, blue components to a 16-bit 565 RGB value. Components
    should be values 0 to 255.
    """
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def split_i16(i):
    """Convert 16 bit integer to two 8 bit integer"""
    return [(i >> 8) & 0xFF, i & 0xFF]


def clamp(v, min_v, max_v):
    return min(max(v, min_v), max_v)


class Display(object):
    """Representation of an ILI9225 TFT LCD."""

    def __init__(self,
                 rs, rst,
                 width=TFTWIDTH,
                 height=TFTHEIGHT):
        """Create an instance of the display using SPI communication. Must
        provide the GPIO pin number for the RS pin and the SPI driver. Can
        optionally provide the GPIO pin number for the reset pin as the rst
        parameter.
        """
        self.rs = rs
        self.rst = rst
        self.width = width
        self.height = height

        # TODO: setup outside
        if not bcm2835_init():
            raise IOError('Could not setup BCM')

        bcm2835_spi_begin()
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST)
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE0)
        bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8)
        bcm2835_spi_chipSelect(BCM2835_SPI_CS0)
        bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW)

    def reset(self):
        """Reset the display, if reset pin is connected."""
        bcm2835_gpio_fsel(self.rs, BCM2835_GPIO_FSEL_OUTP)
        bcm2835_gpio_fsel(self.rst, BCM2835_GPIO_FSEL_OUTP)

        bcm2835_gpio_write(self.rst, LOW)
        bcm2835_delay(100)
        bcm2835_gpio_write(self.rst, HIGH)
        bcm2835_delay(100)

    def setup(self):
        # Start Initial sequence
        self.register(DRIVER_OUTPUT_CTRL, 0x031C)   # set the display line number and display direction
        self.register(LCD_AC_DRIVING_CTRL, 0x0100)  # set 1 line inversion
        self.register(ENTRY_MODE, 0x1030)           # set GRAM write direction and BGR=1.
        self.register(BLANK_PERIOD_CTRL1, 0x0808)   # set the back porch and front porch
        self.register(INTERFACE_CTRL, 0x0000)       # CPU interface
        self.register(OSC_CTRL, 0x0801)             # set Osc  /*0e01*/
        self.register(RAM_ADDR_SET1, 0x0000)        # RAM Address
        self.register(RAM_ADDR_SET2, 0x0000)        # RAM Address

        # Power On sequence
        time.sleep(0.05)
        self.register(POWER_CTRL1, 0x0A00)  # set SAP,DSTB,STB
        self.register(POWER_CTRL2, 0x1038)  # set APON,PON,AON,VCI1EN,VC
        time.sleep(0.05)
        self.register(POWER_CTRL3, 0x1121)  # set BT,DC1,DC2,DC3
        self.register(POWER_CTRL4, 0x0066)  # set GVDD
        self.register(POWER_CTRL5, 0x5F60)  # set VCOMH/VCOML voltage

        # Set GRAM area
        self.register(GATE_SCAN_CTRL, 0x0000)
        self.register(VERTICAL_SCROLL_CTRL1, 0x00DB)
        self.register(VERTICAL_SCROLL_CTRL2, 0x0000)
        self.register(VERTICAL_SCROLL_CTRL3, 0x0000)
        self.register(PARTIAL_DRIVING_POS1, 0x00DB)
        self.register(PARTIAL_DRIVING_POS2, 0x0000)
        self.register(HORIZONTAL_WINDOW_ADDR1, 0x00AF)
        self.register(HORIZONTAL_WINDOW_ADDR2, 0x0000)
        self.register(VERTICAL_WINDOW_ADDR1, 0x00DB)
        self.register(VERTICAL_WINDOW_ADDR2, 0x0000)

        # Adjust GAMMA curve
        self.register(GAMMA_CTRL1, 0x4000)
        self.register(GAMMA_CTRL2, 0x060B)
        self.register(GAMMA_CTRL3, 0x0C0A)
        self.register(GAMMA_CTRL4, 0x0105)
        self.register(GAMMA_CTRL5, 0x0A0C)
        self.register(GAMMA_CTRL6, 0x0B06)
        self.register(GAMMA_CTRL7, 0x0004)
        self.register(GAMMA_CTRL8, 0x0501)
        self.register(GAMMA_CTRL9, 0x0E00)
        self.register(GAMMA_CTRL10, 0x000E)

        time.sleep(0.05)

        self.register(DISP_CTRL1, 0x1017)

    def begin(self):
        """Initialize the display. Should be called once before other calls that
        interact with the display are called.
        """
        self.reset()
        self.setup()
        self.clear(BLACK)

    def command(self, data):
        """Write a byte to the display as command data."""
        bcm2835_gpio_write(self.rs, LOW);
        bcm2835_spi_transfer(c);
        return self

    def register(self, cmd, data):
        """Write a byte command followed by a word of data."""
        bcm2835_gpio_write(self.rs, LOW);
        bcm2835_spi_transfer(cmd);
        bcm2835_gpio_write(self.rs, HIGH);
        bcm2835_spi_write(data);

    def data_array(self, cmd, data):
        """Write a byte command followed by multiple words of data."""
        bcm2835_gpio_write(self.rs, LOW);
        bcm2835_spi_transfer(cmd);
        bcm2835_gpio_write(self.rs, HIGH);
        for d in data:
            bcm2835_spi_write(d);

    def data_repeat(self, cmd, data, count):
        """Write a byte command followed by repeated words of data."""
        bcm2835_gpio_write(self.rs, LOW);
        bcm2835_spi_transfer(cmd);
        bcm2835_gpio_write(self.rs, HIGH);
        for i in range(count):
            bcm2835_spi_write(data);

    def draw_pixel(self, x, y, color):
        if x < 0 or x >= self.width: return
        if y < 0 or y >= self.height: return

        self.register(RAM_ADDR_SET1, x)
        self.register(RAM_ADDR_SET2, y)
        self.register(GRAM_DATA_REG, color)

    def draw_fill_rect(self, x1, y1, x2, y2, color):
        if x1 < 0 or x1 >= self.width: return
        if y1 < 0 or y1 >= self.height: return

        x2 = clamp(x2, 0, self.width - 1)
        y2 = clamp(y2, 0, self.height - 1)

        w = y2 - y1 + 1

        for y in range(y1, y2+1):
            self.register(RAM_ADDR_SET1, x1)
            self.register(RAM_ADDR_SET2, y)
            self.data_repeat(GRAM_DATA_REG, color, w)

    def clear(self, color):
        self.draw_fill_rect(0, 0, self.width-1, self.height-1, color)
