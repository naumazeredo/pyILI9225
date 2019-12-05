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
import wiringpi as wpi


# ILI9225 screen size
ILI9225_TFTWIDTH = 176
ILI9225_TFTHEIGHT = 220

# ILI9225 LCD Registers
ILI9225_DRIVER_OUTPUT_CTRL = 0x01  # Driver Output Control
ILI9225_LCD_AC_DRIVING_CTRL = 0x02  # LCD AC Driving Control
ILI9225_ENTRY_MODE = 0x03  # Entry Mode
ILI9225_DISP_CTRL1 = 0x07  # Display Control 1
ILI9225_BLANK_PERIOD_CTRL1 = 0x08  # Blank Period Control
ILI9225_FRAME_CYCLE_CTRL = 0x0B  # Frame Cycle Control
ILI9225_INTERFACE_CTRL = 0x0C  # Interface Control
ILI9225_OSC_CTRL = 0x0F  # Osc Control
ILI9225_POWER_CTRL1 = 0x10  # Power Control 1
ILI9225_POWER_CTRL2 = 0x11  # Power Control 2
ILI9225_POWER_CTRL3 = 0x12  # Power Control 3
ILI9225_POWER_CTRL4 = 0x13  # Power Control 4
ILI9225_POWER_CTRL5 = 0x14  # Power Control 5
ILI9225_VCI_RECYCLING = 0x15  # VCI Recycling
ILI9225_RAM_ADDR_SET1 = 0x20  # Horizontal GRAM Address Set
ILI9225_RAM_ADDR_SET2 = 0x21  # Vertical GRAM Address Set
ILI9225_GRAM_DATA_REG = 0x22  # GRAM Data Register
ILI9225_GATE_SCAN_CTRL = 0x30  # Gate Scan Control Register
ILI9225_VERTICAL_SCROLL_CTRL1 = 0x31  # Vertical Scroll Control 1 Register
ILI9225_VERTICAL_SCROLL_CTRL2 = 0x32  # Vertical Scroll Control 2 Register
ILI9225_VERTICAL_SCROLL_CTRL3 = 0x33  # Vertical Scroll Control 3 Register
ILI9225_PARTIAL_DRIVING_POS1 = 0x34  # Partial Driving Position 1 Register
ILI9225_PARTIAL_DRIVING_POS2 = 0x35  # Partial Driving Position 2 Register
ILI9225_HORIZONTAL_WINDOW_ADDR1 = 0x36  # Horizontal Address Start Position
ILI9225_HORIZONTAL_WINDOW_ADDR2 = 0x37  # Horizontal Address End Position
ILI9225_VERTICAL_WINDOW_ADDR1 = 0x38  # Vertical Address Start Position
ILI9225_VERTICAL_WINDOW_ADDR2 = 0x39  # Vertical Address End Position
ILI9225_GAMMA_CTRL1 = 0x50  # Gamma Control 1
ILI9225_GAMMA_CTRL2 = 0x51  # Gamma Control 2
ILI9225_GAMMA_CTRL3 = 0x52  # Gamma Control 3
ILI9225_GAMMA_CTRL4 = 0x53  # Gamma Control 4
ILI9225_GAMMA_CTRL5 = 0x54  # Gamma Control 5
ILI9225_GAMMA_CTRL6 = 0x55  # Gamma Control 6
ILI9225_GAMMA_CTRL7 = 0x56  # Gamma Control 7
ILI9225_GAMMA_CTRL8 = 0x57  # Gamma Control 8
ILI9225_GAMMA_CTRL9 = 0x58  # Gamma Control 9
ILI9225_GAMMA_CTRL10 = 0x59  # Gamma Control 10

ILI9225C_INVOFF = 0x20
ILI9225C_INVON = 0x21

ILI9225_BLACK = 0x0000
ILI9225_BLUE = 0x001F
ILI9225_RED = 0xF800
ILI9225_GREEN = 0x07E0
ILI9225_CYAN = 0x07FF
ILI9225_MAGENTA = 0xF81F
ILI9225_YELLOW = 0xFFE0
ILI9225_WHITE = 0xFFFF


def color565(r, g, b):
    """Convert red, green, blue components to a 16-bit 565 RGB value. Components
    should be values 0 to 255.
    """
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def split_i16(i):
    """Convert 16 bit integer to two 8 bit integer"""
    return [(i >> 8) & 0xFF, i & 0xFF]


class ILI9225(object):
    """Representation of an ILI9225 TFT LCD."""

    def __init__(self,
                 rs, rst, cs,
                 width=ILI9225_TFTWIDTH,
                 height=ILI9225_TFTHEIGHT):
        """Create an instance of the display using SPI communication. Must
        provide the GPIO pin number for the RS pin and the SPI driver. Can
        optionally provide the GPIO pin number for the reset pin as the rst
        parameter.
        """
        self._rs = rs
        self._rst = rst
        self._cs = cs
        self.width = width
        self.height = height

        # TODO: setup wiringpi outside
        if wpi.wiringPiSetupGpio() == -1:
            raise IOError('Could not setup WiringPi GPIO')

        if wpi.wiringPiSPISetup(0, 32000000) == -1:
            raise IOError('Could not setup WiringPi SPI')

    def reset(self):
        """Reset the display, if reset pin is connected."""
        wpi.pinMode(self._rs, wpi.OUTPUT)
        wpi.pinMode(self._rst, wpi.OUTPUT)
        wpi.pinMode(self._cs, wpi.OUTPUT)
        wpi.digitalWrite(self._rs, wpi.HIGH)
        wpi.digitalWrite(self._cs, wpi.LOW)

        wpi.digitalWrite(self._rst, wpi.LOW)
        time.sleep(0.1)
        wpi.digitalWrite(self._rst, wpi.HIGH)
        time.sleep(0.1)

    def _init(self):
        # Start Initial sequence
        self.register(ILI9225_DRIVER_OUTPUT_CTRL, 0x031C)   # set the display line number and display direction
        self.register(ILI9225_LCD_AC_DRIVING_CTRL, 0x0100)  # set 1 line inversion
        self.register(ILI9225_ENTRY_MODE, 0x1030)           # set GRAM write direction and BGR=1.
        self.register(ILI9225_BLANK_PERIOD_CTRL1, 0x0808)   # set the back porch and front porch
        self.register(ILI9225_INTERFACE_CTRL, 0x0000)       # CPU interface
        self.register(ILI9225_OSC_CTRL, 0x0801)             # set Osc  /*0e01*/
        self.register(ILI9225_RAM_ADDR_SET1, 0x0000)        # RAM Address
        self.register(ILI9225_RAM_ADDR_SET2, 0x0000)        # RAM Address

        # Power On sequence
        time.sleep(0.05)
        self.register(ILI9225_POWER_CTRL1, 0x0A00)  # set SAP,DSTB,STB
        self.register(ILI9225_POWER_CTRL2, 0x1038)  # set APON,PON,AON,VCI1EN,VC
        time.sleep(0.05)
        self.register(ILI9225_POWER_CTRL3, 0x1121)  # set BT,DC1,DC2,DC3
        self.register(ILI9225_POWER_CTRL4, 0x0066)  # set GVDD
        self.register(ILI9225_POWER_CTRL5, 0x5F60)  # set VCOMH/VCOML voltage

        # Set GRAM area
        self.register(ILI9225_GATE_SCAN_CTRL, 0x0000)
        self.register(ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB)
        self.register(ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000)
        self.register(ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000)
        self.register(ILI9225_PARTIAL_DRIVING_POS1, 0x00DB)
        self.register(ILI9225_PARTIAL_DRIVING_POS2, 0x0000)
        self.register(ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF)
        self.register(ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000)
        self.register(ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB)
        self.register(ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000)

        # Adjust GAMMA curve
        self.register(ILI9225_GAMMA_CTRL1, 0x4000)
        self.register(ILI9225_GAMMA_CTRL2, 0x060B)
        self.register(ILI9225_GAMMA_CTRL3, 0x0C0A)
        self.register(ILI9225_GAMMA_CTRL4, 0x0105)
        self.register(ILI9225_GAMMA_CTRL5, 0x0A0C)
        self.register(ILI9225_GAMMA_CTRL6, 0x0B06)
        self.register(ILI9225_GAMMA_CTRL7, 0x0004)
        self.register(ILI9225_GAMMA_CTRL8, 0x0501)
        self.register(ILI9225_GAMMA_CTRL9, 0x0E00)
        self.register(ILI9225_GAMMA_CTRL10, 0x000E)

        time.sleep(0.05)

        self.register(ILI9225_DISP_CTRL1, 0x1017)

    def begin(self):
        """Initialize the display. Should be called once before other calls that
        interact with the display are called.
        """
        self.reset()
        self._init()

    def command(self, data):
        """Write a byte to the display as command data."""
        wpi.pinMode(self._rs, wpi.LOW)
        buf = bytes([data])
        wpi.wiringPiSPIDataRW(0, buf)
        return self

    def data_bytearray(self, data):
        """Write an array of bytes to the display as display data."""
        # Set RS low for command, high for data.
        wpi.pinMode(self._rs, wpi.HIGH)

        # write data a chunk at a time.
        buf = bytes(data)
        wpi.wiringPiSPIDataRW(0, buf)
        return self

    def register(self, cmd, word):
        """Write a byte command followed by 2 bytes of data."""
        return self.command(cmd).data_bytearray(split_i16(word))

    def draw_pixel(self, x, y, color):
        self.register(ILI9225_RAM_ADDR_SET1, x)
        self.register(ILI9225_RAM_ADDR_SET2, y)
        self.register(ILI9225_GRAM_DATA_REG, color)
