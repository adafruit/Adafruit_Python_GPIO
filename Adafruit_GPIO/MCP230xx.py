# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import math

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.I2C as I2C


class MCP230xxBase(GPIO.BaseGPIO):
    """Base class to represent an MCP230xx series GPIO extender.  Is compatible
    with the Adafruit_GPIO BaseGPIO class so it can be used as a custom GPIO
    class for interacting with device.
    """

    def __init__(self, address, i2c=None, **kwargs):
        """Initialize MCP230xx at specified I2C address and bus number.  If bus
        is not specified it will default to the appropriate platform detected bus.
        """
        # Create I2C device.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(address, **kwargs)
        # Assume starting in ICON.BANK = 0 mode (sequential access).
        # Compute how many bytes are needed to store count of GPIO.
        self.gpio_bytes = int(math.ceil(self.NUM_GPIO/8.0))
        # Buffer register values so they can be changed without reading.
        self.iodir = [0xFF]*self.gpio_bytes  # Default direction to all inputs.
        self.gppu = [0x00]*self.gpio_bytes  # Default to pullups disabled.
        self.gpio = [0x00]*self.gpio_bytes
        # Write current direction and pullup buffer state.
        self.write_iodir()
        self.write_gppu()


    def setup(self, pin, value):
        """Set the input or output mode for a specified pin.  Mode should be
        either GPIO.OUT or GPIO.IN.
        """
        self._validate_pin(pin)
        # Set bit to 1 for input or 0 for output.
        if value == GPIO.IN:
            self.iodir[int(pin/8)] |= 1 << (int(pin%8))
        elif value == GPIO.OUT:
            self.iodir[int(pin/8)] &= ~(1 << (int(pin%8)))
        else:
            raise ValueError('Unexpected value.  Must be GPIO.IN or GPIO.OUT.')
        self.write_iodir()


    def output(self, pin, value):
        """Set the specified pin the provided high/low value.  Value should be
        either GPIO.HIGH/GPIO.LOW or a boolean (True = HIGH).
        """
        self.output_pins({pin: value})

    def output_pins(self, pins):
        """Set multiple pins high or low at once.  Pins should be a dict of pin
        name to pin value (HIGH/True for 1, LOW/False for 0).  All provided pins
        will be set to the given values.
        """
        [self._validate_pin(pin) for pin in pins.keys()]
        # Set each changed pin's bit.
        for pin, value in iter(pins.items()):
            if value:
                self.gpio[int(pin/8)] |= 1 << (int(pin%8))
            else:
                self.gpio[int(pin/8)] &= ~(1 << (int(pin%8)))
        # Write GPIO state.
        self.write_gpio()


    def input(self, pin):
        """Read the specified pin and return GPIO.HIGH/True if the pin is pulled
        high, or GPIO.LOW/False if pulled low.
        """
        return self.input_pins([pin])[0]

    def input_pins(self, pins):
        """Read multiple pins specified in the given list and return list of pin values
        GPIO.HIGH/True if the pin is pulled high, or GPIO.LOW/False if pulled low.
        """
        [self._validate_pin(pin) for pin in pins]
        # Get GPIO state.
        self.gpio = self._device.readList(self.GPIO, self.gpio_bytes)
        # Return True if pin's bit is set.
        return [(self.gpio[int(pin/8)] & 1 << (int(pin%8))) > 0 for pin in pins]


    def pullup(self, pin, enabled):
        """Turn on the pull-up resistor for the specified pin if enabled is True,
        otherwise turn off the pull-up resistor.
        """
        self._validate_pin(pin)
        if enabled:
            self.gppu[int(pin/8)] |= 1 << (int(pin%8))
        else:
            self.gppu[int(pin/8)] &= ~(1 << (int(pin%8)))
        self.write_gppu()

    def write_gpio(self, gpio=None):
        """Write the specified byte value to the GPIO registor.  If no value
        specified the current buffered value will be written.
        """
        if gpio is not None:
            self.gpio = gpio
        self._device.writeList(self.GPIO, self.gpio)

    def write_iodir(self, iodir=None):
        """Write the specified byte value to the IODIR registor.  If no value
        specified the current buffered value will be written.
        """
        if iodir is not None:
            self.iodir = iodir
        self._device.writeList(self.IODIR, self.iodir)

    def write_gppu(self, gppu=None):
        """Write the specified byte value to the GPPU registor.  If no value
        specified the current buffered value will be written.
        """
        if gppu is not None:
            self.gppu = gppu
        self._device.writeList(self.GPPU, self.gppu)


class MCP23017(MCP230xxBase):
    """MCP23017-based GPIO class with 16 GPIO pins."""
    # Define number of pins and registor addresses.
    NUM_GPIO = 16
    IODIR    = 0x00
    GPIO     = 0x12
    GPPU     = 0x0C

    def __init__(self, address=0x20, **kwargs):
        super(MCP23017, self).__init__(address, **kwargs)


class MCP23008(MCP230xxBase):
    """MCP23008-based GPIO class with 8 GPIO pins."""
    # Define number of pins and registor addresses.
    NUM_GPIO = 8
    IODIR    = 0x00
    GPIO     = 0x09
    GPPU     = 0x06

    def __init__(self, address=0x20, **kwargs):
        super(MCP23008, self).__init__(address, **kwargs)
