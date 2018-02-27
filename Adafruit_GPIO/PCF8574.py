'''
Adafruit compatible using BaseGPIO class to represent a PCF8574/A IO expander
Copyright (C) 2015 Sylvan Butler

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.'''

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.I2C as I2C



IN = GPIO.IN
OUT = GPIO.OUT
HIGH = GPIO.HIGH
LOW = GPIO.LOW


class PCF8574(GPIO.BaseGPIO):
    """Class to represent a PCF8574 or PCF8574A GPIO extender. Compatible
    with the Adafruit_GPIO BaseGPIO class so it can be used as a custom GPIO
    class for interacting with device.
    """

    NUM_GPIO = 8

    def __init__(self, address=0x27, busnum=None, i2c=None, **kwargs):
        address = int(address)
        self.__name__ = \
            "PCF8574" if address in range(0x20, 0x28) else \
            "PCF8574A" if address in range(0x38, 0x40) else \
            "Bad address for PCF8574(A): 0x%02X not in range [0x20..0x27, 0x38..0x3F]" % address
        if self.__name__[0] != 'P':
            raise ValueError(self.__name__)
        # Create I2C device.
        i2c = i2c or I2C
        busnum = busnum or i2c.get_default_bus()
        self._device = i2c.get_i2c_device(address, busnum, **kwargs)
        # Buffer register values so they can be changed without reading.
        self.iodir = 0xFF  # Default direction to all inputs is in
        self.gpio = 0x00
        self._write_pins()


    def _write_pins(self):
        self._device.writeRaw8(self.gpio | self.iodir)

    def _read_pins(self):
        return self._device.readRaw8() & self.iodir


    def setup(self, pin, mode):
        self.setup_pins({pin: mode})

    def setup_pins(self, pins):
        if False in [y for x,y in [(self._validate_pin(pin),mode in (IN,OUT)) for pin,mode in pins.items()]]:
            raise ValueError('Invalid MODE, IN or OUT')
        for pin,mode in pins.items():
            self.iodir = self._bit2(self.iodir, pin, mode)
        self._write_pins()


    def output(self, pin, value):
        self.output_pins({pin: value})

    def output_pins(self, pins):
        [self._validate_pin(pin) for pin in pins.keys()]
        for pin,value in pins.items():
            self.gpio = self._bit2(self.gpio, pin, bool(value))
        self._write_pins()


    def input(self, pin):
        return self.input_pins([pin])[0]

    def input_pins(self, pins):
        [self._validate_pin(pin) for pin in pins]
        inp = self._read_pins()
        return [bool(inp & (1<<pin)) for pin in pins]
