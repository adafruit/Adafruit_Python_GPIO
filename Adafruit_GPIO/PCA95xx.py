'''
Adafruit compatible using BaseGPIO class to represent a PCA9555 IO expander
Copyright (C) 2016 Matias Vidal
Ported from: https://github.com/dberlin/PCA95XX

# Copyright 2012 Daniel Berlin

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

# For the PCA 953X and 955X series, the chips with 8 GPIO's have these port numbers
# The chips with 16 GPIO's have the first port for each type at double these numbers
# IE The first config port is 6

INPUT_PORT = 0
OUTPUT_PORT = 1
POLARITY_PORT = 2
CONFIG_PORT = 3

IN = GPIO.IN
OUT = GPIO.OUT
HIGH = GPIO.HIGH
LOW = GPIO.LOW


class PCA9555(GPIO.BaseGPIO):
    """Class to represent a PCA9555  GPIO extender. Compatible
    with the Adafruit_GPIO BaseGPIO class so it can be used as a custom GPIO
    class for interacting with device.
    """
    NUM_GPIO = 16

    def __init__(self, address=0x20, busnum=None, i2c=None, num_gpios=16, **kwargs):
        address = int(address)
        self.__name__ = "PCA955"
        # Create I2C device.
        i2c = i2c or I2C
        busnum = busnum or i2c.get_default_bus()
        self._device = i2c.get_i2c_device(address, busnum, **kwargs)
        self.num_gpios = num_gpios
        
        if self.num_gpios <= 8:
            self.iodir = self._device.readU8(CONFIG_PORT)
            self.outputvalue = self._device.readU8(OUTPUT_PORT)

        elif self.num_gpios > 8 and self.num_gpios <= 16:
            self.iodir = self._device.readU16(CONFIG_PORT<< 1)
            self.outputvalue = self._device.readU16(OUTPUT_PORT << 1)

    def _changebit(self, bitmap, bit, value):
        assert value == 1 or value == 0, "Value is %s must be 1 or 0" % value
        if value == 0:
            return bitmap & ~(1 << bit)
        elif value == 1:
            return bitmap | (1 << bit)

    # Change the value of bit PIN on port PORT to VALUE.  If the
    # current pin state for the port is passed in as PORTSTATE, we
    # will avoid doing a read to get it.  The port pin state must be
    # complete if passed in (IE it should not just be the value of the
    # single pin we are trying to change)
    def _readandchangepin(self, port, pin, value, portstate = None):
        assert pin >= 0 and pin < self.num_gpios, "Pin number %s is invalid, only 0-%s are valid" % (pin, self.num_gpios)
        if not portstate:
          if self.num_gpios <= 8:
             portstate = self._device.readU8(port)
          elif self.num_gpios > 8 and self.num_gpios <= 16:
             portstate = self._device.readU16(port << 1)
        newstate = self._changebit(portstate, pin, value)
        if self.num_gpios <= 8:
            self._device.write8(port, newstate)
        else:
            self._device.write16(port << 1, newstate)
        return newstate

    # Polarity inversion
    def polarity(self, pin, value):
        return self._readandchangepin(POLARITY_PORT, pin, value)

    # Pin direction
    def config(self, pin, mode):
        self.iodir = self._readandchangepin(CONFIG_PORT, pin, mode, self.iodir)
        return self.iodir

    def output(self, pin, value):
        assert self.iodir & (1 << pin) == 0, "Pin %s not set to output" % pin
        self.outputvalue = self._readandchangepin(OUTPUT_PORT, pin, value, self.outputvalue)
        return self.outputvalue

    def input(self, pin):
        assert self.iodir & (1 << pin) != 0, "Pin %s not set to input" % pin
        if self.num_gpios <= 8:
            value = self._device.readU8(INPUT_PORT)
        elif self.num_gpios > 8 and self.num_gpios <= 16:
            value = self._device.readU16(INPUT_PORT << 1)
        return value & (1 << pin)

    def setup(self, pin, mode):
        self.config(pin, mode)

    def cleanup(self, pin=None):
        # nothing to cleanup
        pass
