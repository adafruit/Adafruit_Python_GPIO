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

import unittest

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.SPI as SPI

from MockGPIO import MockGPIO


class TestBitBangSPI(unittest.TestCase):
    def test_pin_modes_set_correctly(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        self.assertDictEqual(gpio.pin_mode, { 1: GPIO.OUT,
                                              2: GPIO.OUT,
                                              3: GPIO.IN,
                                              4: GPIO.OUT })

    def test_ss_set_high_after_initialization(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        self.assertListEqual(gpio.pin_written[4], [1])

    def test_mode_0_write(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        device.write([0x1F])
        # Verify clock
        self.assertListEqual(gpio.pin_written[1], [0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1, 0])
        # Verify MOSI
        self.assertListEqual(gpio.pin_written[2], [0, 0, 0, 1, 1, 1, 1, 1])
        # Verify MISO
        self.assertNotIn(3, gpio.pin_written)
        # Verify SS
        self.assertListEqual(gpio.pin_written[4], [1, 0, 1])

    def test_write_assert_deassert_ss_false(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        device.write([0x1F], assert_ss=False, deassert_ss=False)
        self.assertListEqual(gpio.pin_written[4], [1])

    def test_write_lsbfirst(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        device.set_bit_order(SPI.LSBFIRST)
        device.write([0x1F])
        self.assertListEqual(gpio.pin_written[2], [1, 1, 1, 1, 1, 0, 0, 0])

    def test_invalid_mode_fails(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        self.assertRaises(ValueError, device.set_mode, -1)
        self.assertRaises(ValueError, device.set_mode, 4)

    def test_invalid_bit_order_fails(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        self.assertRaises(ValueError, device.set_bit_order, -1)
        self.assertRaises(ValueError, device.set_bit_order, 2)

    def test_mode_0_read(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        gpio.pin_read[3] = [0, 0, 0, 1, 1, 1, 1, 1]
        result = device.read(1)
        # Verify clock
        self.assertListEqual(gpio.pin_written[1], [0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1, 0])
        # Verify MOSI
        self.assertNotIn(2, gpio.pin_written)
        # Verify MISO
        self.assertNotIn(3, gpio.pin_written)
        # Verify SS
        self.assertListEqual(gpio.pin_written[4], [1, 0, 1])
        # Verify result
        self.assertEqual(result, bytearray([0x1F]))

    def test_read_assert_deassert_ss_false(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        gpio.pin_read[3] = [0, 0, 0, 1, 1, 1, 1, 1]
        result = device.read(1, assert_ss=False, deassert_ss=False)
        self.assertListEqual(gpio.pin_written[4], [1])

    def test_read_multiple_bytes(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        gpio.pin_read[3] = [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
                            0, 0, 0, 1, 1, 1, 1, 1]
        result = device.read(3)
        # Verify clock
        self.assertListEqual(gpio.pin_written[1], [0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1, 0])
        # Verify MOSI
        self.assertNotIn(2, gpio.pin_written)
        # Verify MISO
        self.assertNotIn(3, gpio.pin_written)
        # Verify SS
        self.assertListEqual(gpio.pin_written[4], [1, 0, 1])
        # Verify result
        self.assertEqual(result, bytearray([0x1F, 0xF8, 0x1F]))

    def test_write_multiple_bytes(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        device.write([0x1F, 0xF8, 0x1F])
        # Verify clock
        self.assertListEqual(gpio.pin_written[1], [0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1, 0])
        # Verify MOSI
        self.assertListEqual(gpio.pin_written[2], [0, 0, 0, 1, 1, 1, 1, 1,
                                                   1, 1, 1, 1, 1, 0, 0, 0,
                                                   0, 0, 0, 1, 1, 1, 1, 1])
        # Verify MISO
        self.assertNotIn(3, gpio.pin_written)
        # Verify SS
        self.assertListEqual(gpio.pin_written[4], [1, 0, 1])

    def test_mode_0_transfer(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        gpio.pin_read[3] = [0, 0, 0, 1, 1, 1, 1, 1]
        result = device.transfer([0xF8])
        # Verify clock
        self.assertListEqual(gpio.pin_written[1], [0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1, 0])
        # Verify MOSI
        self.assertListEqual(gpio.pin_written[2], [1, 1, 1, 1, 1, 0, 0, 0])
        # Verify MISO
        self.assertNotIn(3, gpio.pin_written)
        # Verify SS
        self.assertListEqual(gpio.pin_written[4], [1, 0, 1])
        # Verify result
        self.assertEqual(result, bytearray([0x1F]))

    def test_transfer_multiple_bytes(self):
        gpio = MockGPIO()
        device = SPI.BitBang(gpio, 1, 2, 3, 4)
        gpio.pin_read[3] = [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
                            0, 0, 0, 1, 1, 1, 1, 1]
        result = device.transfer([0xF8, 0x1F, 0xF8])
        # Verify clock
        self.assertListEqual(gpio.pin_written[1], [0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1,
                                                   0, 1, 0, 1, 0, 1, 0, 1, 0])
        # Verify MOSI
        self.assertListEqual(gpio.pin_written[2], [1, 1, 1, 1, 1, 0, 0, 0,
                                                   0, 0, 0, 1, 1, 1, 1, 1,
                                                   1, 1, 1, 1, 1, 0, 0, 0])
        # Verify MISO
        self.assertNotIn(3, gpio.pin_written)
        # Verify SS
        self.assertListEqual(gpio.pin_written[4], [1, 0, 1])
        # Verify result
        self.assertEqual(result, bytearray([0x1F, 0xF8, 0x1F]))

    #TODO: Test mode 1, 2, 3

    #TODO: Test null MOSI, MISO, SS
