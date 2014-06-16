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

from mock import Mock, patch

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.SPI as SPI

from MockGPIO import MockGPIO


class TestBaseGPIO(unittest.TestCase):
	def test_set_high_and_set_low(self):
		gpio = MockGPIO()
		gpio.set_high(1)
		gpio.set_low(1)
		self.assertDictEqual(gpio.pin_written, {1: [1, 0]})

	def test_is_high_and_is_low(self):
		gpio = MockGPIO()
		gpio.pin_read[1] = [0, 0, 1, 1]
		self.assertTrue(gpio.is_low(1))
		self.assertFalse(gpio.is_high(1))
		self.assertFalse(gpio.is_low(1))
		self.assertTrue(gpio.is_high(1))


class TestRPiGPIOAdapter(unittest.TestCase):
	def test_setup(self):
		rpi_gpio = Mock()
		adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
		adapter.setup(1, GPIO.OUT)
		rpi_gpio.setup.assert_called_with(1, rpi_gpio.OUT)
		adapter.setup(1, GPIO.IN)
		rpi_gpio.setup.assert_called_with(1, rpi_gpio.IN)

	def test_output(self):
		rpi_gpio = Mock()
		adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
		adapter.output(1, True)
		rpi_gpio.output.assert_called_with(1, True)
		adapter.output(1, False)
		rpi_gpio.output.assert_called_with(1, False)

	def test_input(self):
		rpi_gpio = Mock()
		adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
		rpi_gpio.input = Mock(return_value=True)
		val = adapter.input(1)
		self.assertTrue(val)
		rpi_gpio.input.assert_called_with(1)

	def test_setmode(self):
		rpi_gpio = Mock()
		adapter = GPIO.RPiGPIOAdapter(rpi_gpio, mode=rpi_gpio.BCM)
		rpi_gpio.setmode.assert_called_with(rpi_gpio.BCM)
		adapter = GPIO.RPiGPIOAdapter(rpi_gpio, mode=rpi_gpio.BOARD)
		rpi_gpio.setmode.assert_called_with(rpi_gpio.BOARD)
		adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
		rpi_gpio.setmode.assert_called_with(rpi_gpio.BCM)


class TestAdafruitBBIOAdapter(unittest.TestCase):
	def test_setup(self):
		bbio_gpio = Mock()
		adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
		adapter.setup(1, GPIO.OUT)
		bbio_gpio.setup.assert_called_with(1, bbio_gpio.OUT)
		adapter.setup(1, GPIO.IN)
		bbio_gpio.setup.assert_called_with(1, bbio_gpio.IN)

	def test_output(self):
		bbio_gpio = Mock()
		adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
		adapter.output(1, True)
		bbio_gpio.output.assert_called_with(1, True)
		adapter.output(1, False)
		bbio_gpio.output.assert_called_with(1, False)

	def test_input(self):
		bbio_gpio = Mock()
		adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
		bbio_gpio.input = Mock(return_value=True)
		val = adapter.input(1)
		self.assertTrue(val)
		bbio_gpio.input.assert_called_with(1)


class TestGetPlatformGPIO(unittest.TestCase):
	def test_raspberrypi(self):
		rpi = Mock()
		with patch('platform.platform', 
				   new=Mock(return_value='Linux-3.10.25+-armv6l-with-debian-7.4')):
			with patch.dict('sys.modules', 
							{'RPi': rpi, 'RPi.GPIO': rpi.gpio}):
				gpio = GPIO.get_platform_gpio()
				self.assertIsInstance(gpio, GPIO.RPiGPIOAdapter)

	def test_beagleboneblack(self):
		bbio = Mock()
		with patch('platform.platform', 
				   new=Mock(return_value='Linux-3.8.13-bone47-armv7l-with-debian-7.4')):
			with patch.dict('sys.modules', 
			 				{'Adafruit_BBIO': bbio, 'Adafruit_BBIO.GPIO': bbio.gpio}):
				gpio = GPIO.get_platform_gpio()
				self.assertIsInstance(gpio, GPIO.AdafruitBBIOAdapter)

	def test_otherplatform(self):
		with patch('platform.platform', 
				   new=Mock(return_value='Darwin-13.2.0-x86_64-i386-64bit')):
			self.assertRaises(RuntimeError, GPIO.get_platform_gpio)
