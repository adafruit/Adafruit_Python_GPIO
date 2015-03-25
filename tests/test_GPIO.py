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
import Adafruit_GPIO.Platform as Platform

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

    def test_output_pins(self):
        gpio = MockGPIO()
        gpio.output_pins({0: True, 1: False, 7: True})
        self.assertDictEqual(gpio.pin_written, {0: [1], 1: [0], 7: [1]})


class TestRPiGPIOAdapter(unittest.TestCase):
    def test_setup(self):
        rpi_gpio = Mock()
        adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
        adapter.setup(1, GPIO.OUT)
        rpi_gpio.setup.assert_called_with(1, rpi_gpio.OUT, pull_up_down=rpi_gpio.PUD_OFF)
        adapter.setup(1, GPIO.IN)
        rpi_gpio.setup.assert_called_with(1, rpi_gpio.IN, pull_up_down=rpi_gpio.PUD_OFF)
        adapter.setup(1, GPIO.IN, GPIO.PUD_DOWN)
        rpi_gpio.setup.assert_called_with(1, rpi_gpio.IN, pull_up_down=rpi_gpio.PUD_DOWN)
        adapter.setup(1, GPIO.IN, GPIO.PUD_UP)
        rpi_gpio.setup.assert_called_with(1, rpi_gpio.IN, pull_up_down=rpi_gpio.PUD_UP)

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

    def test_add_event_detect(self):
        rpi_gpio = Mock()
        adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
        adapter.add_event_detect(1, GPIO.RISING)
        rpi_gpio.add_event_detect.assert_called_with(1, rpi_gpio.RISING)

    def test_remove_event_detect(self):
        rpi_gpio = Mock()
        adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
        adapter.remove_event_detect(1)
        rpi_gpio.remove_event_detect.assert_called_with(1)

    def test_add_event_callback(self):
        rpi_gpio = Mock()
        adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
        adapter.add_event_callback(1, callback=self.test_add_event_callback)
        rpi_gpio.add_event_callback.assert_called_with(1, self.test_add_event_callback)

    def test_event_detected(self):
        rpi_gpio = Mock()
        adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
        adapter.event_detected(1)
        rpi_gpio.event_detected.assert_called_with(1)

    def test_wait_for_edge(self):
        rpi_gpio = Mock()
        adapter = GPIO.RPiGPIOAdapter(rpi_gpio)
        adapter.wait_for_edge(1, GPIO.FALLING)
        rpi_gpio.wait_for_edge.assert_called_with(1, rpi_gpio.FALLING)

    def test_cleanup(self):
        rpi_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(rpi_gpio)
        adapter.cleanup()
        rpi_gpio.cleanup.assert_called()

    def test_cleanup_pin(self):
        rpi_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(rpi_gpio)
        adapter.cleanup(1)
        rpi_gpio.cleanup.assert_called_with(1)


class TestAdafruitBBIOAdapter(unittest.TestCase):
    def test_setup(self):
        bbio_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
        adapter.setup(1, GPIO.OUT)
        bbio_gpio.setup.assert_called_with(1, bbio_gpio.OUT, pull_up_down=bbio_gpio.PUD_OFF)
        adapter.setup(1, GPIO.IN)
        bbio_gpio.setup.assert_called_with(1, bbio_gpio.IN, pull_up_down=bbio_gpio.PUD_OFF)
        adapter.setup(1, GPIO.IN, GPIO.PUD_DOWN)
        bbio_gpio.setup.assert_called_with(1, bbio_gpio.IN, pull_up_down=bbio_gpio.PUD_DOWN)
        adapter.setup(1, GPIO.IN, GPIO.PUD_UP)
        bbio_gpio.setup.assert_called_with(1, bbio_gpio.IN, pull_up_down=bbio_gpio.PUD_UP)

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

    def test_add_event_detect(self):
        bbio_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
        adapter.add_event_detect(1, GPIO.RISING)
        bbio_gpio.add_event_detect.assert_called_with(1, bbio_gpio.RISING)

    def test_add_event_detect(self):
        bbio_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
        adapter.add_event_detect(1, GPIO.RISING)
        bbio_gpio.add_event_detect.assert_called_with(1, bbio_gpio.RISING)

    def test_remove_event_detect(self):
        bbio_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
        adapter.remove_event_detect(1)
        bbio_gpio.remove_event_detect.assert_called_with(1)

    def test_add_event_callback(self):
        bbio_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
        adapter.add_event_callback(1, callback=self.test_add_event_callback)
        bbio_gpio.add_event_callback.assert_called_with(1, self.test_add_event_callback)

    def test_event_detected(self):
        bbio_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
        adapter.event_detected(1)
        bbio_gpio.event_detected.assert_called_with(1)

    def test_wait_for_edge(self):
        bbio_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
        adapter.wait_for_edge(1, GPIO.FALLING)
        bbio_gpio.wait_for_edge.assert_called_with(1, bbio_gpio.FALLING)

    def test_cleanup(self):
        bbio_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
        adapter.cleanup()
        bbio_gpio.cleanup.assert_called()

    def test_cleanup_pin(self):
        bbio_gpio = Mock()
        adapter = GPIO.AdafruitBBIOAdapter(bbio_gpio)
        adapter.cleanup(1)
        bbio_gpio.cleanup.assert_called_with(1)


class TestGetPlatformGPIO(unittest.TestCase):
    @patch.dict('sys.modules', {'RPi': Mock(), 'RPi.GPIO': Mock()})
    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.RASPBERRY_PI))
    def test_raspberrypi(self):
        gpio = GPIO.get_platform_gpio()
        self.assertIsInstance(gpio, GPIO.RPiGPIOAdapter)

    @patch.dict('sys.modules', {'Adafruit_BBIO': Mock(), 'Adafruit_BBIO.GPIO': Mock()})
    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.BEAGLEBONE_BLACK))
    def test_beagleboneblack(self):
        gpio = GPIO.get_platform_gpio()
        self.assertIsInstance(gpio, GPIO.AdafruitBBIOAdapter)

    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.UNKNOWN))
    def test_unknown(self):
        self.assertRaises(RuntimeError, GPIO.get_platform_gpio)
