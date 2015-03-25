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

import Adafruit_GPIO.PWM as PWM
import Adafruit_GPIO.Platform as Platform


class TestRPi_PWM_Adapter(unittest.TestCase):
    def test_setup(self):
        rpi_gpio = Mock()
        pwm = PWM.RPi_PWM_Adapter(rpi_gpio)
        pwm.start(1, 50)
        rpi_gpio.PWM.assert_called_with(1, 2000)

    def test_set_duty_cycle_valid(self):
        rpi_gpio = Mock()
        pwm = PWM.RPi_PWM_Adapter(rpi_gpio)
        pwm.start(1, 50)
        pwm.set_duty_cycle(1, 75)
        # Implicit verification that no assertion or other error thrown.

    def test_set_duty_cycle_invalid(self):
        rpi_gpio = Mock()
        pwm = PWM.RPi_PWM_Adapter(rpi_gpio)
        pwm.start(1, 50)
        self.assertRaises(ValueError, pwm.set_duty_cycle, 1, 150)
        self.assertRaises(ValueError, pwm.set_duty_cycle, 1, -10)

    def test_set_frequency(self):
        rpi_gpio = Mock()
        pwm = PWM.RPi_PWM_Adapter(rpi_gpio)
        pwm.start(1, 50)
        pwm.set_frequency(1, 1000)
        # Implicit verification that no assertion or other error thrown.


class TestBBIO_PWM_Adapter(unittest.TestCase):
    def test_setup(self):
        bbio_pwm = Mock()
        pwm = PWM.BBIO_PWM_Adapter(bbio_pwm)
        pwm.start('P9_16', 50)
        bbio_pwm.start.assert_called_with('P9_16', 50, 2000)

    def test_set_duty_cycle_valid(self):
        bbio_pwm = Mock()
        pwm = PWM.BBIO_PWM_Adapter(bbio_pwm)
        pwm.start('P9_16', 50)
        pwm.set_duty_cycle('P9_16', 75)
        bbio_pwm.set_duty_cycle.assert_called_with('P9_16', 75)

    def test_set_duty_cycle_invalid(self):
        bbio_pwm = Mock()
        pwm = PWM.BBIO_PWM_Adapter(bbio_pwm)
        pwm.start('P9_16', 50)
        self.assertRaises(ValueError, pwm.set_duty_cycle, 'P9_16', 150)
        self.assertRaises(ValueError, pwm.set_duty_cycle, 'P9_16', -10)

    def test_set_frequency(self):
        bbio_pwm = Mock()
        pwm = PWM.BBIO_PWM_Adapter(bbio_pwm)
        pwm.start('P9_16', 50)
        pwm.set_frequency('P9_16', 1000)
        bbio_pwm.set_frequency.assert_called_with('P9_16', 1000)


class TestGetPlatformPWM(unittest.TestCase):
    @patch.dict('sys.modules', {'RPi': Mock(), 'RPi.GPIO': Mock()})
    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.RASPBERRY_PI))
    def test_raspberrypi(self):
        pwm = PWM.get_platform_pwm()
        self.assertIsInstance(pwm, PWM.RPi_PWM_Adapter)

    @patch.dict('sys.modules', {'Adafruit_BBIO': Mock(), 'Adafruit_BBIO.PWM': Mock()})
    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.BEAGLEBONE_BLACK))
    def test_beagleboneblack(self):
        pwm = PWM.get_platform_pwm()
        self.assertIsInstance(pwm, PWM.BBIO_PWM_Adapter)

    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.UNKNOWN))
    def test_otherplatform(self):
        self.assertRaises(RuntimeError, PWM.get_platform_pwm)
