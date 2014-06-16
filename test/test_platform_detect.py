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

import Adafruit_GPIO.platform_detect as pd

class TestPlatformDetect(unittest.TestCase):
	def test_raspberry_pi(self):
		with patch('platform.platform', 
				   new=Mock(return_value='Linux-3.10.25+-armv6l-with-debian-7.4')):
			result = pd.platform_detect()
		self.assertEquals(result, pd.RASPBERRY_PI)

	def test_beaglebone_black(self):
		with patch('platform.platform', 
				   new=Mock(return_value='Linux-3.8.13-bone47-armv7l-with-debian-7.4')):
			result = pd.platform_detect()
		self.assertEquals(result, pd.BEAGLEBONE_BLACK)

	def test_unknown(self):
		with patch('platform.platform', 
				   new=Mock(return_value='Darwin-13.2.0-x86_64-i386-64bit')):
			result = pd.platform_detect()
		self.assertEquals(result, pd.UNKNOWN)
