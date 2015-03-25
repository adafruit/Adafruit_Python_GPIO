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

import Adafruit_GPIO.Platform as Platform


class TestPlatformDetect(unittest.TestCase):
    @patch('platform.platform', Mock(return_value='Linux-3.8.13-bone47-armv7l-with-debian-7.4'))
    def test_beaglebone_black(self):
        result = Platform.platform_detect()
        self.assertEquals(result, Platform.BEAGLEBONE_BLACK)

    @patch('platform.platform', Mock(return_value='Darwin-13.2.0-x86_64-i386-64bit'))
    def test_unknown(self):
        result = Platform.platform_detect()
        self.assertEquals(result, Platform.UNKNOWN)


class TestPiRevision(unittest.TestCase):
    def test_revision_1(self):
        with patch('__builtin__.open') as mock_open:
            handle = mock_open.return_value.__enter__.return_value
            handle.__iter__.return_value = iter(['Revision : 0000'])
            rev = Platform.pi_revision()
            self.assertEquals(rev, 1)
        with patch('__builtin__.open') as mock_open:
            handle = mock_open.return_value.__enter__.return_value
            handle.__iter__.return_value = iter(['Revision : 0002'])
            rev = Platform.pi_revision()
            self.assertEquals(rev, 1)
        with patch('__builtin__.open') as mock_open:
            handle = mock_open.return_value.__enter__.return_value
            handle.__iter__.return_value = iter(['Revision : 0003'])
            rev = Platform.pi_revision()
            self.assertEquals(rev, 1)

    def test_revision_2(self):
        with patch('__builtin__.open') as mock_open:
            handle = mock_open.return_value.__enter__.return_value
            handle.__iter__.return_value = iter(['Revision : 000e'])
            rev = Platform.pi_revision()
            self.assertEquals(rev, 2)

    def test_unknown_revision(self):
        with patch('__builtin__.open') as mock_open:
            handle = mock_open.return_value.__enter__.return_value
            handle.__iter__.return_value = iter(['foobar'])
            self.assertRaises(RuntimeError, Platform.pi_revision)

