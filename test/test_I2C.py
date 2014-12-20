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

import logging
import unittest

from mock import Mock, patch

import Adafruit_GPIO.Platform as Platform


# Enable debug logging to stdout during tests.
logging.basicConfig()
logging.getLogger().setLevel(logging.DEBUG)


class MockSMBus(object):
    # Mock the smbus.SMBus class to record all data written to specific
    # addresses and registers in the _written member.
    def __init__(self):
        # _written will store a dictionary of address to register dictionary.
        # Each register dictionary will store a mapping of register value to
        # an array of all written values (in sequential write order).
        self._written = {}
        self._read = {}

    def _write_register(self, address, register, value):
        self._written.setdefault(address, {}).setdefault(register, []).append(value)

    def _read_register(self, address, register):
        return self._read.get(address).get(register).pop(0)

    def write_byte_data(self, address, register, value):
        self._write_register(address, register, value)

    def write_word_data(self, address, register, value):
        self._write_register(address, register, value >> 8 & 0xFF)
        self._write_register(address, register+1, value & 0xFF)

    def write_i2c_block_data(self, address, register, values):
        for i, value in enumerate(values):
            self._write_register(address, register+i, value & 0xFF)

    def read_byte_data(self, address, register):
        return self._read_register(address, register)

    def read_word_data(self, address, register):
        high = self._read_register(address, register)
        low = self._read_register(address, register+1)
        return (high << 8) | low

    def read_i2c_block_data(self, address, length):
        return [self._read_register(address+i) for i in range(length)]


def create_device(address, busnum):
    # Mock the smbus module and inject it into the global namespace so the
    # Adafruit_GPIO.I2C module can be imported.  Also inject a mock SMBus
    # instance to be returned by smbus.SMBus function calls.
    smbus = Mock()
    mockbus = MockSMBus()
    smbus.SMBus.return_value = mockbus
    with patch.dict('sys.modules', {'smbus': smbus}):
        import Adafruit_GPIO.I2C as I2C
        return (I2C.Device(address, busnum), smbus, mockbus)

def safe_import_i2c():
    # Mock the smbus module and inject it into the global namespace so the
    # Adafruit_GPIO.I2C module can be imported.  The imported I2C module is
    # returned so global functions can be called on it.
    with patch.dict('sys.modules', {'smbus': Mock() }):
        import Adafruit_GPIO.I2C as I2C
        return I2C


class TestI2CDevice(unittest.TestCase):

    def test_address_and_bus_set_correctly(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        self.assertEqual(device._bus, mockbus)
        smbus.SMBus.assert_called_with(1)
        self.assertEqual(device._address, 0x1F)

    def test_write8(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        device.write8(0xFE, 0xED)
        self.assertDictEqual(mockbus._written, { 0x1F: { 0xFE: [0xED] }})

    def test_write8_truncates_to_8bits(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        device.write8(0xFE, 0xBEEFED)
        self.assertDictEqual(mockbus._written, { 0x1F: { 0xFE: [0xED] }})

    def test_write16(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        device.write16(0xFE, 0xBEEF)
        self.assertDictEqual(mockbus._written, { 0x1F: { 0xFE: [0xBE],
                                                         0xFF: [0xEF] }})

    def test_write16_truncates_to_8bits(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        device.write16(0xFE, 0xFEEDBEEF)
        self.assertDictEqual(mockbus._written, { 0x1F: { 0xFE: [0xBE],
                                                         0xFF: [0xEF] }})

    def test_writeList(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        device.writeList(0x00, [0xFE, 0xED, 0xBE, 0xEF])
        self.assertDictEqual(mockbus._written, { 0x1F: { 0x00: [0xFE],
                                                         0x01: [0xED],
                                                         0x02: [0xBE],
                                                         0x03: [0xEF] }})

    def test_readU8(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        mockbus._read[0x1F] = { 0xFE: [0xED] }
        value = device.readU8(0xFE)
        self.assertEqual(value, 0xED)

    def test_readS8(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        mockbus._read[0x1F] = { 0xFE: [0xED] }
        value = device.readS8(0xFE)
        self.assertEqual(value, -19)

    def test_readU16(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        mockbus._read[0x1F] = { 0xFE: [0xED], 0xFF: [0x01] }
        value = device.readU16(0xFE)
        self.assertEqual(value, 0xED01)

    def test_readS16(self):
        device, smbus, mockbus = create_device(0x1F, 1)
        mockbus._read[0x1F] = { 0xFE: [0xED], 0xFF: [0x01] }
        value = device.readS16(0xFE)
        self.assertEqual(value, -4863)


class TestGetDefaultBus(unittest.TestCase):
    @patch('Adafruit_GPIO.Platform.pi_revision', Mock(return_value=1))
    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.RASPBERRY_PI))
    def test_raspberry_pi_rev1(self):
        I2C = safe_import_i2c()
        bus = I2C.get_default_bus()
        self.assertEqual(bus, 0)

    @patch('Adafruit_GPIO.Platform.pi_revision', Mock(return_value=2))
    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.RASPBERRY_PI))
    def test_raspberry_pi_rev2(self):
        I2C = safe_import_i2c()
        bus = I2C.get_default_bus()
        self.assertEqual(bus, 1)

    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.BEAGLEBONE_BLACK))
    def test_beaglebone_black(self):
        I2C = safe_import_i2c()
        bus = I2C.get_default_bus()
        self.assertEqual(bus, 1)

    @patch('Adafruit_GPIO.Platform.platform_detect', Mock(return_value=Platform.UNKNOWN))
    def test_unknown(self):
        I2C = safe_import_i2c()
        self.assertRaises(RuntimeError, I2C.get_default_bus)
