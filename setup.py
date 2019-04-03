print('Adafruit GPIO Library')
print('Works best with Python 2.7')
print('THIS INSTALL SCRIPT MAY REQUIRE ROOT/ADMIN PERMISSIONS')
print('Especially if you installed python for "all users" on Windows')
print('\ntry the following in your systems terminal if ensurepip is not sufficient:')
print('$ python -m ensurepip --upgrade')
print('$ python -m pip install --upgrade pip setuptools')

import sys
try:
    import pip
    from setuptools import setup, find_packages
except ImportError:
    import ensurepip
    ensurepip.version()
    ensurepip.bootstrap()
    from setuptools import setup, find_packages

# Define required packages.
requires = ['adafruit-pureio']

# Assume spidev is required on non-windows & non-mac platforms (i.e. linux).
if sys.platform != 'win32' and sys.platform != 'darwin':
    requires.append('spidev')

setup(name              = 'Adafruit_GPIO',
      version           = '1.0.4',
      author            = 'Tony DiCola',
      author_email      = 'tdicola@adafruit.com',
      description       = 'Library to provide a cross-platform GPIO interface on the Raspberry Pi and Beaglebone Black using the RPi.GPIO and Adafruit_BBIO libraries.',
      license           = 'MIT',
      url               = 'https://github.com/adafruit/Adafruit_Python_GPIO/',
      install_requires  = requires,
      test_suite        = 'tests',
      packages          = find_packages())
