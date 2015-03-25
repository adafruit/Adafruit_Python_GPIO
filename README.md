Adafruit Python GPIO Library
============================

Library to provide a cross-platform GPIO interface on the Raspberry Pi and Beaglebone Black using the [RPi.GPIO](https://pypi.python.org/pypi/RPi.GPIO) and [Adafruit_BBIO](https://pypi.python.org/pypi/Adafruit_BBIO) libraries.

The library is currently in an early stage, but you can see how its used in the [Adafruit Nokia LCD library](https://github.com/adafruit/Adafruit_Nokia_LCD) to write Python code that is easily portable between the Raspberry Pi and Beaglebone Black.

Note that you typically don't need to install this library directly as other libraries will depend on it in their setup and automatically install it.  However if you do need to manually install do so by running these commands:

- On a Debian-based Linux like Raspbian, Ubuntu, etc. in a terminal execute:
  
  ```
  sudo apt-get update
  sudo apt-get install build-essential python-pip python-dev python-smbus git
  git clone https://github.com/adafruit/Adafruit_Python_GPIO.git
  cd Adafruit_Python_GPIO
  sudo python setup.py install
  ```

- On Mac OSX, first install PIP by [downloading the python script here](https://bootstrap.pypa.io/get-pip.py) and execute it with `python get-pip.py` in a terminal, then install the [git source control system](http://git-scm.com/downloads).  Then in a terminal execute:
  
  ```
  git clone https://github.com/adafruit/Adafruit_Python_GPIO.git
  cd Adafruit_Python_GPIO
  sudo python setup.py install
  ```

- On Windows, first install the [latest Python 2.7 version](https://www.python.org/downloads/windows/), then install PIP by [downloading the python script here](https://bootstrap.pypa.io/get-pip.py) and execute it with `python get-pip.py` in a terminal, and finally install the [git source control system](http://git-scm.com/downloads).  Then in a git bash prompt execute:
  
  ```
  git clone https://github.com/adafruit/Adafruit_Python_GPIO.git
  cd Adafruit_Python_GPIO
  python setup.py install
  ```

Contributing
------------

For information on contributing, such as how to run tests, etc. please see the [project wiki](https://github.com/adafruit/Adafruit_Python_GPIO/wiki/Running-Tests) on GitHub.
