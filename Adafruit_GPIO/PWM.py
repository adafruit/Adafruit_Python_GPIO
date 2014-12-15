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
import Adafruit_GPIO.Platform as Platform


class RPi_PWM_Adapter(object):
    """PWM implementation for the Raspberry Pi using the RPi.GPIO PWM library."""

    def __init__(self, rpi_gpio, mode=None):
        self.rpi_gpio = rpi_gpio
        # Suppress warnings about GPIO in use.
        rpi_gpio.setwarnings(False)
        # Set board or BCM pin numbering.
        if mode == rpi_gpio.BOARD or mode == rpi_gpio.BCM:
            rpi_gpio.setmode(mode)
        elif mode is not None:
            raise ValueError('Unexpected value for mode.  Must be BOARD or BCM.')
        else:
            # Default to BCM numbering if not told otherwise.
            rpi_gpio.setmode(rpi_gpio.BCM)
        # Store reference to each created PWM instance.
        self.pwm = {}

    def start(self, pin, dutycycle, frequency_hz=2000):
        """Enable PWM output on specified pin.  Set to intiial percent duty cycle
        value (0.0 to 100.0) and frequency (in Hz).
        """
        if dutycycle < 0.0 or dutycycle > 100.0:
            raise ValueError('Invalid duty cycle value, must be between 0.0 to 100.0 (inclusive).')
        # Make pin an output.
        self.rpi_gpio.setup(pin, self.rpi_gpio.OUT)
        # Create PWM instance and save a reference for later access.
        self.pwm[pin] = self.rpi_gpio.PWM(pin, frequency_hz)
        # Start the PWM at the specified duty cycle.
        self.pwm[pin].start(dutycycle)

    def set_duty_cycle(self, pin, dutycycle):
        """Set percent duty cycle of PWM output on specified pin.  Duty cycle must
        be a value 0.0 to 100.0 (inclusive).
        """
        if dutycycle < 0.0 or dutycycle > 100.0:
            raise ValueError('Invalid duty cycle value, must be between 0.0 to 100.0 (inclusive).')
        if pin not in self.pwm:
            raise ValueError('Pin {0} is not configured as a PWM.  Make sure to first call start for the pin.'.format(pin))
        self.pwm[pin].ChangeDutyCycle(dutycycle)

    def set_frequency(self, pin, frequency_hz):
        """Set frequency (in Hz) of PWM output on specified pin."""
        if pin not in self.pwm:
            raise ValueError('Pin {0} is not configured as a PWM.  Make sure to first call start for the pin.'.format(pin))
        self.pwm[pin].ChangeFrequency(frequency_hz)

    def stop(self, pin):
        """Stop PWM output on specified pin."""
        if pin not in self.pwm:
            raise ValueError('Pin {0} is not configured as a PWM.  Make sure to first call start for the pin.'.format(pin))
        self.pwm[pin].stop()
        del self.pwm[pin]


class BBIO_PWM_Adapter(object):
    """PWM implementation for the BeagleBone Black using the Adafruit_BBIO.PWM
    library.
    """

    def __init__(self, bbio_pwm):
        self.bbio_pwm = bbio_pwm

    def start(self, pin, dutycycle, frequency_hz=2000):
        """Enable PWM output on specified pin.  Set to intiial percent duty cycle
        value (0.0 to 100.0) and frequency (in Hz).
        """
        if dutycycle < 0.0 or dutycycle > 100.0:
            raise ValueError('Invalid duty cycle value, must be between 0.0 to 100.0 (inclusive).')
        self.bbio_pwm.start(pin, dutycycle, frequency_hz)

    def set_duty_cycle(self, pin, dutycycle):
        """Set percent duty cycle of PWM output on specified pin.  Duty cycle must
        be a value 0.0 to 100.0 (inclusive).
        """
        if dutycycle < 0.0 or dutycycle > 100.0:
            raise ValueError('Invalid duty cycle value, must be between 0.0 to 100.0 (inclusive).')
        self.bbio_pwm.set_duty_cycle(pin, dutycycle)

    def set_frequency(self, pin, frequency_hz):
        """Set frequency (in Hz) of PWM output on specified pin."""
        self.bbio_pwm.set_frequency(pin, frequency_hz)

    def stop(self, pin):
        """Stop PWM output on specified pin."""
        self.bbio_pwm.stop(pin)


def get_platform_pwm(**keywords):
    """Attempt to return a PWM instance for the platform which the code is being
    executed on.  Currently supports only the Raspberry Pi using the RPi.GPIO
    library and Beaglebone Black using the Adafruit_BBIO library.  Will throw an
    exception if a PWM instance can't be created for the current platform.  The
    returned PWM object has the same interface as the RPi_PWM_Adapter and
    BBIO_PWM_Adapter classes.
    """
    plat = Platform.platform_detect()
    if plat == Platform.RASPBERRY_PI:
        import RPi.GPIO
        return RPi_PWM_Adapter(RPi.GPIO, **keywords)
    elif plat == Platform.BEAGLEBONE_BLACK:
        import Adafruit_BBIO.PWM
        return BBIO_PWM_Adapter(Adafruit_BBIO.PWM, **keywords)
    elif plat == Platform.UNKNOWN:
        raise RuntimeError('Could not determine platform.')
