"""
zumoshield.py - Pololu's Zumo Robot Shield library supporting the MCUs:
    * MicroPython Pyboard Original.
	* Raspberry-Pi Pico

* Author(s):    Braccio M.  from MCHobby (shop.mchobby.be).
                Meurisse D. from MCHobby (shop.mchobby.be).

See project source @ https://github.com/mchobby/micropython-zumo-robot

See example line_follower.py in the project source
"""
#
# The MIT License (MIT)
#
# Copyright (c) 2019 Meurisse D. for MC Hobby
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

__version__ = "0.0.5"
__repo__ = "https://github.com/mchobby/micropython-zumo-robot.git"

# Identify the plateform
import os
is_pico = os.uname().sysname=='rp2' # Otherwise, it is Pypboad
is_pico_w = is_pico and ( 'Pico W' in os.uname().machine )


from machine import Pin, I2C
if is_pico:
	from machine import PWM
else:
	from pyb import Timer
from qtrsensors import QTRSensors
import time

if is_pico:
    BUTTON_PIN = 20
    LED_PIN = 21

    ZUMO_SENSOR_ARRAY_DEFAULT_EMITTER_PIN = 16
    PWM_L=3
    PWM_R=2
    DIR_L=11
    DIR_R=12
    REFLECT_PIN1 = 14
    REFLECT_PIN2 = 17
    REFLECT_PIN3 = 10
    REFLECT_PIN4 = 19
    REFLECT_PIN5 = 18
    REFLECT_PIN6 = 13
else:
    BUTTON_PIN = "Y7"
    LED_PIN = "Y6"

    ZUMO_SENSOR_ARRAY_DEFAULT_EMITTER_PIN = "X7"
    PWM_L="X8"
    PWM_R="X10"
    DIR_L="X9"
    DIR_R="Y5"

    REFLECT_PIN1 = "X2"
    REFLECT_PIN2 = "X22"
    REFLECT_PIN3 = "Y8"
    REFLECT_PIN4 = "X19"
    REFLECT_PIN5 = "X21"
    REFLECT_PIN6 = "X3"


class ZumoReflectanceSensorArray( QTRSensors ):
	def __init__(self):
		arr = [ Pin( REFLECT_PIN1, Pin.IN ), Pin( REFLECT_PIN2, Pin.IN ), Pin( REFLECT_PIN3, Pin.IN ),
				Pin( REFLECT_PIN4, Pin.IN ), Pin( REFLECT_PIN5, Pin.IN ), Pin( REFLECT_PIN6, Pin.IN ) ]
		super().__init__( arr, Pin(ZUMO_SENSOR_ARRAY_DEFAULT_EMITTER_PIN ,Pin.OUT), timeout=2000)


class ZumoShield():
	""" Create all in one class to control the Zumo """
	def __init__( self ):
		self.ir = ZumoReflectanceSensorArray()
		self.led = Pin( LED_PIN, Pin.OUT )

