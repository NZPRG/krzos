#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.cleanup()  # Reset any previous configurations

# Test GPIO pin number for PFWD motor
_gpio_a = 16
_gpio_b = 26

# Setup pins as inputs
GPIO.setup(_gpio_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(_gpio_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def callback_a(channel):
    print(f"Callback on GPIO {_gpio_a}")

def callback_b(channel):
    print(f"Callback on GPIO {_gpio_b}")

try:
    # Add edge detection
    GPIO.add_event_detect(_gpio_a, GPIO.BOTH, callback=callback_a)
    GPIO.add_event_detect(_gpio_b, GPIO.BOTH, callback=callback_b)
    
    # Keep the script running
    input("Press Enter to exit\n")

finally:
    GPIO.cleanup()

