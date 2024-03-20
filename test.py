print("Hello World")

from machine import Pin
import utime
led = Pin(6, Pin.OUT)

while True:
    led.value(1)
    print("ON")
    utime.sleep(1)
    led.value(0)
    print("OFF")
    utime.sleep(1)