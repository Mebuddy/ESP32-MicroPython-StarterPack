from machine import Pin, PWM
import time

led = PWM(Pin(32, Pin.OUT))
led.freq(2000)

while True:
    for duty in range(0, 1024, 10):
        led.duty(duty)
        time.sleep(0.01)

    for duty in range(1023, -1, -10):
        led.duty(duty)
        time.sleep(0.01)
