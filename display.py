from machine import Pin, I2C
import ssd1306
from state import state
from bigDigits import clear, digitBuffers
import utime
import math

pin = Pin(21, Pin.OUT)

# led = Pin(5, Pin.OUT)
# led.value(1)

pin.value(0) #set low to reset OLED
utime.sleep_ms(100)
pin.value(1) #while OLED is running, must set high
utime.sleep_ms(100)
pin.value(0) #set low to reset OLED
utime.sleep_ms(100)
pin.value(1) #while OLED is running, must set high

def getSetDisplay(i2c):
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
    
    oled.contrast(255)

    _heading_digits = [-1, -1, -1]
    _indicatorValue = None
    _status = None
    
    def setDisplay(lines):
        oled.fill(0)
        for i in range(len(lines)):
            oled.text(lines[i], 0, i * 10)

        oled.show()

    def setLargeDisplay(heading, correction = 0, status = None):
        nonlocal _heading_digits
        nonlocal _indicatorValue
        nonlocal _status

        heading_digits = [ 
            math.trunc(heading / 100), 
            math.trunc((heading % 100) / 10), 
            math.trunc(heading % 10)
        ]

        indicatorValue = math.trunc(128 * math.atan((5.027 * state.correction)/157.5) / 3.141592)

        anyDiff = _heading_digits != heading_digits or status != _status or indicatorValue != _indicatorValue

        for d in range(3):
            if heading_digits[d] != _heading_digits[d] or status != _status:
                oled.blit(clear, 42 * d, 0, 1)
                oled.blit(digitBuffers[heading_digits[d]], 42 * d, 0, 0)
                
        if status != '':
            length = len(status or '')
            oled.fill_rect(62 - length * 4, 25, 4 + length * 8, 12, 0)
            oled.text(status, 64 - length * 4, 27)

        oled.fill_rect(0, 61, 128, 3, 0)
        
        if indicatorValue < 0:
            oled.fill_rect(64 + indicatorValue, 62, -indicatorValue, 2, 1)
        else:
            oled.fill_rect(64, 62, indicatorValue, 2, 1)

        if anyDiff:
            _heading_digits = heading_digits
            _indicatorValue = indicatorValue
            _status = status
            oled.show()
    
    return (setDisplay, setLargeDisplay)
