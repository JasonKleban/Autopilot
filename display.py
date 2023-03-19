from machine import Pin, I2C
import ssd1306
from state import state

pin = Pin(21, Pin.OUT)

# led = Pin(5, Pin.OUT)
# led.value(1)

pin.value(0) #set low to reset OLED
pin.value(1) #while OLED is running, must set high

def getSetDisplay(i2c):
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
    
    oled.contrast(255)
    
    def setDisplay(lines):
        oled.fill(0)
        for i in range(len(lines)):
            oled.text(lines[i], 0, i * 10)
        oled.show()
    
    return setDisplay
