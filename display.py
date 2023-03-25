from machine import Pin, I2C
import ssd1306
from state import state
from bigDigits import clear, digitBuffers
import utime

pin = Pin(21, Pin.OUT)

# led = Pin(5, Pin.OUT)
# led.value(1)

pin.value(0) #set low to reset OLED
utime.sleep_ms(10)
pin.value(1) #while OLED is running, must set high

heading = '000'

def getSetDisplay(i2c):
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
    
    oled.contrast(255)
    
    def setDisplay(lines):
        oled.fill(0)
        for i in range(len(lines)):
            oled.text(lines[i], 0, i * 10)
        
        oled.blit(clear, 0, 0, 1)
        oled.blit(digitBuffers[3], 0, 0, 0)
        
        oled.blit(clear, 42, 0, 1)
        oled.blit(digitBuffers[5], 42, 0, 0)
        
        oled.blit(clear, 84, 0, 1)
        oled.blit(digitBuffers[9], 84, 0, 0)

        oled.show()

    # def setHeading(h):
    #     oled.blit(clear, 0, 0, 1)
    #     oled.blit(digitBuffers[0], 0, 0, 0)
    
    return setDisplay
