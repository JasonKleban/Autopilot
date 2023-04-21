
from machine import Pin, RTC
import uasyncio
import utime
from state import state
import math
from pid import PID

#DRV 8825
# 1 EN
# 2 M0
# 3 M1
# 4 M2
# 5 Reset
# 6 Slp
# 7 Step
# 19 Dir --> 46
# 20 fault  --> 45

en = Pin(1, Pin.OUT, Pin.PULL_DOWN)
m0 = Pin(2, Pin.OUT, Pin.PULL_DOWN)
m1 = Pin(3, Pin.OUT, Pin.PULL_DOWN)
m2 = Pin(4, Pin.OUT, Pin.PULL_DOWN)
reset = Pin(5, Pin.OUT, Pin.PULL_DOWN)
slp = Pin(6, Pin.OUT, Pin.PULL_DOWN)
step = Pin(7, Pin.OUT, Pin.PULL_DOWN)
direction = Pin(46, Pin.OUT, Pin.PULL_DOWN)

fault = Pin(45, Pin.IN, Pin.PULL_DOWN)

def devicesOff():
    en.value(1)

maxSpeedSpS = 200.0
maxAccelSpSS = 100.0

def getStepperLoop():
    async def stepperLoop():
        #accelSpSS = 0.0
        #pid = new PID(output_limits=(-maxAccelSpSS, maxAccelSpSS))
        
        reset.value(0)
        slp.value(0)

        await uasyncio.sleep_ms(10)
                    
        reset.value(1)
        slp.value(1)

        while True:
            try:
                #faulted = bool(fault.value())

                #if bool(fault.value()):
                #    state.alarm = state.engaged
                #    en.value(False)
                #else:

                # state.stepCount

                # pid(state.correction)
                
                en.value(not state.engaged)

                direction.value(state.correction < 0)

                if state.engaged and abs(state.correction) > 1:
                    step.value(1)
                    await uasyncio.sleep_ms(2)
                    step.value(0)

                #print('Step')

            except OSError as e:
              print(e)

            await uasyncio.sleep_ms(5)

    return stepperLoop
