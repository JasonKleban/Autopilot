
from machine import Pin, RTC
import uasyncio
import utime
from state import state
import math
from pid import PID

# def devicesOff():
#     light.value(0)

# 1 EN
# 2 M0
# 3 M1
# 4 M2
# 5 Reset
# 6 Slp
# 7 Step
# 19 Dir
# 20 Fault

en = Pin(1, Pin.OUT, Pin.PULL_DOWN)
m0 = Pin(2, Pin.OUT, Pin.PULL_DOWN)
m1 = Pin(3, Pin.OUT, Pin.PULL_DOWN)
m2 = Pin(4, Pin.OUT, Pin.PULL_DOWN)
reset = Pin(5, Pin.OUT, Pin.PULL_DOWN)
slp = Pin(6, Pin.OUT, Pin.PULL_DOWN)
step = Pin(7, Pin.OUT, Pin.PULL_DOWN)
direction = Pin(19, Pin.OUT, Pin.PULL_DOWN)

fault = Pin(20, Pin.IN, Pin.PULL_DOWN)

maxSpeedSpS = 200.0
maxAccelSpSS = 100.0

def getStepperLoop():
    async def stepperLoop():
        accelSpSS = 0.0
        pid = new PID(output_limits=(-maxAccelSpSS, maxAccelSpSS))

        while True:
            try:
                faulted = bool(fault.value())

                if bool(fault.value()):
                    state.alarm = state.engaged
                    en.value(False)
                else:
                    en.value(state.engaged)

                    state.stepCount

                    pid(state.correction)

                    direction.value(0 < state.correction)
                    step.value(1)
                    await uasyncio.sleep_ms(2)
                    step.value(0)

            except OSError as e:
              print(e)

            await uasyncio.sleep_ms(250)

    return stepperLoop
