
from machine import Pin, RTC, Signal, PWM
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
# 45 Dir
# 46 Fault

maxSpeedSpS = 20000.0
#maxAccelSpSS = 100.0

en = Pin(1, Pin.OUT, Pin.PULL_DOWN)
m0 = Pin(2, Pin.OUT, Pin.PULL_DOWN)
m1 = Pin(3, Pin.OUT, Pin.PULL_DOWN)
m2 = Pin(4, Pin.OUT, Pin.PULL_DOWN)
reset = Pin(5, Pin.OUT, Pin.PULL_DOWN)
slp = Pin(6, Pin.OUT, Pin.PULL_DOWN)
# step = Pin(7, Pin.OUT, Pin.PULL_DOWN)
step = None
direction = Pin(46, Pin.OUT, Pin.PULL_DOWN)
fault = Pin(45, Pin.IN, Pin.PULL_DOWN)

divisor_speeds = {
    1: (0, 0, 0),
    2: (1, 0, 0),
    4: (0, 1, 0),
    8: (1, 1, 0),
    16: (0, 0, 1),
    32: (1, 0, 1)
}

def setSpeed(divisor):
    m = divisor_speeds[divisor]
    m0.value(m[0])
    m1.value(m[1])
    m2.value(m[2])

def devicesOff():
    en.value(1)

def getStepperLoop():
    async def stepperLoop():
        global step
        
        #accelSpSS = 0.0
        pid = PID(output_limits=(-maxSpeedSpS * 32, maxSpeedSpS * 32), sample_time = None)
        
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

                recommendation = int(pid(state.correction) * 10.0)
                abs_rec = abs(recommendation)
                direction.value(0 < recommendation)
                
                # print('{:+08.4f} full steps per second'.format(recommendation))
                
                en.value(not state.engaged)

                if state.engaged:
                    if abs_rec > 1:
                        if abs_rec * 32 < maxSpeedSpS:
                            setSpeed(32)
                            step = PWM(Pin(7), freq=abs_rec * 32, duty=512)
                        elif abs_rec * 16 < maxSpeedSpS:
                            setSpeed(16)
                            step = PWM(Pin(7), freq=abs_rec * 16, duty=512)
                        elif abs_rec * 8 < maxSpeedSpS:
                            setSpeed(8)
                            step = PWM(Pin(7), freq=abs_rec * 8, duty=512)
                        elif abs_rec * 4 < maxSpeedSpS:
                            setSpeed(4)
                            step = PWM(Pin(7), freq=abs_rec * 4, duty=512)
                        elif abs_rec * 2 < maxSpeedSpS:
                            setSpeed(2)
                            step = PWM(Pin(7), freq=abs_rec * 2, duty=512)
                        else:
                            setSpeed(1)
                            step = PWM(Pin(7), freq=abs_rec * 1, duty=512)
                    elif step is not None:
                        # full step mode for strongest hold
                        setSpeed(1)

                        #no pwm though
                        step.deinit()
                        step = None

                #print('Step')

            except OSError as e:
              print(e)

            await uasyncio.sleep_ms(5)

    return stepperLoop
