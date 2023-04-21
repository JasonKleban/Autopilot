
from machine import Pin, RTC
import uasyncio
import utime
from state import state
import math
from bno055 import BNO055

import ubinascii, uhashlib

light = Pin(26, Pin.OUT, Pin.PULL_DOWN)
switch = Pin(47, Pin.IN, Pin.PULL_DOWN)
buzzer = Pin(40, Pin.OUT, Pin.PULL_DOWN)

rtc = RTC()

def devicesOff():
    light.value(0)

def getReadLoop(i2c, setLargeDisplay):
    
    imu = BNO055(i2c, transpose=(0, 2, 1), sign=(0, 1, 0))
    imu.set_offsets(bytearray(ubinascii.unhexlify('e7ff0a00eeff31fd54fdf7ff00000000feffe803dc02')))

    async def readLoop():
        nonlocal imu

        while True:
            state.status = 'OK'
            nextEngaged = switch.value()
            nextHeading = imu.euler()[0]

            state.heading = round(nextHeading)
            state.heading = 0 if state.heading == 360 else state.heading

            if not state.calibrated and nextEngaged:
                state.wasEngagedBeforeCalibrated = True

            if state.wasEngagedBeforeCalibrated and not nextEngaged:
                state.wasEngagedBeforeCalibrated = False
                
            alarm = state.wasEngagedBeforeCalibrated

            buzzer.value(alarm)

            #once calibrated, stays calibrated
            if not state.calibrated:
                try:
                    #print('Calibration: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
                    state.calibrated = imu.calibrated()

                    print('Calibration Progress: {} of 3, {} of 3, {} of 3, {} of 3'.format(*imu.cal_status()))

                    setLargeDisplay(state.heading, 0, 'CALIBRATING')

                    if state.calibrated:
                        state.calibration = imu.sensor_offsets()
                        print(hex(int(ubinascii.hexlify(state.calibration).decode(), 16)))
                        buzzer.value(1)
                        await uasyncio.sleep_ms(250)
                        buzzer.value(0)
                        
                except OSError as e:
                    state.status = 'ESensor'
                    print(e)
            else:
                try:
                    if (nextEngaged != state.engaged and nextEngaged) or (not nextEngaged):
                        state.idealHeading = round(nextHeading)
                        state.stepCount = 0
                            
                    state.engaged = not state.wasEngagedBeforeCalibrated and nextEngaged
                    state.correction = (state.idealHeading - state.heading + 540) % 360 - 180

                    upseconds = utime.time() - state.boot

                    indicatorValue = math.trunc(16 * math.atan((5.027 * state.correction)/157.5) / 3.141592)

                    setLargeDisplay(state.heading, indicatorValue, '' if state.engaged else 'STANDBY')
                    
                    print('Heading     {:4.0f} roll {:+5.0f} pitch {:+5.0f}'.format(*imu.euler()))

                    #buzzer.value(45 < abs(diff))

                except OSError as e:
                    print(e)

            await uasyncio.sleep_ms(0)

    return readLoop
