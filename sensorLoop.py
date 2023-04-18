
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
    
    imu = BNO055(i2c, transpose=(0, 1, 2), sign=(0, 0, 0))
    imu.set_offsets(bytearray(ubinascii.unhexlify('e1ff0400ebff4bfe1d0232ff0000feff0000e803a702')))

    async def readLoop():
        nonlocal imu

        while True:
            state.status = 'OK'

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
                except OSError as e:
                    state.status = 'ESensor'
                    print(e)
            else:
                try:
                    # print('Orientation: {:0.2f}, {:0.2f}, {:0.2f}'.format(*imu.euler()))
                    # print('Quaternion: {:0.2f}, {:0.2f}, {:0.2f}, {:0.2f}'.format(*imu.quaternion()))

                    nextHeading = imu.euler()[0]
                    nextEngaged = switch.value()

                    state.heading = round(nextHeading)
                    state.heading = 0 if state.heading == 360 else state.heading

                    if (nextEngaged != state.engaged and nextEngaged) or (not nextEngaged):
                        state.idealHeading = nextHeading
                        state.stepCount = 0
                            
                    state.engaged = nextEngaged
                    state.correction = (state.idealHeading - state.heading + 540) % 360 - 180

                    upseconds = utime.time() - state.boot

                    indicatorValue = math.trunc(16 * math.atan((5.027 * state.correction)/157.5) / 3.141592)

                    setLargeDisplay(state.heading, indicatorValue, '' if state.engaged else 'STANDBY')

                    #buzzer.value(state.alarm)
                    #buzzer.value(45 < abs(diff))

                except OSError as e:
                    print(e)

            await uasyncio.sleep_ms(250)

    return readLoop
