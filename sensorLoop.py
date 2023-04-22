
from machine import Pin, RTC
import uasyncio
import utime
from state import state
import math
from bno055 import BNO055

import ubinascii, uhashlib

light = Pin(35, Pin.OUT, Pin.PULL_DOWN)
switch = Pin(47, Pin.IN, Pin.PULL_DOWN)
buzzer = Pin(40, Pin.OUT, Pin.PULL_DOWN)

rtc = RTC()

def devicesOff():
    light.value(0)

def getReadLoop(i2c, setLargeDisplay):
    
    imu = BNO055(i2c, transpose=(1, 2, 0), sign=(1, 1, 0))
    imu.set_offsets(bytearray(ubinascii.unhexlify('f3ff1a00defff3ff6dfd2bfd010000000000e803c002')))

    async def readLoop():
        nonlocal imu
        
        lastCalibrationStatus = ()

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
                    
                    nextCalibrationStatus = list(imu.cal_status())
                    
                    #print(nextCalibrationStatus)
                    
                    if lastCalibrationStatus != nextCalibrationStatus:
                        print('Calibration Progress: {} of 3, {} of 3, {} of 3, {} of 3'.format(*nextCalibrationStatus))
                        lastCalibrationStatus = nextCalibrationStatus

                    setLargeDisplay(state.heading, 0, 'CALIBRATING')

                    if state.calibrated:
                        state.calibration = imu.sensor_offsets()
                        print(hex(int(ubinascii.hexlify(state.calibration).decode(), 16)))
                        buzzer.value(1)
                        light.value(1)
                        await uasyncio.sleep_ms(250)
                        buzzer.value(0)
                        light.value(0)
                        
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
                    
                    #print('Heading     {:4.0f} roll {:+5.0f} pitch {:+5.0f}'.format(*imu.euler()))
                    #print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
                    #print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
                    
                    print('Heading     {:4.0f} roll {:+5.0f} pitch {:+5.0f}  Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.euler(), *imu.gravity()))

                    #buzzer.value(45 < abs(diff))

                except OSError as e:
                    print(e)

            await uasyncio.sleep_ms(500)

    return readLoop
