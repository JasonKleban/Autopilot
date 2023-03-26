
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
    #imu.set_offsets(bytearray(ubinascii.unhexlify('22001e00e6ff40009702f3fdffffffff0000e803de04')))

    async def readLoop():
        nonlocal imu

        while True:
            try:
                state.status = 'OK'

                #once calibrated, stays calibrated
                if not state.calibrated:
                    state.calibrated = imu.calibrated()
                # else:
                #     if imu.calibrated():
                #         calibration = imu.sensor_offsets()
                #         print(hex(int(ubinascii.hexlify(calibration).decode(), 16)))
                
                q = imu.quaternion()

                yaw = math.atan2(
                    2.0 * (q[1] * q[2] + q[0] * q[3]), 
                    q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
                pitch = -math.asin(
                    2.0 * (q[1] * q[3] - q[0] * q[2]))
                roll = math.atan2(
                    2.0 * (q[0] * q[1] + q[2] * q[3]), 
                    q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])

                    # print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
                # print('Temperature {}°C'.format(imu.temperature()))
                # print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
                # print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
                # print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
                # print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
                # print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
                print('Heading     {:4.0f} roll {:+5.0f} pitch {:+5.0f}'.format(*imu.euler()))
                # print('{:+0.5f}, {:+0.5f}, {:+0.5f}, {:+0.5f}'.format(*imu.quaternion()))
                #print('Heading2    {:4.0f} roll {:+5.0f} pitch {:+5.0f}'.format(yaw, roll, pitch))

                nextHeading = imu.euler()[0]
                nextEngaged = switch.value()

                state.heading = round(nextHeading)
                state.heading = 0 if state.heading == 360 else state.heading

                if (nextEngaged != state.engaged and nextEngaged) or (not nextEngaged):
                    state.idealHeading = nextHeading
                    state.stepCount = 0
                        
                state.engaged = nextEngaged
                state.correction = (state.idealHeading - state.heading + 540) % 360 - 180

            except OSError as e:
              state.status = 'ESensor'
              print(e)

            try:
                upseconds = utime.time() - state.boot

                indicatorValue = math.trunc(16 * math.atan((5.027 * state.correction)/157.5) / 3.141592)

                indicator = f'{'<' * -indicatorValue:>8}' if indicatorValue < 0 else f'        {'>' * indicatorValue}'

                setLargeDisplay(state.heading, indicatorValue, '' if state.engaged else 'STANDBY' if state.calibrated else 'CALIBRATING')
                # setDisplay([
                #     #station.ifconfig()[0],
                #     f'uptime:{upseconds}s',
                #     f'h:{state.heading:+4.0f}*',
                #     indicator if state.engaged else '    STANDBY',
                # ])

                #buzzer.value(state.alarm)
                #buzzer.value(45 < abs(diff))

            except OSError as e:
              print(e)

            await uasyncio.sleep_ms(250)

    return readLoop
