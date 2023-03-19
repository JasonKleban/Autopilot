
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

def getReadLoop(i2c, setDisplay):
    
    imu = BNO055(i2c, transpose=(2, 1, 0), sign=(1, 0, 0))
    imu.set_offsets(bytearray(ubinascii.unhexlify('22001e00e6ff40009702f3fdffffffff0000e803de04')))

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
                    
                    # print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
                # print('Temperature {}°C'.format(imu.temperature()))
                # print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
                # print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
                # print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
                # print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
                # print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
                # print('Heading     {:4.0f} roll {:+5.0f} pitch {:+5.0f}'.format(*imu.euler()))

                nextHeading = imu.euler()[0]
                nextEngaged = switch.value()

                state.heading = nextHeading

                if (nextEngaged != state.engaged and nextEngaged) or (not nextEngaged):
                    state.idealHeading = nextHeading
                        
                state.engaged = nextEngaged
                #state.alarm = switch.value()

            except OSError as e:
              state.status = 'ESensor'
              print(e)

            try:
                upseconds = utime.time() - state.boot
                
                diff = (state.idealHeading - state.heading + 540) % 360 - 180
                
                # print(diff)
                # correction = diff if 180 < abs(diff) else ((360 * 2) - diff) % 360

                indicatorValue = math.trunc(16 * math.atan((5.027 * diff)/157.5) / 3.141592)

                indicator = f'{'<' * -indicatorValue:>8}' if indicatorValue < 0 else f'        {'>' * indicatorValue}'

                setDisplay([
                    #station.ifconfig()[0],
                    f'uptime:{upseconds}s',
                    f'c:{state.calibrated}',
                    f'h:{state.heading:+4.0f}*',
                    f'i:{diff:+4.0f}*',
                    indicator if state.engaged else '    STANDBY',
                ])

                #buzzer.value(state.alarm)
                buzzer.value(45 < abs(diff))

            except OSError as e:
              print(e)

            await uasyncio.sleep_ms(250)

    return readLoop
