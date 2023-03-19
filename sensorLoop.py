
from machine import Pin, RTC
import uasyncio
import utime
from state import state
import math
from bno055 import BNO055

light = Pin(26, Pin.OUT, Pin.PULL_DOWN)
switch = Pin(47, Pin.IN, Pin.PULL_DOWN)
buzzer = Pin(40, Pin.OUT, Pin.PULL_DOWN)

rtc = RTC()

def devicesOff():
    light.value(0)

def getReadLoop(i2c, setDisplay):
    
    imu = BNO055(i2c)
    calibrated = False

    async def readLoop():
        nonlocal imu

        while True:
            try:
                state.status = 'OK'

                #once calibrated, stays calibrated
                if not state.calibrated:
                    state.calibrated = imu.calibrated()
                    # print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
                # print('Temperature {}°C'.format(imu.temperature()))
                # print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
                # print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
                # print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
                # print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
                # print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
                # print('Heading     {:4.0f} roll {:+5.0f} pitch {:+5.0f}'.format(*imu.euler()))

                state.yaw = imu.euler()[0]
                state.engaged = switch.value()
                state.alarm = switch.value()

            except OSError as e:
              state.status = 'ESensor'
              print(e)

            try:
                upseconds = utime.time() - state.boot

                setDisplay([
                    #station.ifconfig()[0],
                    f'uptime:{upseconds}s',
                    f'c:{state.calibrated}',
                    f'e:{state.engaged}',
                    f'h:{state.yaw:+4.0f}*',
                    state.status
                ])

                buzzer.value(state.alarm)

            except OSError as e:
              print(e)

            await uasyncio.sleep_ms(250)

    return readLoop
