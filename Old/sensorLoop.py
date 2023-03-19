
from machine import Pin, RTC
import uasyncio
import utime
from state import state
from mahonyQuaternion import MahonyQuaternion
from madgwickQuaternion import MadgwickQuaternion
import math

from mpu6500 import MPU6500, SF_G, SF_DEG_S
from mpu9250 import MPU9250
from ak8963 import AK8963

light = Pin(26, Pin.OUT, Pin.PULL_DOWN)

rtc = RTC()

def devicesOff():
    light.value(0)

DEG_TO_RAD = 0.017453292519943295769236907684886
RAD_TO_DEG = 57.295779513082320876798154814105

def getReadLoop(i2c, setDisplay):

    async def readLoop():

        mpu6500 = None
        mpu9250 = None

        try:
            mpu6500 = MPU6500(
                i2c,
                gyro_offset=(-0.06308424, 0.005061981, -0.01411032),
                accel_offset=(-0.4055954, 0.07087928, -0.3054762))

            mpu9250 = MPU9250(
                i2c,
                mpu6500=mpu6500,
                mag_offset=(2.233887, 78.51885, -11.19727),
                mag_scale=(1.002542, 0.9729811, 1.025887))

            #print(mpu6500.calibrate(count=2000, delay=5))
            #print(mpu9250.calibrate_mag(count=2000, delay=10))

        except OSError as e:
            state.status = 'ESensor'
            print(e)
            return

        mahony = MahonyQuaternion()
        #madgwick = MadgwickQuaternion()

        lastUpdate = utime.time_ns()
        deltaT = 0.0

        while True:
            try:
                state.status = 'OK'

                now = utime.time_ns()
                upseconds = utime.time() - state.boot
                deltaT = ((now - lastUpdate) / 1000000.0)
                lastUpdate = now

                aN, aE, aD = -mpu9250.acceleration[0], mpu9250.acceleration[1], -mpu9250.acceleration[2]
                gN, gE, gD = -mpu9250.gyro[0], mpu9250.gyro[1], -mpu9250.gyro[2]
                mN, mE, mD = -mpu9250.magnetic[1], mpu9250.magnetic[0], mpu9250.magnetic[2]

                for _ in range(10):
                    mahony.update(
                        aN, aE, aD,
                        gN, gE, gD,
                        mN, mE, mD,
                        deltaT
                    )

                q = mahony.get_q()

                state.roll = math.atan2(
                        q[0] * q[1] + q[3] * q[2],
                        0.5 - (q[1] * q[1] - q[2] * q[2])
                    ) * RAD_TO_DEG
                state.pitch = math.asin(-2.0 * (q[3] * q[1] - q[2] * q[0])) * RAD_TO_DEG
                state.yaw = math.atan2(
                        q[3] * q[0] + q[1] * q[2],
                        0.5 - q[2] * q[2] - q[3] * q[3]
                    ) * RAD_TO_DEG
                
                if state.yaw < 0:
                    state.yaw += 360
                
                #state.yaw += 10.5
                
                state.yaw += 720
                
                state.yaw %= 360

                #precursors = f'acce: {aN:+06.2f}, {aE:+06.2f}, {aD:+06.2f}'
                #precursors = f'gyro: {gN:+06.2f}, {gE:+06.2f}, {gD:+06.2f}'
                #precursors = f'magn: {mN:+07.2f}, {mE:+07.2f}, {mD:+07.2f}' # |mag|: {mag_mag:05.2f};'

                q_log = f'q: {q[0]:+4.2f} {q[1]:+4.2f} {q[2]:+4.2f} {q[3]:+4.2f}'

                final = f'roll:{state.roll:+09.4f}, pitch:{state.pitch:+09.4f}, yaw:{state.yaw:03.0f}'

                print(final)

            except OSError as e:
              state.status = 'ESensor'
              print(e)

            try:
                upseconds = utime.time() - state.boot

                setDisplay([
                    #station.ifconfig()[0],
                    f'uptime:{upseconds}s',
                    f'r:{state.roll:+07.2f}\'',
                    f'p:{state.pitch:+07.2f}\'',
                    f'y:{state.yaw:03.0f}\'',
                    state.status
                ])
            except OSError as e:
              print(e)

            await uasyncio.sleep_ms(50)

    return readLoop
