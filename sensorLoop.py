
from machine import Pin, RTC
import uasyncio
import utime
from state import state
from mahonyQuaternion import MahonyQuaternion
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
        
        lastUpdate = utime.time_ns()
        deltat = 0.0
        
        mpu6500 = None
        mpu9250 = None
        
        try:
            mpu6500 = MPU6500(
                i2c,
                accel_sf=SF_G) #,
                #gyro_sf=SF_DEG_S) #,
                #gyro_offset=(-0.06217736, 0.004287855, -0.01491513),
                #accel_offset=(-0.004601356, 0.1052795, 0.2896795))
            
            mpu9250 = MPU9250(
                i2c,
                mpu6500=mpu6500,
                mag_offset=(-6.254883, 81.39698, -20.41348),
                mag_scale=(0.9332827, 1.106261, 0.976022))
            
            #print(mpu6500.calibrate())
            
        except OSError as e:
            state.status = 'ESensor'
            print(e)
            return

        mahony = MahonyQuaternion()

        while True:
            try:
                state.status = 'OK'

                now = utime.time_ns()
                upseconds = utime.time() - state.boot
                deltat = ((now - lastUpdate) / 1000000.0)
                lastUpdate = now

                mahony.update(
                    mpu9250.acceleration[0], -mpu9250.acceleration[1], mpu9250.acceleration[2],
                    mpu9250.gyro[0] * DEG_TO_RAD, -mpu9250.gyro[1] * DEG_TO_RAD, mpu9250.gyro[2] * DEG_TO_RAD,
                    mpu9250.magnetic[1] * 0.01, -mpu9250.magnetic[0] * 0.01, -mpu9250.magnetic[2] * 0.01,
                    deltat
                )
                
                mag_mag = math.sqrt(
                    (mpu9250.magnetic[0] * 0.01) ** 2 +
                    (mpu9250.magnetic[1] * 0.01) ** 2 +
                    (mpu9250.magnetic[2] * 0.01) ** 2)

                q = mahony.get_q()

                state.yaw   = (math.atan2(2.0 * (q[1] * q[2] + q[0]
                                * q[3]), q[0] * q[0] + q[1]
                                * q[1] - q[2] * q[2] - q[3]
                                * q[3])) * RAD_TO_DEG
                state.pitch = (-math.asin(2.0 * (q[1] * q[3] - q[0]
                                * q[2]))) * RAD_TO_DEG
                state.roll  = math.atan2(2.0 * (q[0] * q[1] + q[2]
                                * q[3]), q[0] * q[0] - q[1]
                                * q[1] - q[2] * q[2] + q[3]
                                * q[3])
                
                precursors = f'acce: {mpu9250.acceleration[0]:+06.2f}, {-mpu9250.acceleration[1]:+06.2f}, {mpu9250.acceleration[2]:+06.2f}; ' + \
                             f'gyro: {mpu9250.gyro[0] * DEG_TO_RAD:+06.2f}, {-mpu9250.gyro[1] * DEG_TO_RAD:+06.2f}, {mpu9250.gyro[2] * DEG_TO_RAD:+06.2f}; ' + \
                             f'magn: {mpu9250.magnetic[1] * 0.01:+07.2f}, {-mpu9250.magnetic[0] * 0.01:+07.2f}, {-mpu9250.magnetic[2] * 0.01:+07.2f}; |mag|: {mag_mag:05.2f}; ' + \
                             f'q: {q[0]:+4.2f} {q[1]:+4.2f} {q[2]:+4.2f} {q[3]:+4.2f}'
                            
                final = f'roll:{state.roll:+09.4f}, pitch:{state.pitch:+09.4f}, yaw:{state.yaw:+09.4f}'
                
                print(precursors)
                #print(final)

            except OSError as e:
              state.status = 'ESensor'
              print(e)

            try:
                upseconds = utime.time() - state.boot

                setDisplay([
                    #station.ifconfig()[0],
                    f'uptime:{upseconds}s',
                    f'r:{state.roll:+07.2f}*',
                    f'p:{state.pitch:+07.2f}*',
                    f'y:{state.yaw:+07.2f}*',
                    state.status
                ])
            except OSError as e:
              print(e)

            await uasyncio.sleep_ms(250)

    return readLoop
