import gc
import utime
from simple_pid import PID

#define I2C_HARDWARE 1
from machine import Pin, SoftI2C, Timer

from t825 import TicI2C
from bno055 import BNO055

print(f'Hello')

i2c = SoftI2C(scl=Pin(5), sda=Pin(16), freq=100000)
tic = TicI2C(i2c)
imu = BNO055(i2c)

light1 = Pin(2, Pin.OUT) # LED
light2 = Pin(16, Pin.OUT) # LED
light1.value(0)
light2.value(0)

#tic.set_step_mode(5)
#tic.set_max_speed(500000)
#tic.set_max_accel(2147483647)
#tic.set_max_decel(0)

def move():
    position = tic.get_current_position()
    print(f"Current position is {position}.")

    new_target = -200 if position > 0 else 200
    print(f"Setting target position to {new_target}.")

    tic.energize()
    tic.exit_safe_start()
    tic.set_target_position(new_target)
    utime.sleep_ms(500)
    utime.sleep_ms(500)
    utime.sleep_ms(500)
    tic.deenergize()

#timer = Timer(-1)
#timer.init(period=3000, mode=Timer.PERIODIC, callback=lambda t:move())

calibrated = False
while True:
    utime.sleep_ms(1000)
    if not calibrated:
        calibrated = imu.calibrated()
        print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
    print('Temperature {}°C'.format(imu.temperature()))
    print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
    print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
    print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
    print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
    print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
    print('Heading     {:4.0f} roll {:+5.0f} pitch {:+5.0f}'.format(*imu.euler()))

# try:
#   #setDisplay([ "Hello!", station.ifconfig()[0] ])

#   loop = uasyncio.get_event_loop()

#   def _handle_exception(loop, context):
# #    devicesOff()
#     print(context)
#     loop.stop()

#   loop.set_exception_handler(_handle_exception)

# #  server = uasyncio.start_server(web_page, "", 80, backlog=5)
# #  loop.create_task(server)
#   loop.create_task(readLoop())
#   loop.run_forever()
#   loop.close()
# finally:
#   print("GOODBYE")
#   devicesOff()
# #  if (server != ()):
# #      server.close()
#   uasyncio.new_event_loop()  # Clear uasyncio stored state
