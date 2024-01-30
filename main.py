import gc
import uasyncio
import display
import utime
import sys
from display import getSetDisplay
#from webserver import web_page
from sensorLoop import getReadLoop, devicesOff
from stepper import getStepperLoop, devicesOff as devicesOffStepper

#define I2C_HARDWARE 1
from machine import Pin, SoftI2C

#server = ()

utime.sleep_ms(500)

i2cDisplay = SoftI2C(scl=Pin(18), sda=Pin(17), freq=115200)
i2cOther = SoftI2C(scl=Pin(42), sda=Pin(41), freq=400)
(setDisplay, setLargeDisplay) = getSetDisplay(i2cDisplay)

#loop = None

try:
  setDisplay([ "Hello!" ]) #, station.ifconfig()[0] ])

  stepperLoop = getStepperLoop()
  readLoop = getReadLoop(i2cOther, setLargeDisplay)
  loop = uasyncio.get_event_loop()

  def _handle_exception(loop, context):
    sys.print_exception(context["exception"])
    devicesOff()
    loop.stop()

  loop.set_exception_handler(_handle_exception)
  
  # utime.sleep_ms(500)
  
#  server = uasyncio.start_server(web_page, "", 80, backlog=5)
#  loop.create_task(server)
  loop.create_task(readLoop())
  loop.create_task(stepperLoop())
  loop.run_forever()
  loop.close()
finally:
  print("GOODBYE")
  devicesOffStepper()
  devicesOff()
#  if (server != ()):
#      server.close()
  #if loop:
  #  loop.stop()
  uasyncio.new_event_loop()  # Clear uasyncio stored state

