import gc
import uasyncio
import display
from display import getSetDisplay
#from webserver import web_page
from sensorLoop import getReadLoop, devicesOff

#define I2C_HARDWARE 1
from machine import Pin, SoftI2C

#server = ()

# 1 EN
# 2 M0
# 3 M1
# 4 M2
# 5 Reset
# 6 Slp
# 7 Step
# 19 Dir
# 20 fault
# 40 buzzer
# 41 scl
# 42 sda
# 47 switch

i2cDisplay = SoftI2C(scl=Pin(18), sda=Pin(17), freq=115200)
#i2cOther = SoftI2C(scl=Pin(18), sda=Pin(17), freq=400000)
setDisplay = getSetDisplay(i2cDisplay)
#readLoop = getReadLoop(i2cOther, setDisplay)

try:
  setDisplay([ "Hello!" ]) #, station.ifconfig()[0] ])

  loop = uasyncio.get_event_loop()

  def _handle_exception(loop, context):
#    devicesOff()
    print(context)
    loop.stop()

  loop.set_exception_handler(_handle_exception)
  
#  server = uasyncio.start_server(web_page, "", 80, backlog=5)
#  loop.create_task(server)
  loop.create_task(readLoop())
  loop.run_forever()
  loop.close()
finally:
  print("GOODBYE")
  devicesOff()
#  if (server != ()):
#      server.close()
  uasyncio.new_event_loop()  # Clear uasyncio stored state

