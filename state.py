import utime

class Object(object):
    pass

state = Object()
state.status = ""
state.boot = utime.time()
state.heading = 0.0
state.idealHeading = 0.0
state.pitch = 0.0
state.roll = 0.0
state.calibrated = False
state.engaged = False
state.alarm = False