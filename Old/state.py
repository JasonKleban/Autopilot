import utime

class Object(object):
    pass

state = Object()
state.status = ""
state.boot = utime.time()
state.yaw = 0.0
state.pitch = 0.0
state.roll = 0.0
