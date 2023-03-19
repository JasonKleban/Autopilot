from machine import Pin, SoftI2C

class TicI2C:
  def __init__(self, i2c, address = 14):
    self.i2c = i2c
    self.address = address

  # Sends the "Exit safe start" command.
  def exit_safe_start(self):
    self.i2c.writeto(self.address, bytearray([0x83]))
    
  # Sends the "Reset command timeout" command.
  def reset_command_timeout(self):
    self.i2c.writeto(self.address, bytearray([0x8C]))
    
  # Sends the "energize" command.
  def energize(self):
    self.i2c.writeto(self.address, bytearray([0x85]))
    
  # Sends the "set_max_speed" command.
  # microsteps per 10,000s
  def set_max_speed(self, speed):
    self.i2c.writeto(self.address, bytearray([0xE6, speed]))
    
  # Sends the "set_max_accel" command.
  # microsteps per 100 s²
  def set_max_accel(self, speed):
    self.i2c.writeto(self.address, bytearray([0xEA, speed]))
    
  # Sends the "set_max_decel" command.
  # microsteps per 100 s²
  def set_max_decel(self, speed):
    self.i2c.writeto(self.address, bytearray([0xE9, speed]))
    
  # Sends the "set_step_mode" command.
  def set_step_mode(self, mode):
    self.i2c.writeto(self.address, bytearray([0x94, mode]))
    
  # Sends the "deenergize" command.
  def deenergize(self):
    self.i2c.writeto(self.address, bytearray([0x86]))

  # Sets the target position.
  #
  # For more information about what this command does, see the
  # "Set target position" command in the "Command reference" section of the
  # Tic user's guide.
  def set_target_position(self, target):
    self.i2c.writeto(self.address, bytearray([0xE0,
      target >> 0 & 0xFF,
      target >> 8 & 0xFF,
      target >> 16 & 0xFF,
      target >> 24 & 0xFF]))

  # Gets one or more variables from the Tic.
  def get_variables(self, offset, length):
    self.i2c.writeto(self.address, bytearray([0xA1,offset]))
    return list(self.i2c.readfrom(self.address, length))

  # Gets the "Current position" variable from the Tic.
  def get_current_position(self):
    b = self.get_variables(0x22, 4)
    position = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24)
    if position >= (1 << 31):
      position -= (1 << 32)
    return position
