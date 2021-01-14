import inputs
import serial
import time


gamepad_state = {
  "ABS_Y": 0,
  "ABS_X": 0,
  "BTN_THUMBR": 0,
  "ABS_RZ": 0,
  "ABS_Z": 0,
  "BTN_THUMBL": 0,
  "ABS_HAT0Y": 0,
  "ABS_HAT0X": 0,
  "BTN_WEST": 0,
  "BTN_NORTH": 0,
  "BTN_EAST": 0,
  "BTN_SOUTH": 0,
  "BTN_SELECT": 0,
  "BTN_START": 0,
  "BTN_MODE": 0,
  "BTN_TR": 0,
  "BTN_TR2": 0,
  "BTN_TL": 0,
  "BTN_TL2": 0
}

def adjust_value(num, state):
  if state in ["ABS_RZ"]:
    gamepad_state[state] = 128 - num
  if state in ["ABS_Z"]:
    gamepad_state[state] = num - 127
  if state in ["ABS_TR2", "ABS_TL2"] and num == 255:
    pass
    # gamepad_state[state] = 1

def get_command():
  commands = [val for val in gamepad_state.keys() if gamepad_state[val] != 0]
  print("gamepads: ", gamepad_state)
  print("commands: ", commands)
  if len(commands) == 0:
    return "S"
  if len(commands) == 1:
    c = commands[0]
    if c == "ABS_RZ":
      if gamepad_state[c] > 0:
        return "F " + str(gamepad_state[c])
      elif gamepad_state[c] < 0:
          return "B " + str(-1 * gamepad_state[c])
    if c == "ABS_Z":
      if gamepad_state[c] > 0:
        return "R " + str(gamepad_state[c])
      elif gamepad_state[c] < 0:
        return "L " + str(-1 * gamepad_state[c])
  if len(commands) == 2:
    if all(element in ["ABS_RZ", "ABS_Z"] for element in commands):
      return "N "
    if all(element in ["ABS_RZ", "BTN_TR2"] for element in commands):
      print("raspberryPi moving forward")
  return "M"




def handleJoystick():
  print("in handling joystick")
  with serial.Serial('/dev/ttyACM0', 115200, timeout=0.1) as arduino:
    while True:
      events = inputs.get_gamepad()
      for event in events:
        print("ev_type: ", event.ev_type, " code: ", event.code, " type: ", type(event.code), " state: ", event.state)
        if event.ev_type == "Key" or event.ev_type == "Absolute":
          adjust_value(event.state, event.code)
          if event.code in ["ABS_RZ", "ABS_Z"]:
            handleSerialWrite(arduino)
          # test_serial(arduino, event.code, event.state)




def handleSerialWrite(dev):
  print("in test serial")
  cmd = get_command()
  print("cmd: ", cmd)
  dev.write(str.encode(cmd))
  # time.sleep(0.1)
  # while True:
    # data = dev.readline()
    # if data:
      # print(data[:-2])
      # break


def main():
  handleJoystick()
  # test_serial()

if __name__ == '__main__':
  main()



