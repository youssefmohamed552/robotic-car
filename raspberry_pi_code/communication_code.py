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
  val = (num - 255) if state in ["ABS_Y", "ABS_X", "ABS_RZ", "ABS_Z"] else num
  val = 1 if state in ["ABS_TR2", "ABS_TL2"] and num == 255 else 0



def handleJoystick():
  print("in handling joystick")
with serial.Serial('/dev/ttyACM0', 115200, timeout=0.1) as arduino:
    while True:
      events = inputs.get_gamepad()
      for event in events:
        print("ev_type: ", event.ev_type, " code: ", event.code, " type: ", type(event.code), " state: ", event.state)
        if event.ev_type == "Key" or event.ev_type == "Absolute":
          handleSerialWrite(arduino, event.code, event.state)
          # test_serial(arduino, event.code, event.state)




def handleSerialWrite(dev, code, value):
  print("in test serial")
  dev.write(str.encode(gamepad_map[code] + str(value)))
  # time.sleep(0.1)
  while True:
    data = dev.readline()
    if data:
      print(data[:-2])
      # break


def main():
  handleJoystick()
  # test_serial()

if __name__ == '__main__':
  main()



