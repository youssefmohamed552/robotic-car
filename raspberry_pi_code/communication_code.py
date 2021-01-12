import inputs
import serial
import time


gamepad_map = {
  "ABS_Y": "JRY ",
  "ABS_X": "JRX ",
  "BTN_THUMBR": "JRP ",
  "ABS_RY": "JLY ",
  "ABS_RX": "JLX ",
  "BTN_THUMBL": "JLP ",
  "ABS_HAT0Y": "BRY ",
  "ABS_HAT0X": "BRX ",
  "BTN_WEST": "Y ",
  "BTN_NORTH": "X ",
  "BTN_EAST": "B ",
  "BTN_SOUTH": "A ",
  "BTN_SELECT": "SL ",
  "BTN_START": "ST ",
  "BTN_MODE": "H ",
  "BTN_TR": "R1 ",
  "ABS_RZ": "R2 ",
  "BTN_TL": "L1 ",
  "ABS_Z": "L2 "
}

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
      break


def main():
  handleJoystick()
  # test_serial()

if __name__ == '__main__':
  main()



