from threading import Thread
import serial
from matplotlib import pyplot as plt

THRUSTER_SERIAL_PORT = 'COM14'

TENSO_SIZE = 1000

# thruster module id
TENSOMETER_MODULE_ID = (0x01)
tenso_data = []

############## app helper functions #########



############## app serial communication #####



############## app data center ###############

def tensometer_data(message):
    print("\nTensometer data.")
    if (len(tenso_data) < TENSO_SIZE):
        tenso_value = message[2]
        tenso_value |= message[3]<<8
        tenso_data.append(tenso_value)

def data_center(i, message):
    switcher={
            1:tensometer_data,
            }
    func=switcher.get(i, lambda :'Invalid')
    return func(message)

############## app control center ############

def tensometer_offset():
    print("\nTensometer offset.")
    tenso_message = [0] * 10
    tenso_message[0] = TENSOMETER_MODULE_ID
    tenso_message[1] = 1
    tenso_message[2] = 3
    print(tenso_message)
    thruster_serial.write(tenso_message)


def tensometer_calib():
    print("\nTensometer calibration.")
    weight = int(input("Enter weight :> "))
    tenso_message = [0] * 10
    tenso_message[0] = TENSOMETER_MODULE_ID
    tenso_message[1] = 3
    tenso_message[2] = 4
    tenso_message[3] = weight&0xFF
    tenso_message[4] = weight>>8
    print(tenso_message)
    thruster_serial.write(tenso_message)

def tensometer_start():
    print("\nTensometer start.")
    rate = int(input("Enter rate :> "))
    tenso_message = [0] * 10
    tenso_message[0] = TENSOMETER_MODULE_ID
    tenso_message[1] = 3
    tenso_message[2] = 1
    tenso_message[3] = rate&0xFF
    tenso_message[4] = rate>>8
    print(tenso_message)
    thruster_serial.write(tenso_message)

def tensometer_stop():
    print("\nTensometer stop.")
    tenso_message = [0] * 10
    tenso_message[0] = TENSOMETER_MODULE_ID
    tenso_message[1] = 1
    tenso_message[2] = 2
    print(tenso_message)
    thruster_serial.write(tenso_message)

def tensometer_plot():
    print("\nTensometer set rate.")
    # open plot dispaly
    x = list(range(0, len(tenso_data)))
    y = tenso_data
    print(len(x))
    print(len(y))
    plt.cla()
    plt.clf()
    plt.close()
    plt.plot(x, y)
    plt.savefig("tenso_plot.png")

def tensometer_get_measurements():
    print("\nTensometer get measurement.")
    print(tenso_data)
    tenso_data.clear()
    print(tenso_data)

def tensometer_get_value():
    print("\nTensometer offset.")
    tenso_message = [0] * 10
    tenso_message[0] = TENSOMETER_MODULE_ID
    tenso_message[1] = 1
    tenso_message[2] = 5
    print(tenso_message)
    thruster_serial.write(tenso_message)

def control_center(i):
    switcher={
            1:tensometer_offset,
            2:tensometer_calib,
            3:tensometer_start,
            4:tensometer_plot,
            5:tensometer_get_measurements,
            6:tensometer_get_value,
            7:tensometer_stop,
            }
    func=switcher.get(i, lambda :'Invalid')
    return func()

############### app thread ##################

def thread_receive_messages():

    while True:
        if (stop_threads == True):
            break
        serial_message = thruster_serial.read(size=2)
        tens = list(serial_message)
        if (len(serial_message) == 2):
            #data_center(int(serial_message[0]), serial_message)
            value = int(tens[0]) | int(tens[1])<<8 
            if (len(tenso_data) < (TENSO_SIZE-10) and value < 10000):
                print("Value: ", end='')
                print(value)
                tenso_data.append(int(tens[0]) | int(tens[1])<<8)

def thread_dispaly_menu():
     while True:
        print("""\nCommands:
0 - Exit app;
1 - Tensometer offset;
2 - Tensometer calibration;
3 - Tensometer start;
4 - Plot tensometer data;
5 - Get measurements;
6 - Get value;
7 - Tensometer stop;
""", end='')
        control_num = int(input("Enter :> "))
        if (control_num == 0):
            break
        control_center(control_num)

############ app main start #####################

if __name__ == "__main__":

    # open file save 
    tenso_file = open("tenso.txt", "w")
    
    # open serial port
    thruster_serial = serial.Serial(
    port=THRUSTER_SERIAL_PORT,
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

    stop_threads = False
    # send command thread
    thread_seial_send = Thread(target = thread_dispaly_menu)
    thread_seial_send.start()

    # thread receive data
    thread_serial_receive = Thread(target = thread_receive_messages)
    thread_serial_receive.start()

    thread_seial_send.join()
    stop_threads = True
    # thread_serial_receive.terminate()

    # close serial port and file save
    tenso_file.close()

    print("app finished...exiting")
