from threading import Thread
import serial
from matplotlib import pyplot as plt

THRUSTER_SERIAL_PORT = 'COM8'

# thruster module id
TENSOMETER_MODULE_ID = (0x01)
tenso_data_size = 0
tenso_data = []

############## app helper functions #########



############## app serial communication #####



############## app data center ###############

def tensometer_data(message):
    print("\nTensometer data.")
    if (len(tenso_data) < tenso_data_size):
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
    offset = int(input("Enter offset :> "))
    tenso_message = [TENSOMETER_MODULE_ID, 2, offset&0xFF, offset>>8]
    print(tenso_message)

def tensometer_calib():
    print("\nTensometer calibration.")

def tensometer_set_rate():
    print("\nTensometer set rate.")

def tensometer_plot():
    print("\nTensometer set rate.")
    # open plot dispaly
    x = list(range(0, tenso_data_size))
    y = tenso_data
    plt.plot(x, y)
    plt.savefig("tenso_plot.png")

def tensometer_get_measurement():
    print("\nTensometer get measurement.")
    tenso_data_size = int(input("Enter :> "))
    tenso_data.clear()

    # receive serial thread 
    stop_threads = False
    thread_serial_receive = Thread(target = thread_receive_messages)
    thread_serial_receive.start()

def control_center(i):
    switcher={
            1:tensometer_offset,
            2:tensometer_calib,
            3:tensometer_set_rate,
            4:tensometer_plot,
            5:tensometer_get_measurement,
            }
    func=switcher.get(i, lambda :'Invalid')
    return func()

############### app thread ##################

def thread_receive_messages():

    while True:
        if (stop_threads == True):
            break
        serial_message = thruster_serial.read()
        if (len(serial_message) != 0):
            data_center(int(serial_message[0]), serial_message)

def thread_dispaly_menu():
     while True:
        print("""\nCommands:
0 - Exit app;
1 - Tensometer offset;
2 - Tensometer calibration;
3 - Tensometer set rate;
4 - Plot tensometer data;
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

    # send command thread
    thread_seial_send = Thread(target = thread_dispaly_menu)
    thread_seial_send.start()

    # data plot thread


    thread_seial_send.join()
    stop_threads = True
    # thread_serial_receive.terminate()

    # close serial port and file save
    tenso_file.close()

    print("app finished...exiting")
