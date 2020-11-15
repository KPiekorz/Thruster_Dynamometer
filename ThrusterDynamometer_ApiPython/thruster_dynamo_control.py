import multiprocessing
import serial

THRUSTER_SERIAL_PORT = 'COM8'

############## app helper functions #########



############## app serial communication #####



############## app data center ###############

def tensometer_data():
    print("\nTensometer data.")

def data_center(i):
    switcher={
            1:tensometer_data,
            }
    func=switcher.get(i, lambda :'Invalid')
    return func()

############## app control center ############

def tensometer_offset():
    print("\nTensometer offset.")

def tensometer_calib():
    print("\nTensometer calibration.")

def tensometer_set_rate():
    print("\nTensometer set rate.")

def control_center(i):
    switcher={
            1:tensometer_offset,
            2:tensometer_calib,
            3:tensometer_set_rate,
            }
    func=switcher.get(i, lambda :'Invalid')
    return func()

############### app thread ##################

def thread_receive_messages():
    while True:
        pass

def thread_dispaly_menu():
     while True:
        print("""\nCommands:
0 - Exit app;
1 - Tensometer offset;
2 - Tensometer calibration;
3 - Tensometer set rate;
""", end='')
#        control_num = int(input("Enter :> "))
 #       if (control_num == 0):
  #          break
   #     control_center(control_num)

############ app main start #####################

if __name__ == "__main__":

    # open file save 

    
    # open serial port
    thruster_serial = serial.Serial(
    port=THRUSTER_SERIAL_PORT,
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

    # receive serial thread 
    thread_serial_receive = multiprocessing.Process(target = thread_receive_messages)
    thread_serial_receive.start()

    # send command thread
    thread_seial_send = multiprocessing.Process(target = thread_dispaly_menu)
    thread_seial_send.start()

    # data plot thread


    thread_seial_send.join()
    # thread_serial_receive.terminate()

    # close serial port and file save


    print("app finished...exiting")
