import threading
import socket
import queue
from file_operations import read_frame_from_file


# HOST = '127.0.0.1'
# PORT = 1234
HOST = '169.254.0.55'
PORT = 80

FRAME_SIZE = 142

queue_receiveframe = queue.Queue()
queue_fileoperation = queue.Queue()

###################################################################################################################################


class Switcher(object):
    def numbers_to_methods_to_strings(self, argument, parametr):
        method_name = 'number_' + argument
        method = getattr(self, method_name, lambda: "nothing")
        return method()

    def number_0(parametr):     # zatrzymuje prace hamowni
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.send(b'[0|0|0|0|0]')
            s.close()

    def number_1(parametr):     # rozpoczna prace hamowni
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.send(b'[1|0|0|0|0]')
            # rcv_frame = s.recv(len(b'[1|0|0|0|0]'))
            # rcv_frame = rcv_frame.decode("utf-8")
            # print(f"Odebrano: {rcv_frame}")
            s.close()

    def number_2(parametr):     # zmiania predkosci silnia
        wartos_pwm = input("Podaj wartos pwm (500-1000): ")
        command = f"[2|{(wartos_pwm)}|0|0|0]"
        print(command)
        command = command.encode('utf-8')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.send(command)
            s.close()

    def number_3(parametr):     # zapis danych z czujikow - uruchomienie tasku ReceiveFrame
        ile_ramek = input("Podaj iloc ramke do odebrania: ")
        queue_receiveframe.put(ile_ramek)


    def number_4(parametr):     # podzielenie danych na czytelne do matlaba - uruchomienie tasku ThreadFileOperations
        queue_fileoperation.put(1)

    def number_5(parametr):     # zakonczenie pracy aplikacji - skasownaie wszystkich taskow
        queue_receiveframe.put(0)
        queue_fileoperation.put(0)

    def number_6(parametr):     # wybranie trybu pracy hamowni
        def switch_tryby_pracy(argument):
            switcher = {
                1: '5',     # long test
                2: '6',     # short test
                3: '7'      # throttle test
            }
            return switcher.get(argument, 'nie ma takiego trybu')

        tryb = int(input("Podaj tryb pracy (1 - long test (1 min stalej pracy), 2 - short test (wszystko do ustawienia) lub 3 - throttle test (maks i dol)): "))
        rise_time = int(input("Podaj rise time (sec): "))
        stay_time = int(input("Podaj stay time (tylko short) (sec): "))
        fall_time = int(input("Podaj fall time (sec): "))
        command = f"[{switch_tryby_pracy(tryb)}|0|{rise_time}|{stay_time}|{fall_time}]"
        # print(command)
        command = command.encode('utf-8')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.send(command)
            s.close()

    def number_7(parametr):     # tenso offset
        command = f"[3|0|0|0|0]"
        command = command.encode('utf-8')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.send(command)
            s.close()

    def number_8(parametr):     # tenso kalibrcja
        waga_wzorcowa = input("Podaj wage wzorcowa (gram): ")
        command = f"[4|{(waga_wzorcowa)}|0|0|0]"
        print(command)
        command = command.encode('utf-8')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.send(command)
            s.close()


###################################################################################################################################


class ThreadCmd(threading.Thread):
    def __init__(self, id, name, i):
        threading.Thread.__init__(self)
        self.id = id
        self.name = name
        self.i = i

    def run(self):
        print("Alikacja obslugi hamowni")
        switch = Switcher()
        while True:
            print("""
Komendy:\n
0 - Stop (zatrzymuje wszystko);
1 - Start (rozpoczyna pomiary z czujnikow, uruchamia pwm do silnika);
2 - Zmiana predkosci silnika;
3 - Zapis pomiarow z czujnikow;
4 - Przerobienie na dane do matlab;
5 - Zakonczenie pracy aplikacji;
6 - Uruchomienie trybu pracy hamowni;
7 - Tensometr offset (przed startem);
8 - Tensometr kalibracja (przed startem, po offset);
                  """)
            wybrana_komenda = input("Wybierz komende: ")
            switch.numbers_to_methods_to_strings(wybrana_komenda, 0)
            print("\n")
            if wybrana_komenda == '5':
                break


class ThreadReceiveFrame(threading.Thread):
    def __init__(self, id, name, i):
        threading.Thread.__init__(self)
        self.id = id
        self.name = name
        self.i = i

    def run(self):
        while True:
            msg = queue_receiveframe.get()
            if int(msg) == 0:
                break
            else:
                print(msg)
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((HOST, PORT))
                    with open('Data_dumps/frame.txt', 'w+') as f:
                        for i in range(int(msg)):
                            rcv_frame = s.recv(FRAME_SIZE)  # odbior stalej dlugosci ramki
                            rcv_frame = rcv_frame.decode("utf-8")
                            rcv_frame += '\n'
                            f.write(rcv_frame)
                        f.close()
                    s.close()
                print("ThreadReceiveFrame - zapis zakonczony!!!")


class ThreadFileOperations(threading.Thread):
    def __init__(self, id, name, i):
        threading.Thread.__init__(self)
        self.id = id
        self.name = name
        self.i = i

    def run(self):
        while True:
            message = queue_fileoperation.get()
            if message == 0:
                break
            else:
                read_frame_from_file()


class ThreadPlot(threading.Thread):
    def __init__(self, id, name, i):
        threading.Thread.__init__(self)
        self.id = id
        self.name = name
        self.i = i

    def run(self):
        pass


###################################################################################################################################


if __name__ == "__main__":
    threadcmd = ThreadCmd(1, "First Thread", 1)
    threadreceiveframe = ThreadReceiveFrame(2, "Second Thread", 2)
    threadfileoperations = ThreadFileOperations(3, "Third Thread", 3)

    threadcmd.start()
    threadreceiveframe.start()
    threadfileoperations.start()

    threadcmd.join()
    threadreceiveframe.join()
    threadfileoperations.join()

    print("Byeeeeee!!!")