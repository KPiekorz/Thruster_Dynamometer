import re


def time_to_sec(min, sec, milis):
    milis_to_sec = milis/1000
    min_to_sec = min*60
    return min_to_sec+sec+milis_to_sec


def erase_files():
    open("Data_dumps/temp.txt", "w+").close()
    open("Data_dumps/pressure.txt", "w+").close()
    open("Data_dumps/vibration.txt", "w+").close()
    open("Data_dumps/shunt.txt", "w+").close()
    open("Data_dumps/hal.txt", "w+").close()
    open("Data_dumps/tenso.txt", "w+").close()
    open("Data_dumps/repair_frame.txt", "w+").close()



def write_to_temp_file(dict_frame):
    temp_file = open("Data_dumps/temp.txt", "a+")
    temp = f"{str(dict_frame['Time'])},  {str(dict_frame['Temperature'][0])},  {str(dict_frame['Temperature'][1])}\n"
    temp_file.write(temp)
    temp_file.close()


def write_to_pres_shunt_tenso_file(dict_frame):
    press_file = open("Data_dumps/pressure.txt", "a+")
    shunt_file = open("Data_dumps/shunt.txt", "a+")
    tenso_file = open("Data_dumps/tenso.txt", "a+")
    press = f"{str(dict_frame['Time'])},  {str(dict_frame['Pressure'])}\n"
    shunt = f"{str(dict_frame['Time'])},  {str(dict_frame['Shunt'])}\n"
    tenso = f"{str(dict_frame['Time'])},  {str(dict_frame['Tensometer'])}\n"
    press_file.write(press)
    shunt_file.write(shunt)
    tenso_file.write(tenso)
    press_file.close()
    shunt_file.close()
    tenso_file.close()


def write_to_vibro_hal_file(dict_frame):
    time = dict_frame['Time']
    vibro_file = open("Data_dumps/vibration.txt", "a+")
    hal_file = open("Data_dumps/hal.txt", "a+")

    for i in range(10):
        vibro = f"{str(time)},  {str(dict_frame['Vibration'][i])}\n"
        hal = f"{str(time)},  {str(dict_frame['Hal'][i])}\n"
        vibro_file.write(vibro)
        hal_file.write(hal)
        time += 0.0001
        time = round(time, 4)

    vibro_file.close()
    hal_file.close()


def make_dict_from_frame(frame):
    field_values = re.findall(r"[\.\w']+", frame)
    # zamieniamy liczby na int
    field_values = list(map(float, field_values))
    time = time_to_sec(min=field_values[0], sec=field_values[1], milis=field_values[2])
    temp = field_values[3:5]
    press = field_values[5]
    vibro_table = field_values[6:16]
    shunt = field_values[16]
    hal_table = field_values[17:27]
    tenso = field_values[27]

    field_values = []
    field_values.append(time)
    field_values.append(temp)
    field_values.append(press)
    field_values.append(vibro_table)
    field_values.append(shunt)
    field_values.append(hal_table)
    field_values.append(tenso)

    field_names = ['Time', 'Temperature', 'Pressure', 'Vibration', 'Shunt', 'Hal', 'Tensometer']
    return dict(zip(field_names, field_values))


def rapairs_frame(frames, index):
    new_frame = frames[index].rstrip()
    # return f"{new_frame}{frames[index+1]}-"
    return f"{new_frame}{frames[index+1]}"


def read_frame_from_file():
    '''tworz pliki z podzialem na wyniki z poszczegolnych czujnikow'''

    frame_files = open("Data_dumps/frame.txt", "r")
    frames = frame_files.readlines()
    frame_files.close()

    erase_files()

    repair_frame = open("Data_dumps/repair_frame.txt", "a+")

    for num, frame in enumerate(frames):
        if len(frame) == 143 and frame[0] == '[':
            repair_frame.write(frame)
        else:
            if num < (len(frames)-1):
                if frame[0] == '[' and len(frames[num+1]) == (144 - len(frame)):
                    reparied_frame = rapairs_frame(frames, num)
                    repair_frame.write(reparied_frame)

    repair_frame.close()

    frame_files = open("Data_dumps/repair_frame.txt", "r")
    frames = frame_files.readlines()
    frame_files.close()

    for num, frame in enumerate(frames):
        dict_frame = make_dict_from_frame(frame)
        write_to_temp_file(dict_frame)
        write_to_pres_shunt_tenso_file(dict_frame)
        write_to_vibro_hal_file(dict_frame)
