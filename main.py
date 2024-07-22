import threading
import pymcprotocol
import time

# If you use Q series PLC
pymc3e = pymcprotocol.Type3E()
# If you use ascii byte communication, (Default is "binary")
# pymc3e.setaccessopt(commtype="ascii")
pymc3e.connect("192.168.200.10", 3001)
print("python3 connected!!!!!")

def reading_resister_plc(resister_name):
    d_val, dword_values = pymc3e.randomread(word_devices=[resister_name], dword_devices=[])
    return d_val[0]

def write_resister_plc(resister_name, write_data):
    pymc3e.randomwrite(word_devices=[resister_name], word_values=[write_data], dword_devices=[], dword_values=[])


def plc_communication():
    write_data = 1
    while True:
        # d_2010_val, dword_values = pymc3e.randomread(word_devices=["D2010"], dword_devices=[])
        d_2010_val = reading_resister_plc("D2010")
        print("D2010 --> ", d_2010_val)
        if write_data == 1:
            write_resister_plc("D2000", write_data)
            write_data = 0
        elif write_data == 0:
            write_resister_plc("D2000", write_data)
            write_data = 1
        time.sleep(3)
        


# Create a thread for the PLC communication
plc_thread = threading.Thread(target=plc_communication)

# Start the thread
plc_thread.start()

# You can do other tasks in the main thread
while True:
    d_2011_val = reading_resister_plc("D2011")
    d_2005_val = reading_resister_plc("D2005")
    print("D2011 --> ", d_2011_val, "D2005 --> ", d_2005_val)

    # Main thread tasks
    time.sleep(1)
