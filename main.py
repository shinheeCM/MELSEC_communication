import threading
import pymcprotocol
import time

def plc_communication():
    # If you use Q series PLC
    pymc3e = pymcprotocol.Type3E()

    # If you use ascii byte communication, (Default is "binary")
    # pymc3e.setaccessopt(commtype="ascii")
    pymc3e.connect("192.168.200.10", 3001)

    print("python3 connected!!!!!")

    write_data = 1
    while True:
        word_values, dword_values = pymc3e.randomread(word_devices=["D2010"], dword_devices=[])
        print("D2010 --> ", word_values)
        if write_data == 1:
            pymc3e.randomwrite(word_devices=["D2000"], word_values=[write_data], 
                               dword_devices=[], dword_values=[])
            write_data = 0
        elif write_data == 0:
            pymc3e.randomwrite(word_devices=["D2000"], word_values=[write_data], 
                               dword_devices=[], dword_values=[])
            write_data = 1
        time.sleep(3)

# Create a thread for the PLC communication
plc_thread = threading.Thread(target=plc_communication)

# Start the thread
plc_thread.start()

# You can do other tasks in the main thread
while True:
    # Main thread tasks
    time.sleep(1)
