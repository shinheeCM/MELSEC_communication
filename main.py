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
        # print("D2010 --> ", d_2010_val)
        if write_data == 1:
            write_resister_plc("D2000", write_data)
            write_data = 0
        elif write_data == 0:
            write_resister_plc("D2000", write_data)
            write_data = 1
        time.sleep(3)

def plc_communication1():
    # loop_ = True
    while True:
        d_2011_val = reading_resister_plc("D2011")
        d_2005_val = reading_resister_plc("D2005")
        d_2012_val = reading_resister_plc("D2012")
        d_2014_val = reading_resister_plc("D2014")
        d_2014_val = reading_resister_plc("D2014")
        # print("d_2014_val ==> ", d_2014_val)
        
        # if(d_2011_val and not d_2005_val and d_2012_val and not d_2014_val):
        #     loop_ = True

        if d_2014_val == 2:
            print("d_2014_val == ")
            print(d_2011_val, d_2005_val, d_2012_val, d_2014_val)
            time.sleep(3)
            write_resister_plc("D2001", 0)
            # loop_ = False
            

        if(d_2011_val==1 and d_2005_val==0 and d_2012_val==1 and d_2014_val==0):
            print("found ----> ")
            write_resister_plc("D2003", 1)
            time.sleep(3)
            write_resister_plc("D2001", 1)
            time.sleep(1)
            write_resister_plc("D2003", 0)
            # time.sleep(3)
            # write_resister_plc("D2014", 1)
            if d_2014_val == 1:
                write_resister_plc("D2006", 1)

            

            # break


def plc_communication2():
    while True:
        d_2011_val = reading_resister_plc("D2011")
        d_2005_val = reading_resister_plc("D2005")
        d_2012_val = reading_resister_plc("D2012")
        d_2014_val = reading_resister_plc("D2014")

        print(d_2011_val, d_2005_val, d_2012_val, d_2014_val)
        if(d_2011_val and not d_2005_val and d_2012_val and not d_2014_val):
            print("found ----> ")
            write_resister_plc("D2003", 1)
            time.sleep(3)
            write_resister_plc("D2001", 1)
            time.sleep(3)
            write_resister_plc("D2003", 0)
            # time.sleep(3)
            write_resister_plc("D2014", 1)
            time.sleep(3)
            write_resister_plc("D2006", 1)
            # time.sleep(3)
            write_resister_plc("D2014", 2)
            time.sleep(3)
            write_resister_plc("D2001", 0)
            time.sleep(3)



# Create a thread for the PLC communication
plc_thread = threading.Thread(target=plc_communication)
# Start the thread
plc_thread.start()

# plc_communication1()
plc_thread1 = threading.Thread(target=plc_communication1)
# Start the thread
plc_thread1.start()
# You can do other tasks in the main thread

        # print("")
    # print("D2011 --> ", d_2011_val, "D2005 --> ", d_2005_val)

    # Main thread tasks
    
