# https://pypi.org/project/pymcprotocol/
import pymcprotocol
import time
#If you use Q series PLC
pymc3e = pymcprotocol.Type3E()

#If you use ascii byte communication, (Default is "binary")
# pymc3e.setaccessopt(commtype="ascii")
pymc3e.connect("192.168.200.10", 3001)


print("python3 connected!!!!!")

# cpu_type, cpu_code = pymc3e.read_cputype()
# print(cpu_type)
# wordunits_values = pymc3e.batchread_wordunits(headdevice="D2010", readsize=10)
# word_values, dword_values = pymc3e.randomread(word_devices=["D2010"])

write_data = 1
while 1:
    word_values, dword_values = pymc3e.randomread(word_devices=["D2010"], dword_devices=[])
    print("D2010 --> ",word_values)
    if write_data == 1:
        pymc3e.randomwrite(word_devices=["D2000"], word_values=[write_data], 
                    dword_devices=[], dword_values=[])
        write_data = 0
    elif write_data ==0:
        pymc3e.randomwrite(word_devices=["D2000"], word_values=[write_data], 
                    dword_devices=[], dword_values=[])
        write_data = 1
    time.sleep(3)

