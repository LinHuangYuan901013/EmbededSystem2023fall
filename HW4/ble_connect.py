# %%
from bluepy.btle import Scanner, DefaultDelegate,UUID, Peripheral
from binascii import hexlify
import struct
# %%
class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print("Received new data from", dev.addr)


class NotifyDelegate(DefaultDelegate):
    # Constructor (run once on startup)
    def __init__(self, params):
        DefaultDelegate.__init__(self)

    # func is caled on notifications
    def handleNotification(self, cHandle, data):
        print("Notification from Handle: 0x" + format(cHandle,'02X') )
        print(data)
        # print(type(data))
        # print(hexlify(data))
        # Convert each byte to an integer
        int1 = int.from_bytes(data[0:1], byteorder='big')
        int2 = int.from_bytes(data[1:2], byteorder='big')
        int3 = int.from_bytes(data[2:3], byteorder='big')

        print(int1, int2, int3)

axis=['x','y','z']
# %%
scanner = Scanner().withDelegate(ScanDelegate())
devices = scanner.scan(10.0)
n=0
addr = []
for dev in devices:
    print ("%d: Device %s (%s), RSSI=%d dB" % (n, dev.addr,dev.addrType, dev.rssi))
    addr.append(dev.addr)
    n += 1
    for (adtype, desc, value) in dev.getScanData():
        print (" %s = %s" % (desc, value))
# choose pairing device
number = input('Enter your device number: ')
print ('Device', number)
num = int(number)
print (addr[num])
# connecting to the device
print ("Connecting...")
p_device = Peripheral(addr[num], 'random')
p_device.withDelegate(NotifyDelegate(p_device))
services = p_device.getServices()

# displays all services
print("displays all services:")
for service in services:
    print(service)


# %% get specified service
service_uuid = UUID(0x180D)
uuidlist = {0x1800, 0x1801, 0x1802}
c_service = p_device.getServiceByUUID(service_uuid)
print("c_service: ", c_service)

# %%
characteristics = c_service.getCharacteristics()
# characteristics = p_device.getCharacteristics()
# displays all characteristics
print("# displays all characteristics: ")
for char in characteristics:
    print(char)
# %%
notify_char = characteristics[0]



# %%
try: 

    while True:
        counter = 0

        for ch in characteristics:
            print(axis[counter%3],"axis value = ")
            print(int.from_bytes(ch.read(),"little",signed=True))
            counter+=1
        print()
        
finally:
    p_device.disconnect()

# %%