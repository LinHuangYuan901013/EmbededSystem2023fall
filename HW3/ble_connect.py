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
        print(hexlify(data))


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
service_uuid = UUID(0xfff0)
c_service = p_device.getServiceByUUID(service_uuid)
print("c_service: ", c_service)

# %%
characteristics = c_service.getCharacteristics()
# displays all characteristics
print("# displays all characteristics: ")
for char in characteristics:
    print(char)
# %%
notify_char = characteristics[0]

#%%
hEcg=notify_char.getHandle()
for descriptor in p_device.getDescriptors(hEcg,c_service.hndEnd):
    if (descriptor.uuid==0x2902):
        print(f'Client Characteristic Configuration found at handle 0x{format(descriptor.handle,"02X")}')
        hEcgCCC=descriptor.handle

p_device.writeCharacteristic(hEcgCCC,bytes([1, 0]))


# %%
try: 
    while True:
        if p_device.waitForNotifications(1.0):
            # handleNotification() was called
            continue

        print("Waiting... Waited more than one sec for notification")

# %%
finally:
    p_device.disconnect()

# %%