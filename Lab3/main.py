from bluepy.btle import Peripheral, UUID
from bluepy.btle import Scanner, DefaultDelegate


complete_local_name = "ShuCheniPhone"

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print ("Discovered device", dev.addr)
        elif isNewData:
            print ("Received new data from", dev.addr)

scanner = Scanner().withDelegate(ScanDelegate())
devices = scanner.scan(10.0)
n=0
target_n = 0
addr = []

for dev in devices:
    print ("%d: Device %s (%s), RSSI=%d dB" % (n, dev.addr, dev.addrType, dev.rssi))
    addr.append(dev.addr)
    
    
    for (adtype, desc, value) in dev.getScanData():
        print (" %s = %s" % (desc, value))
        if(value == complete_local_name):
            print("found", complete_local_name)
            target_n = n
    n += 1
# number = input('Enter your device number: ')
# print ('Device', number)
# num = int(number)
num = target_n
print('Device', num)
print (addr[num])
#
print ("Connecting...")
dev = Peripheral(addr[num], 'random')
#
# print ("Services...")
# for svc in dev.services:
#     print (str(svc))

testService = dev.getServiceByUUID(UUID(0x1111))
ch = testService.getCharacteristics(UUID(0x2222))[0]
for descriptor in ch.getDescriptors():
    print(descriptor, descriptor.uuid)
    if(descriptor.uuid == 0x2902):
        CCCD_handle = descriptor.handle
        print("Before writing to CCCD:", end="")
        print(dev.readCharacteristic(CCCD_handle))
        dev.writeCharacteristic(CCCD_handle, bytes([0x02, 0x00]), withResponse=True)

        print("After writing to CCCD:", end="")
        print(dev.readCharacteristic(CCCD_handle))
        if (dev.readCharacteristic(CCCD_handle) == bytes([0, 0])):
            print("Notification is turned off")
        else:
            NotificationFlag = 1
            print("Notification is turned on")

        if (NotificationFlag == 1):
            print("Waiting for notifications")
            NotificationCount = 0
            while True:
                if dev.waitForNotifications(1.0):
                    # handleNotification() was called
                    NotificationCount += 1
                    print("Notification count:", NotificationCount)
                    continue
    # 00002902-0000-1000-8000-00805f9b34fb
#     # if(desriptor.uuid == )
#
# try:
#     testService = dev.getServiceByUUID(UUID(0x1111))
#     for ch in testService.getCharacteristics():
#         print (str(ch))
# #
#     # ch = dev.getCharacteristics(uuid=UUID(0x2222))[0]
#     # if (ch.supportsRead()):
#     #     print(ch.read())

# finally:
#     dev.disconnect()
