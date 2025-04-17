from bluepy.btle import Peripheral, UUID
from bluepy.btle import Scanner, DefaultDelegate
import threading

complete_local_name = "ShuCheniPhone"
target_mac = "e4:94:56:76:5a:d6"

ACC_UUID = ("00e00000-0001-11e1-ac36-0002a5d5c51b")
SERVICE_UUID = UUID("00000000-0001-11e1-ac36-0002a5d5c51b")

LOCK = threading.Lock()

def ahhhh(WC):
    while(True):
        input()
        with LOCK:
            FREQ = int(input("Assign new frequency"))
            WC.write(FREQ.to_bytes(2, "big"), withResponse=True)


class NotifyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        print("Notification received:")
        print("Handle:", cHandle)
        x = int.from_bytes(data[0:2], byteorder='little', signed=True)
        y = int.from_bytes(data[2:4], byteorder='little', signed=True)
        z = int.from_bytes(data[4:6], byteorder='little', signed=True)
        print("x:",x,"y:",y,"z:",z)



class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

#
print ("Connecting...")
dev = Peripheral(target_mac, 'random')
print("connected!")
WC = dev.getCharacteristics(uuid = SERVICE_UUID)[0]
acc_char = dev.getCharacteristics(uuid=ACC_UUID)[0]
cccd = acc_char.getDescriptors(UUID(0x2902))[0]
cccd.write(b'\x01\x00', withResponse=True)

dev.setDelegate(NotifyDelegate())

t=threading.Thread(target = ahhhh, args=(WC,))
t.start()

while True:
    if dev.waitForNotifications(1.0):
        with LOCK:
            # handleNotification() was called
            print("Notification count:")
            continue
