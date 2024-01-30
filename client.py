import asyncio
from bleak import BleakClient
import binascii
import keyboard
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
import sys

MACaddress = "your_ble_controller_mac_address_here"
CHARACTERISTIC_UUID = "your_command_characteristic_uuid_here"

def get_current_volume_level():
    devices = AudioUtilities.GetSpeakers()
    interface = devices.Activate(
        IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
    volume = cast(interface, POINTER(IAudioEndpointVolume))
    return volume.GetMasterVolumeLevelScalar()

def calculate_new_volume_level(offset):
    current_volume = get_current_volume_level()
    new_volume = max(min(current_volume + offset / 100, 1.0), 0.0)
    return new_volume

def set_volume_level(offset):
    new_volume = calculate_new_volume_level(offset)
    devices = AudioUtilities.GetSpeakers()
    interface = devices.Activate(
        IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
    volume = cast(interface, POINTER(IAudioEndpointVolume))
    volume.SetMasterVolumeLevelScalar(new_volume, None)

async def main():
    # Connect to the first discovered device
    async with BleakClient(MACaddress) as client:
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

        while (True):
            await asyncio.sleep(1)

async def notification_handler(sender, data):

    try:
        value = binascii.hexlify(data).decode("utf-8")
        value = int(value, base=16)
        
        print(f"Received notification from {sender}: {value}")

        match(value):
            case 0:
                print("Audio up +1")
                set_volume_level(+1)
            case 1:
                print("Audio down -1")
                set_volume_level(-1)
            case 2:
                print("Audio up +5")
                set_volume_level(+5)
            case 3:
                print("Audio down -5")
                set_volume_level(-5)
            case 4:
                print("Play")
                keyboard.send("space")
            case 5:
                print("Pause")
                keyboard.send("space")
            case 6:
                print("Skip")
            case 7:
                print("Return")
            case 8:
                print("Exit / Enter fullscreen")
                keyboard.send("f11")
            case 9:
                print("Delete current tab")
                keyboard.send("ctrl+w")
            case 10:
                print("Open new tab")
                keyboard.send("ctrl+t")
            case 11:
                print("Reserved")
            case 12:
                print("Reserved")
            case 13:
                print("Reserved")
            case 14:
                print("Reserved")
            # exit button
            case 15:
                print("Exiting...")
                sys.exit()
    
    except Exception as e:
        print("Error:", e)

            
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program terminated by user.")
    except SystemExit:
        print("Program exited.")
