import evdev
from evdev import InputDevice, categorize, ecodes, list_devices

# List all input devices and find the Xbox controller
devices = [InputDevice(path) for path in list_devices()]
controller = None

for device in devices:
    if 'Xbox' in device.name or 'xbox' in device.name:
        controller = device
        break

if not controller:
    print("No Xbox controller found.")
    exit()

print(f"Using controller: {controller.name} at {controller.path}")

# Read and print all events
for event in controller.read_loop():
    if event.type in [ecodes.EV_KEY, ecodes.EV_ABS]:
        print(categorize(event))
        print(event.value)
