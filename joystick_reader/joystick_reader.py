import evdev


from evdev import ecodes


# required codes for device to be considered a joystick
joystick_EV_ABS_codes = ['ABS_Y', 'ABS_X', 'ABS_RY', 'ABS_RX', 'ABS_HAT0Y', 'ABS_HAT0X']
joystick_EV_KEY_codes = ['BTN_SOUTH', 'BTN_EAST', 'BTN_WEST', 'BTN_NORTH', 'BTN_TL', 'BTN_TR',
                         'BTN_TL2', 'BTN_TR2', 'BTN_START']
required_codes = [ ecodes.ecodes[key] for key in (joystick_EV_KEY_codes + joystick_EV_ABS_codes) ]

# check if device contains the required codes
joystick_device = None
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
    device_capabilities = device.capabilities(absinfo=False)
    if (all(key in device_capabilities.keys() for key in [ecodes.EV_KEY, ecodes.EV_ABS])
        and all((code in (device_capabilities[ecodes.EV_KEY] + device_capabilities[ecodes.EV_ABS]) for code in required_codes))):
        joystick_device = device
        break
if not joystick_device:
    print("No joystick device connected")
    exit(1)


# start grabbing data from the joystick
for event in device.read_loop():
    print(evdev.categorize(event))
