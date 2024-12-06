# tu_ros2_mmi

## Prerequisites

### WSL2 Setup (Optional)

If you want to open `.ino` files with the Arduino IDE in Windows while working in WSL2:

- Mount WSL2 as a network drive in Windows
- See: [Arduino IDE Issue #1797](https://github.com/arduino/arduino-ide/issues/1797)
- Solution detailed [here](https://superuser.com/questions/1738361/how-to-mount-a-wsl2-folder-as-a-network-drive-in-windows-10)

### USB Passthrough Setup

1. Install [usbipd-win](https://github.com/dorssel/usbipd-win):

```powershell
winget install usbipd
```

2. Connect USB device:

```powershell
# List available devices
usbipd list

# Bind device
usbipd bind --busid=XXX

# Attach to WSL2
usbipd attach --wsl --busid=XXX
```

3. Verify connection in WSL2:

```bash
lsusb
```

### Arduino Development Workflow

When modifying Arduino code:

1. Disconnect device from WSL2
2. Flash sketch using Arduino IDE in Windows
3. Reattach to WSL2: `usbipd attach --wsl --busid=XXX`
4. Device will be available at `/dev/ttyACM0` (or similar)

### Keyboard Input

This project uses [sshkeyboard](https://sshkeyboard.readthedocs.io/en/latest/) for keyboard input in WSL2:

- Works in headless environments
- No X server dependency
- No root access required
- No external dependencies
