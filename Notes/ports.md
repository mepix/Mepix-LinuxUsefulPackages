# Ports

## Serial Ports

### Symlink Serial Ports

Ports are assigned sequentially as devices are discovered. As such, it can be difficult to connect to a specific USB Serial device. The following approach will create a symlink between a static user-defined port and a variable `/dev` port using the [`udev` rules](https://wiki.debian.org/udev).

1. List USB device attributes:
```sh
udevadm info --name=/dev/ttyACMx --attribute-walk
```

2. Create (or open) a file `/etc/udev/rules.d/99-usb-serial.rules`
3. Add a line that includes the unique attributes such as `idVendor` or `idProduct`:
```sh
KERNEL=="ttyACM*", ATTRS{idVendor}=="6472", SYMLINK+="sensor_0"
```
4. Reload udevadm rules:
```sh
udevadm control --reload-rules
```

5. Verify the creation of the port
```sh
ls /dev/sens*
```

More information and related commands can be found [here](https://msadowski.github.io/linux-static-port/) and [here](https://unix.stackexchange.com/questions/66901/how-to-bind-usb-device-under-a-static-name).
