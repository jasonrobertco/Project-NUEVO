# Raspberry Pi Setup

## Enable UART on a Pi 5
Edit config.txt
```bash
sudo nano /boot/firmware/config.txt
```

Find the uart enable option and edit:
```bash
enable_uart=1
dtoverlay=uart0-pi5
```

Also disable serial console:
```bash
sudo nano /boot/firmware/cmdline.txt
```

Remove anything like:
```bash
console=serial0,115200
console=ttyAMA0,115200
```
Save the file and reboot.

## Verify
```
ls -l /dev/ttyAMA0
```

You should see something like:
```bash
crw-rw---- 1 root dialout 204, 64 Mar 10 14:02 /dev/ttyAMA0
```

The current bridge runtime uses:

```text
/dev/ttyAMA0 @ 200000 baud
```
