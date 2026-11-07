```bash
test "$(md5 -q flash_dump_4mb.bin)" = "$(awk '{print $1}' flash_dump_4mb.bin.md5)"
esptool --port /dev/cu.usbserial-2140 --baud 921600 erase-flash
esptool --port /dev/cu.usbserial-2140 --baud 921600 write-flash 0x0 flash_dump_4mb.bin
esptool --port /dev/cu.usbserial-2140 --baud 921600 verify-flash 0 flash_dump_4mb.bin
```