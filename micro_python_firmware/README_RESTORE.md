```bash
md5sum -c flash_dump_4mb.bin.md5
esptool --port /dev/ttyUSB0 --baud 921600 erase-flash
esptool --port /dev/ttyUSB0 --baud 921600 write-flash 0x0 flash_dump_4mb.bin
esptool --port /dev/ttyUSB0 --baud 921600 verify-flash 0 flash_dump_4mb.bin
```