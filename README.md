__DGW-Tiny__

Combination of Alex Taradov's DGW usb-hid gateway and USBTiny usb stack for a simple usb-cdc device on a SAMD21. Intended as a template for other projects. 

https://github.com/ataradov/dgw

https://github.com/hathach/tinyusb


__TODO__
+ set up systick/millis/delay
+ on resume from linux host, cdc receive ATPORTCFG#, or #ATPORTCFG, etc... removed modemmanager
+ clean up inclue/ and startup, move to atmel style
+ the switch from dgw linker and startup scripts to asf increased .bss by about 9k( pre-allocated stack)
- make master branch empty
- move tud functions to separate file?
- uf2 handover with msc device?
- add msc class with uf2 handover?

__dx1 bootloader problems__
1. Start with regular load.
link at 0x0
load
dump flash
2. Same with bootloader
link at 0x400
convert elf to dfu
load bootloader
dfu upload .dfu
dump flash
3. add offset to noboot dump
4. compare dumps
