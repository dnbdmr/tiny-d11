__DGW-Tiny__

Combination of Alex Taradov's DGW usb-hid gateway and USBTiny usb stack for a simple usb-cdc device on a SAMD11/21. Intended as a template for other projects. 

https://github.com/ataradov/dgw

https://github.com/hathach/tinyusb


__TODO__
+ move gpio go separate file
+ set up systick/millis/delay
+ on resume from linux host, cdc receive ATPORTCFG#, or #ATPORTCFG, etc... removed modemmanager
+ clean up inclue/ and startup, move to atmel style
+ the switch from dgw linker and startup scripts to asf increased .bss by about 9k( pre-allocated stack)
+ move tud functions to separate file?
