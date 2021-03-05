tar ext /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe__SWLINK____Firmware_v1.7.1__B6C7C8FC-if00
mon swdp_scan
att 1

# To work with flashing from SAMDx1 dfu bootloader
mon vector_catch disable reset
