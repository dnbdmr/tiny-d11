##############################################################################
BUILD = build
BIN = dgw
PORT = /dev/serial/by-id/usb-DNBDMR_TinyUSB_D11D_CEF3EF1D514D4B53334A2020FF071B46-if00

##############################################################################

CC = /home/daniel/src/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-gcc
OBJCOPY = /home/daniel/src/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-objcopy
OBJDUMP = /home/daniel/src/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-objdump
SIZE = /home/daniel/src/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-size

CFLAGS += -Wextra -Wall --std=gnu11 -DDEBUG -ggdb3 -Og
#CFLAGS += -Wextra -Wall --std=gnu11 -Os -flto
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m0plus -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d
CFLAGS += --specs=nano.specs

LDFLAGS += -mcpu=cortex-m0plus -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--start-group -lm -Wl,--end-group
LDFLAGS += -Wl,-Map=$(BUILD)/$(BIN).map
LDFLAGS += -Wl,--script=../linker/samd11c14.ld
#LDFLAGS += -Wl,-flto

#LIBS = --specs=nano.specs -u _printf_float -u _scanf_float

INCLUDES += \
  -I../tinyusb/src \
  -I../tinyusb/hw/mcu/microchip/samd11/include/ \
  -I../tinyusb/hw/mcu/microchip/samd11/CMSIS/Core/Include/ \
  -I..

SRCS += \
  ../main.c \
  ../startup_samd11.c \
  ../tinyusb/src/tusb.c \
  ../tinyusb/src/common/tusb_fifo.c \
  ../tinyusb/src/device/usbd.c \
  ../tinyusb/src/device/usbd_control.c \
  ../tinyusb/src/portable/microchip/samd/dcd_samd.c \
  ../tinyusb/src/class/cdc/cdc_device.c \
  ../usb_descriptors.c \
  ../i2c_master.c \
  ../htu21.c \
  ../debug.c \
  ../dma.c \
  ../adc.c \
  ../rtc.c \
  ../pwm.c \
  ../utils.c \
  ../usb_utils.c \
  ../gpio.c

DEFINES += \
  -D__SAMD11D14AM__ \
  -DDONT_USE_CMSIS_INIT \
  -DF_CPU=48000000 \
  -DSTARTUP_FROM_RESET

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))

.PHONY: all
all: $(BUILD)/$(BIN).elf size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

$(BUILD)/$(BIN).dfu: $(BUILD)/$(BIN).elf
	@echo DFU $@
	./dx1elf2dfu  $^ $@

.PHONY: fdfu
fdfu: $(BUILD)/$(BIN).dfu
	@echo Flash DFU $^
	-./force_reset.py --period 0.5 $(PORT)
	dfu-util -D $^

.PHONY: disasm
disasm: $(BUILD)/$(BIN).lss;

$(BUILD)/$(BIN).lss: $(BUILD)/$(BIN).elf
	@echo DISASSEMBLE $^
	$(OBJDUMP) -d -S $^ > $@

$(OBJS): | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

$(BUILD):
	@mkdir -p $(BUILD)

.PHONY: size
size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

.PHONY: clean
clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)
