##############################################################################
BUILD = build
BIN = dgw

##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CFLAGS += -Wextra -Wall --std=gnu11 -DDEBUG -ggdb3 -Og
#CFLAGS += -Wextra -Wall --std=gnu11 -Os
#CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m0plus -mthumb
#CFLAGS += --specs=nano.specs -u _printf_float -u _scanf_float
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0plus -mthumb
LDFLAGS += -Wl,--gc-sections
#LDFLAGS += --specs=nano.specs -u _printf_float -u _scanf_float
#LDFLAGS += --specs=nosys.specs
#LDFLAGS += --specs=rdimon.specs
LDFLAGS += -Wl,--start-group -lm -Wl,--end-group
LDFLAGS += -Wl,-Map=$(BUILD)/$(BIN).map
LDFLAGS += -Wl,--script=../linker/samd11c14.ld

INCLUDES += \
  -I../include \
  -I../tinyusb/src \
  -I..

SRCS += \
  ../main.c \
  ../startup_samd11.c \
  ../tinyusb/src/tusb.c \
  ../tinyusb/src/common/tusb_fifo.c \
  ../tinyusb/src/device/usbd.c \
  ../tinyusb/src/device/usbd_control.c \
  ../tinyusb/src/portable/microchip/samd21/dcd_samd21.c \
  ../tinyusb/src/class/cdc/cdc_device.c \
  ../usb_descriptors.c \
  ../adc.c

DEFINES += \
  -D__SAMD11C14A__ \
  -DDONT_USE_CMSIS_INIT \
  -DF_CPU=48000000 

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

program: $(BUILD)/$(BIN).bin
	@echo PROGRAM $^
	./force_reset.py --caterina /dev/ttyACM2
	./boss.sh $^

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@mkdir -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)

