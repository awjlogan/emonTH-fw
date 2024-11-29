##############################################################################
BUILD = build
BIN = emonTH
OUT = bin
##############################################################################
.PHONY: all directory clean size

# Path to toolchain, e.g. /path/to/bin/ Leave empty if already on path.
TC_PATH =
CC = $(TC_PATH)arm-none-eabi-gcc
OBJCOPY = $(TC_PATH)arm-none-eabi-objcopy
SIZE = $(TC_PATH)arm-none-eabi-size

ifeq ($(OS), Windows_NT)
  MKDIR = gmkdir
else
  MKDIR = mkdir
endif

CFLAGS += -W -Wall -Wextra -Wpedantic --std=c17 -Os -g3
CFLAGS += -fno-diagnostics-show-caret -fno-common
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -Wuninitialized
CFLAGS += -Wshadow -Wdouble-promotion -Wundef
CFLAGS += -mcpu=cortex-m23 -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m23 -mthumb -nostartfiles
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -Wl,--script=./linker/saml10e14.ld

INCLUDES += \
  -I./include/saml10 \
  -I./third_party/RFM69 \
  -I./third_party/qfplib \
  -I./src/

SRCS += $(wildcard ./src/*.c)

DEFINES += \
  -D__SAML10E14A__ \
  -DDONT_USE_CMSIS_INIT \
  -D__ARM_FEATURE_DSP=0

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))
OBJS += $(BUILD)/qfplib.o

# Always update the build information. This forces this to run every time. Exit
# if this fails - likely to be a path of Python version issue.
BUILD_INFO := $(shell python3 ./scripts/build_info.py ./src/emonTH_build_info.c)
ifeq ($(strip $(BUILD_INFO)), )
$(error 1)
endif

VERSION_INFO := $(shell python3 ./scripts/version_info.py)

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@
	@cp $@ $(OUT)/$(VERSION_INFO).elf

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@
	@cp $@ $(OUT)/$(VERSION_INFO).hex

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@
	@cp $@ $(OUT)/$(VERSION_INFO).bin

$(BUILD)/qfplib.o:
	@echo AS $@
	@$(CC) $(CFLAGS) third_party/qfplib/qfplib.s -c -o $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@$(MKDIR) -p $(BUILD)
	@$(MKDIR) -p $(OUT)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)
	@-rm -f $(OUT)/emonTH*

-include $(wildcard $(BUILD)/*.d)
