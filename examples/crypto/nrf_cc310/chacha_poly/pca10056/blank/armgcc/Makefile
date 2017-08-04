PROJECT_NAME     := chacha_poly_pca10056
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../../../../..
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := chacha_poly_gcc_nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(PROJ_DIR)/integration_test_cc_chacha_poly_vectors.c \
  $(PROJ_DIR)/main.c \
  $(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/external/nrf_cc310/common/integration_test_plat_utils.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/toolchain/system_nrf52840.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/external/segger_rtt \
  ../config \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(PROJ_DIR) \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/external/nrf_cc310/include \
  $(SDK_ROOT)/external/nrf_cc310/common \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/toolchain/gcc \

# Libraries common to all targets
LIB_FILES += \
  $(SDK_ROOT)/external/nrf_cc310/lib/libcc310_gcc_0.9.0.a \

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCHACHA_INTEGTEST
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DDEBUG
CFLAGS += -DDEBUG_NRF
CFLAGS += -DDX_CC_TEE
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DNRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE=2048
CFLAGS += -DNRF_LOG_ENABLED=1
CFLAGS += -DNRF_LOG_USES_RTT=1
CFLAGS += -DNRF_SDK_PRESENT
CFLAGS += -DSEGGER_RTT_BLOCKING
CFLAGS += -D__STACK_SIZE=2048
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums 

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCHACHA_INTEGTEST
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DDEBUG
ASMFLAGS += -DDEBUG_NRF
ASMFLAGS += -DDX_CC_TEE
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE=2048
ASMFLAGS += -DNRF_LOG_ENABLED=1
ASMFLAGS += -DNRF_LOG_USES_RTT=1
ASMFLAGS += -DNRF_SDK_PRESENT
ASMFLAGS += -DSEGGER_RTT_BLOCKING
ASMFLAGS += -D__STACK_SIZE=2048

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs


# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52840_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf52840_xxaa

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash erase

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex
	@echo Flashing: $<
	nrfjprog -f nrf52 --program $< --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall
