PROJECT_NAME     := ant_connectivity_s212_uart_pca10040
TARGETS          := nrf52832_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../../../..
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf52832_xxaa.out: \
  LINKER_SCRIPT  := ant_connectivity_gcc_nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/components/libraries/experimental_memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_acknowledge_message_tx.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_active_search_sharing_cycles_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_active_search_sharing_cycles_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_adv_burst_config_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_broadcast_message_tx.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_capabilities_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_assign.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_close.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_id_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_id_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_low_priority_rx_search_timout_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_open_with_offset.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_period_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_period_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_radio_freq_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_radio_freq_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_radio_tx_power_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_rx_search_timeout_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_status_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_channel_unassign.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_coex_config_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_coex_config_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_crypto_channel_enable.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_crypto_info_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_crypto_info_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_crypto_key_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_cw_test_mode.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_cw_test_mode_init.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_enable.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_event.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_event_rx.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_id_list_add.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_id_list_config.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_lib_config_clear.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_lib_config_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_lib_config_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_network_address_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_prox_search_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_rx_scan_mode_start.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_search_channel_priority_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_search_waveform_set.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_stack_reset.c \
  $(SDK_ROOT)/components/serialization/common/struct_ser/ant/ant_struct_serialization.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers/ant_version_get.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/common/ble_dtm_init.c \
  $(SDK_ROOT)/components/serialization/common/ble_serialization.c \
  $(SDK_ROOT)/components/serialization/common/cond_field_serialization.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/common/conn_mw.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/middleware/conn_mw_ant.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/common/conn_mw_nrf_soc.c \
  $(SDK_ROOT)/components/serialization/connectivity/hal/dtm_uart.c \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ble/serializers/nrf_soc_conn.c \
  $(SDK_ROOT)/components/serialization/common/struct_ser/ble/nrf_soc_struct_serialization.c \
  $(SDK_ROOT)/components/serialization/connectivity/ser_conn_cmd_decoder.c \
  $(SDK_ROOT)/components/serialization/connectivity/ser_conn_dtm_cmd_decoder.c \
  $(SDK_ROOT)/components/serialization/connectivity/ser_conn_error_handling.c \
  $(SDK_ROOT)/components/serialization/connectivity/ser_conn_event_encoder.c \
  $(SDK_ROOT)/components/serialization/connectivity/ser_conn_handlers.c \
  $(SDK_ROOT)/components/serialization/connectivity/ser_conn_pkt_decoder.c \
  $(SDK_ROOT)/components/serialization/connectivity/ser_conn_reset_cmd_decoder.c \
  $(SDK_ROOT)/components/serialization/common/ser_dbg_sd_str.c \
  $(SDK_ROOT)/components/serialization/common/transport/ser_hal_transport.c \
  $(SDK_ROOT)/components/serialization/common/transport/ser_phy/ser_phy_uart.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(PROJ_DIR)/main.c \
  $(SDK_ROOT)/components/ble/ble_dtm/ble_dtm.c \
  $(SDK_ROOT)/components/ble/ble_dtm/ble_dtm_hw_nrf52.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52.S \
  $(SDK_ROOT)/components/toolchain/system_nrf52.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ant.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/serialization/connectivity/hal \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ble/serializers \
  $(SDK_ROOT)/components/libraries/experimental_log \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/common \
  $(SDK_ROOT)/components/serialization/connectivity \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/serialization/common \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/drivers_nrf/uart \
  $(SDK_ROOT)/components/serialization/common/transport/ser_phy/config \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/softdevice/s212/headers/nrf52 \
  $(SDK_ROOT)/components/serialization/common/transport/ser_phy \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/serializers \
  $(SDK_ROOT)/components/libraries/experimental_log/src \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/softdevice/s212/headers \
  ../config \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/serialization/connectivity/codecs/ant/middleware \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/libraries/experimental_memobj \
  $(SDK_ROOT)/components/serialization/common/struct_ser/ble \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/serialization/common/transport \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/serialization/common/struct_ser/ant \

# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DANT_STACK_SUPPORT_REQD
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DNRF52
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -DS212
CFLAGS += -DSER_CONNECTIVITY
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -D__HEAP_SIZE=1024
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
ASMFLAGS += -DANT_STACK_SUPPORT_REQD
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52_PAN_74
ASMFLAGS += -DS212
ASMFLAGS += -DSER_CONNECTIVITY
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -D__HEAP_SIZE=1024
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
default: nrf52832_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf52832_xxaa

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash erase

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex
	@echo Flashing: $<
	nrfjprog -f nrf52 --program $< --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall
