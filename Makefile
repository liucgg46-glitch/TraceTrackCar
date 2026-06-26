TARGET := DroneProject
BUILD_DIR := build

PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE := $(PREFIX)size

SHELL := powershell.exe
.SHELLFLAGS := -NoProfile -ExecutionPolicy Bypass -Command

MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS := -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx

INCLUDES := \
  -Ilibraries/CMSIS/Device/ST/STM32F4xx/Include \
  -Ilibraries/CMSIS/Include \
  -Ilibraries/STM32F4xx_StdPeriph_Driver/inc \
  -Iuser \
  -IBSP \
  -IDriver \
  -IAlgorithm \
  -IVL53L1_core \
  -IVL53L1_platform \
  -IAPP \
  -IRoute

CFLAGS := $(MCU) $(DEFS) $(INCLUDES) -std=gnu11 -O0 -g3 -ffunction-sections -fdata-sections -Wall -MMD -MP
ASFLAGS := $(MCU) -x assembler-with-cpp -g3 -MMD -MP
LDSCRIPT := STM32F407ZGTx_FLASH.ld
LDFLAGS := $(MCU) -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs
LDLIBS := -lc -lm -lnosys

STARTUP := libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f40_41xxx.s

SRCS := \
  libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/misc.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cec.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dfsdm.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma2d.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dsi.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash_ramfunc.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fmpi2c.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_lptim.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ltdc.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_qspi.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sai.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spdifrx.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
  libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c \
  user/main.c \
  user/stm32f4xx_it.c \
  user/bsp_led.c \
  BSP/bsp_gpio.c \
  BSP/bsp_pwm.c \
  BSP/bsp_encoder.c \
  BSP/bsp_adc.c \
  BSP/bsp_key.c \
  BSP/bsp_exti.c \
  BSP/bsp_uart.c \
  BSP/bsp_i2c.c \
  BSP/bsp_spi.c \
  BSP/bsp_all.c \
  BSP/bsp_systick.c \
  Algorithm/pid.c \
  Algorithm/line_detect.c \
  Algorithm/line_track.c \
  Algorithm/heading_estimator.c \
  Algorithm/odometer.c \
  Driver/test.c \
  Driver/drv_encoder.c \
  Driver/drv_motor.c \
  Driver/driver_all.c \
  Driver/drv_gray_4051.c \
  Driver/drv_gray_mcu_i2c.c \
  Driver/drv_gray_sensor.c \
  Driver/drv_lcd_tft.c \
  Driver/drv_lcd_font.c \
  Driver/drv_oled_i2c.c \
  Driver/drv_oled_font.c \
  Driver/drv_oled_image.c \
  APP/app_task_port.c \
  APP/nb_wait.c \
  APP/scheduler.c \
  APP/chassis.c \
  APP/app_all.c \
  APP/motion_action.c \
  APP/line_follow_app.c \
  APP/sensor_manager.c \
  APP/lcd_ui.c \
  APP/oled_ui.c \
  VL53L1_core/vl53l1_api.c \
  VL53L1_core/vl53l1_api_calibration.c \
  VL53L1_core/vl53l1_api_core.c \
  VL53L1_core/vl53l1_api_debug.c \
  VL53L1_core/vl53l1_api_preset_modes.c \
  VL53L1_core/vl53l1_api_strings.c \
  VL53L1_core/vl53l1_core.c \
  VL53L1_core/vl53l1_core_support.c \
  VL53L1_core/vl53l1_error_strings.c \
  VL53L1_core/vl53l1_register_funcs.c \
  VL53L1_core/vl53l1_silicon_core.c \
  VL53L1_core/vl53l1_wait.c \
  VL53L1_platform/vl53l1_platform.c \
  VL53L1_platform/vl53l1_platform_init.c \
  Route/route_common.c \
  Route/route_manager.c \
  Route/route_profile_basic.c \
  Route/route_profile_hjduino.c

OBJS := $(addprefix $(BUILD_DIR)/,$(SRCS:.c=.o))
OBJS += $(BUILD_DIR)/$(STARTUP:.s=.o)
DEPS := $(OBJS:.o=.d)

.PHONY: all clean size

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

$(BUILD_DIR)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@New-Item -ItemType Directory -Force -Path '$(BUILD_DIR)' | Out-Null
	$(CC) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD_DIR)/%.o: %.c
	@New-Item -ItemType Directory -Force -Path '$(dir $@)' | Out-Null
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	@New-Item -ItemType Directory -Force -Path '$(dir $@)' | Out-Null
	$(AS) $(ASFLAGS) -c $< -o $@

size: $(BUILD_DIR)/$(TARGET).elf
	$(SIZE) $<

clean:
	@if (Test-Path '$(BUILD_DIR)') { Remove-Item -Recurse -Force '$(BUILD_DIR)' }

-include $(DEPS)
