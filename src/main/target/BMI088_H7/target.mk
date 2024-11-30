H743xI_TARGETS += $(TARGET)
HSE_VALUE    = 8000000
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_bmi088.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \