ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_SPI TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/ports/AVR/TINY/SPIv1/hal_adc_lld.c
endif
else
PLATFORMSRC += ${CHIBIOS}/os/hal/ports/AVR/TINY/SPIv1/hal_adc_lld.c
endif

PLATFORMINC += ${CHIBIOS}/os/hal/ports/AVR/TINY/SPIv1
