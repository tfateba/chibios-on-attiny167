ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_I2C TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/ports/AVR/TINY/I2Cv1/hal_adc_lld.c
endif
else
PLATFORMSRC += ${CHIBIOS}/os/hal/ports/AVR/TINY/I2Cv1/hal_adc_lld.c
endif

PLATFORMINC += ${CHIBIOS}/os/hal/ports/AVR/TINY/I2Cv1
