ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_USB TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/ports/AVR/TINY/USBv1/hal_usb_lld.c
endif
else
PLATFORMSRC += ${CHIBIOS}/os/hal/ports/AVR/TINY/USBv1/hal_usb_lld.c
endif

PLATFORMINC += ${CHIBIOS}/os/hal/ports/AVR/TINY/USBv1
