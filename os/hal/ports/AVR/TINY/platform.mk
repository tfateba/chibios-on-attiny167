# List of all the AVR platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/AVR/TINY/hal_lld.c

# Drivers compatible with the platform.
PLATFORMINC = ${CHIBIOS}/os/hal/ports/AVR/TINY
include ${CHIBIOS}/os/hal/ports/AVR/TINY/ADCv1/driver.mk
include ${CHIBIOS}/os/hal/ports/AVR/TINY/EXTv1/driver.mk
include ${CHIBIOS}/os/hal/ports/AVR/TINY/GPIOv1/driver.mk
include ${CHIBIOS}/os/hal/ports/AVR/TINY/I2Cv1/driver.mk
include ${CHIBIOS}/os/hal/ports/AVR/TINY/SPIv1/driver.mk
include ${CHIBIOS}/os/hal/ports/AVR/TINY/TIMv1/driver.mk
include ${CHIBIOS}/os/hal/ports/AVR/TINY/USARTv1/driver.mk
include ${CHIBIOS}/os/hal/ports/AVR/TINY/USBv1/driver.mk
