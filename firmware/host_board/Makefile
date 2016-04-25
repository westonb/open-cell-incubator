# Makefile for STM32F051 using cmsis and GNU toolchain.

# Files to include
C_SRC  = $(wildcard Src/*.c)
C_SRC += $(wildcard Drivers/STM32F0xx_HAL_Driver/Src/*.c)
C_SRC += $(wildcard Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c)
C_SRC += $(wildcard Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c)
C_INC  = $(wildcard Inc/*.h)

OBJECTS = $(patsubst %.c,%.o,$(C_SRC)) Src/startup.o

INC  = -IInc
INC += -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32F0xx/Include
INC += -IDrivers/STM32F0xx_HAL_Driver/Inc -IDrivers/STM32F0xx_HAL_Driver/Inc/Legacy
INC += -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

# Assembler, compiler, and linker flags
AFLAGS  = -mlittle-endian -mthumb -mcpu=cortex-m0 -march=armv6-m
CFLAGS  = $(AFLAGS) -g -O1 $(INC) -std=c99 -Wall #debugging flag is on this line
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -DSTM32F072xB --specs=nosys.specs #testing this
CFLAGS += -fdata-sections -ffunction-sections
#LFLAGS  = $(CFLAGS) -nostartfiles -Tflash.ld -Wl,-Map=main.map -Wl,--gc-sections
LFLAGS  = $(CFLAGS) -Tflash.ld -Wl,-Map=main.map -Wl,--gc-sections -mthumb -mcpu=cortex-m0 -march=armv6-m -lnosys  #testing this

# Targets
all: main.bin
clean:
	-rm -f $(OBJECTS) *.elf *.bin *.map
main.bin: main.elf
	arm-none-eabi-objcopy -O binary main.elf main.bin
main.elf: $(OBJECTS)
	arm-none-eabi-gcc $(LFLAGS) -o main.elf $(OBJECTS) $(LIBRARIES)
Src/startup.o: Src/startup.s
	arm-none-eabi-as $(AFLAGS) Src/startup.s -o Src/startup.o
%.o: %.c $(C_INC)
	arm-none-eabi-gcc $(CFLAGS) -c -o $@ $<

%.S: %.c $(C_INC)
	 arm-none-eabi-gcc $(CFLAGS) -S -o $@ $<
gdb: main.elf
	arm-none-eabi-gdb -x init.gdb
load: main.elf
	arm-none-eabi-gdb -x init.gdb -ex "mon reset halt" -ex load
loadquit: main.elf
	arm-none-eabi-gdb -batch -x init.gdb -ex "mon reset halt" -ex load -ex kill -ex quit
ocd:
	openocd -f board_openocd.cfg
