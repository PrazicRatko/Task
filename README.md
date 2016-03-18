#STM32 board with integrated FreeRTOS in Qemu emulator

##About

FreeRTOS ported to STM32F103RBT6, based on the CORTEX-M3 ARM CPU.

The current version is based on FreeRTOS 9.0.0. 

The application is doing simple UART task.

##Prerequisites

- GNU ARM Embedded Toolchain: [download and installation guide](https://launchpad.net/~terry.guo/+archive/ubuntu/gcc-arm-embedded)


- Qemu with an STM32 microcontroller implementation: [download and installation guide](https://github.com/beckus/qemu_stm32)

##Build

Compile every source file with next command:


`$ arm-none-eabi-gcc -Wall -mcpu=cortex-m3 -mlittle-endian -mthumb -I path_to_the_header_files/include -D STM32F10x -Os -c source_file_name.c -o source_file_name.o`

Link all .o files with next command:


`$ arm-none-eabi-gcc -mcpu=cortex-m3 -mlittle-endian -mthumb -D STM32F10x -T path_to_the_linker_script/stm32.ld -Wl,--gc-sections list_all_o_files.o -o stm32_application.elf`

Convert ELF binary into binary format:


`$ arm-none-eabi-objcopy -O binary stm32_application.elf stm32_application.bin`


##Run

To run the target image in Qemu, enter the following command:


`path_to_the_installation_file/qemu-system-arm -machine stm32-p103 -nographic -m 513M -kernel stm32_application.bin`

The application will run infinitely so it must be stopped manually by "killing" the instance of Qemu (an "equivalent" to switching off the board).

##License

All source and header files from FreeRTOS and CMSIS are licensed under the modified GPL license. 

For the avoidance of any doubt refer to the comment included at the top of each source and header file for license and copyright information.
