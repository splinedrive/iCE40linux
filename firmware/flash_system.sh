#!/bin/sh
# Bulk erase the flash
iceprog -b

# Flash the Root FS image at offset=6M
iceprog -n -X -o 6144k buildroot/output/images/rootfs.ubi

# Flash the kernel at offset=256k
iceprog -n -X -o  256k buildroot/output/images/Image

# Flash the device tree at offset=192k
iceprog -n -X -o  192k dt/ice40linux.dtb

# Flash the machine mode bios at offset=128k
iceprog -n -X -o  128k bios/bios.bin

# Flash the FPGA bitstream at the beginning of flash
iceprog -n -X     ../gateware/riscv_linux/build-tmp/riscv_linux_init.bin
