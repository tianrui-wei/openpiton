#!/bin/bash
cp /home/eva/piton/uboot/spl/u-boot-spl.bin /home/eva/piton/piton_bin/u-boot-spl.riscv
cd build
sims -sys=manycore -vcs_build -debug_all -x_tiles=1 -y_tiles=1 u-boot-spl.riscv -ariane -precompiled -vcs_build_args="-full64"
#sims -sys=manycore -vcs_run -gui -x_tiles=1 -y_tiles=1 u-boot-spl.riscv -ariane -precompiled -vcs_build_args="-full64"
sims -sys=manycore -vcs_run -nosimslog -x_tiles=1 -y_tiles=1 u-boot-spl.riscv -ariane -precompiled -vcs_build_args="-full64"
