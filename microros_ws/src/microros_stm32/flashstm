#!/bin/bash


if [[ $# -ne 1 ]]
then
    echo "usage: flashstm <path_to_elf_file>"
    exit 1
fi

path_to_elf_file=$1
openocd -f interface/stlink.cfg -c "set WORKAREASIZE 0x2000" -f target/stm32f4x.cfg -c "program $path_to_elf_file verify reset"
