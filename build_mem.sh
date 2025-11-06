#!/bin/bash

# Exit on any error
set -e

# Input file (assembly)
ASM_FILE="program.S"

# Output files
OBJ_FILE="program.o"
# ELF_FILE="program.elf"
BIN_FILE="program.bin"
MEM_FILE="program.mem"

# Check that assembler is installed
if ! command -v riscv64-unknown-elf-as &> /dev/null; then
    echo "Error: RISC-V toolchain not found in PATH."
    echo "Make sure riscv64-unknown-elf-as is installed and in your PATH."
    exit 1
fi

echo "Assembling $ASM_FILE..."
riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 "$ASM_FILE" -o "$OBJ_FILE"

echo "Generating raw binary..."
riscv64-unknown-elf-objcopy -O binary "$OBJ_FILE" "$BIN_FILE"

echo "Converting binary to memory file..."
xxd -p -c 1 "$BIN_FILE" > "$MEM_FILE"

echo "Done. Output written to $MEM_FILE"