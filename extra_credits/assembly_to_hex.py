from riscv_converter import RiscvAssembler
# Just add your locations here and run script.
ASSEMBLY_FILE_LOCATION = "extra_credits/bubblesort_large.asm"
OUTPUT_LOCATION = "extra_credits/output.mem"

with open(ASSEMBLY_FILE_LOCATION, "r") as f:
    code = f.read()


assembler = RiscvAssembler(code)
machine_code_hex = assembler.assemble()

output_code = ""
for code in machine_code_hex:
    output_code = output_code + f"{code[:2]}\n" + f"{code[2:4]}\n" + f"{code[4:6]}\n" + f"{code[6:]}\n"

with open(OUTPUT_LOCATION, "w") as f:
    f.write(output_code)
