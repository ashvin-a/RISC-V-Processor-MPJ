#! Dont use this thing! THIS IS INCOMPLETE
import struct
from collections import OrderedDict

# --- Utility Functions ---

def sign_extend(value, bits):
    """Sign-extends a value up to 32 bits."""
    sign_bit = 1 << (bits - 1)
    return (value & (sign_bit - 1)) - (value & sign_bit)

def decode_instruction(instruction_word):
    """
    Decodes a 32-bit RISC-V instruction word into assembly.
    This function implements a comprehensive disassembler logic based on the
    instruction's bit fields (Opcode, Funct3, Funct7).
    """
   
    # 7-bit opcode
    opcode = instruction_word & 0x7F
   
    # Register names lookup (standard RISC-V calling convention names)
    REGISTERS_REV = {
        0: 'zero', 1: 'ra', 2: 'sp', 3: 'gp', 4: 'tp', 5: 't0', 6: 't1', 7: 't2',
        8: 's0', 9: 's1', 10: 'a0', 11: 'a1', 12: 'a2', 13: 'a3', 14: 'a4', 15: 'a5',
        16: 'a6', 17: 'a7', 18: 's2', 19: 's3', 20: 's4', 21: 's5', 22: 's6', 23: 's7',
        24: 's8', 25: 's9', 26: 's10', 27: 's11', 28: 't3', 29: 't4', 30: 't5', 31: 't6',
    }
   
    # Helper to get register name
    def reg_name(reg_num):
        return REGISTERS_REV.get(reg_num, f'x{reg_num}')

    # Extract common fields
    funct3 = (instruction_word >> 12) & 0x7
    funct7 = (instruction_word >> 25) & 0x7F
    rd = (instruction_word >> 7) & 0x1F
    rs1 = (instruction_word >> 15) & 0x1F
    rs2 = (instruction_word >> 20) & 0x1F
   
   
    # --- R-Type (Opcode 0b0110011: Register-Register ALU) ---
    if opcode == 0b0110011:
        instr_map = {
            # funct3: {funct7: instr_name}
            0b000: {0b0000000: "add", 0b0100000: "sub"}, # ADD/SUB
            0b001: {0b0000000: "sll"},                   # SLL
            0b010: {0b0000000: "slt"},                   # SLT
            0b011: {0b0000000: "sltu"},                  # SLTU
            0b100: {0b0000000: "xor"},                   # XOR
            0b101: {0b0000000: "srl", 0b0100000: "sra"}, # SRL/SRA
            0b110: {0b0000000: "or"},                    # OR
            0b111: {0b0000000: "and"},                   # AND
        }
       
        if funct3 in instr_map and funct7 in instr_map[funct3]:
            instr_name = instr_map[funct3][funct7]
            # Pseudoinstruction detection: ADD rd, zero, rs2 is sometimes MOV rd, rs2
            if instr_name == "add" and rs1 == 0:
                 return f"mv {reg_name(rd)}, {reg_name(rs2)}"
           
            return f"{instr_name} {reg_name(rd)}, {reg_name(rs1)}, {reg_name(rs2)}"
       
    # --- I-Type (Opcode 0b0010011: Immediate ALU) and (Opcode 0b0000011: Load) ---
    elif opcode in (0b0010011, 0b0000011, 0b1110011, 0b1100111):
        imm_12 = (instruction_word >> 20) & 0xFFF
        imm = sign_extend(imm_12, 12)
        shamt = imm_12 & 0x1F # For shifts, immediate is shamt[4:0]
       
        # Immediate ALU (0b0010011)
        if opcode == 0b0010011:
            if funct3 == 0b000:
                if rs1 == 0 and imm == 0:
                    return f"nop"
                # Pseudo-instruction li (Load Immediate) detection
                if rs1 == 0:
                    return f"li {reg_name(rd)}, {imm}"
                return f"addi {reg_name(rd)}, {reg_name(rs1)}, {imm}"
           
            instr_map = {
                0b001: "slli",  # SLLI (funct7 must be 0)
                0b010: "slti",  # SLTI
                0b011: "sltiu", # SLTIU
                0b100: "xori",  # XORI
                0b101: ("srli", "srai"), # SRLI (funct7=0) / SRAI (funct7=0b0100000)
                0b110: "ori",   # ORI
                0b111: "andi",  # ANDI
            }
           
            if funct3 in instr_map:
                instr_info = instr_map[funct3]
               
                if funct3 in (0b001, 0b101): # Shift instructions need funct7 check
                    if funct7 == 0b0000000:
                        instr_name = instr_info if isinstance(instr_info, str) else instr_info[0]
                        return f"{instr_name} {reg_name(rd)}, {reg_name(rs1)}, {shamt}"
                    elif funct7 == 0b0100000 and funct3 == 0b101:
                        instr_name = instr_info[1] # SRAI
                        return f"{instr_name} {reg_name(rd)}, {reg_name(rs1)}, {shamt}"
                elif isinstance(instr_info, str):
                    # Standard I-Type
                    return f"{instr_info} {reg_name(rd)}, {reg_name(rs1)}, {imm}"

        # Load Instructions (0b0000011)
        elif opcode == 0b0000011:
            instr_map = {
                0b000: "lb",
                0b001: "lh",
                0b010: "lw", # Exists in original
                0b100: "lbu",
                0b101: "lhu",
            }
            if funct3 in instr_map:
                instr_name = instr_map[funct3]
                return f"{instr_name} {reg_name(rd)}, {imm}({reg_name(rs1)})"

        # JALR (0b1100111)
        elif opcode == 0b1100111 and funct3 == 0b000:
            if rd == 0 and imm == 0 and rs1 != 0:
                # Pseudo-instruction RET: JALR zero, rs1, 0
                return f"ret"
            if rd == 0:
                # Pseudo-instruction JR: JALR zero, rs1, imm
                return f"jr {reg_name(rs1)}"
            return f"jalr {reg_name(rd)}, {reg_name(rs1)}, {imm}"

        # System/Environment Instructions (0b1110011)
        elif opcode == 0b1110011 and funct3 == 0b000:
            if imm_12 == 0b000000000000:
                return f"ecall"
            elif imm_12 == 0b000000000001:
                return f"ebreak" # Exists in original
            elif imm_12 == 0b000100000010: # MRET
                return f"mret"
   
    # --- S-Type (Opcode 0b0100011: Store) ---
    elif opcode == 0b0100011:
        # Immediate calculation for S-Type (bits [11:5] and [4:0])
        imm_11_5 = (instruction_word >> 25) & 0x7F
        imm_4_0 = (instruction_word >> 7) & 0x1F
        imm_12 = (imm_11_5 << 5) | imm_4_0
        imm = sign_extend(imm_12, 12)

        instr_map = {
            0b000: "sb",
            0b001: "sh",
            0b010: "sw", # Exists in original
        }
       
        if funct3 in instr_map:
            instr_name = instr_map[funct3]
            return f"{instr_name} {reg_name(rs2)}, {imm}({reg_name(rs1)})"
       
    # --- B-Type (Opcode 0b1100011: Branch) ---
    elif opcode == 0b1100011:
       
        # Immediate calculation for B-Type
        imm_12 = (instruction_word >> 31) & 0x1
        imm_10_5 = (instruction_word >> 25) & 0x3F
        imm_4_1 = (instruction_word >> 8) & 0xF
        imm_11 = (instruction_word >> 7) & 0x1
       
        # Combine and sign-extend the 12-bit offset (multiplied by 2)
        imm_raw = (imm_12 << 11) | (imm_11 << 10) | (imm_10_5 << 4) | (imm_4_1 << 0)
        offset = sign_extend(imm_raw << 1, 13)
       
        instr_map = {
            0b000: "beq", # Exists in original
            0b001: "bne",
            0b100: "blt",
            0b101: "bge",
            0b110: "bltu", # Exists in original
            0b111: "bgeu",
        }
       
        if funct3 in instr_map:
            instr_name = instr_map[funct3]
            # Branch target is calculated relative to PC, which is current_address + offset
            return f"{instr_name} {reg_name(rs1)}, {reg_name(rs2)}, <pc + {offset}>"

    # --- U-Type (Opcode 0b0110111: LUI / Opcode 0b0010111: AUIPC) ---
    elif opcode in (0b0110111, 0b0010111):
        imm_20 = (instruction_word >> 12) & 0xFFFFF
        # U-Type immediates are 20 bits, treated as bits 31:12
        imm_val = imm_20 << 12

        if opcode == 0b0110111: # LUI
            # Pseudo-instruction LI (if the upper 20 bits are sufficient) is handled by ADDI
            return f"lui {reg_name(rd)}, 0x{imm_val >> 12:x}"
        elif opcode == 0b0010111: # AUIPC
            return f"auipc {reg_name(rd)}, 0x{imm_val >> 12:x}"

    # --- J-Type (Opcode 0b1101111: JAL) ---
    elif opcode == 0b1101111:
       
        # Immediate calculation for J-Type (bits 20, 10:1, 11, 19:12)
        imm_20 = (instruction_word >> 31) & 0x1
        imm_10_1 = (instruction_word >> 21) & 0x3FF
        imm_11 = (instruction_word >> 20) & 0x1
        imm_19_12 = (instruction_word >> 12) & 0xFF
       
        # Combine the 20-bit offset (multiplied by 2)
        imm_raw = (imm_20 << 19) | (imm_19_12 << 11) | (imm_11 << 10) | (imm_10_1 << 0)
        offset = sign_extend(imm_raw << 1, 21)
       
        # Pseudo-instruction J (JAL zero, offset)
        if rd == 0:
            return f"j <pc + {offset}>"
       
        return f"jal {reg_name(rd)}, <pc + {offset}>"
       
    return f".word 0x{instruction_word:08x}"

def disassemble_binary_data(binary_data: bytes, base_address: int = 0x0):
    """
    Reads raw bytes (simulating a binary file) and disassembles them.
   
    Args:
        binary_data: A bytes object containing the raw machine code.
        base_address: The starting memory address for the first instruction.
       
    Returns:
        A list of tuples: (address, hex_code, assembly_line)
    """
    instructions = []
    current_address = base_address
   
    # Read 4 bytes at a time (32-bit instructions)
    for i in range(0, len(binary_data), 4):
        chunk = binary_data[i:i+4]
       
        # Ensure we have a full 4 bytes
        if len(chunk) < 4:
            break
           
        try:
            # Unpack the 4 bytes into a single unsigned integer (little-endian: '<I')
            instruction_word, = struct.unpack('<I', chunk)
           
            # Decode the instruction
            assembly_line = decode_instruction(instruction_word)
           
            instructions.append((
                current_address,
                f"{instruction_word:08x}",
                assembly_line
            ))
           
        except struct.error:
            instructions.append((current_address, chunk.hex(), "ERROR: Unpacking failed"))
        except Exception as e:
            instructions.append((current_address, chunk.hex(), f"ERROR: Decoding failed - {e}"))
           
        current_address += 4 # RISC-V instructions are 4 bytes wide

    return instructions

def disassemble_hex_string(hex_string, base_address=0):
    """
    Parses a string of hex values (text) into binary data and disassembles it.
    """
    # Clean the string (remove newlines, spaces)
    clean_hex = hex_string.replace('\n', '').replace(' ', '')
   
    # Ensure length is a multiple of 8 (4 bytes * 2 chars/byte)
    if len(clean_hex) % 8 != 0:
        print(f"Warning: Truncating incomplete hex data at end of string.")
        clean_hex = clean_hex[:-(len(clean_hex) % 8)]
       
    try:
        # Convert hex string to bytes
        binary_data = bytes.fromhex(clean_hex)
        return disassemble_binary_data(binary_data, base_address)
    except ValueError as e:
        return [(base_address, "INVALID", f"Error parsing hex string: {e}")]

# --- Execution Example ---

if __name__ == "__main__":
    # --- usage example 1: Text-based Hex String ---
    print("--- Example 1: Disassembling a Hex String ---")
   
    # This example contains two instructions:
    # 0x00052523 -> sw zero, 10(a0) (instruction may vary based on exact bitfields)
    # 0x00900513 -> addi a0, zero, 9 (li a0, 9)
    example_hex = """9302d009
23205010"""
   
    results = disassemble_hex_string(example_hex, base_address=0x1000)
    for addr, hex_val, asm in results:
        print(f"0x{addr:04x} | {hex_val} | {asm}")

    # --- usage example 2: Reading from a Binary File ---
    # To use this, you would place your 'program.bin' or 'program.hex' in the same folder.
