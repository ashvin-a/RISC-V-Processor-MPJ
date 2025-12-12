`timescale 1ns/1ps
/////////////////////////// FETCH MODULE ///////////////////////////
module fetch (
    input  wire clk,
    input  wire rst,
    input  wire i_clu_halt,
    input  wire i_pc_write_en,
    
    // Branch correction signals
    input  wire i_mispredict,                // NEW: From HCU
    input  wire [31:0] i_recovery_addr,      // NEW: Correct address to jump to

    // Normal PC inputs
    output wire [31:0] o_instr_mem_rd_addr,
    output wire [31:0] o_pc_plus_4,
    output reg  [31:0] PC,
    output wire [31:0] o_next_pc    
);

    wire [31:0] next_pc;
    wire [31:0] t_pc_plus_4;
    
    assign t_pc_plus_4 = PC + 4;

    // Logic for Next PC:
    // 1. High Priority: If mispredict, go to recovery address immediately.
    // 2. Normal: PC + 4 (Since we don't have a BTB to jump in Fetch, we fetch normally).
    assign next_pc = (i_mispredict) ? i_recovery_addr : t_pc_plus_4;

    assign o_instr_mem_rd_addr = (i_pc_write_en) ? PC : PC - 4;
    assign o_pc_plus_4 = t_pc_plus_4;

    always @(posedge clk) begin
        if(rst) begin
            PC <= 0;
        end
        else begin
            if (!i_clu_halt)
                if (i_pc_write_en)
                PC <= next_pc;
        end
    end
    assign o_next_pc = next_pc;
endmodule
/////////////////////////// MAIN CONTROL UNIT ///////////////////////////
module control_unit(
    input  wire [31:0] i_clu_inst,
    output wire o_clu_Branch,
    output wire o_clu_halt,
    output wire o_clu_MemRead,
    output wire o_clu_MemtoReg,
    output wire [1:0]o_clu_ALUOp,
    output wire o_clu_MemWrite,
    output wire [1:0] o_clu_ALUSrc,
    output wire o_clu_RegWrite,
    output wire [1:0]o_clu_lui_auipc_mux_sel,       // The Mux in between reg and alu for lui and auipc instruction implementation
    output wire [1:0]o_clu_branch_instr_alu_sel,    // Should be invalid by default
    output wire o_clu_pc_o_rs1_data_mux_sel,
    output wire [2:0]o_clu_ld_st_type_sel,
    output wire [2:0]o_sign_or_zero_ext_data_mux,    // This signal will go to 5:1 MUX which will choose between ZERO extend , SIGN EXTEND or NO EXTEND on the read data from the datamem - ONLY FOR LOAD
    input wire reg_valid
);

assign o_clu_Branch =  !reg_valid ? 1'b0 : ((i_clu_inst[6:0] == 7'b110_0011)|| (i_clu_inst[6:0] == 7'b110_1111) || (i_clu_inst[6:0] == 7'b110_0111)); // Branch enabled for BRANCH , JAL and JALR instruction types

assign o_clu_halt =    !reg_valid ? 1'b0 : (i_clu_inst[6:0] == 7'b111_0011); // Halt - For ebreak condition - (ecall also will trigger the same as it has the same opcode)

assign o_clu_MemRead =  !reg_valid ? 1'b0 : (i_clu_inst[6:0] == 7'b000_0011); // Load

assign o_clu_MemtoReg = !reg_valid ? 1'b0 : (i_clu_inst[6:0] == 7'b000_0011) ? 1'b1 : // Load
                        (i_clu_inst[6:0] == 7'b110_0011) ? 1'bx : // Branch
                        (i_clu_inst[6:0] == 7'b010_0011) ? 1'b0 : // Store
                        (i_clu_inst[6:0] == 7'b110_1111) ? 1'b0 : // JAL
                        (i_clu_inst[6:0] == 7'b110_0111) ? 1'b0 : // JALR
                        1'b0;

assign o_clu_MemWrite = !reg_valid ? 1'b0 : (i_clu_inst[6:0] == 7'b010_0011);   // Store

assign o_clu_ALUSrc = !reg_valid ? 1'b0 :   ((i_clu_inst[6:0] == 7'b001_0011)|| // I type
                        (i_clu_inst[6:0] == 7'b011_0111) || // LUI
                        (i_clu_inst[6:0] == 7'b001_0111) || // AUIPC
                        (i_clu_inst[6:0] == 7'b000_0011) || // Load
                        (i_clu_inst[6:0] == 7'b010_0011)) ? 2'b01 : // Store
                        ((i_clu_inst[6:0] == 7'b110_1111) || (i_clu_inst[6:0] == 7'b110_0111)) ? 2'b00 :  //JAL and JALR type
                        2'b10;   // Branch and R type

assign o_clu_RegWrite = !reg_valid ? 1'b0 : (i_clu_inst[6:0] == 7'b011_0011) || // R type
                        (i_clu_inst[6:0] == 7'b001_0011) || // I type
                        (i_clu_inst[6:0] == 7'b011_0111) || // LUI
                        (i_clu_inst[6:0] == 7'b001_0111) || // AUIPC
                        (i_clu_inst[6:0] == 7'b000_0011) || // Load
                        (i_clu_inst[6:0] == 7'b110_1111) || // Jal
                        (i_clu_inst[6:0] == 7'b110_0111);   // Jalr

assign o_clu_ALUOp =   !reg_valid ? 2'b0 :  ((i_clu_inst[6:0] == 7'b011_0011) || (i_clu_inst[6:0] == 7'b001_0011)) ?  2'b10 : // R and I type 
                        (i_clu_inst[6:0] == 7'b110_0011) ? 2'b01 : //Branch
                        2'b00;

assign o_clu_branch_instr_alu_sel  =  !reg_valid ? 2'b0 :   (i_clu_inst[6:0] == 7'b110_0011)? (      //Check for BRANCH Instruction type
                                        (i_clu_inst[14:12] == 3'b000)? 2'b00  : //beq
                                        (i_clu_inst[14:12] == 3'b001)? 2'b01  : //bne
                                        (i_clu_inst[14:12] == 3'b100)? 2'b10  : //blt
                                        (i_clu_inst[14:12] == 3'b101)? 2'b11  : //bge
                                        (i_clu_inst[14:12] == 3'b110)? 2'b10  : //bltu
                                        (i_clu_inst[14:12] == 3'b111)? 2'b11  : 2'bxx) : //bgeu
                                        ((i_clu_inst[6:0] == 7'b110_1111) || (i_clu_inst[6:0] == 7'b110_0111)) ?  2'b01 : // JAl and JALR - forcing to ~o_eq check which is always true for PC+4 vs 32'b0
                                        2'bxx; //Don't care
                                        //2'b00; // Moving it to Non X values for avoiding X propogation

assign o_clu_lui_auipc_mux_sel =     !reg_valid ? 2'b0 :    (i_clu_inst[6:0] == 7'b011_0111) ? 2'b01 : // LUI
                                        (i_clu_inst[6:0] == 7'b001_0111) ? 2'b10 : // AUICP
                                        ((i_clu_inst[6:0] == 7'b110_1111) || (i_clu_inst[6:0] == 7'b110_0111)) ? 2'b11 : // JAL, JALR
                                        2'b00; // Register File

assign o_clu_pc_o_rs1_data_mux_sel =   !reg_valid ? 1'b0 :  (i_clu_inst[6:0] == 7'b110_0111); // JALR - Select o_rs1_data instead of PC

assign o_sign_or_zero_ext_data_mux =  !reg_valid ? 3'b0 :   (i_clu_inst[6:0] == 7'b000_0011) ? ( // Check for Load instruction
                                        (i_clu_inst[14:12] == 3'b000) ? 3'b001 : // lb
                                        (i_clu_inst[14:12] == 3'b001) ? 3'b011 : // lh
                                        (i_clu_inst[14:12] == 3'b010) ? 3'b100 : // lw
                                        (i_clu_inst[14:12] == 3'b100) ? 3'b000 : // lbu
                                        (i_clu_inst[14:12] == 3'b101) ? 3'b010 : // lhu
                                        3'b100) :
                                        3'b100; 

// LOAD and STORE INSTRCUTION TYPE SEL FOR MASK GENERATION
assign o_clu_ld_st_type_sel = !reg_valid ? 3'b0 : (i_clu_inst[6:0] == 7'b000_0011) ? (                  // Check for Load instruction
                              (i_clu_inst[14:12] == 3'b000)? 3'b000  :              // Byte - lb 
                              (i_clu_inst[14:12] == 3'b001)? 3'b001  :              // Half - lh 
                              (i_clu_inst[14:12] == 3'b010)? 3'b010  :              // Word - lw 
                              (i_clu_inst[14:12] == 3'b100)? 3'b011  :              // Byte - unsigned - lbu
                              (i_clu_inst[14:12] == 3'b101)? 3'b100  : 3'b010) :    // Half - unsigned - lhu
                              (i_clu_inst[6:0] == 7'b010_0011) ? (                  // Check for STORE instruction
                              (i_clu_inst[14:12] == 3'b000)? 3'b101  :              // Byte - sb 
                              (i_clu_inst[14:12] == 3'b001)? 3'b110  :              // Half - sh 
                              (i_clu_inst[14:12] == 3'b010)? 3'b111  : 3'b111)      // Word - sw
                              : 3'bxxx ;                                            // SHOULD NOT HAPPEN - Don't care
                              //: 3'b000 ;                                            // SHOULD NOT HAPPEN - Don't care - // Moving it to Non X values for avoiding X propogation
                              
endmodule

/////////////////////////// ALU MODULE ///////////////////////////
// The arithmetic logic unit (ALU) is responsible for performing the core
// calculations of the processor. It takes two 32-bit operands and outputs
// a 32 bit result based on the selection operation - addition, comparison,
// shift, or logical operation. 
module alu (
    // 3'b000: addition/subtraction if `i_sub` asserted
    // 3'b001: shift left logical
    // 3'b010,
    // 3'b011: set less than/unsigned if `i_unsigned` asserted
    // 3'b100: exclusive or
    // 3'b101: shift right logical/arithmetic if `i_arith` asserted
    // 3'b110: or
    // 3'b111: and
    input  wire [ 2:0] i_opsel,
    // When asserted, addition operations should subtract instead.
    // This is only used for `i_opsel == 3'b000` (addition/subtraction).
    input  wire        i_sub,
    // When asserted, comparison operations should be treated as unsigned.
    // This is only used for branch comparisons and set less than.
    // For branch operations, the ALU result is not used, only the comparison
    // results.
    input  wire        i_unsigned,
    // When asserted, right shifts should be treated as arithmetic instead of
    // logical. This is only used for `i_opsel == 3'b011` (shift right).
    input  wire        i_arith,
    // First 32-bit input operand.
    input  wire [31:0] i_op1,
    // Second 32-bit input operand.
    input  wire [31:0] i_op2,
    // 32-bit output result. Any carry out (from addition) should be ignored.
    output wire [31:0] o_result,
    // Equality result. This is used downstream to determine if a
    // branch should be taken. // BEQ
    output wire        o_eq,
    // Set less than result. This is used downstream to determine if a
    // branch should be taken.
    output wire        o_slt // BLT and BLTU
);

    wire [4:0]temp = i_op2[4:0];
    wire [31:0] sll = i_op1 << temp;
    wire [31:0] srl = i_op1 >> temp;
    wire [31:0] sra = $signed(i_op1) >>> temp;
    wire [31:0] slt_signed = ($signed(i_op1) < $signed(i_op2)) ? 32'b1 : 32'b0;
    wire [31:0] slt_unsigned = (i_op1 < i_op2) ? 32'b1 : 32'b0;

    assign o_eq  = (i_op1 == i_op2);
    assign o_slt =  i_unsigned ? (i_op1 <  i_op2) : ($signed(i_op1) <  $signed(i_op2));
    assign o_result =       (i_opsel == 3'b000)? ((i_sub) ? (i_op1 - i_op2) : (i_op1 + i_op2)) :
                            (i_opsel == 3'b001)? (i_op1 << temp) : 
                            ((i_opsel == 3'b010) || (i_opsel == 3'b011)) ? (i_unsigned ? slt_unsigned : slt_signed) : 
                            (i_opsel == 3'b100)? (i_op1 ^ i_op2) :  
                            (i_opsel == 3'b101)? ((i_arith)? sra : srl) : 
                            (i_opsel == 3'b110)? (i_op1 | i_op2) : 
                            (i_opsel == 3'b111)? (i_op1 & i_op2) : 
                            32'h0;
    
endmodule

/////////////////////////// ALU WRAPPER ///////////////////////////

module alu_wrapper (
    // 4 bit input from ALU Control block
    input  wire [ 3:0] i_alu_ctrl_opsel,
    // First 32-bit input operand.
    input  wire [31:0] i_rf_op1,
    // Second 32-bit input operand.
    input  wire [31:0] i_rf_op2,
    // Signal for Branch instruction selection coming from alu control
    input wire [1:0]i_clu_branch_instr_alu_sel,
    // 32-bit output result. Any carry out (from addition) should be ignored.
    output wire [31:0] o_alu_result,
    // Equality result. This is used downstream to determine if a
    // branch should be taken. (case of BLT/U , BGE/U , BNE , BEQ)
    output wire        o_alu_Zero
);
    wire [ 2:0] i_opsel;
    wire        i_sub;
    wire        i_unsigned;
    wire        i_arith;
    wire        o_eq;
    wire        o_slt;
    alu alu_inst(
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith),
        .i_op1(i_rf_op1),
        .i_op2(i_rf_op2),
        .o_result(o_alu_result),
        .o_eq(o_eq),
        .o_slt(o_slt)
    );

    // INPUTS TO THE INSIDE ALU
    assign i_unsigned = (i_alu_ctrl_opsel == 4'b1001)? 1'b1 : 1'b0; // Whenever it is SLTU , we will enable the unsigned signal high
    assign i_arith    = (i_alu_ctrl_opsel == 4'b0111)? 1'b1 : 1'b0;
    assign i_sub      = (i_alu_ctrl_opsel == 4'b0001)? 1'b1 : 1'b0;
    assign i_opsel    = (i_alu_ctrl_opsel == 4'b0000)? 3'b000 :    //ADD
                        (i_alu_ctrl_opsel == 4'b0001)? 3'b000 :    //SUB   
                        (i_alu_ctrl_opsel == 4'b0010)? 3'b111 :    //AND   
                        (i_alu_ctrl_opsel == 4'b0011)? 3'b110 :    //OR   
                        (i_alu_ctrl_opsel == 4'b0100)? 3'b100 :    //XOR   
                        (i_alu_ctrl_opsel == 4'b0101)? 3'b001 :    //SLL   
                        (i_alu_ctrl_opsel == 4'b0110)? 3'b101 :    //SRL   
                        (i_alu_ctrl_opsel == 4'b0111)? 3'b101 :    //SRA   
                        (i_alu_ctrl_opsel == 4'b1000)? 3'b010 :    //SLT
                        (i_alu_ctrl_opsel == 4'b1001)? 3'b010 :    //SLTU   
                        (i_alu_ctrl_opsel == 4'b1010)? 3'b011 :    //PASS_B - LUI   - Can I use 011 for LUI?
                        3'bXXX; //Don't care               
    
    // OUTPUTS
    assign o_alu_Zero = (i_clu_branch_instr_alu_sel == 2'b00) ?   o_eq :  //BEQ
                        (i_clu_branch_instr_alu_sel == 2'b01) ?  ~o_eq :  //BNE
                        (i_clu_branch_instr_alu_sel == 2'b10) ?  o_slt :  //BLT / BLTU 
                        ~o_slt;                                           //BGE / BGEU
                        
endmodule
/////////////////////////// ALU CONTROL ///////////////////////////
module alu_control( input  wire [1:0]  i_clu_alu_op,
                    input  wire [2:0] funct3,
                    input  wire [6:0] funct7,
                    input  wire opcode_5thbit_add_sub, // TODO - Change the name from add_sub to R_I as this for that purpose
                    output wire [3:0]  o_alu_control_sel
                    );

assign o_alu_control_sel = 
    (i_clu_alu_op == 2'b00) ? 4'b0000 : // Forced Addition (S, U, J) - Here we can have I' and S as well
    (i_clu_alu_op == 2'b01) ?           // Forced Subtraction (B) - BEQ , BNE - This needs to be updated - Add a func3 check here
            ((funct3 == 3'b000) ? 4'b1000 : //beq 
             (funct3 == 3'b001) ? 4'b1000 : //bne
             (funct3 == 3'b100) ? 4'b1000 : //blt 
             (funct3 == 3'b101) ? 4'b1000 : //bge
             (funct3 == 3'b110) ? 4'b1001 : //bltu 
             (funct3 == 3'b111) ? 4'b1001 : //bgeu
             4'bxxxx) :
    (i_clu_alu_op == 2'b10) ? (
            (funct3 == 3'b000) ? (opcode_5thbit_add_sub? ((funct7[5]) ? 4'b0001 : 4'b0000) : 4'b0000) : // sub/add
            (funct3 == 3'b001) ? 4'b0101 : // sll
            (funct3 == 3'b010) ? 4'b1000 : // slt //BLT
            (funct3 == 3'b011) ? 4'b1001 : // sltu
            (funct3 == 3'b100) ? 4'b0100 : // xor
            (funct3 == 3'b101) ? ((funct7[5]) ? 4'b0111 : 4'b0110) : // sra/srl
            (funct3 == 3'b110) ? 4'b0011 : // or
            (funct3 == 3'b111) ? 4'b0010 : // and
            4'bxxxx) :
    (i_clu_alu_op == 2'b11) ? 4'bxxxx : 4'bxxxx;

endmodule

/////////////////////////// HART MODULE ///////////////////////////
module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word. This
    // is not representative of a realistic memory interface; it has been
    // modeled as more similar to a DFF or SRAM to simplify phase 3. In
    // later phases, you will replace this with a more realistic memory.
    //
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,
    // Instruction word fetched from memory, available on the same cycle.
    input  wire [31:0] i_imem_rdata,
    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle. Reads are combinational - values are available immediately after
    // updating the address and asserting read enable. Writes occur on (and
    // are visible at) the next clock edge.
    //
    // Read/write address for the data memory. This should be 32-bit aligned
    // (i.e. the two LSB should be zero). See `o_dmem_mask` for how to perform
    // half-word and byte accesses at unaligned addresses.
    output wire [31:0] o_dmem_addr,
    // When asserted, the memory will perform a read at the aligned address
    // specified by `i_addr` and return the 32-bit word at that address
    // immediately (i.e. combinationally). It is illegal to assert this and
    // `o_dmem_wen` on the same cycle.
    output wire        o_dmem_ren,
    // When asserted, the memory will perform a write to the aligned address
    // `o_dmem_addr`. When asserted, the memory will write the bytes in
    // `o_dmem_wdata` (specified by the mask) to memory at the specified
    // address on the next rising clock edge. It is illegal to assert this and
    // `o_dmem_ren` on the same cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned address at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    //
    // To perform a half-word read at address 0x00001002, align `o_dmem_addr`
    // to 0x00001000, assert `o_dmem_ren`, and set the mask to 0b1100 to
    // indicate that only the upper two bytes should be read. Only the upper
    // two bytes of `i_dmem_rdata` can be assumed to have valid data; to
    // calculate the final value of the `lh[u]` instruction, shift the rdata
    // word right by 16 bits and sign/zero extend as appropriate.
    //
    // To perform a byte write at address 0x00002003, align `o_dmem_addr` to
    // `0x00002000`, assert `o_dmem_wen`, and set the mask to 0b1000 to
    // indicate that only the upper byte should be written. On the next clock
    // cycle, the upper byte of `o_dmem_wdata` will be written to memory, with
    // the other three bytes of the aligned word unaffected. Remember to shift
    // the value of the `sb` instruction left by 24 bits to place it in the
    // appropriate byte lane.
    output wire [ 3:0] o_dmem_mask,
    // The 32-bit word read from data memory. When `o_dmem_ren` is asserted,
    // this will immediately reflect the contents of memory at the specified
    // address, for the bytes enabled by the mask. When read enable is not
    // asserted, or for bytes not set in the mask, the value is undefined.
    input  wire [31:0] i_dmem_rdata,
	// The output `retire` interface is used to signal to the testbench that
    // the CPU has completed and retired an instruction. A single cycle
    // implementation will assert this every cycle; however, a pipelined
    // implementation that needs to stall (due to internal hazards or waiting
    // on memory accesses) will not assert the signal on cycles where the
    // instruction in the writeback stage is not retiring.
    //
    // Asserted when an instruction is being retired this cycle. If this is
    // not asserted, the other retire signals are ignored and may be left invalid.
    output wire        o_retire_valid,
    // The 32 bit instruction word of the instrution being retired. This
    // should be the unmodified instruction word fetched from instruction
    // memory.
    output wire [31:0] o_retire_inst,
    // Asserted if the instruction produced a trap, due to an illegal
    // instruction, unaligned data memory access, or unaligned instruction
    // address on a taken branch or jump.
    output wire        o_retire_trap,
    // Asserted if the instruction is an `ebreak` instruction used to halt the
    // processor. This is used for debugging and testing purposes to end
    // a program.
    output wire        o_retire_halt,
    // The first register address read by the instruction being retired. If
    // the instruction does not read from a register (like `lui`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs1_raddr,
    // The second register address read by the instruction being retired. If
    // the instruction does not read from a second register (like `addi`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs2_raddr,
    // The first source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs1 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs1_rdata,
    // The second source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs2 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs2_rdata,
    // The destination register address written by the instruction being
    // retired. If the instruction does not write to a register (like `sw`),
    // this should be 5'd0.
    output wire [ 4:0] o_retire_rd_waddr,
    // The destination register data written to the register file in the
    // writeback stage by this instruction. If rd is 5'd0, this field is
    // ignored and can be treated as a don't care.
    output wire [31:0] o_retire_rd_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc,
    ////////////Added the below new signals for Project Phase 5////////////
    // The following data memory retire interface is used to record the
    // memory transactions completed by the instruction being retired.
    // As such, it mirrors the transactions happening on the main data
    // memory interface (o_dmem_* and i_dmem_*) but is delayed to match
    // the retirement of the instruction. You can hook this up by just
    // registering the main dmem interface signals into the writeback
    // stage of your pipeline.
    // All these fields are don't-care for instructions that do not
    // access data memory (o_retire_dmem_ren and o_retire_dmem_wen
    // not asserted).
    // NOTE: This interface is new for phase 5 in order to account for
    // the delay between data memory accesses and instruction retire.
    //
    // The 32-bit data memory address accessed by the instruction.
    output wire [31:0] o_retire_dmem_addr,
    // The byte masked used for the data memory access.
    output wire [ 3:0] o_retire_dmem_mask,
    // Asserted if the instruction performed a read (load) from data memory.
    output wire        o_retire_dmem_ren,
    // Asserted if the instruction performed a write (store) to data memory.
    output wire        o_retire_dmem_wen,
    // The 32-bit data read from memory by a load instruction.
    output wire [31:0] o_retire_dmem_rdata,
    // The 32-bit data written to memory by a store instruction.
    output wire [31:0] o_retire_dmem_wdata

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS,
`endif
);

// Wire declarations
wire [5:0] i_imm_format;
wire [31:0] t_rs2_rdata;
wire [31:0] i_dmem_alu_muxout_data;
wire [31:0] o_rs1_rdata;
wire [31:0] rs2_rdata_imm_mux_data;
wire [3:0] o_alu_control_sel;
wire [1:0] t_clu_ALUSrc;
wire t_clu_MemtoReg, t_clu_branch;
wire t_clu_halt;
wire [31:0] PC_current_val;
wire [31:0] t_lui_auipc_mux_data;
wire [1:0]t_clu_lui_auipc_mux_sel;
wire [2:0]t_sign_or_zero_ext_data_mux;
wire [1:0] t_clu_branch_instr_alu_sel;
wire [1:0]t_clu_alu_op;
wire [31:0] i_dmem_rdata_sign_or_zero_ext_mux_data;
wire t_rd_wen;
wire [31:0] t_pc_plus_4; 
wire [31:0] t_pc_o_rs1_data_mux_pcaddr;
wire t_clu_pc_o_rs1_data_mux_sel;
wire t_alu_o_Zero;
wire t_dmem_wen;
wire [3:0]t_dmem_mask;
wire t_dmem_ren;
wire [2:0]t_clu_ld_st_type_sel;
wire [31:0] t_immediate_out_data;
wire t_pc_write_en;
wire [4:0] t_i_rd_waddr;
wire t_i_rd_wen;
wire [31:0] t_pc_o_rs1_data_mux_imm_add_data; // New wire added for the imm and PC added value which is passed through EX/MEM Pipieline register
wire t_alu_o_Zero_clu_Branch_and; // NEW INPUT ADDED FOR THE ANDing in MEM stage
wire [31:0] t_pc_o_rs1_data_mux_imm_add_EX_stage_data; //TODO - Should go to EX/MEM Pipeline Register
wire [31:0] f_rs2_rdata;
wire [31:0] t_o_alu_result; // Getting the Alu result before pipelining
wire [31:0] fwd_ID_EX_o_rs1_rdata;
wire [31:0] fwd_ID_EX_o_rs2_rdata;
wire [4:0] t_rd_waddr_EXMEM;
wire [4:0] t_rd_waddr_MEMWB;
wire t_rd_wen_EXMEM;
wire t_rd_wen_MEMWB;
wire t_dmem_wen_forwarding;
wire t_dmem_ren_forwarding;
wire [1:0]t_forward_A;
wire [1:0]t_forward_B;
wire t_forward_store;
wire [31:0] fwd_EX_MEM_o_alu_result;
wire [31:0] fwd_muxout_Adata;
wire [31:0] fwd_muxout_Bdata;
wire [31:0] t_o_next_pc; //Added for o_nextpc
wire t_id_ex_alu_o_Zero_clu_Branch_and;
wire IF_ID_write_en;
wire IF_ID_flush;
wire [63:0]IF_ID_temp;
wire t_predict_taken;      // Output from BHT
wire t_mispredict;         // Output from HCU
wire [197:0]ID_EX_temp;

// Logic to calculate Recovery Address in EX stage
// If Predict=Taken (1) but Actual=NotTaken (0) -> We should have gone to PC+4 (of that branch)
// If Predict=NotTaken (0) but Actual=Taken (1) -> We should have gone to Target (PC+Imm)
wire [31:0] t_recovery_pc;
assign t_recovery_pc = (ID_EX_temp[197]) ? ID_EX_temp[175:144] : t_pc_o_rs1_data_mux_imm_add_EX_stage_data;
// ID_EX[197] is the propogated prediction bit

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////MODULE INSTANTIATIONS AND PIPLEINE DEFINITIONS START HERE//////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////###Fetch Module Section###///////
fetch fetch_inst(
    .clk(i_clk),
    .rst(i_rst),
    .i_clu_halt(t_clu_halt), // Halt is received immediately from the control unit (No flop)
    .i_pc_write_en(t_pc_write_en),
    .i_mispredict(t_mispredict),          // From Hazard Unit
    .i_recovery_addr(t_recovery_pc),      // Calculated recovery logic
    .PC(PC_current_val),
    .o_pc_plus_4(t_pc_plus_4),
    .o_instr_mem_rd_addr(o_imem_raddr),
    .o_next_pc(t_o_next_pc)
);


branch_predictor bht_inst (
    .clk(i_clk),
    .rst(i_rst),
    .i_fetch_pc(PC_current_val),
    .i_ex_pc(ID_EX_temp[143:112]),              // PC stored in ID_EX pipeline
    .i_ex_branch_was_taken(t_alu_o_Zero_clu_Branch_and), // Actual outcome in EX
    .i_ex_is_branch_instr(ID_EX_temp[184]),     // t_clu_branch signal in ID_EX
    .o_predict_taken(t_predict_taken)
);
//////////////////////////////////////////////////////////////
//Introducing Valid signal for Invalidating instructions//////
//////////////////////////////////////////////////////////////
reg f_valid;
wire reg_valid;
always @ (posedge i_clk) begin
    if (i_rst)
        f_valid <= 1'b0;
    else
        f_valid <= 1'b1;
end
assign reg_valid = f_valid & !IF_ID_flush;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////*IF_ID Pipeline Register Implementation*/////////////////////////////////////////////////////////////////////
/////////////////////////////////////{t_predict_taken, t_pc_plus_4[31:0], PC_current_val[31:0]}///////////////////////////////////////////////////////
/////////////////////////////////    {IF_ID[64]      ,  IF_ID[63:32],      IF_ID[31:0]         }//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//TODO : Enable Clear and ready
reg [64:0]IF_ID;
assign IF_ID_temp = {t_predict_taken, t_pc_plus_4[31:0],PC_current_val[31:0]};
always @ (posedge i_clk) begin
    if (i_rst)
            IF_ID <= 65'b0;
    else if (IF_ID_flush)
            IF_ID <= 65'b0;
    else if (IF_ID_write_en)
            IF_ID <= IF_ID_temp;
end
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg reg_IF_ID_flush;
always @ (posedge i_clk) begin
    if (i_rst)
        reg_IF_ID_flush <= 1'b0; 
    else
        reg_IF_ID_flush <= IF_ID_flush;
end
wire [31:0] hcu_flush_mux_imem_rdata;
assign hcu_flush_mux_imem_rdata = (reg_IF_ID_flush) ?  32'd0 : i_imem_rdata[31:0]; //Making sure we clear the instruction data going to the ID_EX Flush //TODO - GIVE HEXA of addi x0,x0,0 instead of 32'h0
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
wire [31:0] t_i_imem_to_rf_instr;
assign t_i_imem_to_rf_instr = hcu_flush_mux_imem_rdata; //NOT pipelining any more (To accomodate 1 clock cycle delay of the INSTR MEM)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////Register File Instance///////////////////////////////////////////////////////
rf #(.BYPASS_EN(1)) rf(
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_rs1_raddr(t_i_imem_to_rf_instr[19:15]), // Replaced it with the pipelined reg out instruction
    .i_rs2_raddr(t_i_imem_to_rf_instr[24:20]), // Replaced it with the pipelined reg out instruction
    .i_rd_waddr(t_i_rd_waddr), //Coming from MEM_WB Pipeline register
    .i_rd_wen(t_i_rd_wen), //Has to come from the MEM_WB pipeline register
    .i_rd_wdata(i_dmem_alu_muxout_data), // Coming from MEM_WB Pipeline register
    .o_rs1_rdata(o_rs1_rdata),
    .o_rs2_rdata(t_rs2_rdata), // This signal is going only to the Pipeline ID_EX
    .reg_valid(reg_valid)
);
/////////////////////////////////////////////////////////Control Unit Instance///////////////////////////////////////////////////////
control_unit control_unit_inst(
    .i_clu_inst(t_i_imem_to_rf_instr), // Replaced it with the pipelined reg out instruction
    .o_clu_Branch(t_clu_branch),
    .o_clu_halt(t_clu_halt),
    .o_clu_MemRead(t_dmem_ren),
    .o_clu_MemtoReg(t_clu_MemtoReg),
    .o_clu_ALUOp(t_clu_alu_op),
    .o_clu_MemWrite(t_dmem_wen),
    .o_clu_ALUSrc(t_clu_ALUSrc),
    .o_clu_RegWrite(t_rd_wen),
    .o_clu_lui_auipc_mux_sel(t_clu_lui_auipc_mux_sel),
    .o_clu_branch_instr_alu_sel(t_clu_branch_instr_alu_sel),
    .o_clu_pc_o_rs1_data_mux_sel(t_clu_pc_o_rs1_data_mux_sel),
    .o_clu_ld_st_type_sel(t_clu_ld_st_type_sel),
    .o_sign_or_zero_ext_data_mux(t_sign_or_zero_ext_data_mux),
    .reg_valid(reg_valid)
);
///////////////////////////////Immediate format decoding -- > Updated to do the Decoding in the ID stage ///////////////////////////////
assign i_imm_format =   !reg_valid ? 6'b0 :
    (t_i_imem_to_rf_instr[6:0] == 7'b0110011)? 6'b000001 : // R
    ((t_i_imem_to_rf_instr[6:0] == 7'b0010011)  || (t_i_imem_to_rf_instr[6:0] == 7'b1100111))? 6'b000010 : // I and Jalr
    (t_i_imem_to_rf_instr[6:0] == 7'b0000011)? 6'b000010 : // I (Load)
    (t_i_imem_to_rf_instr[6:0] == 7'b0100011)? 6'b000100 : // S
    (t_i_imem_to_rf_instr[6:0] == 7'b1100011)? 6'b001000 : // B
    ((t_i_imem_to_rf_instr[6:0] == 7'b0110111) || (t_i_imem_to_rf_instr[6:0] == 7'b0010111))? 6'b010000 : // U
    (t_i_imem_to_rf_instr[6:0] == 7'b1101111)? 6'b100000 : // Jal
    6'bXXXXXX;
//////////////////////////////////////////////////////////Immediate Generator Instance////////////////////////////////////////////////////
imm imm_decode_inst(
    .i_inst(t_i_imem_to_rf_instr), // Replaced it with the pipelined reg out instruction
    .i_format(i_imm_format),
    .o_immediate(t_immediate_out_data),
    .reg_valid(reg_valid)
);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////Pipeline enabled parallel to the ID_EX Pipeline Only to store the i_rs1_addr and i_rs2_addr///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////{ ID_EX_FWD_ADDR_PIPE[16:10]               ID_EX_FWD_ADDR_PIPE[9:5]                         ,  ID_EX_FWD_ADDR_PIPE[4:0]};///////////
///      { t_i_imem_to_rf_instr[6:0]/opcode[6:0]    i_rs2_addr[4:0]/t_i_imem_to_rf_instr[24:20]      ,  i_rs1_addr[4:0]/t_i_imem_to_rf_instr[19:15]}//////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg [16:0]ID_EX_FWD_ADDR_PIPE;
wire [16:0]ID_EX_fwd_addr_pipe_temp;
assign ID_EX_fwd_addr_pipe_temp = {t_i_imem_to_rf_instr[6:0],t_i_imem_to_rf_instr[24:20],t_i_imem_to_rf_instr[19:15]};//Instruction from IMEM is directly connected to next pipeline
always @ (posedge i_clk) begin
    if (i_rst)
            ID_EX_FWD_ADDR_PIPE <= 17'b0;
    else begin
            ID_EX_FWD_ADDR_PIPE <= ID_EX_fwd_addr_pipe_temp;
    end
end
///#################################################################################################################################################################################
////////////////#######################################*ID_EX Pipeline Register Implementation*#######################################//////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////Assigning a 20 bit Wire to group all the Control Signal together for pipelining///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//{ID_EX[197]         ,ID_EX[196]         , ID_EX[195], ID_EX[194],                 ID_EX[193], ID_EX[192], ID_EX[191:189],            ID_EX[188:186],                   ID_EX[185],     ID_EX[184],   ID_EX[183:182],                  ID_EX[181:180],    ID_EX[179:178],    ID_EX[177:176]||||/////////////
//{predictor_bit         ,  t_rd_wen,   t_clu_pc_o_rs1_data_mux_sel,t_dmem_wen, t_dmem_ren, t_clu_ld_st_type_sel[2:0], t_sign_or_zero_ext_data_mux[2:0], t_clu_MemtoReg, t_clu_branch, t_clu_branch_instr_alu_sel[1:0], t_clu_alu_op[1:0], t_clu_ALUSrc[1:0], t_clu_lui_auipc_mux_sel[1:0]}//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
wire [20:0] Control_input_ID_EX;
assign Control_input_ID_EX = {t_clu_halt,t_rd_wen,t_clu_pc_o_rs1_data_mux_sel,t_dmem_wen,t_dmem_ren,t_clu_ld_st_type_sel[2:0],t_sign_or_zero_ext_data_mux[2:0],t_clu_MemtoReg,
                             t_clu_branch,t_clu_branch_instr_alu_sel[1:0],t_clu_alu_op[1:0],t_clu_ALUSrc[1:0],t_clu_lui_auipc_mux_sel[1:0]};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////{ID_EX[197]     ,ID_EX[196:176],        ID_EX[175:144],        ID_EX[143:112],             ID_EX[111:80],      ID_EX[79:48],      ID_EX[47:41], ID_EX[40:38], ID_EX[37],    ID_EX[36:5],           ID_EX[4:0]};///////////
///      {t_predict_taken,Control Signals[20:0], t_pc_plus_4,           PC_current_val,             o_rs1_rdata[31:0],  t_rs2_rdata[31:0], func7,        func3,        opcode5thbit, t_immediate_out_data,  wr_addr[4:0]}//////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg [197:0]ID_EX;
wire ID_EX_flush;
assign ID_EX_temp = {IF_ID[64], Control_input_ID_EX[20:0],IF_ID[63:32],IF_ID[31:0],o_rs1_rdata[31:0],t_rs2_rdata[31:0],t_i_imem_to_rf_instr[31:25],t_i_imem_to_rf_instr[14:12],t_i_imem_to_rf_instr[5],t_immediate_out_data[31:0],t_i_imem_to_rf_instr[11:7]};//Instruction from IMEM is directly connected to next pipeline
always @ (posedge i_clk) begin
    if (i_rst)
        ID_EX <= 198'b0;
    else if (ID_EX_flush)
        ID_EX <= 198'b0;
    else
        ID_EX <= ID_EX_temp;
    end

wire [31:0] fwd_t_rs2_rdata; // This the STORE ONLY WRITE DATA COMING FROM RF RS2 Port all the way pipelined till DMEM
assign fwd_t_rs2_rdata  =   (t_forward_B == 2'b00 ) ? ID_EX[79:48] : //Default Condition - No Forwarding
                            (t_forward_B == 2'b01 || t_forward_B == 2'b10) ? fwd_muxout_Bdata : // Forwarding - Both (EX-EX and MEM- EX)
                            32'bxx;
assign rs2_rdata_imm_mux_data       = (ID_EX[179:178] == 2'b00) ? 32'b0 : 
                                      (ID_EX[179:178] == 2'b01) ? ID_EX[36:5]  : // t_immediate_out_data     - Updated to Reg Data from Pipileine ID_EX
                                      (ID_EX[179:178] == 2'b10) ? fwd_muxout_Bdata : // t_rs2_rdata          - Updated to Reg Data from Pipileine ID_EX
                                       fwd_muxout_Bdata;                             // t_rs2_rdata          - Updated to Reg Data from Pipileine ID_EX  
assign t_lui_auipc_mux_data         = (ID_EX[177:176] == 2'b00)? fwd_muxout_Adata :  //Default      //o_rs1_rdata     Updated to Reg Data from Pipileine ID_EX
                                      (ID_EX[177:176] == 2'b01)? 32'b0 :          //LUI     
                                      (ID_EX[177:176] == 2'b10)? ID_EX[143:112] : //AUIPC        //PC_current_val  Updated to Reg Data from Pipileine ID_EX
                                      (ID_EX[177:176] == 2'b11)? ID_EX[175:144] : //Jal, Jalr    //t_pc_plus_4     Updated to Reg Data from Pipileine ID_EX
                                      32'bx;   
assign t_pc_o_rs1_data_mux_pcaddr   = (ID_EX[194]) ? fwd_muxout_Adata :                                                        // o_rs1_rdata Updated to Reg Data from Pipeline ID_EX
                                       ID_EX[143:112];                                // Select o_rs1_data for jalr instruction else retain pc //PC_current_val  Updated to Reg Data from Pipeline ID_EX    

assign t_pc_o_rs1_data_mux_imm_add_EX_stage_data = t_pc_o_rs1_data_mux_pcaddr + ID_EX[36:5]; //Connected to the MEM_EX Pipeline
////////////////////////////////////////////////////////////////////////////////////////////////
assign t_id_ex_alu_o_Zero_clu_Branch_and = t_alu_o_Zero & ID_EX[184]; //NEW AND FOR CHU Branch prediction - In ID EX Stage - Now using the same for Actual Branch prediction to fetch
////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////ALU Control Instance///////////////////////////////////////// 
alu_control alu_control_inst( 
    .i_clu_alu_op(ID_EX[181:180]), // t_clu_alu_op
    .funct3(ID_EX[40:38]), // funct3 coming from the ID_EX Pipeline register
    .funct7(ID_EX[47:41]), // funct7 coming from the ID_EX Pipeline register
    .opcode_5thbit_add_sub(ID_EX[37]), // opcode5thbit coming from the ID_EX Pipeline register
    .o_alu_control_sel(o_alu_control_sel)
);
////////////////////////////////////////ALU Wrapper Instance/////////////////////////////////////////
alu_wrapper alu_wrapper_inst(
    .i_alu_ctrl_opsel(o_alu_control_sel),
    .i_rf_op1(t_lui_auipc_mux_data), //replaced it with Muxed out data from the 4:1 mux
    .i_rf_op2(rs2_rdata_imm_mux_data),
    .i_clu_branch_instr_alu_sel(ID_EX[183:182]), //t_clu_branch_instr_alu_sel
    .o_alu_result(t_o_alu_result), //Redefined a new temp wire for connecting it to MEM_EX pipeline below
    .o_alu_Zero(t_alu_o_Zero) // Being sent to the MEM_EX Pipeline
);
//////////////////////////////////Mask Generation in the EX Stage/////////////////////////////////////
assign t_dmem_mask =    ((ID_EX[191:189] == 3'b000) || (ID_EX[191:189] == 3'b011) || (ID_EX[191:189] == 3'b101)) ? ( // For lb, lbu, sb - BYTE
                            (t_o_alu_result[1:0] == 2'b00) ? 4'b0001 : // Eg. 2000
                            (t_o_alu_result[1:0] == 2'b01) ? 4'b0010 : // Eg. 2001
                            (t_o_alu_result[1:0] == 2'b10) ? 4'b0100 : // Eg. 2002
                            (t_o_alu_result[1:0] == 2'b11) ? 4'b1000 : // Eg. 2003
                            4'bxxxx)
                        :
                        ((ID_EX[191:189] == 3'b001) || (ID_EX[191:189] == 3'b100) || (ID_EX[191:189] == 3'b110)) ? ( // For lh, lhu, sh - HALF
                            (t_o_alu_result[1:0] == 2'b00) ? 4'b0011 : // Eg. 2000
                            // (t_o_alu_result[1:0] == 2'b01) Eg. 2001 - Not a valid case for half
                            (t_o_alu_result[1:0] == 2'b10) ? 4'b1100 : // Eg. 2002
                            // (t_o_alu_result[1:0] == 2'b11) Eg. 2003 - Not a valid case for half
                            4'bxxxx)
                        :
                        4'b1111;                      
/////////////#######################################*#####################################################################################################################////////
/////////////#######################################*EX_MEM Pipeline Register Implementation*#######################################//////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////Assigning a 9 bit Wire to group all the Control Signal together for EX_MEM pipeline/////////
//////////{t_clu_halt, t_sign_or_zero_ext_data_mux[2:0], t_rd_wen,    t_dmem_wen,  t_dmem_ren,  t_clu_MemtoReg, t_clu_branch}///////////////////
//////////{EX_MEM[114]        , EX_MEM[113:111],                  EX_MEM[110], EX_MEM[109], EX_MEM[108], EX_MEM[107],    EX_MEM[106]  }///////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
wire [8:0] Control_input_EX_MEM;
assign Control_input_EX_MEM = {ID_EX[196],ID_EX[188:186],ID_EX[195],ID_EX[193],ID_EX[192],ID_EX[185],ID_EX[184]}; // Mapped to the Signals as above description
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////|||{EX_MEM[114:106],             EX_MEM[105:74],                                EX_MEM[73],    EX_MEM[72:69],            EX_MEM[68:37],         EX_MEM[36:5],       EX_MEM[4:0]}|||||///
///      |||{Control_input_EX_MEM[8:0], t_pc_o_rs1_data_mux_imm_add_EX_stage_data[31:0], t_alu_o_Zero , t_dmem_mask[3:0]          t_o_alu_result[31:0],  t_rs2_rdata[31:0],  wr_addr[4:0]|||||///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg [114:0]EX_MEM;
wire [114:0]EX_MEM_temp;
assign EX_MEM_temp = {Control_input_EX_MEM[8:0],t_pc_o_rs1_data_mux_imm_add_EX_stage_data[31:0],t_alu_o_Zero,t_dmem_mask[3:0],t_o_alu_result[31:0],fwd_t_rs2_rdata[31:0],ID_EX[4:0]};//Updated f_rs2_data to a Muxed out data from the forwarded data from EX
always @ (posedge i_clk) begin
    if (i_rst)
            EX_MEM <= 115'b0;
    else begin
            EX_MEM <= EX_MEM_temp;
    end
end
////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////AND Logic implemented in MEM Stage/////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
assign  t_alu_o_Zero_clu_Branch_and = EX_MEM[73] & EX_MEM[106]; // AND of t_clu_branch and t_alu_o_Zero
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////!!!!!!!!!!IMPORTANT!!!!!!!!!!!!!!!////////////////
//assign t_pc_o_rs1_data_mux_imm_add_data =  EX_MEM[105:74]; // The PC + imm value executed in EX is Pipelined to MEM and is now being sent to Fetch (For Branch and Jumps)
assign t_pc_o_rs1_data_mux_imm_add_data =  t_pc_o_rs1_data_mux_imm_add_EX_stage_data; // The PC + imm value executed in EX is Pipelined to MEM and is now being sent to Fetch (For Branch and Jumps)
//// Avoiding the Flopping from ID_EX to EX_MEM before giving it to fetch
////////////////////////////////////////////////////////////////////////////////////////////////
///////////Muxes - Changing the assignments to receive the data from MEM_EX pipeline///////////
assign o_dmem_wen = EX_MEM[109];
assign o_dmem_ren = EX_MEM[108];
assign o_dmem_addr = (EX_MEM[109] || EX_MEM[108]) ? {EX_MEM[68:39],2'b0} : EX_MEM[68:37]; //Only when w_en or ren is set we are to align the addresses

assign f_rs2_rdata  = EX_MEM[36:5]; // The Pipelined o_rs2_data coming from REG for STORE instruction
// For store instructions only 
assign o_dmem_wdata =   (EX_MEM[72:69] == 4'b0001) ? {{24{f_rs2_rdata[7]}},f_rs2_rdata[7:0]} :                   // Applicable for sb
                        (EX_MEM[72:69] == 4'b0010) ? {{16{f_rs2_rdata[15]}},f_rs2_rdata[15:8],8'b0} :             // Applicable for sb
                        (EX_MEM[72:69] == 4'b0100) ? {{8{f_rs2_rdata[23]}},f_rs2_rdata[23:16],16'b0} :            // Applicable for sb
                        (EX_MEM[72:69] == 4'b1000) ? {f_rs2_rdata[7:0],24'b0} :                   // Applicable for sb
                        (EX_MEM[72:69] == 4'b0011) ? {{16{f_rs2_rdata[15]}},f_rs2_rdata[15:0]} :  // Applicable for sh - SIGN EXTENSION by default
                        (EX_MEM[72:69] == 4'b1100) ? {f_rs2_rdata[15:0],16'b0} :                  // Applicable for sh
                        (EX_MEM[72:69] == 4'b1111) ? f_rs2_rdata :  
                        32'bxxxx;                                                               // Default case

//MASK Implementation
assign o_dmem_mask = EX_MEM[72:69]; // Changed it to value coming from MEM_EX Pipeline - t_dmem_mask
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////############################################################################################################################################################////////
////////////#######################################*MEM_WB Pipeline Register Implementation*#######################################//////////////////////////////////////////
/////Assigning a 78 bit Wire to group all the RETIRE Signals together for MEM_WB pipeline/////////
///////////###{t_sign_or_zero_ext_data_mux[2:0], t_dmem_mask[3:0],  t_clu_halt         ,  o_dmem_ren,  o_dmem_wen,  o_dmem_mask[3:0], o_dmem_addr[31:0], o_dmem_wdata[31:0]}####////////////
/////////#####{        MEM_WB[116:114]         , MEM_WB[113:110] ,  MEM_WB[109]        ,  MEM_WB[108], MEM_WB[107], MEM_WB[106:103],  MEM_WB[102:71],   MEM_WB[70:39]    }#######////////////
wire [77:0] retire_bus;
assign retire_bus = {EX_MEM[113:111],EX_MEM[72:69],EX_MEM[114],o_dmem_ren,o_dmem_wen,o_dmem_mask[3:0],o_dmem_addr[31:0],o_dmem_wdata[31:0]};
/////////////////////////////////////////////////////////////////////////////////////////////////
/////Assigning a 2 bit Wire to group all the Control Signal together for MEM_WB pipeline/////////
//////////{      t_rd_wen,         t_clu_MemtoReg  }/////////////////////////////////////////////
//////////{ (MEM_WB[38]/EX_MEM[110], MEM_WB[37]/EX_MEM[107]) }/////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
wire [1:0] Control_input_MEM_WB;
assign Control_input_MEM_WB = {EX_MEM[110],EX_MEM[107]}; // Mapped to the Signals as in above description
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////|||{MEM_WB[109:39]    MEM_WB[38:37],               MEM_WB[36:5],                      ,        MEM_WB[4:0]}|||||//////////////////
///      |||{retire_bus[70:0], Control_input_MEM_WB[1:0],  (EX_MEM[68:37]/t_o_alu_result[31:0]),  (EX_MEM[4:0]/wr_addr[4:0])|||||//////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg [116:0]MEM_WB;
wire [116:0]MEM_WB_temp;
assign MEM_WB_temp = {retire_bus[77:0],Control_input_MEM_WB[1:0],EX_MEM[68:37],EX_MEM[4:0]};
always @ (posedge i_clk) begin
    if (i_rst)
            MEM_WB <= 117'b0;
    else begin
            MEM_WB <= MEM_WB_temp;
    end
end
//For Load instructions only --> Muxing shifted to the MEM_WB stage - But i_dmem_rdata is still not flopped
assign  i_dmem_rdata_sign_or_zero_ext_mux_data  =   (MEM_WB[113:110] == 4'b0001) && (MEM_WB[116:114] == 3'b001) ?  {{24{i_dmem_rdata[7]}},i_dmem_rdata[7:0]}         :   // lb and MASK = 4'b0001
                                                    (MEM_WB[113:110] == 4'b0001) && (MEM_WB[116:114] == 3'b000) ?  {24'b0,i_dmem_rdata[7:0]}                         :   // lbu and MASK = 4'b0001
                                                    (MEM_WB[113:110] == 4'b0010) && (MEM_WB[116:114] == 3'b001) ?  {{16{i_dmem_rdata[15]}},i_dmem_rdata[15:8],8'b0}  :   // lb and MASK = 4'b0010 - sign ext
                                                    (MEM_WB[113:110] == 4'b0010) && (MEM_WB[116:114] == 3'b000) ?  {16'b0,i_dmem_rdata[15:8],8'b0}                   :   // lbu and MASK = 4'b0010
                                                    (MEM_WB[113:110] == 4'b0100) && (MEM_WB[116:114] == 3'b001) ?  {{8{i_dmem_rdata[23]}},i_dmem_rdata[23:16],16'b0} :   // lb and MASK = 4'b0100 - sign ext
                                                    (MEM_WB[113:110] == 4'b0100) && (MEM_WB[116:114] == 3'b000) ?  {8'b0,i_dmem_rdata[23:16],16'b0}                  :   // lbu and MASK = 4'b0100
                                                    (MEM_WB[113:110] == 4'b1000) && (MEM_WB[116:114] == 3'b001) ?  {i_dmem_rdata[31:24],24'b0}                       :   // lb and MASK = 4'b1000
                                                    (MEM_WB[113:110] == 4'b1000) && (MEM_WB[116:114] == 3'b000) ?  {i_dmem_rdata[31:24],24'b0}                       :   // lbu and MASK = 4'b1000
                                                    (MEM_WB[113:110] == 4'b0011) && (MEM_WB[116:114] == 3'b011) ?  {{16{i_dmem_rdata[15]}},i_dmem_rdata[15:0]}       :   // lh and MASK = 4'b0011
                                                    (MEM_WB[113:110] == 4'b0011) && (MEM_WB[116:114] == 3'b010) ?  {16'b0,i_dmem_rdata[15:0]}                        :   // lhu and MASK = 4'b0011
                                                    (MEM_WB[113:110] == 4'b1100) && (MEM_WB[116:114] == 3'b011) ?  {{16{i_dmem_rdata[31]}},i_dmem_rdata[31:16]}      :   // lh and MASK = 4'b1100
                                                    (MEM_WB[113:110] == 4'b1100) && (MEM_WB[116:114] == 3'b010) ?  {16'b0,i_dmem_rdata[31:16]}                       :   // lhu and MASK = 4'b1100
                                                    (MEM_WB[113:110] == 4'b1111) && (MEM_WB[116:114] == 3'b100) ?  i_dmem_rdata                                      :   // lw and MASK = 4'b1111
                                                                                //i_dmem_rdata ;
                                                                                32'bx ; //Should not happen
/////////////////THIS WILL BE IN THE LAST STAGE OF THE PIPELINE///////////////////////////
assign i_dmem_alu_muxout_data                     =    MEM_WB[37] ? i_dmem_rdata_sign_or_zero_ext_mux_data[31:0] : MEM_WB[36:5]; // The Data going to be written to the Register - Either the Alu result or Data Read from Memory in case of Load instruction
assign t_i_rd_waddr                               =    MEM_WB[4:0]; // Pipeline Register Write Address (Source Instruction)
assign t_i_rd_wen                                 =    MEM_WB[38]; // Pipelined Register Write Enable  (Source Control Unit)
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////Forwarding Unit Instance wires//////////////////////////
assign fwd_EX_MEM_o_alu_result = EX_MEM[68:37];
assign t_rd_waddr_EXMEM = EX_MEM[4:0]; // Change it to ID
assign t_rd_waddr_MEMWB = MEM_WB[4:0]; // Change it to EX_MEM
assign t_rd_wen_EXMEM = EX_MEM[110]; // Change it to ID
assign t_rd_wen_MEMWB =  MEM_WB[38]; // Change it to EX_MEM
assign fwd_ID_EX_o_rs1_rdata = ID_EX[111:80];
assign fwd_ID_EX_o_rs2_rdata = ID_EX[79:48];
assign t_dmem_wen_forwarding = EX_MEM[109]; 
assign t_dmem_ren_forwarding =  EX_MEM[108]; 
//////////////////////////////////////FORWARDING UNIT INSTANCE///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
forwarding_unit forwarding_inst(
    .i_rs1_IDEX_addr(ID_EX_FWD_ADDR_PIPE[4:0]),
    .i_rs2_IDEX_addr(ID_EX_FWD_ADDR_PIPE[9:5]),
    .i_rd_waddr_EXMEM(t_rd_waddr_EXMEM),
    .i_rd_waddr_MEMWB(t_rd_waddr_MEMWB),
    .i_clu_RegWrite_EXMEM(t_rd_wen_EXMEM),
    .i_clu_RegWrite_MEMWB(t_rd_wen_MEMWB),
    .i_clu_MemWrite_EXMEM(t_dmem_wen_forwarding), //Only for MEM to MEM
    .i_clu_MemRead_EXMEM(t_dmem_ren_forwarding),  //Only for MEM to MEM
    .i_opcode(ID_EX_FWD_ADDR_PIPE[16:10]), //Added Opcode
    .o_forward_A(t_forward_A), 
    .o_forward_B(t_forward_B),
    .o_forward_store(t_forward_store) 
);
//////////////////////////Muxed Out Data after forwarding////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
assign   fwd_muxout_Adata      =    (t_forward_A == 2'b00) ?  fwd_ID_EX_o_rs1_rdata :
                                    (t_forward_A == 2'b01) ?  i_dmem_alu_muxout_data:   //MEM to EX
                                    (t_forward_A == 2'b10) ?  fwd_EX_MEM_o_alu_result : //EX to EX
                                    fwd_ID_EX_o_rs1_rdata;

assign   fwd_muxout_Bdata      =    (t_forward_B == 2'b00) ?  fwd_ID_EX_o_rs2_rdata :
                                    (t_forward_B == 2'b01) ?  i_dmem_alu_muxout_data:   //MEM to EX
                                    (t_forward_B == 2'b10) ?  fwd_EX_MEM_o_alu_result : //EX to EX
                                    fwd_ID_EX_o_rs2_rdata;
////////////////////////////////////////////////////////////////////////////////////////////////
wire o_hcu_retire_valid; //TODO - Do we need this wire????
////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Hazard control Unit Instantiation //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
hazard_control_unit hazard_control_unit_inst (
    .i_hcu_branch (t_clu_branch), //
    .i_hcu_lui_auipc_mux_sel (t_clu_lui_auipc_mux_sel), 
    .i_hcu_IF_ID_PC_current_val(IF_ID[31:0]),
    .i_hcu_ID_EX_MemRead(ID_EX[192]),  //memRead - EX_MEM[108]
    .i_hcu_ID_EX_rd(ID_EX[4:0]),  //rd_addr from ID_EX - EX_MEM[4:0]
    .i_hcu_IF_ID_rs1(t_i_imem_to_rf_instr[19:15]), //Unflopped rs1_addr -  ID_EX_FWD_ADDR_PIPE[4:0]
    .i_hcu_IF_ID_rs2(t_i_imem_to_rf_instr[24:20]), //Unflopped rs2_addr -  ID_EX_FWD_ADDR_PIPE[9:5]
    .i_hcu_IF_ID_MemWrite(t_dmem_wen), // Should we change to the ID_EX signal? - Can be removed
    .i_ex_prediction_bit(ID_EX[197]),           // The pipelined prediction bit
    .i_ex_actual_outcome(t_id_ex_alu_o_Zero_clu_Branch_and), // Actual result
    .i_ex_is_branch(ID_EX[184]),                // Is it a branch instr?
    .o_ex_mispredict_detected(t_mispredict),    // Connect to wire t_mispredict
    .o_hcu_IF_ID_flush(IF_ID_flush), 
    .o_hcu_ID_EX_flush(ID_EX_flush),
    .o_hcu_PCWriteEn(t_pc_write_en),
    .o_hcu_IF_ID_write_en(IF_ID_write_en),
    .i_hcu_IF_ID_opcode(t_i_imem_to_rf_instr[6:0]), //Unflopped opcode from instruction - ID_EX_FWD_ADDR_PIPE[16:10]
    .o_hcu_retire_valid(o_hcu_retire_valid),
    .i_hcu_branch_predictor(t_id_ex_alu_o_Zero_clu_Branch_and) // ANDED value from MEM Stage given to HCU for Control Hazards t_alu_o_Zero_clu_Branch_and--> t_id_ex_alu_o_Zero_clu_Branch_and
);
////////////////////////////////////////////////////////////////////////////////////////////////
//////////////Unmasked Store Data flopping for retire purposes//////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
reg [31:0]reg_MEM_WB_f_rs2_rdata;
always @ (posedge i_clk) begin
    if (i_rst)
            reg_MEM_WB_f_rs2_rdata <= 32'b0;
    else begin
            reg_MEM_WB_f_rs2_rdata <= f_rs2_rdata;
    end
end
////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Valid Signal generated at  IF ID Stage and Pipelined till MEM_WB////////
////////////////////////////////////////////////////////////////////////////////////////////////
   reg IF_ID_valid;
    always @(posedge i_clk) begin
        if(i_rst) begin
            IF_ID_valid <= 1'b0; // PRESET = 1
        end
        else if (IF_ID_flush)
            IF_ID_valid <= 1'b0;
        else begin
            IF_ID_valid <= 1'b1;
        end
    end
    reg ID_EX_o_retire_valid;
    always @ (posedge i_clk) begin
        if (i_rst)
                ID_EX_o_retire_valid <= 1'b0;
        else if (ID_EX_flush)
           ID_EX_o_retire_valid <= 1'b0;
        else begin
                ID_EX_o_retire_valid <= IF_ID_valid;
        end
    end
    reg EX_MEM_o_retire_valid;
    always @ (posedge i_clk) begin
        if (i_rst)
                EX_MEM_o_retire_valid <= 1'b0;
        else begin
                EX_MEM_o_retire_valid <= ID_EX_o_retire_valid;
        end
    end
    reg MEM_WB_o_retire_valid;
    always @ (posedge i_clk) begin
        if (i_rst)
                MEM_WB_o_retire_valid <= 1'b0;
        else begin
                MEM_WB_o_retire_valid <= EX_MEM_o_retire_valid;
        end
    end
////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////instruction pipelined from ID_EX to MEM_WB for retiring////////
////////////////////////////////////////////////////////////////////////////////////////////////
    reg [31:0]ID_EX_o_retire_inst;
    always @ (posedge i_clk) begin
        if (i_rst)
                ID_EX_o_retire_inst <= 32'b0;
        else begin
                ID_EX_o_retire_inst <= t_i_imem_to_rf_instr[31:0];
        end
    end
    reg [31:0]EX_MEM_o_retire_inst;
    always @ (posedge i_clk) begin
        if (i_rst)
                EX_MEM_o_retire_inst <= 32'b0;
        else begin
                EX_MEM_o_retire_inst <= ID_EX_o_retire_inst;
        end
    end
    reg [31:0]MEM_WB_o_retire_inst;
    always @ (posedge i_clk) begin
        if (i_rst)
                MEM_WB_o_retire_inst <= 32'b0;
        else begin
                MEM_WB_o_retire_inst <= EX_MEM_o_retire_inst;
        end
    end
////////////////////////////////////////////////////////////////////////////////////////////////
/////nextpc for IF and ID///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//t_o_next_pc;
   reg [31:0]IF_ID_o_next_pc;
    always @(posedge i_clk) begin
        if(i_rst) begin
            IF_ID_o_next_pc <= 32'b0;
        end
        else begin
            IF_ID_o_next_pc <= t_o_next_pc;
        end
    end
    reg [31:0]ID_EX_o_next_pc;
    always @ (posedge i_clk) begin
        if (i_rst)
                ID_EX_o_next_pc <= 32'b0;
        else begin
                ID_EX_o_next_pc <= IF_ID_o_next_pc;
        end
    end
////////////////////////////////////////////////////////////////////////////////////////////////
/////pc and nextpc///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
    reg [63:0]EX_MEM_o_retire_pc_pc4;
    always @ (posedge i_clk) begin
        if (i_rst)
                EX_MEM_o_retire_pc_pc4 <= 64'b0;
        else begin
                EX_MEM_o_retire_pc_pc4 <= {ID_EX_o_next_pc,ID_EX[143:112]}; //nextpc and PC
        end
    end

    reg [63:0]MEM_WB_o_retire_pc_pc4;
    always @ (posedge i_clk) begin
        if (i_rst)
                MEM_WB_o_retire_pc_pc4 <= 64'b0;
        else begin
                MEM_WB_o_retire_pc_pc4 <= EX_MEM_o_retire_pc_pc4;
        end
    end
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
    //ID_EX[111:80],      ID_EX[79:48],    
    //o_rs1_rdata[31:0],  t_rs2_rdata[31:0]
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
    reg [63:0]EX_MEM_o_retire_o_rs1_rs2_data;
    always @ (posedge i_clk) begin
        if (i_rst)
                EX_MEM_o_retire_o_rs1_rs2_data <= 64'b0;
        else begin
                //EX_MEM_o_retire_o_rs1_rs2_data <= {ID_EX[111:80],ID_EX[79:48]}; // To be changed to ALU operand inouts and outputs
                EX_MEM_o_retire_o_rs1_rs2_data <= {rs2_rdata_imm_mux_data,t_lui_auipc_mux_data}; // To be changed to ALU operand inouts and outputs
        end
    end

    reg [63:0]MEM_WB_o_retire_o_rs1_rs2_data;
    always @ (posedge i_clk) begin
        if (i_rst)
                MEM_WB_o_retire_o_rs1_rs2_data <= 64'b0;
        else begin
                MEM_WB_o_retire_o_rs1_rs2_data <= EX_MEM_o_retire_o_rs1_rs2_data;
        end
    end
///////////////////////////////////////////////////////////////////
//////////////////////////RETIRE SIGNALS///////////////////////////
///////////////////////////////////////////////////////////////////
assign o_retire_valid     =   MEM_WB_o_retire_valid;
assign o_retire_rs1_raddr =   MEM_WB_o_retire_inst[19:15];         
assign o_retire_rs1_rdata =   MEM_WB_o_retire_o_rs1_rs2_data[31:0]; //rs1_data         
assign o_retire_rs2_raddr =   MEM_WB_o_retire_inst[24:20];  
assign o_retire_pc        =   MEM_WB_o_retire_pc_pc4[31:0];                 
assign o_retire_next_pc   =   MEM_WB_o_retire_pc_pc4[63:32];
assign o_retire_inst      =   MEM_WB_o_retire_inst[31:0];
assign o_retire_halt      =   MEM_WB[109]; // Pipelined the halt from MCU all the way to the last pipeline and to the Design Boundary
assign o_retire_trap      =   0; //Temporary assignment - Need to be modified //TODO          
assign o_retire_rd_waddr  =   t_i_rd_wen?MEM_WB[4:0]:5'b0;
assign o_retire_rd_wdata  =   i_dmem_alu_muxout_data;         
assign o_retire_rs2_rdata =   o_retire_dmem_wen?  reg_MEM_WB_f_rs2_rdata : MEM_WB_o_retire_o_rs1_rs2_data[63:32];  //Retire Data for rs2_data can come also from the STORE TYPE where we have a seperate connection all the way to WB
////////////////////////////////////////////////////////////////////////////////////////////////
//New Phase-5 Wires being assigned from the MEM_WB Pipeline register as instructed in the Phase 5 Specification
// The 32-bit data memory address accessed by the instruction.
assign o_retire_dmem_addr = MEM_WB[102:71];
// The byte masked used for the data memory access.
assign o_retire_dmem_mask = MEM_WB[106:103];
// Asserted if the instruction performed a read (load) from data memory.
assign o_retire_dmem_ren = MEM_WB[108];
// Asserted if the instruction performed a write (store) to data memory.
assign o_retire_dmem_wen = MEM_WB[107];
// The 32-bit data written to memory by a store instruction.
assign o_retire_dmem_wdata = MEM_WB[70:39] ;
// The 32-bit data read from memory by a load instruction.
assign o_retire_dmem_rdata = i_dmem_rdata[31:0]; // Picking the Raw data for dmem LOAD o_retire

endmodule