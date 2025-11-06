/////////////////////////////////////////////////////////////
///////////////////////// FORWARDING UNIT////////////////////
/////////////////////////////////////////////////////////////

module forwarding_unit (
    input wire[4:0] i_rs1_IDEX_addr,
    input wire[4:0] i_rs2_IDEX_addr,
    input wire[4:0] i_rd_waddr_EXMEM,
    input wire[4:0] i_rd_waddr_MEMWB,
    input wire i_clu_RegWrite_EXMEM,
    input wire i_clu_RegWrite_MEMWB,
    input wire i_clu_MemWrite_EXMEM,
    input wire i_clu_MemRead_EXMEM, // Doubt on whether its EXMEM or MEMWB. But does it matter?
    input wire [6:0]i_opcode,
    output wire[1:0] o_forward_A, 
    output wire[1:0] o_forward_B,
    output wire o_forward_store  //? We might need this solely for handling MEM-to-MEM
);

// Will these things hold up for Jal and branch things?
///For RS1_data
assign o_forward_A =
    (i_clu_RegWrite_EXMEM && (i_rd_waddr_EXMEM != 0) && (i_rd_waddr_EXMEM == i_rs1_IDEX_addr)) ? 2'b10 : // EX-to-EX
    (i_clu_RegWrite_MEMWB && (i_rd_waddr_MEMWB != 0) && (i_rd_waddr_MEMWB == i_rs1_IDEX_addr)) ? 2'b01 : // MEM-to-EX
    2'b00; // No forwarding

//For RS2 data
assign o_forward_B = 
    ((i_clu_RegWrite_EXMEM == 1 && i_rd_waddr_EXMEM != 0) && (i_rd_waddr_EXMEM == i_rs2_IDEX_addr)) ?  (2'b10): // EX-to-EX Forwarding for rs2
    ((i_clu_RegWrite_MEMWB == 1 && i_rd_waddr_MEMWB != 0) && (i_rd_waddr_MEMWB == i_rs2_IDEX_addr)) ?  (2'b01):// MEM-to-EX Forwarding for rs2
    2'b00; // No forwarding

// MEM-to-MEM Forwarding
assign o_forward_store = ((i_clu_MemRead_EXMEM == 1) && (i_clu_MemWrite_EXMEM == 1)) ? 1 : 0;
    // This condition is incomplete. Need to check rs1 and rs2 data and other stuffs 🚶‍♂️.
    // TODO: Add conditions to prioritize EX/MEM over MEM/WB so that we get the latest data.

// // Will these things hold up for Jal and branch things?
// assign o_forward_A =
//     (i_clu_RegWrite_EXMEM && (i_rd_waddr_EXMEM != 0) && (i_rd_waddr_EXMEM == i_rs1_IDEX_addr)) ? 2'b10 : // EX-to-EX
//     (i_clu_RegWrite_MEMWB && (i_rd_waddr_MEMWB != 0) && (i_rd_waddr_MEMWB == i_rs1_IDEX_addr)) ? 2'b01 : // MEM-to-EX
//     2'b00; // No forwarding

// assign o_forward_B = 
//     ((i_clu_RegWrite_EXMEM == 1 && i_rd_waddr_EXMEM != 0) && (i_rd_waddr_EXMEM == i_rs2_IDEX_addr)) ?  (2'b10): // EX-to-EX Forwarding for rs2
//     ((i_clu_RegWrite_MEMWB == 1 && i_rd_waddr_MEMWB != 0) && (i_rd_waddr_MEMWB == i_rs2_IDEX_addr)) ?  (2'b01):// MEM-to-EX Forwarding for rs2
//     2'b00; // No forwarding

// // MEM-to-MEM Forwarding
// assign o_forward_store = ((i_clu_MemRead_EXMEM == 1) && (i_clu_MemWrite_EXMEM == 1)) ? 1 : 0;
//     // This condition is incomplete. Need to check rs1 and rs2 data and other stuffs 🚶‍♂️.
//     // TODO: Add conditions to prioritize EX/MEM over MEM/WB so that we get the latest data.
endmodule