/////////////////////////////////////////////////////////////
///////////////////////// FORWARDING UNIT////////////////////
/////////////////////////////////////////////////////////////

module forwarding_unit (
    input wire[4:0] i_rs1_IDEX_data,
    input wire[4:0] i_rs2_IDEX_data,
    input wire[4:0] i_rd_EXMEM,
    input wire[4:0] i_rd_MEMWB,
    input wire i_clu_RegWrite_EXMEM,
    input wire i_clu_RegWrite_MEMWB,
    input wire i_clu_MemWrite_EXMEM,
    input wire i_clu_MemRead_EXMEM, // Doubt on whether its EXMEM or MEMWB. But does it matter?
    output wire[1:0] o_forward_A, 
    output wire[1:0] o_forward_B,
    output wire o_forward_store  //? We might need this solely for handling MEM-to-MEM
);

// Will these things hold up for Jal and branch things?
assign o_forward_A = ((i_clu_RegWrite_EXMEM == 1 && i_rd_EXMEM != 0) ? 
                            (i_rd_EXMEM == i_rs1_IDEX_data) ?  // EX-to-EX Forwarding for rs1
                                (2b'10) :
                      (i_clu_RegWrite_MEMWB == 1 && i_rd_MEMWB != 0) ?
                            (i_rd_MEMWB == i_rs1_IDEX_data) ?  // MEM-to-EX Forwarding for rs1
                                (2b'01))

assign o_forward_B = ((i_clu_RegWrite_EXMEM == 1 && i_rd_EXMEM != 0) ?
                            (i_rd_EXMEM == i_rs2_IDEX_data) ?  // EX-to-EX Forwarding for rs2
                                (2b'10):
                        (i_clu_RegWrite_MEMWB == 1 && i_rd_MEMWB != 0) ?
                            (i_rd_MEMWB == i_rs2_IDEX_data) ?  // MEM-to-EX Forwarding for rs2
                                (2b'01))

// MEM-to-MEM Forwarding
if (i_clu_MemRead_EXMEM == 1) && (i_clu_MemWrite_EXMEM == 1) begin
    o_forward_store = 1
    // This condition is incomplete. Need to check rs1 and rs2 data and other stuffs 🚶‍♂️.
    // TODO: Add conditions to prioritize EX/MEM over MEM/WB so that we get the latest data.
end


endmodule