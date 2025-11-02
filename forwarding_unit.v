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
    input wire i_clu_MemWrite,
    input wire i_clu_MemWrite,
    output wire[1:0] o_forward_A, 
    output wire[1:0] o_forward_B
);

assign o_forward_A = ((i_clu_RegWrite_EXMEM == 1 && i_rd_EXMEM != 0) ? 
                            (i_rd_EXMEM == i_rs1_IDEX_data) ?  
                                (2b'10) :
                      (i_clu_RegWrite_MEMWB == 1 && i_rd_MEMWB != 0) ?
                            (i_rd_MEMWB == i_rs1_IDEX_data) ?  
                                (2b'01))

assign o_forward_B = ((i_clu_RegWrite_EXMEM == 1 && i_rd_EXMEM != 0) ?
                            (i_rd_EXMEM == i_rs2_IDEX_data) ?
                                (2b'10):
                        (i_clu_RegWrite_MEMWB == 1 && i_rd_MEMWB != 0) ?
                            (i_rd_MEMWB == i_rs2_IDEX_data) ?  
                                (2b'01))

endmodule