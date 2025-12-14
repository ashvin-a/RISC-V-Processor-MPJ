/////////////////////////////////////////////////////////////////
///////////////////////// HAZARD CONTROL UNIT////////////////////
/////////////////////////////////////////////////////////////////

// Hazard control unit works at the decode stage for stalls (esp for load-use cases, etc) an at the exceute stage for flushes (eg branch and jump cases)
// None of the hazard signals should be flopped - their effects can be flopped and implemented in the next cycle

// Expected outputs: -    Stall when Load-Use case (except for when use is store instruction)
//                        Stall when there is as functional hazard_control_unit - When I stall I stall Decode and Fectch stage
//                        Flush when there is a Jump or there is a mispredicted branch
// Outputs controlled -
//                      For Stall:
//                             PCWriteEn = 0
//                             IF_ID_write_en = 0
//                             ID_EX_write_en = 0
//                     For Flush:
//                             IF_ID_flush = 0;
//                             ID_EX_flush = 0;
// Inputs Used to decide -
//                     For Control Hazard:
//                             ID_EX_o_clu_lui_auipc_mux_sel  - Jump instruction if this is 11
//                             target_addr - the add calculated at the ex stage
//                             pc - 4 - the pc value pointing to the next instruction of the branch instruction
//                     For Data Hazard:
//                             ID_EX_MemRead - Load instruction if this is 1
//                             ID_EX_rd
//                             IF_ID_rs1
//                             IF_ID_rs2
//                             IF_ID_MemWrite

module hazard_control_unit( 
    
    input wire i_hcu_branch,                                 // available in hart as t_clu_branch
    input wire [1:0] i_hcu_lui_auipc_mux_sel,                      // available in hart as t_clu_lui_auipc_mux_sel
    input wire [31:0] i_hcu_IF_ID_PC_current_val      ,             // available in hart as IF_ID[31:0] 
    input wire i_hcu_ID_EX_MemRead         ,                 // available in hart as t_dmem_ren
    input wire [4:0] i_hcu_ID_EX_rd            ,                   // available in hart as ID_EX[4:0]
    input wire [4:0] i_hcu_IF_ID_rs1       ,                       // available in hart as t_i_imem_to_rf_instr[19:15]
    input wire [4:0] i_hcu_IF_ID_rs2      ,                        // available in hart as t_i_imem_to_rf_instr[24:20]
    input wire i_hcu_IF_ID_MemWrite  ,                       // available in hart as t_dmem_wen
    input wire [6:0] i_hcu_IF_ID_opcode,                           // available in hart as t_i_imem_to_rf_instr[6:0]
    input wire i_hcu_branch_predictor,
    input wire i_ex_prediction_bit,    // The prediction bit carried down from Fetch->ID->EX
    input wire i_ex_actual_outcome,    // The result of the AND gate in EX (1=Taken, 0=Not)
    input wire i_ex_is_branch,         // Is the instruction in EX actually a branch opcode?
    output wire o_hcu_IF_ID_flush,
    output wire o_hcu_ID_EX_flush,
    output wire o_hcu_PCWriteEn,
    output wire o_hcu_IF_ID_write_en,
    output wire o_hcu_retire_valid,
    output wire o_ex_mispredict_detected // Tell Fetch to load the correct address
);

wire      control_hazard;
wire [4:0]t_hcu_IF_ID_rs1;
wire [4:0]t_hcu_IF_ID_rs2;
//wire [4:0]t_hcu_ID_EX_rd;

// assign control_hazard = (i_hcu_branch) ? (((i_hcu_lui_auipc_mux_sel == 2'b11) //JAL /JALR
//                                          || (i_hcu_branch_predictor)) ? 1'b1 : 1'b0 ) 
//                                          : 1'b0;
assign control_hazard = i_hcu_branch_predictor;

wire   load_use_hazard;
wire misprediction;
// If it is a branch, and (Prediction != Outcome), we have a problem.
assign misprediction = (i_ex_is_branch) && (i_ex_prediction_bit != i_ex_actual_outcome);
assign load_use_hazard = ((i_hcu_ID_EX_MemRead == 1) && ((i_hcu_ID_EX_rd == t_hcu_IF_ID_rs1) || (i_hcu_ID_EX_rd == t_hcu_IF_ID_rs2)) && (i_hcu_ID_EX_rd != 5'b0)) ? 1'b1 : 1'b0;
assign o_hcu_retire_valid = ~load_use_hazard;
// assign o_hcu_IF_ID_flush    =      control_hazard;
assign o_hcu_IF_ID_flush    =      misprediction || control_hazard;
assign o_hcu_PCWriteEn      =      ~load_use_hazard;
assign o_hcu_IF_ID_write_en =      ~load_use_hazard;       // will I get garbage values in the stall cycle as only IF_ID is disabled
assign o_hcu_ID_EX_flush    =      load_use_hazard || misprediction;
assign o_ex_mispredict_detected = misprediction;

assign t_hcu_IF_ID_rs1 = ((i_hcu_IF_ID_opcode == 7'b0110111)||(i_hcu_IF_ID_opcode == 7'b0010111)||(i_hcu_IF_ID_opcode == 7'b1101111)) ? 5'b00000 : i_hcu_IF_ID_rs1;
assign t_hcu_IF_ID_rs2 = ((i_hcu_IF_ID_opcode == 7'b0110011)||(i_hcu_IF_ID_opcode == 7'b0100011)||(i_hcu_IF_ID_opcode == 7'b1100011)) ?  i_hcu_IF_ID_rs2 : 5'b00000; //
//ssign t_hcu_ID_EX_rd  = ((i_hcu_IF_ID_opcode == 7'b0100011)||(i_hcu_IF_ID_opcode == 7'b1100011)) ? 5'b00000 : i_hcu_ID_EX_rd; //Store and Branch - No Rd and thus have to make it Zero

endmodule


// #todo:
// i_pc_write_en - added in fetch module - make according changes in the modules it is instantiated - decide it has an valid inputs coming to it all times
// ID_EX_write_en - make sure it has proper values at all times
// IF_ID_write_en - make sure it has proper values at all times

// replace IF_ID_rs1 -> ID_rs1 => More appropriate naming

// rd == 0 (x0) we dont have to stall - incl rd!=x0
// rs1 = rd or rs2=rd does not excatly mean raw - recheck with that example in the ppt
//To make sure we clear the new ID EX Pipe for addresses

// Mostly trash:
// // Data Hazard
// //assign o_hcu_PCWriteEn =      (o_hcu_PCWriteEn)                         ?   (((i_huc_ID_EX_MemRead == 1) && ((i_hcu_ID_EX_rd == i_hcu_IF_ID_rs1) || (i_hcu_ID_EX_rd == i_hcu_IF_ID_rs2))  && (i_hcu_IF_ID_MemWrite != 1'b0)) ? 1'b0 : 1'b1) : 1'b1;
// assign o_hcu_IF_ID_write_en = (o_hcu_IF_ID_write_en)                    ?   (((i_huc_ID_EX_MemRead == 1) && ((i_hcu_ID_EX_rd == i_hcu_IF_ID_rs1) || (i_hcu_ID_EX_rd == i_hcu_IF_ID_rs2))  && (i_hcu_IF_ID_MemWrite != 1'b0)) ? 1'b0 : 1'b1) : 1'b1;
// assign o_hcu_ID_EX_write_en = (o_hcu_ID_EX_write_en)                    ?   (((i_huc_ID_EX_MemRead == 1) && ((i_hcu_ID_EX_rd == i_hcu_IF_ID_rs1) || (i_hcu_ID_EX_rd == i_hcu_IF_ID_rs2))  && (i_hcu_IF_ID_MemWrite != 1'b0)) ? 1'b0 : 1'b1) : 1'b1;
