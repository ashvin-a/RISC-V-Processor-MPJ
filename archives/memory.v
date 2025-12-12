/////////////////////////////////////////////////
/////////////////Instruction Memory/////////////////////
/////////////////////////////////////////////////
//[Q2] What is the size of the instruction memory? - Assuming 16KB for now
//[Q] Who is initialising the insturction memory? How are the instructions initially written into the memory?
module instruction_memory(
    input  wire clk,
    input  wire rst_n,
    input  wire [31:0]i_instr_mem_rd_addr,
    output wire [31:0] o_inst
);
    reg [31:0] instr_mem [255:0];//16 KB Memory with 32 bit instruction in each location
    reg [31:0] t_inst_reg; //temporary reg for o_inst
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            t_inst_reg <= 0; // [Q3] Should it be don't care instead of 0?
            for (i = 0;i<256;i=i+1) begin
                instr_mem[i] <= 32'b0;
            end
        end
        else if (i_instr_mem_rd_addr)
            t_inst_reg <= instr_mem [i_instr_mem_rd_addr];
    end
    assign o_inst = t_inst_reg;
endmodule
/////////////////////////////////////////////////
/////////////////Data Memory/////////////////////
/////////////////////////////////////////////////
//[Q] Shall I assume that that instructions will always be 32 bit? - I guess Yes
//[Q] Will there be a negative case where both MemWrite and MemRead will be high?
//[Q] Who is initialising the data memory? And Should I be initializing to zero??
module data_memory(
    input  wire clk,
    input  wire rst_n,
    input  wire i_clu_MemWrite,
    input  wire i_clu_MemRead,
    input  wire [31:0]i_data_mem_addr,
    input  wire [31:0] i_data_mem_wr_data,
    output wire [31:0] o_data_mem_rd_data 
);
    reg [31:0] data_mem [255:0];//16 KB Memory with 32 bit data in each location
    reg [31:0] t_data_mem_rd_data_reg;
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for (i = 0;i<256;i=i+1) begin
                data_mem[i] <= 32'b0;
            end
        end
        if (i_data_mem_addr && i_clu_MemRead && !i_clu_MemWrite)
            t_data_mem_rd_data_reg <= data_mem [i_data_mem_addr];
        if (i_data_mem_addr && !i_clu_MemRead && i_clu_MemWrite) // Wrote a seperate if - independent
                data_mem[i_data_mem_addr] <= i_data_mem_wr_data;
    end
    assign o_data_mem_rd_data = t_data_mem_rd_data_reg;

    //Also add a mux at the end

endmodule