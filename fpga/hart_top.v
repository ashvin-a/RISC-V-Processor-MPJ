module hart_top (
    input clk, 
    input rst,
    output LED0,
    output LED1,
    output LED2,
    output LED3,
    output LED4,
    output LED5,
    output LED6
    );
    // Instruction memory interface.
    reg  [31:0] imem_rdata, dmem_rdata;
    wire [31:0] imem_raddr, dmem_addr;
    // Data memory interface.
    wire        dmem_ren, dmem_wen;
    wire [31:0] dmem_wdata;
    wire [ 3:0] dmem_mask;

    // Instruction retire interface.
    wire        valid, trap, halt;
    wire [31:0] inst;
    wire [ 4:0] rs1_raddr, rs2_raddr;
    wire [31:0] rs1_rdata, rs2_rdata;
    wire [ 4:0] rd_waddr;
    wire [31:0] rd_wdata;
    wire [31:0] pc, next_pc;

    hart #(
        .RESET_ADDR (32'h0)
    ) dut (
        .i_clk        (clk),
        .i_rst        (rst),
        .o_imem_raddr (imem_raddr),
        .i_imem_rdata (imem_rdata),
        .o_dmem_addr  (dmem_addr),
        .o_dmem_ren   (dmem_ren),
        .o_dmem_wen   (dmem_wen),
        .o_dmem_wdata (dmem_wdata),
        .o_dmem_mask  (dmem_mask),
        .i_dmem_rdata (dmem_rdata),
        .o_retire_valid     (valid),
        .o_retire_inst      (inst),
        .o_retire_trap      (trap),
        .o_retire_halt      (halt),
        .o_retire_rs1_raddr (rs1_raddr),
        .o_retire_rs1_rdata (rs1_rdata),
        .o_retire_rs2_raddr (rs2_raddr),
        .o_retire_rs2_rdata (rs2_rdata),
        .o_retire_rd_waddr  (rd_waddr),
        .o_retire_rd_wdata  (rd_wdata),
        .o_retire_pc        (pc),
        .o_retire_next_pc   (next_pc)
    );

    initial begin
        $readmemh("program.mem", imem);
    end

    // The tesbench uses separate instruction and data memory banks.
    reg [7:0] imem [0:1023];
    reg [7:0] dmem [0:1023];

    wire [7:0] imem_raddr1;
    wire [7:0] imem_raddr2;
    wire [7:0] imem_raddr3;
    wire [7:0] imem_raddr0;

    assign imem_raddr3 = imem[imem_raddr + 3];
    assign imem_raddr2 = imem[imem_raddr + 2];
    assign imem_raddr1 = imem[imem_raddr + 1];
    assign imem_raddr0 = imem[imem_raddr + 0];
    // Instruction memory read.
    always @(*) begin
        imem_rdata = {imem[imem_raddr + 3], imem[imem_raddr + 2], imem[imem_raddr + 1], imem[imem_raddr + 0]};
    end

    // Data memory read. Masks are ignored since it is always safe
    // to access the full bytes in this memory.
    always @(*) begin
        if (dmem_ren)
            dmem_rdata = {dmem[dmem_addr + 3], dmem[dmem_addr + 2], dmem[dmem_addr + 1], dmem[dmem_addr + 0]};
        else
            dmem_rdata = 32'h0;
    end
    // Synchronous data memory write. Masks must be respected.
    // The byte order is little-endian.
    always @(posedge clk) begin
        if (dmem_wen & dmem_mask[0])
            dmem[dmem_addr + 0] <= dmem_wdata[ 7: 0];
        if (dmem_wen & dmem_mask[1])
            dmem[dmem_addr + 1] <= dmem_wdata[15: 8];
        if (dmem_wen & dmem_mask[2])
            dmem[dmem_addr + 2] <= dmem_wdata[23:16];
        if (dmem_wen & dmem_mask[3])
            dmem[dmem_addr + 3] <= dmem_wdata[31:24];
    end

    assign LED0 = imem[0][0];
    assign LED1 = imem[0][1];
    assign LED2 = imem[0][2];
    assign LED3 = rd_wdata[3];
    assign LED4 = rd_wdata[4];
    assign LED5 = rd_wdata[5];
    assign LED6 = rd_wdata[6];
    assign LED7 = rd_wdata[7];

endmodule
