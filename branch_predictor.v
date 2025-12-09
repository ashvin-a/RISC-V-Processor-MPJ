module branch_predictor (
    input wire clk,
    input wire rst,
    
    // READ PORT (Fetch Stage)
    input wire [31:0] i_fetch_pc,      // Current PC in Fetch
    output wire o_predict_taken,       // 1 = Predict Taken, 0 = Predict Not Taken

    // WRITE/UPDATE PORT (Execute Stage)
    input wire [31:0] i_ex_pc,         // PC of the branch currently in EX stage
    input wire i_ex_branch_was_taken,  // The actual result (AND gate output) from EX
    input wire i_ex_is_branch_instr    // High if the instruction in EX is actually a branch
);

    // 2-bit Saturating Counters Table (e.g., 64 entries)
    // 00: Strong Not Taken, 01: Weak Not Taken
    // 10: Weak Taken, 11: Strong Taken
    reg [1:0] bht [63:0]; 

    // Hashing function: simply use bits [7:2] of the PC
    wire [5:0] fetch_index = i_fetch_pc[7:2];
    wire [5:0] ex_index = i_ex_pc[7:2];

    // --- PREDICTION LOGIC ---
    // Predict Taken if counter >= 2 (binary 10 or 11)
    assign o_predict_taken = bht[fetch_index][1]; 

    // --- UPDATE LOGIC ---
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 64; i = i + 1) bht[i] <= 2'b01; // Initialize to Weak Not Taken
        end
        else if (i_ex_is_branch_instr) begin
            case (bht[ex_index])
                2'b00: bht[ex_index] <= (i_ex_branch_was_taken) ? 2'b01 : 2'b00;
                2'b01: bht[ex_index] <= (i_ex_branch_was_taken) ? 2'b10 : 2'b00;
                2'b10: bht[ex_index] <= (i_ex_branch_was_taken) ? 2'b11 : 2'b01;
                2'b11: bht[ex_index] <= (i_ex_branch_was_taken) ? 2'b11 : 2'b10;
            endcase
        end
    end
endmodule