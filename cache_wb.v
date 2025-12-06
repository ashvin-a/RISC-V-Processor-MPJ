`default_nettype none

module cache (
    input  wire        i_clk,
    input  wire        i_rst,
    // Memory Interface
    input  wire        i_mem_ready,
    output wire [31:0] o_mem_addr,
    output wire        o_mem_ren,
    output wire        o_mem_wen,
    output wire [31:0] o_mem_wdata,
    input  wire [31:0] i_mem_rdata,
    input  wire        i_mem_valid,
    // CPU Interface
    output wire        o_busy,
    input  wire [31:0] i_req_addr,
    input  wire        i_req_ren,
    input  wire        i_req_wen,
    input  wire [ 3:0] i_req_mask,
    input  wire [31:0] i_req_wdata,
    output wire [31:0] o_res_rdata
);

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam O = 4;           // 16 byte lines
    localparam S = 5;           // 32 sets
    localparam DEPTH = 2 ** S;
    localparam T = 32 - O - S;  // 23 bit tag
    localparam D = 2 ** O / 4;  // 4 words per line

    // -------------------------------------------------------------------------
    // Cache State Arrays
    // -------------------------------------------------------------------------
    reg [31:0] datas0 [DEPTH-1:0][D-1:0];
    reg [31:0] datas1 [DEPTH-1:0][D-1:0];
    reg [T-1:0] tags0 [DEPTH-1:0];
    reg [T-1:0] tags1 [DEPTH-1:0];
    reg [1:0]   valid [DEPTH-1:0];
    reg [1:0]   dirty [DEPTH-1:0]; // NEW: Dirty bits [Set][Way]
    reg         lru   [DEPTH-1:0];

    // -------------------------------------------------------------------------
    // Address Decoding
    // -------------------------------------------------------------------------
    wire [T-1:0] in_tag    = i_req_addr[31 : O+S];
    wire [S-1:0] in_set    = i_req_addr[O+S-1 : O];
    wire [1:0]   in_offset = i_req_addr[3:2];

    // -------------------------------------------------------------------------
    // Hit Detection
    // -------------------------------------------------------------------------
    wire hit0 = valid[in_set][0] && (tags0[in_set] == in_tag);
    wire hit1 = valid[in_set][1] && (tags1[in_set] == in_tag);
    wire hit  = hit0 || hit1;
    wire hit_way = hit1; 
    
    // Victim Selection for Misses
    wire current_victim = lru[in_set]; 

    // -------------------------------------------------------------------------
    // Data Read & Masking (For Hits)
    // -------------------------------------------------------------------------
    wire [31:0] rdata0 = datas0[in_set][in_offset];
    wire [31:0] rdata1 = datas1[in_set][in_offset];
    
    assign o_res_rdata = hit0 ? rdata0 : rdata1;

    // Masking for IDLE Write Hits
    wire [31:0] hit_word = (hit_way == 0) ? rdata0 : rdata1;
    reg  [31:0] hit_merged_wdata;

    always @(*) begin
        hit_merged_wdata[ 7: 0] = i_req_mask[0] ? i_req_wdata[ 7: 0] : hit_word[ 7: 0];
        hit_merged_wdata[15: 8] = i_req_mask[1] ? i_req_wdata[15: 8] : hit_word[15: 8];
        hit_merged_wdata[23:16] = i_req_mask[2] ? i_req_wdata[23:16] : hit_word[23:16];
        hit_merged_wdata[31:24] = i_req_mask[3] ? i_req_wdata[31:24] : hit_word[31:24];
    end

    // -------------------------------------------------------------------------
    // Shadow Registers (Snapshot)
    // -------------------------------------------------------------------------
    reg [31:0] saved_addr;
    reg [31:0] saved_wdata;
    reg [3:0]  saved_mask;
    reg        saved_wen;
    reg        saved_ren;
    reg        saved_victim; // Latched Victim Way
    
    wire [T-1:0] saved_tag    = saved_addr[31 : O+S];
    wire [S-1:0] saved_set    = saved_addr[O+S-1 : O];
    wire [1:0]   saved_offset = saved_addr[3:2];

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    localparam S_IDLE     = 2'b00;
    localparam S_EVICT    = 2'b01; // NEW: Flush dirty line to memory
    localparam S_ALLOCATE = 2'b10; // Fetch new line
    localparam S_UPDATE   = 2'b11; // Update metadata

    reg [1:0] state;
    reg [1:0] refill_cnt;
    reg       req_sent;

    // Busy Logic:
    // 1. Miss detected in IDLE
    // 2. Currently processing Miss (Any state != IDLE)
    // Note: We do NOT stall on Write Hits anymore (Write-Back is fast!)
    assign o_busy = ((i_req_ren || i_req_wen) && !hit && state == S_IDLE) || 
                    (state != S_IDLE);

    // -------------------------------------------------------------------------
    // Memory Interface Outputs
    // -------------------------------------------------------------------------
    reg [31:0] mem_addr_comb;
    reg        mem_ren_comb;
    reg        mem_wen_comb;
    reg [31:0] mem_wdata_comb;

    assign o_mem_addr  = mem_addr_comb;
    assign o_mem_ren   = mem_ren_comb;
    assign o_mem_wen   = mem_wen_comb;
    assign o_mem_wdata = mem_wdata_comb;

    always @(*) begin
        mem_addr_comb  = 0;
        mem_ren_comb   = 0;
        mem_wen_comb   = 0;
        mem_wdata_comb = 0;

        case (state)
            S_IDLE: begin
                // No Memory Access on Hit (Write-Back)
            end

            S_EVICT: begin
                // Write Back Old Line
                // Address: Reconstructed from OLD Tag + Set + Counter
                // Data: Read from Cache Arrays
                mem_wen_comb = !req_sent; 
                if (saved_victim == 0) begin
                    mem_addr_comb  = {tags0[saved_set], saved_set, refill_cnt, 2'b00};
                    mem_wdata_comb = datas0[saved_set][refill_cnt];
                end else begin
                    mem_addr_comb  = {tags1[saved_set], saved_set, refill_cnt, 2'b00};
                    mem_wdata_comb = datas1[saved_set][refill_cnt];
                end
            end

            S_ALLOCATE: begin
                // Fetch New Line
                mem_addr_comb = {saved_tag, saved_set, refill_cnt, 2'b00};
                mem_ren_comb  = !req_sent; 
            end

            S_UPDATE: begin
                // No Memory Access (Cache Update Only)
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // Sequential Logic
    // -------------------------------------------------------------------------
    integer k, j;
    initial begin
        for(k=0; k<DEPTH; k=k+1) begin 
            valid[k]=0; lru[k]=0; dirty[k]=0;
            for(j=0; j<D; j=j+1) begin datas0[k][j]=0; datas1[k][j]=0; end
        end
        state = S_IDLE;
    end

    always @(posedge i_clk) begin
        if (i_rst) begin
            state <= S_IDLE;
            refill_cnt <= 0;
            req_sent <= 0;
            for(k=0; k<DEPTH; k=k+1) begin 
                valid[k]<=0; lru[k]<=0; dirty[k]<=0;
                for(j=0; j<D; j=j+1) begin datas0[k][j]<=0; datas1[k][j]<=0; end
            end
        end else begin
            case (state)
                S_IDLE: begin
                    if (i_req_ren || i_req_wen) begin
                        if (hit) begin
                            // --- HIT ---
                            lru[in_set] <= ~hit_way; 
                            if (i_req_wen) begin
                                // Update Data
                                if (hit_way == 0) datas0[in_set][in_offset] <= hit_merged_wdata;
                                else              datas1[in_set][in_offset] <= hit_merged_wdata;
                                
                                // Set Dirty Bit
                                dirty[in_set][hit_way] <= 1'b1;
                            end
                        end 
                        else begin
                            // --- MISS ---
                            saved_addr   <= i_req_addr;
                            saved_wdata  <= i_req_wdata;
                            saved_mask   <= i_req_mask;
                            saved_ren    <= i_req_ren;
                            saved_wen    <= i_req_wen;
                            saved_victim <= current_victim; // Lock victim choice
                            
                            // Check if Victim is Dirty
                            if (valid[in_set][current_victim] && dirty[in_set][current_victim]) begin
                                state <= S_EVICT; // Must Write Back first
                            end else begin
                                state <= S_ALLOCATE; // Clean, proceed to fetch
                            end
                            
                            refill_cnt <= 0;
                            req_sent <= 0;
                        end
                    end
                end

                S_EVICT: begin
                    // Flush dirty cache line to memory
                    if (!req_sent && i_mem_ready) req_sent <= 1;

                    // Assuming memory accepts write when Ready is high
                    if (i_mem_ready) begin
                        if (refill_cnt == 2'b11) begin
                            state <= S_ALLOCATE; // Done flushing, now fetch
                            refill_cnt <= 0;
                            req_sent <= 0;
                        end else begin
                            refill_cnt <= refill_cnt + 1;
                            req_sent <= 0;
                        end
                    end
                end

                S_ALLOCATE: begin
                    if (!req_sent && i_mem_ready) req_sent <= 1;

                    if (i_mem_valid) begin
                        if (saved_victim == 0) datas0[saved_set][refill_cnt] <= i_mem_rdata;
                        else                   datas1[saved_set][refill_cnt] <= i_mem_rdata;

                        if (refill_cnt == 2'b11) begin
                            state <= S_UPDATE; 
                        end else begin
                            refill_cnt <= refill_cnt + 1;
                            req_sent <= 0; 
                        end
                    end
                end

                S_UPDATE: begin
                    // Update Tag/Valid
                    if (saved_victim == 0) tags0[saved_set] <= saved_tag;
                    else                   tags1[saved_set] <= saved_tag;
                    
                    valid[saved_set][saved_victim] <= 1'b1;

                    if (saved_wen) begin
                        // Write Miss: Apply the write to the cache
                        if (saved_victim == 0) begin
                            datas0[saved_set][saved_offset][ 7: 0] <= saved_mask[0] ? saved_wdata[ 7: 0] : datas0[saved_set][saved_offset][ 7: 0];
                            datas0[saved_set][saved_offset][15: 8] <= saved_mask[1] ? saved_wdata[15: 8] : datas0[saved_set][saved_offset][15: 8];
                            datas0[saved_set][saved_offset][23:16] <= saved_mask[2] ? saved_wdata[23:16] : datas0[saved_set][saved_offset][23:16];
                            datas0[saved_set][saved_offset][31:24] <= saved_mask[3] ? saved_wdata[31:24] : datas0[saved_set][saved_offset][31:24];
                        end else begin
                            datas1[saved_set][saved_offset][ 7: 0] <= saved_mask[0] ? saved_wdata[ 7: 0] : datas1[saved_set][saved_offset][ 7: 0];
                            datas1[saved_set][saved_offset][15: 8] <= saved_mask[1] ? saved_wdata[15: 8] : datas1[saved_set][saved_offset][15: 8];
                            datas1[saved_set][saved_offset][23:16] <= saved_mask[2] ? saved_wdata[23:16] : datas1[saved_set][saved_offset][23:16];
                            datas1[saved_set][saved_offset][31:24] <= saved_mask[3] ? saved_wdata[31:24] : datas1[saved_set][saved_offset][31:24];
                        end
                        // Mark Dirty (Write Allocate)
                        dirty[saved_set][saved_victim] <= 1'b1;
                    end else begin
                        // Read Miss: Clean data from memory
                        dirty[saved_set][saved_victim] <= 1'b0;
                    end
                    
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule