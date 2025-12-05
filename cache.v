`default_nettype none

module cache (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // External memory interface. See hart interface for details. This
    // interface is nearly identical to the phase 5 memory interface, with the
    // exception that the byte mask (`o_mem_mask`) has been removed. This is
    // no longer needed as the cache will only access the memory at word
    // granularity, and implement masking internally.
    input  wire        i_mem_ready,
    output wire [31:0] o_mem_addr,
    output wire        o_mem_ren,
    output wire        o_mem_wen,
    output wire [31:0] o_mem_wdata,
    input  wire [31:0] i_mem_rdata,
    input  wire        i_mem_valid,
    // Interface to CPU hart. This is nearly identical to the phase 5 hart memory
    // interface, but includes a stall signal (`o_busy`), and the input/output
    // polarities are swapped for obvious reasons.
    //
    // The CPU should use this as a stall signal for both instruction fetch
    // (IF) and memory (MEM) stages, from the instruction or data cache
    // respectively. If a memory request is made (`i_req_ren` for instruction
    // cache, or either `i_req_ren` or `i_req_wen` for data cache), this
    // should be asserted *combinationally* if the request results in a cache
    // miss.
    //
    // In case of a cache miss, the CPU must stall the respective pipeline
    // stage and deassert ren/wen on subsequent cycles, until the cache
    // deasserts `o_busy` to indicate it has serviced the cache miss. However,
    // the CPU must keep the other request lines constant. For example, the
    // CPU should not change the request address while stalling.
    output wire        o_busy,
    // 32-bit read/write address to access from the cache. This should be
    // 32-bit aligned (i.e. the two LSBs should be zero). See `i_req_mask` for
    // how to perform half-word and byte accesses to unaligned addresses.
    input  wire [31:0] i_req_addr,
    // When asserted, the cache should perform a read at the aligned address
    // specified by `i_req_addr` and return the 32-bit word at that address,
    // either immediately (i.e. combinationally) on a cache hit, or
    // synchronously on a cache miss. It is illegal to assert this and
    // `i_dmem_wen` on the same cycle.
    input  wire        i_req_ren,
    // When asserted, the cache should perform a write at the aligned address
    // specified by `i_req_addr` with the 32-bit word provided in
    // `o_req_wdata` (specified by the mask). This is necessarily synchronous,
    // but may either happen on the next clock edge (on a cache hit) or after
    // multiple cycles of latency (cache miss). As the cache is write-through
    // and write-allocate, writes must be applied to both the cache and
    // underlying memory.
    // It is illegal to assert this and `i_dmem_ren` on the same cycle.
    input  wire        i_req_wen,
    // The memory interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    input  wire [ 3:0] i_req_mask,
    // The 32-bit word to write to memory, if the request is a write
    // (i_req_wen is asserted). Only the bytes corresponding to set bits in
    // the mask should be written into the cache (and to backing memory).
    input  wire [31:0] i_req_wdata,
    // THe 32-bit data word read from memory on a read request.
    output wire [31:0] o_res_rdata
);
    // These parameters are equivalent to those provided in the project
    // 6 specification. Feel free to use them, but hardcoding these numbers
    // rather than using the localparams is also permitted, as long as the
    // same values are used (and consistent with the project specification).
    //
    // 32 sets * 2 ways per set * 16 bytes per way = 1K cache
    localparam O = 4;            // 4 bit offset => 16 byte cache line
    localparam S = 5;            // 5 bit set index => 32 sets
    localparam DEPTH = 2 ** S;   // 32 sets
    localparam W = 2;            // 2 way set associative, NMRU
    localparam T = 32 - O - S;   // 23 bit tag
    localparam D = 2 ** O / 4;   // 16 bytes per line / 4 bytes per word = 4 words per line

    // The following memory arrays model the cache structure. As this is
    // an internal implementation detail, you are *free* to modify these
    // arrays as you please.

    // Backing memory, modeled as two separate ways.
    reg [   31:0] datas0 [DEPTH - 1:0][D - 1:0];
    reg [   31:0] datas1 [DEPTH - 1:0][D - 1:0];
    reg [T - 1:0] tags0  [DEPTH - 1:0];
    reg [T - 1:0] tags1  [DEPTH - 1:0];
    reg [1:0] valid [DEPTH - 1:0];
    reg       lru   [DEPTH - 1:0];

    integer i;

    always @ (posedge i_clk)
        if (i_rst) begin
            for (i=0; i<32; i = i + 1) begin
                valid[i] <= 0;
                lru[i] <= 0;
                tags0[i] <= 0;
                tags1[i] <= 0;
            end
        end

    integer k, j;

    always @(posedge i_clk) begin
        if (i_rst) begin
            for (k = 0; k < DEPTH; k = k + 1) begin
                for (j = 0; j < D; j = j + 1) begin
                    datas0[k][j] <= 32'b0;
                    datas1[k][j] <= 32'b0;
                end
            end
        end
    end

    // Fill in your implementation here.
    ////Local wires////
    reg  cnt_4_clr; // To clear the counter once 4 times count is done 
    reg  cnt_4;
    reg  load_miss_cache_write;
    reg  store_miss_cache_write;
    reg  store_hit_way0;
    reg  store_hit_way1;
    wire [31:0] t_masked_i_req_wdata;

    reg read_hit_way0;
    reg read_hit_way1;
    reg miss_from_chk_tag;

    reg  w_o_busy;
    reg reg_o_busy;
    reg  [31:0] w_o_res_rdata;
    reg  [31:0] w_o_mem_addr;
    reg  w_o_mem_wen;
    reg  w_o_mem_ren;
    reg  [31:0] w_o_mem_wdata;

    wire [4:0] set_index;
    wire [1:0] offset;
    wire [22:0] tag;
    wire cnt_4_done;

    reg [1:0] counter_4;
    reg w_o_busy_clr;
    reg way;
    reg hit;
    reg hold_lru;

    wire [4:0] f_set_index;
    wire [1:0] f_offset;
    wire [22:0] f_tag;

    reg  new_store_hit_way0;
    reg  new_store_hit_way1;
    reg  do_the_write_cache;
    
    reg mem_access_store_hit;
    reg mem_access_store_miss;
    reg mem_access_load_miss;

    reg load_miss_cache_write_done;
    reg load_miss_cache_write_done_clr;
    /////////////////////////////////////////////////////////////////////////
    //// Registers to store the inut request signals in case of a miss///////
    ////////////////////////////////////////////////////////////////////////
    reg reg_i_req_ren;
    reg reg_i_req_wen;
    reg [31:0] reg_i_req_addr;
    reg [31:0] reg_i_req_wdata;

    reg f_i_req_wen;
    reg f_i_req_ren;
    reg [31:0] f_t_masked_i_req_wdata;
    reg [31:0] f_i_req_addr;
    //###################################//#############///////
    ////DATA MASKING TO BE DONE HERE///////////////////////////
    //###################################//#############///////
    // For store instructions only 
    assign t_masked_i_req_wdata =   i_req_wdata;
    // assign t_masked_i_req_wdata =   (i_req_mask == 4'b0001) ? {{24{i_req_wdata[7]}},i_req_wdata[7:0]} :                   // Applicable for sb
    //                     (i_req_mask == 4'b0010) ? {{16{i_req_wdata[15]}},i_req_wdata[15:8],8'b0} :             // Applicable for sb
    //                     (i_req_mask == 4'b0100) ? {{8{i_req_wdata[23]}},i_req_wdata[23:16],16'b0} :            // Applicable for sb
    //                     (i_req_mask == 4'b1000) ? {i_req_wdata[7:0],24'b0} :                   // Applicable for sb
    //                     (i_req_mask == 4'b0011) ? {{16{i_req_wdata[15]}},i_req_wdata[15:0]} :  // Applicable for sh - SIGN EXTENSION by default
    //                     (i_req_mask == 4'b1100) ? {i_req_wdata[15:0],16'b0} :                  // Applicable for sh
    //                     (i_req_mask == 4'b1111) ? i_req_wdata :  
    //                     32'bxxxx;                                                               // Default case

    //////Local assignments////////////
    assign set_index = i_req_addr[8:4];
    assign offset = i_req_addr[3:2];
    assign tag = i_req_addr[31:9];    

    assign o_busy = w_o_busy;
    assign o_res_rdata = w_o_res_rdata;
    assign o_mem_addr = w_o_mem_addr;
    assign o_mem_wen = w_o_mem_wen;
    assign o_mem_ren = w_o_mem_ren; // Illegal to keep both as the same
    assign o_mem_wdata = w_o_mem_wdata;

    //////////////////////////////////////////////////////
    ///////////////LRU //////////////////////////////////
    ///////////////////////////////////////////////////
    always @ (posedge i_clk)
        if (i_rst) begin
            for (i=0; i<32; i = i + 1) begin
                lru[i] <= 0;
            end
        end
        else begin
            if (hit && !hold_lru) begin
                if (way == lru[set_index])
                    lru[set_index] <= ~lru[set_index];
                    //lru[set_index] = ~lru[set_index];
                else 
                    //lru[set_index] = lru[set_index];
                    lru[set_index] <= lru[set_index];
            end
            else if (!hit && !hold_lru)
                //lru[set_index] = ~lru[set_index];
                lru[set_index] <= ~lru[set_index];
            else if (hold_lru)
                //lru[set_index] = lru[set_index];
                lru[set_index] <= lru[set_index];
        end

    ///////////////////////////////////////////////////
    ////////////////////cache write////////////////////
    ///////////////////////////////////////////////////
    always @ (negedge i_clk)
        if (i_rst) begin
            load_miss_cache_write_done <= 0;
        end
        else if (load_miss_cache_write_done_clr) begin
            load_miss_cache_write_done <= 0;
        end
        else if (cnt_4_done && i_mem_valid) begin
            load_miss_cache_write_done <= 1'b1;
        end

    always @ (posedge i_clk)
        if (store_hit_way0) begin// write to that particular set/way in the cache
            datas0[set_index][offset] <= t_masked_i_req_wdata; 
        end
        else if (store_hit_way1) begin
            // datas1[set_index][offset] <= t_masked_i_req_wdata;
            if(i_req_mask == 4'b1111)
                datas1[set_index][offset] <= t_masked_i_req_wdata;
            else if(i_req_mask == 4'b0011)
                datas1[set_index][offset] <= {datas1[set_index][offset][31:16],t_masked_i_req_wdata[15:0]};
            else if(i_req_mask == 4'b1100)
                datas1[set_index][offset] <= {t_masked_i_req_wdata[31:16],datas1[set_index][offset][15:0]};
        end
        //else if (new_store_hit_way0 && store_miss_cache_write) begin// write to that particular set/way in the cache
        else if (do_the_write_cache && store_miss_cache_write) begin// write to that particular set/way in the cache
            // datas0[f_set_index][f_offset] <= reg_i_req_wdata; 
            if (!lru[f_set_index]) begin
                datas0[f_set_index][f_offset] <= reg_i_req_wdata; 
                datas0[f_set_index][counter_4] <= i_mem_rdata; // Data coming from the Memory
                if(cnt_4_done)
                    valid[f_set_index][0] <= 1'b1;
                else                     
                    valid[f_set_index][0] <= valid[f_set_index][0];
                tags0[f_set_index] <= f_tag;
            end
            else if (lru[f_set_index]) begin
                datas1[f_set_index][f_offset] <= reg_i_req_wdata;
                datas1[f_set_index][counter_4] <= i_mem_rdata; // Data coming from the Memory 
                if(cnt_4_done)
                    valid[f_set_index][1] <= 1'b1;
                else                     
                    valid[f_set_index][1] <= valid[f_set_index][1];
                tags1[f_set_index] <= f_tag;
            end            
        end
        //else if (new_store_hit_way1 && store_miss_cache_write) begin
        else if (do_the_write_cache && store_miss_cache_write) begin
            // datas1[f_set_index][f_offset] <= reg_i_req_wdata;
            if (!lru[f_set_index]) begin
                datas0[f_set_index][f_offset] <= reg_i_req_wdata; 
                datas0[f_set_index][counter_4] <= i_mem_rdata; // Data coming from the Memory
                if(cnt_4_done)
                    valid[f_set_index][0] <= 1'b1;
                else                     
                    valid[f_set_index][0] <= valid[f_set_index][0];
                tags0[f_set_index] <= f_tag;
            end
            else if (lru[f_set_index]) begin
                datas1[f_set_index][f_offset] <= reg_i_req_wdata;
                datas1[f_set_index][counter_4] <= i_mem_rdata; // Data coming from the Memory 
                if(cnt_4_done)
                    valid[f_set_index][1] <= 1'b1;
                else                     
                    valid[f_set_index][1] <= valid[f_set_index][1];
                tags1[f_set_index] <= f_tag;
            end              
        end
        else if (load_miss_cache_write | store_miss_cache_write) begin
            if (!lru[f_set_index]) begin
                datas0[f_set_index][counter_4] <= i_mem_rdata; // Data coming from the Memory
                if(cnt_4_done)
                    valid[f_set_index][0] <= 1'b1;
                else                     
                    valid[f_set_index][0] <= valid[f_set_index][0];
                tags0[f_set_index] <= f_tag;
            end
            else if (lru[f_set_index]) begin
                datas1[f_set_index][counter_4] <= i_mem_rdata; // Data coming from the Memory 
                if(cnt_4_done)
                    valid[f_set_index][1] <= 1'b1;
                else                     
                    valid[f_set_index][1] <= valid[f_set_index][1];
                tags1[f_set_index] <= f_tag;
            end
        end
    //////////////////////////////////////////////////////////////////////////
    //// Register to flop Store Hit as it will be used in different states////
    //////////////////////////////////////////////////////////////////////////
    reg f_store_hit;
    always @ (posedge i_clk)
        if(i_rst)
            f_store_hit <= 0;
        else 
            f_store_hit <= (store_hit_way0 | store_hit_way1);
    //////////////////////////////////////
    //// counter to count 4 times////////
    /////////////////////////////////////
    always @ (posedge i_clk)
        if(i_rst)
            counter_4 <= 0;
        else if (cnt_4_clr)
            counter_4 <= 0;
        else if (cnt_4)
            counter_4 <= counter_4 + 2'b1;
    assign cnt_4_done = &counter_4;
    
    ///////////////////////////////////////////////////////////////////////////////
    ////whenever there is a miss storing the instruction info in a register////////
    ///////////////////////////////////////////////////////////////////////////////
    always @ (posedge i_clk)
        if (i_rst) begin
            reg_i_req_ren   <= 0;
            reg_i_req_wen   <= 0;
            reg_i_req_addr  <= 0;
            reg_i_req_wdata <= 0;
        end
        else if (miss_from_chk_tag) begin
            reg_i_req_ren   <= i_req_ren;
            reg_i_req_wen   <= i_req_wen;
            reg_i_req_addr  <= i_req_addr;
            reg_i_req_wdata <= t_masked_i_req_wdata;
        end

    assign f_set_index = reg_i_req_addr[8:4];
    assign f_offset = reg_i_req_addr[3:2];
    assign f_tag = reg_i_req_addr[31:9];            
    ////////////////////////////////////////////////////////////////////////////////////
    ////whenever there is a store hit storing the instruction info in a register////////
    ///////////////////////////////////////////////////////////////////////////////////
    always @ (posedge i_clk)
        if (i_rst) begin
            f_i_req_ren   <= 0;
            f_i_req_wen   <= 0;
            f_i_req_addr  <= 0;
            f_t_masked_i_req_wdata <= 0;
        end    
        else if (store_hit_way0 | store_hit_way1) begin
            f_i_req_wen <= i_req_wen;
            f_i_req_ren <= i_req_ren;
            f_t_masked_i_req_wdata <= t_masked_i_req_wdata; 
            f_i_req_addr <= i_req_addr;
        end

    always @(*) begin

        // normal cache read
        if (valid[set_index][0] && (tags0[set_index] == tag) && i_req_ren && !i_req_wen)
            w_o_res_rdata = datas0[set_index][offset];
        else if (valid[set_index][1] && (tags1[set_index] == tag) && i_req_ren && !i_req_wen)
            w_o_res_rdata = datas1[set_index][offset];

        // miss-writeback read override
        if (!o_busy && load_miss_cache_write_done) begin
            if (valid[f_set_index][0] && (tags0[f_set_index] == f_tag) && reg_i_req_ren && !reg_i_req_wen)
                w_o_res_rdata = datas0[f_set_index][f_offset];
            else if (valid[f_set_index][1] && (tags1[f_set_index] == f_tag) && reg_i_req_ren && !reg_i_req_wen)
                w_o_res_rdata = datas1[f_set_index][f_offset];
        end
    end

    /////////////////////////////////////
    ///////State Machine////////////////
    /////////////////////////////////////
    localparam CHK_TAG = 2'b00;
    localparam MEM_ACCESS = 2'b01;
    localparam WRITE_CACHE = 2'b10;
    reg [1:0] state,next_state;

    always @ (posedge i_clk)
        if(i_rst) 
            state <= CHK_TAG;
        else 
            state <= next_state;

    always @ (*) begin
        mem_access_store_hit = 0;
        mem_access_store_miss = 0;
        mem_access_load_miss = 0;

        w_o_busy = 0;
        w_o_mem_addr = 0;
        w_o_mem_wen = 0;
        w_o_mem_ren = 0; // Illegal to keep both as the same : TODO
        w_o_mem_wdata = 0;
        cnt_4_clr = 0; // To clear the counter once 4 times count is done 
        cnt_4 = 0;
        load_miss_cache_write = 0;
        store_miss_cache_write = 0;
        load_miss_cache_write_done_clr = 0;

        store_hit_way0 = 0;
        store_hit_way1 = 0;
        hit = 0;
        way = 0;
        hold_lru = 0;

        read_hit_way0 = 0;
        read_hit_way1 = 0;
        miss_from_chk_tag = 0;

        new_store_hit_way0 = 0;
        new_store_hit_way1 = 0;
        do_the_write_cache = 0;

        next_state = state;

            case (state)
                CHK_TAG: begin
                        load_miss_cache_write_done_clr = 1;
                        //We need this illegal condition enforced or else it will switch to a the next state
                        //even when no transactions are coming to the cache
                        if (i_req_ren == i_req_wen)
                            next_state = CHK_TAG;                        
                        else if(valid[set_index][0] && (tags0[set_index] == tag) && i_req_ren) begin// LOAD HIT - Way 0
                            way = 0; 
                            hit = 1;
                            hold_lru = 0;
                            read_hit_way0 = 1;
                        end
                        else if (valid[set_index][1] && (tags1[set_index] == tag) && i_req_ren) begin// LOAD HIT - Way 1
                            way = 1;
                            hit = 1;
                            hold_lru = 0;
                            read_hit_way1 = 1;
                        end
                        else if(valid[set_index][0] && (tags0[set_index] == tag) && i_req_wen) begin // STORE HIT - Way 0  - Write to this Cache
                            way = 0;
                            hit = 1;
                            store_hit_way0 = 1; // To ensure write to cache is done even it is a hit which will be followed by write through to mem
                            if (i_mem_ready) begin // WRITE THROUGH :  Check for whether the Memory is ready or not
                                hold_lru = 0;
                                w_o_mem_wen   = i_req_wen;
                                w_o_mem_ren   = i_req_ren;
                                w_o_mem_wdata = t_masked_i_req_wdata;  
                                w_o_mem_addr  = i_req_addr;
                            end
                            else begin
                                hold_lru = 1;
                                // next_state = MEM_ACCESS;
                                next_state = CHK_TAG;                            
                                // w_o_busy = 1;                            
                            end
                        end
                        else if (valid[set_index][1] && (tags1[set_index] == tag) && i_req_wen) begin // STORE HIT - Way 1 - Write to this Cache
                            way = 1;
                            hit = 1;
                            store_hit_way1 = 1;
                            if (i_mem_ready) begin // WRITE THROUGH :  Check for whether the Memory is ready or not
                                hold_lru = 0;
                                w_o_mem_wen   = i_req_wen;
                                w_o_mem_ren   = i_req_ren;
                                w_o_mem_wdata = t_masked_i_req_wdata;  
                                w_o_mem_addr  = i_req_addr;
                            end  
                            else begin
                                hold_lru = 1;
                                // next_state = MEM_ACCESS;                            
                                next_state = CHK_TAG;                            
                                // w_o_busy = 1;                            
                            end                                                      
                        end
                        else begin  // GOING TO READ THE BLOCK FROM MEMORY IN CASE OF BOTH THE MISSES
                            next_state = MEM_ACCESS;
                            miss_from_chk_tag = 1;
                            w_o_busy = 1;
                            hold_lru = 1;
                        end
                end

                MEM_ACCESS: begin
                        hold_lru = 1;
                        // if (f_store_hit) begin // STORE HIT - Write through  to MEM (checking the flopped store hit value)
                        //     mem_access_store_hit = 1; // temp variable for debug 
                        //     if (i_mem_ready) begin // WRITE THROUGH :  Check for whether the Memory is ready or not
                        //         w_o_mem_wen   = f_i_req_wen;
                        //         w_o_mem_ren   = f_i_req_ren;
                        //         w_o_mem_wdata = f_t_masked_i_req_wdata;  
                        //         w_o_mem_addr  = f_i_req_addr;
                        //         next_state = CHK_TAG; // Have to go back to Check Tag once Write through to Memory is completed
                        //     end 
                        // end
                        //else if (reg_i_req_ren && !reg_i_req_wen) begin  // LOAD MISS - Checking for the read enable high case
                        if (reg_i_req_ren && !reg_i_req_wen) begin  // LOAD MISS - Checking for the read enable high case
                            w_o_busy = 1;
                            mem_access_load_miss = 1; // temp variable for debug 
                            if (i_mem_ready) begin // Check for whether the Memory is ready or not
                                    w_o_mem_wen = reg_i_req_wen;
                                    w_o_mem_ren = reg_i_req_ren;
                                    w_o_mem_addr = {reg_i_req_addr[31:4],counter_4,2'b00};                      
                                    next_state = WRITE_CACHE; // We need to write the read value into the cache - But we need to do it on a counter 4 basis to read 4 words one by one
                                    w_o_busy = 1;
                            end
                        end
                        else if (!reg_i_req_ren && reg_i_req_wen) begin // STORE MISS - Read block from MEM - Write to Cache and Update Cache and Write through  to MEM
                            w_o_busy = 1;
                            mem_access_store_miss = 1; // temp variable for debug 
                            if (i_mem_ready) begin // Check for whether the Memory is ready or not
                                    w_o_mem_wen = !reg_i_req_wen; // We need to read now first
                                    w_o_mem_ren = !reg_i_req_ren; // We need to read now first
                                    w_o_mem_addr = {reg_i_req_addr[31:4],counter_4,2'b00}; // TODO : BASE ADDRESS OF THE BLOCK ADDRESS IS REQUIRED FIRST AND THEN INCREMENT USING THE COUNT - Do I need to flop it before i give it?? - I dont think so
                                    next_state = WRITE_CACHE; // We need to write the read value into the cache - But we need to do it on a counter 4 basis to read 4 words one by one
                                    w_o_busy = 1;
                            end
                        end
                end

                WRITE_CACHE : begin
                    w_o_busy = 1;
                    hold_lru = 1;
                    if (i_mem_valid && reg_i_req_ren) begin// LOAD MISS - Waiting for valid will ensure that we wait for how much ever time the memory takes to give back the valid data
                        load_miss_cache_write = 1; // Write the data coming from the Memory to the cache
                        if (cnt_4_done == 1'b0) begin
                            cnt_4 = 1; // Increment the counter only when Write cache is done 
                            next_state = MEM_ACCESS;
                            hold_lru = 1;
                        end
                        else if (cnt_4_done == 1'b1) begin
                            cnt_4_clr = 1;
                            next_state = CHK_TAG;
                            hold_lru = 0;
                        end    
                    end

                    else if (i_mem_valid && reg_i_req_wen) begin// STORE MISS - Waiting for valid will ensure that we wait for how much ever time the memory takes to give back the valid data
                        store_miss_cache_write = 1; // Write the data coming from the Memory to the cache
                        if (cnt_4_done == 1'b0) begin
                            cnt_4 = 1; // Increment the counter only when Write cache is done 
                            next_state = MEM_ACCESS;
                            hold_lru = 1;
                        end
                        else if (cnt_4_done == 1'b1) begin
                            cnt_4_clr = 1;
                            // Do the writing to cache  ///  && (tags0[set_index] == tag)
                            do_the_write_cache = 1;
                            if (valid[f_set_index][lru[f_set_index]]) begin
                                if(!lru[f_set_index]) begin //  Way 0 
                                    if (tags0[f_set_index] == f_tag) begin
                                        new_store_hit_way0 = 1;
                                    end
                                else begin
                                    if (tags1[f_set_index] == f_tag) begin
                                        new_store_hit_way1 = 1;
                                        end
                                    end
                                end
                            end
                            // Send the write through stuff to memory
                            if (i_mem_ready) begin // WRITE THROUGH :  Check for whether the Memory is ready or not
                                hold_lru = 0;
                                w_o_mem_wen   = reg_i_req_wen;
                                w_o_mem_ren   = reg_i_req_ren;
                                w_o_mem_wdata = reg_i_req_wdata;  
                                w_o_mem_addr  = reg_i_req_addr;
                                next_state    = CHK_TAG;
                            end
                        end    
                    end
                end
                default : begin
                    next_state = CHK_TAG;
                end
            endcase
        end
endmodule

`default_nettype wire
