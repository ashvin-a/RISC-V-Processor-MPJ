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

    // Fill in your implementation here.

    ////Local wires////
    reg  cnt_4_clr; // To clear the counter once 4 times count is done 
    reg  cnt_4;
    reg  load_miss_cache_write;
    reg  store_miss_cache_write;
    reg  f_lru; // Is it ok to assign them as 0 at first?
    reg  store_hit_way0;
    reg  store_hit_way1;
    wire [31:0] t_masked_i_req_wdata;

    reg read_hit_way0;
    reg read_hit_way1;
    reg miss_from_chk_tag;

    reg  w_o_busy;
    //reg  reg_o_busy;
    reg  [31:0] w_o_res_rdata;
    reg  [31:0] w_o_mem_addr;
    reg  w_o_mem_wen;
    reg  w_o_mem_ren;
    reg  [31:0] w_o_mem_wdata;

    wire [4:0] set_index;
    wire [3:0] offset;
    wire [22:0] tag;
    wire cnt_4_done;

    reg [1:0] counter_4;
    reg reg_i_mem_inputs;
    reg w_o_busy_clr;
    reg way;
    reg hit;
    reg hold_lru;

    wire [4:0] f_set_index;
    wire [3:0] f_offset;
    wire [22:0] f_tag;

    reg  new_store_hit_way0;
    reg  new_store_hit_way1;
    
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
    assign t_masked_i_req_wdata =   (i_req_mask == 4'b0001) ? {{24{i_req_wdata[7]}},i_req_wdata[7:0]} :                   // Applicable for sb
                        (i_req_mask == 4'b0010) ? {{16{i_req_wdata[15]}},i_req_wdata[15:8],8'b0} :             // Applicable for sb
                        (i_req_mask == 4'b0100) ? {{8{i_req_wdata[23]}},i_req_wdata[23:16],16'b0} :            // Applicable for sb
                        (i_req_mask == 4'b1000) ? {i_req_wdata[7:0],24'b0} :                   // Applicable for sb
                        (i_req_mask == 4'b0011) ? {{16{i_req_wdata[15]}},i_req_wdata[15:0]} :  // Applicable for sh - SIGN EXTENSION by default
                        (i_req_mask == 4'b1100) ? {i_req_wdata[15:0],16'b0} :                  // Applicable for sh
                        (i_req_mask == 4'b1111) ? i_req_wdata :  
                        32'bxxxx;                                                               // Default case

    //////Local assignments////////////
    assign set_index = i_req_addr[8:4];
    assign offset = i_req_addr[3:0];
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
                    lru[set_index] = ~lru[set_index];
                else 
                    lru[set_index] = lru[set_index];
            end
            else if (!hit && !hold_lru)
                lru[set_index] = ~lru[set_index];
            else if (hold_lru)
                lru[set_index] = lru[set_index];
        end
    
    always @ (posedge i_clk)
        if (store_hit_way0) begin// write to that particular set/way in the cache
            datas0[set_index][offset] <= t_masked_i_req_wdata; 
        end
        else if (store_hit_way1) begin
            datas1[set_index][offset] <= t_masked_i_req_wdata; 
        end

    always @ (posedge i_clk)
        if (new_store_hit_way0) begin// write to that particular set/way in the cache
            datas0[f_set_index][f_offset] <= reg_i_req_wdata; 
        end
        else if (new_store_hit_way1) begin
            datas1[f_set_index][f_offset] <= reg_i_req_wdata; 
        end

    always @ (posedge i_clk)
        if (load_miss_cache_write | store_miss_cache_write) begin
            if (!lru[set_index]) begin
                datas0[set_index][{(counter_4),2'b00}] <= i_mem_rdata; // Data coming from the Memory
                if(cnt_4_done)
                    valid[set_index][0] <= 1'b1;
                else                     
                    valid[set_index][0] <= valid[set_index][0];
                tags0[set_index] <= tag;
            end
            else if (lru[set_index]) begin
                datas1[set_index][{counter_4,2'b00}] <= i_mem_rdata; // Data coming from the Memory 
                if(cnt_4_done)
                    valid[set_index][1] <= 1'b1;
                else                     
                    valid[set_index][1] <= valid[set_index][1];
                tags1[set_index] <= tag;
            end
        end

    // always @ (posedge i_clk)
    //     if (load_miss_cache_write | store_miss_cache_write) begin
    //         if (valid[set_index][0] == 0 && i_mem_valid) begin
    //             // lru[set_index] <= 1'b1;
    //             datas0[set_index][{(counter_4),2'b00}] <= i_mem_rdata; // Data coming from the Memory
    //             if(cnt_4_done)
    //                 valid[set_index][0] <= 1'b1;
    //             else                     
    //                 valid[set_index][0] <= valid[set_index][0];
    //             tags0[set_index] <= tag;
    //         end
    //         else if (valid[set_index][1] == 0 && i_mem_valid) begin
    //             // lru[set_index] <= 1'b0;
    //             datas1[set_index][{counter_4,2'b00}] <= i_mem_rdata; // Data coming from the Memory 
    //             if(cnt_4_done)
    //                 valid[set_index][1] <= 1'b1;
    //             else                     
    //                 valid[set_index][1] <= valid[set_index][1];
    //             tags1[set_index] <= tag;
    //         end
    //         else begin// Eviction and Write overwrite - Eviction and write a full block - 4 transactions - 4 words
    //             if(lru[set_index] && i_mem_valid) begin
    //                 datas1[set_index][{counter_4,2'b00}] <= i_mem_rdata; // Data coming from the Memory 
    //                 tags1[set_index] <= tag;
    //             end
    //             else if(!lru[set_index] && i_mem_valid) begin
    //                 datas0[set_index][{counter_4,2'b00}] <= i_mem_rdata; // Data coming from the Memory 
    //                 tags0[set_index] <= tag;
    //             end
    //         end
    //     end

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
    
    ///////////////////////////////////////////////////////
    //// Register to flop Store lru assignment/////////////
    ///////////////////////////////////////////////////////
    always @ (posedge i_clk)
        lru[set_index] <= f_lru;

    // /////////////////////////////////////////
    // //// Register to flop o_busy/////////////
    // /////////////////////////////////////////
    // always @ (posedge i_clk)
    //     if (w_o_busy)
    //         reg_o_busy <= 1'b1;
    //     else if (w_o_busy_clr)
    //         reg_o_busy <= 1'b0;




    always @ (posedge i_clk) // I dont want to store anything if its not a miss
        //if (miss_from_chk_tag) begin
        if (w_o_busy) begin
            reg_i_req_ren   <= i_req_ren;
            reg_i_req_wen   <= i_req_wen;
            reg_i_req_addr  <= i_req_addr;
            reg_i_req_wdata <= t_masked_i_req_wdata;
        end

    assign f_set_index = reg_i_req_addr[8:4];
    assign f_offset = reg_i_req_addr[3:0];
    assign f_tag = reg_i_req_addr[31:9];            

    always @ (posedge i_clk)
        if (store_hit_way0 | store_hit_way1) begin
            f_i_req_wen <= i_req_wen;
            f_i_req_ren <= i_req_ren;
            f_t_masked_i_req_wdata <= t_masked_i_req_wdata; 
            f_i_req_addr <= i_req_addr;
        end

    localparam CHK_TAG = 2'b00;
    localparam MEM_ACCESS = 2'b01;
    localparam WRITE_CACHE = 2'b10;
    reg [1:0] state,next_state;
    /////////////////////////////////////
    ///////State Machine////////////////
    /////////////////////////////////////
    always @ (posedge i_clk)
        if(i_rst)
            state <= CHK_TAG;
        else 
            state <= next_state;

    always @ (*) begin
        w_o_busy = 0;
        w_o_res_rdata = 0;
        w_o_mem_addr = 0;
        w_o_mem_wen = 0;
        w_o_mem_ren = 0; // Illegal to keep both as the same TODO
        w_o_mem_wdata = 0;
        cnt_4_clr = 0; // To clear the counter once 4 times count is done 
        cnt_4 = 0;
        load_miss_cache_write = 0;
        store_miss_cache_write = 0;
        f_lru = 0; // Is it ok to assign them as 0 at first?
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

        next_state = state;
        if (!i_rst) begin  // To avoid reset condition - BIG TODOOOOOOOOOOOOOOOO !!!!
        //if (reg_valid) begin  // To avoid reset condition - BIG TODOOOOOOOOOOOOOOOO !!!!
            case (state)
                CHK_TAG: begin
                        if(valid[set_index][0] && (tags0[set_index] == tag) && i_req_ren) begin// LOAD HIT - Way 0
                            w_o_res_rdata = datas0[set_index][offset];
                            f_lru = 1; // Flopping it so that it is not lost once lru gets updated
                            way = 0; 
                            hit = 1;
                            hold_lru = 0;
                            read_hit_way0 = 1;
                        end
                        else if (valid[set_index][1] && (tags1[set_index] == tag) && i_req_ren) begin// LOAD HIT - Way 1
                            w_o_res_rdata = datas1[set_index][offset];
                            f_lru = 0; // Flopping it so that it is not lost once lru gets updated
                            way = 1;
                            hit = 1;
                            hold_lru = 0;
                            read_hit_way1 = 1;
                        end
                        else if(valid[set_index][0] && (tags0[set_index] == tag) && i_req_wen) begin // STORE HIT - Way 0  - Write to this Cache
                            way = 0;
                            hit = 1;
                            store_hit_way0 = 1; // To ensure write to cache is done even it is a hit which will be followed by write through to mem
                            f_lru = 1; // Flopping it so that it is not lost once lru gets updated
                            if (i_mem_ready) begin // WRITE THROUGH :  Check for whether the Memory is ready or not
                                hold_lru = 0;
                                w_o_mem_wen   = i_req_wen;
                                w_o_mem_ren   = i_req_ren;
                                w_o_mem_wdata = t_masked_i_req_wdata;  
                                w_o_mem_addr  = i_req_addr;
                            end
                            else begin
                                hold_lru = 1;
                                next_state = MEM_ACCESS;
                                w_o_busy = 1;                            
                            end
                        end
                        else if (valid[set_index][1] && (tags1[set_index] == tag) && i_req_wen) begin // STORE HIT - Way 1 - Write to this Cache
                            way = 1;
                            hit = 1;
                            store_hit_way1 = 1;
                            f_lru = 0; // Flopping it so that it is not lost once lru gets updated
                            if (i_mem_ready) begin // WRITE THROUGH :  Check for whether the Memory is ready or not
                                hold_lru = 0;
                                w_o_mem_wen   = i_req_wen;
                                w_o_mem_ren   = i_req_ren;
                                w_o_mem_wdata = t_masked_i_req_wdata;  
                                w_o_mem_addr  = i_req_addr;
                            end  
                            else begin
                                hold_lru = 1;
                                next_state = MEM_ACCESS;                            
                                w_o_busy = 1;                            
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
                        if (f_store_hit) begin // STORE HIT - Write through  to MEM (checking the flopped store hit value)
                            if (i_mem_ready) begin // WRITE THROUGH :  Check for whether the Memory is ready or not
                                w_o_mem_wen   = f_i_req_wen;
                                w_o_mem_ren   = f_i_req_ren;
                                w_o_mem_wdata = f_t_masked_i_req_wdata;  
                                w_o_mem_addr  = f_i_req_addr;
                                next_state = CHK_TAG; // Have to go back to Check Tag once Write through to Memory is completed
                            end 
                        end
                        else if (reg_i_req_ren && !reg_i_req_wen) begin  // LOAD MISS - Checking for the read enable high case
                            w_o_busy = 1;
                            if (i_mem_ready) begin // Check for whether the Memory is ready or not
                                    w_o_mem_wen = !reg_i_req_wen;
                                    w_o_mem_ren = reg_i_req_ren;
                                    w_o_mem_addr = {reg_i_req_addr[31:4],counter_4,2'b00};                      
                                    next_state = WRITE_CACHE; // We need to write the read value into the cache - But we need to do it on a counter 4 basis to read 4 words one by one
                                    w_o_busy = 1;
                            end
                        end
                        else if (!reg_i_req_ren && reg_i_req_wen) begin // STORE MISS - Read block from MEM - Write to Cache and Update Cache and Write through  to MEM
                            w_o_busy = 1;
                            if (i_mem_ready) begin // Check for whether the Memory is ready or not
                                    w_o_mem_wen = 0;
                                    w_o_mem_ren = 1;
                                    w_o_mem_addr = {reg_i_req_addr[31:0],counter_4,2'b00}; // TODO : BASE ADDRESS OF THE BLOCK ADDRESS IS REQUIRED FIRST AND THEN INCREMENT USING THE COUNT - Do I need to flop it before i give it?? - I dont think so
                                    next_state = WRITE_CACHE; // We need to write the read value into the cache - But we need to do it on a counter 4 basis to read 4 words one by one
                                    w_o_busy = 1;
                            end
                        end
                end
                WRITE_CACHE : begin
                    w_o_busy = 1;
                    // if (reg_i_req_ren) begin
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
                    //end

                    if (i_mem_valid && reg_i_req_wen) begin// STORE MISS - Waiting for valid will ensure that we wait for how much ever time the memory takes to give back the valid data
                        store_miss_cache_write = 1; // Write the data coming from the Memory to the cache
                        if (cnt_4_done == 1'b0) begin
                            cnt_4 = 1; // Increment the counter only when Write cache is done 
                            next_state = MEM_ACCESS;
                            hold_lru = 1;
                        end
                        else if (cnt_4_done == 1'b1) begin
                            cnt_4_clr = 1;
                            
                            // Do the writing to cache  ///  && (tags0[set_index] == tag)
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
                                next_state = CHK_TAG;
                            end

                        end    
                    end
                    // else if (i_mem_valid && reg_i_req_wen) begin// STORE MISS
                    //     store_miss_cache_write = 1; // Write the block of data coming from the Memory to the cache first and then update that address location in the cache
                    //     if (cnt_4_done == 1'b0) begin
                    //         if(store_miss_cache_write_done) begin
                    //             cnt_4 = 1; // Increment the counter only when Write cache is done so that it gets captured by the MEM_ACCESS state when it sents the new read address to the MEM
                    //             next_state = MEM_ACCESS;
                    //         end
                    //         else 
                    //             next_state = WRITE_CACHE;
                    //     end
                    //     else if (cnt_4_done == 1'b1) begin
                    //         cnt_4_clr = 1;
                    //         w_o_busy_clr = 1; // We will explicitly make o_busy low because we know that now the particular address is part of the cache and it will be a Hit if checked again
                    //         if(valid[set_index][0] && (tags0[set_index] == tag) && i_req_wen) begin //  Way 0  - Write to this Cache - // We expect the i_req_addr input to give us the location of the cache to which we should write.
                    //             store_hit_way0 = 1;
                    //             next_state = MEM_ACCESS; // Then go to MEM ACCESS for writing it through to memory
                    //         end
                    //         else if (valid[set_index][1] && (tags1[set_index] == tag) && reg_i_req_wen) begin //  Way 1 - Write to this Cache
                    //             store_hit_way1 = 1;
                    //             next_state = MEM_ACCESS; // Then go to MEM ACCESS for writing it through to memory
                    //         end
                    //     end
                    // end
                end
                default : begin
                    next_state = CHK_TAG;
                end
            endcase
        end
    end
endmodule
// Do we need to mask for the out data in case of load?
// Make sure o_busy assignments through states are correct
// Do i need to make cnt_4_clr = 1 as default?
// the cnt_4 increment in WRITE_CACHE will the counter increment be captured in the next MEM_ACCESS STATE?
// To write mask logic for the data
// Does the read happen combinationally from the Memory?
// Do we need to make o_busy high whenever even during the STORE HIT MEM ACCESS CASE
// Read the descirption to see they are asking us to mask the last 2 bits to 0 of addresses all the time - whether we are doing it or not
`default_nettype wire
