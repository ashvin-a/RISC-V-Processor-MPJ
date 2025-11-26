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

    // Fill in your implementation here.

    ////Local wires////
    reg  cnt_4_clr = 0; // To clear the counter once 4 times count is done
    reg  cnt_4 = 0;
    reg  load_miss_cache_write = 0;
    reg  store_miss_cache_write = 0;
    reg  f_lru = 0; // Is it ok to assign them as 0 at first?
    reg  store_hit_way0 = 0;
    reg  store_hit_way1 = 0;
    wire [31:0] t_masked_i_req_wdata;

    reg    w_o_busy = 0;
    reg  [31:0]  w_o_res_rdata = 0;
    reg  [31:0]  w_o_mem_addr = 0;
    reg    w_o_mem_wen = 0;
    reg    w_o_mem_ren = 1; // Illegal to keep both as the same
    reg  [31:0]  w_o_mem_wdata = 0;

    wire [4:0] set_index;
    wire [3:0] offset;
    wire [22:0] tag;
    wire cnt_4_done;
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

    ////////////////////////////////////////////////////
    //// Register to Write to Cache during Store Hit////
    ////////////////////////////////////////////////////
    /// no reset condition for cache write needed //////
    ////////////////////////////////////////////////////
    reg load_miss_cache_write_done;
    reg store_miss_cache_write_done;
    reg store_hit_way0_done; // to communicate to state machine that write to cache is done in this case
    reg store_hit_way1_done; // to communicate to state machine that write to cache is done in this case
    always @ (posedge i_clk)
        if (store_hit_way0) begin// write to that particular set/way in the cache
            store_hit_way0_done <= 1;
            datas0[set_index][offset] <= t_masked_i_req_wdata; // TODO : To mask it in the cache module
        end
        else if (store_hit_way1) begin
            store_hit_way1_done <= 1;
            datas1[set_index][offset] <= t_masked_i_req_wdata; // TODO : To mask it in the cache module
        end

    always @ (posedge i_clk)
        if(i_rst) begin
            load_miss_cache_write_done  <= 0;
            store_miss_cache_write_done <= 0;
        end
        else if (load_miss_cache_write | store_miss_cache_write) begin
            load_miss_cache_write_done  <= 1;
            store_miss_cache_write_done <= 1;
            if (valid[0] == 0) begin
                lru[set_index] <= 1;
                datas0[set_index][offset] <= i_mem_rdata; // Data coming from the Memory -> TODO : To mask it in the cache module
            end
            else if (valid[1] == 0) begin
                lru[set_index] <= 0;
                datas1[set_index][offset] <= i_mem_rdata; // Data coming from the Memory -> TODO : To mask it in the cache module
            end
            else begin// Eviction and Write overwrite
                if(lru[set_index]) begin
                    datas1[set_index][offset] <= i_mem_rdata; // Data coming from the Memory -> TODO : To mask it in the cache module
                end
                else if(!lru[set_index]) begin
                    datas0[set_index][offset] <= i_mem_rdata; // Data coming from the Memory -> TODO : To mask it in the cache module
                end
                lru[set_index] <= ~lru[set_index];
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
    reg [1:0] counter_4;
    always @ (posedge i_clk)
        if(i_rst)
            counter_4 <= 0;
        else if (cnt_4_clr)
            counter_4 <= 0;
        else if (cnt_4)
            counter_4 <= counter_4 + 2'b1;
    assign cnt_4_done = &counter_4;
   
    //////////////////////////////////////////////////////////////////////////
    //// Register to flop Store lru assignment/////////////
    //////////////////////////////////////////////////////////////////////////
    always @ (posedge i_clk)
        lru[set_index] <= f_lru;

    //////Local assignments////////////
    assign set_index = i_req_addr[8:4];
    assign offset = i_req_addr[3:0];
    assign tag = i_req_addr[31:9];            

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
        w_o_mem_ren = 1; // Illegal to keep both as the same
        w_o_mem_wdata = 0;
        cnt_4_clr = 0; // To clear the counter once 4 times count is done
        cnt_4 = 0;
        load_miss_cache_write = 0;
        store_miss_cache_write = 0;
        f_lru = 0; // Is it ok to assign them as 0 at first?
        store_hit_way0 = 0;
        store_hit_way1 = 0;
        next_state = state;

        case (state)
            CHK_TAG: begin
                    if(valid[set_index][0] && (tags0[set_index] == tag) && i_req_ren) begin// LOAD HIT - Way 0
                        w_o_res_rdata = datas0[set_index][offset];
                        f_lru = 1; // Flopping it so that it is not lost once lru gets updated
                    end
                    else if (valid[set_index][1] && (tags1[set_index] == tag) && i_req_ren) begin// LOAD HIT - Way 1
                        w_o_res_rdata = datas1[set_index][offset];
                        f_lru = 0; // Flopping it so that it is not lost once lru gets updated
                    end
                    else if(valid[set_index][0] && (tags0[set_index] == tag) && i_req_wen) begin // STORE HIT - Way 0  - Write to this Cache
                        store_hit_way0 = 1;
                        next_state = MEM_ACCESS;
                        f_lru = 1; // Flopping it so that it is not lost once lru gets updated
                    end
                    else if (valid[set_index][1] && (tags1[set_index] == tag) && i_req_wen) begin // STORE HIT - Way 1 - Write to this Cache
                        store_hit_way1 = 1;
                        next_state = MEM_ACCESS;
                        f_lru = 0; // Flopping it so that it is not lost once lru gets updated
                    end
                    else begin                               // GOING TO READ THE BLOCK FROM MEMORY IN CASE OF BOTH THE MISSES
                        next_state = MEM_ACCESS;
                        w_o_busy = 1; // Will make busy whenever we have a MISS and we are trying to acess the memory
                    end
            end
            MEM_ACCESS: begin
                if (i_mem_ready) begin // Check for whether the Memory is ready or not
                    if (f_store_hit) begin // STORE HIT - Write through  to MEM (checking the flopped store hit value)
                        w_o_mem_wen = 1;
                        w_o_mem_ren = 0;
                        w_o_mem_wdata = t_masked_i_req_wdata;  // TODO : Do I need to flop it before i give it?? -  - I dont think so
                        w_o_mem_addr = i_req_addr; // TODO : Do I need to flop it before i give it?? - I dont think so - DO I NEED TO MAKE THE LAST 2 BITS TO 0?
                        next_state = CHK_TAG; // Have to go back to Check Tag once Write through to Memory is completed
                    end
                    else if (i_req_ren && !i_req_wen) begin  // LOAD MISS - Checking for the read enable high case
                        w_o_mem_wen = 0;
                        w_o_mem_ren = 1;
                        w_o_mem_addr = {i_req_addr[31:2],counter_4}; // TODO : BASE ADDRESS OF THE BLOCK ADDRESS IS REQUIRED FIRST AND THEN INCREMENT USING THE COUNT - Do I need to flop it before i give it?? - I dont think so                      
                        next_state = WRITE_CACHE; // We need to write the read value into the cache - But we need to do it on a counter 4 basis to read 4 words one by one
                        w_o_busy = 1; // TODO : Is it Redundant??
                    end
                    else if (!i_req_ren && i_req_wen) begin // STORE MISS - Read block from MEM - Write to Cache and Update Cache and Write through  to MEM
                        //READ a BLOCK FROM THE MEMORY - 4 consecutive read requests
                        w_o_mem_wen = 0;
                        w_o_mem_ren = 1;
                        w_o_mem_addr = {i_req_addr[31:2],counter_4}; // TODO : BASE ADDRESS OF THE BLOCK ADDRESS IS REQUIRED FIRST AND THEN INCREMENT USING THE COUNT - Do I need to flop it before i give it?? - I dont think so
                        next_state = WRITE_CACHE; // We need to write the read value into the cache - But we need to do it on a counter 4 basis to read 4 words one by one
                        w_o_busy = 1; // TODO : Is it Redundant??
                    end
                end
            end
            WRITE_CACHE : begin
                if (i_mem_valid && i_req_ren) begin// LOAD MISS - Waiting for valid will ensure that we wait for how much ever time the memory takes to give back the valid data - But how will we increment the cou
                    load_miss_cache_write = 1; // Write the data coming from the Memory to the cache
                    // We will wait until cache write is done before next request - to ensure that we get the incremented counter value in the next MEM_ACCESS state above
                    if (cnt_4_done == 1'b0) begin
                        if (load_miss_cache_write_done) begin
                            cnt_4 = 1; // Increment the counter only when Write cache is done so that
                            next_state = MEM_ACCESS;
                        end
                        else
                            next_state = WRITE_CACHE; // Stay in this state until we have the writeto cache done in case of load miss                            
                    end
                    else if (cnt_4_done == 1'b1) begin
                        if (load_miss_cache_write_done) begin                        
                            cnt_4_clr = 1;
                            next_state = CHK_TAG; // This will ensure o_busy also will default to 0
                        end    
                        else
                            next_state = WRITE_CACHE; // Stay in this state until we have the writeto cache done in case of load miss                                          
                    end
                   
                end
                else if (i_mem_valid && i_req_wen) begin// STORE MISS
                    store_miss_cache_write = 1; // Write the block of data coming from the Memory to the cache first and then update that address location in the cache
                    if (cnt_4_done == 1'b0) begin
                        if(store_miss_cache_write_done) begin
                            cnt_4 = 1; // Increment the counter only when Write cache is done so that it gets captured by the MEM_ACCESS state when it sents the new read address to the MEM
                            next_state = MEM_ACCESS;
                        end
                        else
                            next_state = WRITE_CACHE;
                    end
                    else begin
                        cnt_4_clr = 1;
                        w_o_busy = 0; // We will explicitly make o_busy low because we know that now the particular address is part of the cache and it will be a Hit if checked again
                        if(valid[set_index][0] && (tags0[set_index] == tag) && i_req_wen) begin //  Way 0  - Write to this Cache - // We expect the i_req_addr input to give us the location of the cache to which we should write.
                            store_hit_way0 = 1;
                            if(store_hit_way0_done) // TODO : Should we wait for the Cache write to be completed before writing it through to the Memory?
                                next_state = MEM_ACCESS; // Then go to MEM ACCESS for writing it through to memory
                        end
                        else if (valid[set_index][1] && (tags1[set_index] == tag) && i_req_wen) begin //  Way 1 - Write to this Cache
                            store_hit_way1 = 1;
                            if(store_hit_way1_done) // TODO : Should we wait for the Cache write to be completed before writing it through to the Memory?
                                next_state = MEM_ACCESS; // Then go to MEM ACCESS for writing it through to memory
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

// Do we need to mask for the out data in case of load?
// Make sure o_busy assignments through states are correct
// Do i need to make cnt_4_clr = 1 as default?
// the cnt_4 increment in WRITE_CACHE will the counter increment be captured in the next MEM_ACCESS STATE?
// To write mask logic for the data
// Does the read happen combinationally from the Memory?
// Do we need to make o_busy high whenever even during the STORE HIT MEM ACCESS CASE
// Read the descirption to see they are asking us to mask the last 2 bits to 0 of addresses all the time - whether we are doing it or not

`default_nettype wire