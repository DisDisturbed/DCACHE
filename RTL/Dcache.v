`timescale 1ns / 1ps
`default_nettype none

// author yusuf mirac göcen (3rd year ee student at itü)
// specifications
// blocking write-back data cache
//
// organization assumptions:
// - direct mapped
// - 64 lines
// - 16 words per line (64 bytes per line)
// - write-back, write-allocate
// - single outstanding miss
// - fixed 16-beat incr burst for refill and eviction
// - no axi id support
//
// address breakdown (32-bit):
// [31:12] tag
// [11:6]  line index (64 lines)
// [5:2]   word index (16 words per line)
// [1:0]   byte offset



// TODO: Implement Word First forwarding to reduce stall cycles on read misses.
//core should hold outputs stable ortherwise it will not work

module dcache #(
      parameter ADDR_WIDTH = 32,
      parameter DATA_WIDTH = 32,
      parameter TAG_BITS   = ADDR_WIDTH - 12,
      parameter LINE_SIZE  = 16
)(
      // clock and reset
      input  wire                    clk,
      input  wire                    rst,

      // core side interface
      input  wire [ADDR_WIDTH-1:0]   core_addr_i,
      input  wire                    core_req_i,
      input  wire                    core_wen_i,
      input  wire [3:0]              core_wstrb_i,
      input  wire [DATA_WIDTH-1:0]   core_wdata_i,
      output wire [DATA_WIDTH-1:0]   core_rdata_o,
      output reg                     stall_o,

      // AXI4 MASTER INTERFACE

      // 1. Write Address Channel
      output reg                     m_axi_awvalid,
      input  wire                    m_axi_awready, 
      output wire [31:0]             m_axi_awaddr,
      // REQUIRED STATIC SIGNALS
      output wire [3:0]              m_axi_awid,    
      output wire [7:0]              m_axi_awlen,   
      output wire [2:0]              m_axi_awsize,  
      output wire [1:0]              m_axi_awburst, 

      // 2. Write Data Channel
      output reg                     m_axi_wvalid, 
      input  wire                    m_axi_wready,  
      output wire [31:0]             m_axi_wdata,
      output wire [3:0]              m_axi_wstrb,   
      output reg                     m_axi_wlast,

      // 3. Write Response Channel
      input  wire                    m_axi_bvalid,
      output wire                    m_axi_bready,  
      input  wire [1:0]              m_axi_bresp,   
      input  wire [3:0]              m_axi_bid,    

      // 4. Read Address Channel
      output reg                     m_axi_arvalid, 
      input  wire                    m_axi_arready, 
      output wire [31:0]             m_axi_araddr,
      // REQUIRED STATIC SIGNALS
      output wire [3:0]              m_axi_arid,    
      output wire [7:0]              m_axi_arlen,   
      output wire [2:0]              m_axi_arsize,  
      output wire [1:0]              m_axi_arburst, 

      // 5. Read Data Channel
      input  wire                    m_axi_rvalid,
      output wire                    m_axi_rready, 
      input  wire [31:0]             m_axi_rdata,
      input  wire                    m_axi_rlast,   
      input  wire [1:0]              m_axi_rresp,   
      input  wire [3:0]              m_axi_rid      
);

    // AXI4 CONSTANT DRIVERS 
    assign m_axi_awid    = 4'd0;
    assign m_axi_awlen   = 8'd15;       // Burst Length: 15+1 = 16 beats
    assign m_axi_awsize  = 3'b010;      // Burst Size: 4 Bytes (32-bit)
    assign m_axi_awburst = 2'b01;       // Burst Type: INCR

    assign m_axi_wstrb   = 4'b1111;     // Cache eviction writes full words

    assign m_axi_bready  = 1'b1;        // Always accept write responses (no stalls on B channel)

    assign m_axi_arid    = 4'd0;
    assign m_axi_arlen   = 8'd15;       // Burst Length: 15+1 = 16 beats
    assign m_axi_arsize  = 3'b010;      // Burst Size: 4 Bytes
    assign m_axi_arburst = 2'b01;       // Burst Type: INCR

    // During refill, we are always ready to consume data as fast as it comes
    // (Assuming D-Cache RAM writes are 1 cycle)
    assign m_axi_rready  = 1'b1;

    // special mmio address used to trigger software flush
    // send the number of the line to this address and cache will send it to the main memory
    localparam FLUSH_CTRL_ADDR = 32'h0000BEEF;

    // stage0 pipeline registers
    // this stage captures the core request when not stalled
    // tag ram output aligns with this stage

    reg [TAG_BITS-1:0] addr_tag_stage0;      // tag extracted from core address
    reg                req_stage0;           // latched request
    reg                we_stage0;            // latched write enable (1 = write)
    reg [5:0]          line_index_stage0;    // index into tag/data ram
    reg [3:0]          word_index_stage0;    // word offset inside line
    reg                is_flush_stage0;      // indicates flush command
    reg [5:0]          flush_line_stage0;    // line to flush
    reg [3:0]          wstrb_stage0;         // byte enable
    reg [31:0]         wdata_stage0;         // write data

    // active low from core changed for the internal use
    wire core_wr_cmd = !core_wen_i;

    // detect flush mmio write
    wire is_flush_mmio_write =
        core_req_i && core_wr_cmd && (core_addr_i == FLUSH_CTRL_ADDR);
    
    
    // stage0 pipeline capture
    // Add a registered "request accepted" flag
reg req_accepted;

always @(posedge clk) begin
    if (rst) begin
        addr_tag_stage0   <= {TAG_BITS{1'b0}};
        req_stage0        <= 1'b0;
        we_stage0         <= 1'b0;
        line_index_stage0 <= 6'd0;
        word_index_stage0 <= 4'd0;
        is_flush_stage0   <= 1'b0;
        flush_line_stage0 <= 6'd0;
        wstrb_stage0      <= 4'd0;
        wdata_stage0      <= 32'd0;
        req_accepted      <= 1'b0;
    end 
    else if (state == WAIT) begin 
        if (we_stage0 || is_flush_stage0) begin
            req_stage0      <= 1'b0;
            is_flush_stage0 <= 1'b0;
        end
        req_accepted <= 1'b0;  // NEW: Clear on WAIT->IDLE transition
    end 
    // KEY CHANGE: Only latch when IDLE AND no request pending
    else if (state == IDLE && !req_accepted && core_req_i) begin 
        addr_tag_stage0   <= core_addr_i[31:12];
        req_stage0        <= core_req_i;
        we_stage0         <= core_wr_cmd;
        line_index_stage0 <= core_addr_i[11:6];
        word_index_stage0 <= core_addr_i[5:2];
        is_flush_stage0   <= is_flush_mmio_write;
        flush_line_stage0 <= is_flush_mmio_write ? core_wdata_i[11:6] : 6'd0;
        wstrb_stage0      <= core_wstrb_i;
        wdata_stage0      <= core_wdata_i;
        req_accepted      <= 1'b1;  // NEW: Mark request as captured
    end
    else if (state == IDLE && !core_req_i) begin
        req_stage0   <= 1'b0;
        req_accepted <= 1'b0;  // NEW: Clear when no request
    end
end

    // -------------------------------------------------------------------------
    // tag ram format
    // [21] dirty
    // [20] valid
    // [19:0] tag
    // -------------------------------------------------------------------------

    wire [21:0] tag_ram_out;

    wire stored_dirty = tag_ram_out[21];
    wire stored_valid = tag_ram_out[20];
    wire [19:0] stored_tag = tag_ram_out[19:0];

    // hit and miss detection
    wire HIT  = stored_valid &&
                (stored_tag == addr_tag_stage0) &&
                !is_flush_stage0;

    wire MISS = req_stage0 && !HIT && !is_flush_stage0;

    // fsm states
    // idle           : normal operation, detect hit or miss
    // send_addr_read : issue read burst address
    // refill         : receive 16 beats and write into data ram
    // wait           : one cycle cleanup state before returning to idle
    // push_addr      : issue write-back address for dirty eviction
    // push_data      : send 16 beats of dirty line
    // push_lookup    : flush lookup stage

    localparam IDLE           = 3'd0,
               SEND_ADDR_READ = 3'd1,
               REFILL         = 3'd2,
               WAIT           = 3'd3,
               PUSH_ADDR      = 3'd4,
               PUSH_DATA      = 3'd5,
               PUSH_LOOKUP    = 3'd6,
               WAIT_BRESP     = 3'd7;
    
    reg [2:0] state, next_state;

    reg [3:0] burst_count;        // counts 16-beat bursts
    reg       is_refilling;       // indicates refill phase

    // saved miss information
    reg [TAG_BITS-1:0] saved_tag;
    reg [5:0]          saved_line_index;

    reg                flush_active;

    // write after refill commit registers
    reg pending_write_commit;
    reg [3:0] pending_wstrb;
    reg [31:0] pending_wdata;
    reg [3:0] pending_word_idx;
    reg do_write_commit;

    // sequential logic for miss 
    always @(posedge clk) begin
        if (rst) begin
            saved_tag <= 0;
            saved_line_index <= 0;
            flush_active <= 0;
            pending_write_commit <= 0;
            pending_wstrb <= 0;
            pending_wdata <= 0;
            pending_word_idx <= 0;
            do_write_commit <= 0;
        end else begin
            do_write_commit <= 1'b0;

            if (state == IDLE) begin 
                if (is_flush_stage0) begin
                    saved_line_index <= flush_line_stage0;
                    saved_tag        <= {TAG_BITS{1'b0}};
                    flush_active     <= 1'b1;
                    pending_write_commit <= 1'b0;
                end
                else if (MISS) begin
                    saved_tag        <= addr_tag_stage0;
                    saved_line_index <= line_index_stage0;
                    flush_active     <= 1'b0;

                    // if miss was write, commit after refill
                    if (we_stage0) begin
                        pending_write_commit <= 1'b1;
                        pending_wstrb        <= wstrb_stage0;
                        pending_wdata        <= wdata_stage0;
                        pending_word_idx     <= word_index_stage0;
                    end else begin
                        pending_write_commit <= 1'b0;
                    end
                end
            end
            else if (state == REFILL &&
                     burst_count == 4'd15 &&
                     m_axi_rvalid &&
                     pending_write_commit) begin
                // trigger write commit after final refill beat
                do_write_commit <= 1'b1;
            end
            else if (state == WAIT) begin
                do_write_commit <= 1'b0;
                flush_active <= 1'b0;
                pending_write_commit <= 1'b0;
            end
        end
    end
    
    
    
    
    
    

    //  Performance Counters 
    reg [31:0] hit_count;
    reg [31:0] miss_count;
    reg [31:0] eviction_count;
    reg [31:0] hit_clk;
    reg [31:0] miss_clk;
    
    always @(posedge clk) begin
        
        if (rst) begin
            hit_count <= 32'b0;
            miss_count <= 32'b0;
            eviction_count <= 32'b0;
            hit_clk <= 32'b0;
            miss_clk <= 32'b0;
        end else begin
            if (HIT) 
               hit_clk <= hit_clk + 1;
            if (MISS)
               miss_clk <= miss_clk +1;
            if (state == IDLE && (doing_write_hit || doing_read_hit))
                hit_count <= hit_count + 1;
                
            if (state == IDLE && MISS)
                miss_count <= miss_count + 1;
            if (state == PUSH_ADDR && next_state == PUSH_DATA)
                eviction_count <= eviction_count + 1;
        end
    end
    
    
    // address muxing between hit path and miss path

    wire [5:0] active_line_index;
    wire [3:0] active_word_index;
    wire [ADDR_WIDTH-1:0] active_tag_addr;

    assign active_tag_addr =
    (state == IDLE && !req_accepted) ? core_addr_i :  // Only use current addr if no request pending
    (state == IDLE && req_accepted)  ? {addr_tag_stage0, line_index_stage0, 6'b0} : // Use latched addr
    {saved_tag, saved_line_index, 6'b0};  // Use saved addr for miss handling

    // hit detection helpers
    wire doing_write_hit =
        (state == IDLE) && req_stage0 && we_stage0 && HIT;

    wire doing_read_hit =
        (state == IDLE) && req_stage0 && !we_stage0 && HIT;

    // select line index source
    assign active_line_index =
    (state == IDLE && doing_read_hit && req_accepted) ? line_index_stage0 :
    (state == IDLE && doing_write_hit && req_accepted) ? line_index_stage0 :
    saved_line_index;

    // select word index source
    assign active_word_index =
        (do_write_commit) ? pending_word_idx :
        (state == IDLE && (doing_write_hit || doing_read_hit)) ?
            word_index_stage0 :
        (state == PUSH_DATA) ?
            (m_axi_wready ? burst_count + 4'd1 : burst_count) :
        burst_count;

    // data ram control

    reg ram_wr_en;
    reg tag_wr_en;
    reg dirty_bit_in;
    reg valid_bit_in;

    wire [31:0] ram_wdata =
        is_refilling    ? m_axi_rdata :
        do_write_commit ? pending_wdata :
                          wdata_stage0;

    wire [3:0] ram_wstrb =
        is_refilling    ? 4'b1111 :
        do_write_commit ? pending_wstrb : 
                          wstrb_stage0;

    wire actual_ram_wen =
        ram_wr_en || doing_write_hit || do_write_commit;
    // data ram instance
    dcache_data_ram data_ram_inst (
        .clk_i(clk),
        .req_i(1'b1),
        .wr_en(actual_ram_wen),
        .wstrb_i(ram_wstrb),
        .line_idx_i(active_line_index),
        .word_idx_i(active_word_index),
        .wdata_i(ram_wdata),
        .rdata_o(core_rdata_o)
    );
    // write-back data comes from data ram read port
    assign m_axi_wdata = core_rdata_o;
    // tag ram instance
    dcache_tag_ram tag_ram_inst (
        .clk_i(clk),
        .rst_i(rst),
        .addr_i(active_tag_addr),
        .dirty_in(dirty_bit_in),
        .valid_in(valid_bit_in),
        .wr_en(tag_wr_en),
        .tag_o(tag_ram_out)
    );

    // memory burst addresses
    assign m_axi_araddr = {saved_tag, saved_line_index, 6'b0};
    assign m_axi_awaddr  = {stored_tag, saved_line_index, 6'b0};

    // state register and burst counter

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            burst_count <= 4'd0;
        end else begin
            state <= next_state;

            if (state == REFILL && m_axi_rvalid)
                burst_count <= burst_count + 1;
            else if (state == PUSH_DATA && m_axi_wready)
                burst_count <= burst_count + 1;
            else if (state == IDLE ||
                     state == SEND_ADDR_READ ||
                     state == PUSH_ADDR ||
                     state == PUSH_LOOKUP)
                burst_count <= 4'd0;
        end
    end
    // fsm combinational logic
    // controls stall, ram writes, tag writes and axi signals

    always @(*) begin
        next_state = state;

        stall_o = 1'b0;
        ram_wr_en = 1'b0;
        tag_wr_en = 1'b0;
        dirty_bit_in = 1'b0;
        valid_bit_in = 1'b0;
        is_refilling = 1'b0;

        m_axi_arvalid = 1'b0;
        m_axi_awvalid = 1'b0;
        m_axi_wvalid = 1'b0;
        m_axi_wlast = 1'b0;

        case (state)
            // normal operation
            IDLE: begin
                // write hit: update data and set dirty
                if (doing_write_hit) begin
                    tag_wr_en = 1'b1;
                    dirty_bit_in = 1'b1;
                    valid_bit_in = 1'b1;
                end
                // flush request
                if (is_flush_stage0) begin
                    stall_o = 1'b1;
                    next_state = PUSH_LOOKUP;
                end
                // miss handling
                else if (MISS) begin
                    stall_o = 1'b1;
                    if (stored_dirty)
                        next_state = PUSH_ADDR;
                    else
                        next_state = SEND_ADDR_READ;
                end
            end
            // flush lookup stage
            PUSH_LOOKUP: begin
                stall_o = 1'b1;
               next_state = PUSH_ADDR;
            end

            // send eviction address
            PUSH_ADDR: begin
                stall_o = 1'b1;
                if (!stored_dirty) begin
                    if (flush_active)
                        next_state = WAIT;
                    else
                        next_state = SEND_ADDR_READ;
                end else begin
                    m_axi_awvalid = 1'b1;
                    if (m_axi_awready)
                        next_state = PUSH_DATA;
                end
            end

            // send 16 beats of dirty data
           PUSH_DATA: begin
                stall_o = 1'b1;
                m_axi_wvalid = 1'b1;
                if (burst_count == 4'd15) m_axi_wlast = 1'b1;
                
                if (m_axi_wready && burst_count == 4'd15) begin
                    tag_wr_en = 1'b1;
                    dirty_bit_in = 1'b0;
                    valid_bit_in = 1'b1;
                    
                    next_state = WAIT_BRESP; 
                end
            end
            // issue read burst address
            SEND_ADDR_READ: begin
                stall_o = 1'b1;
                is_refilling = 1'b1;
                m_axi_arvalid = 1'b1;
                if (m_axi_arready)
                    next_state = REFILL;
            end
            // receive 16 beats and write to data ram inside the cache
            REFILL: begin
                stall_o = 1'b1;
                is_refilling = 1'b1;
                if (m_axi_rvalid) begin
                    ram_wr_en = 1'b1;
                    if (burst_count == 4'd15) begin
                        tag_wr_en = 1'b1;
                        valid_bit_in = 1'b1;
                        dirty_bit_in = 1'b0;
                        next_state = WAIT;
                    end
                end
            end
            // one-cycle wait state
            WAIT: begin
                stall_o = 1'b1;
                // pending write after refill
                if (do_write_commit) begin
                    tag_wr_en = 1'b1;
                    dirty_bit_in = 1'b1;
                    valid_bit_in = 1'b1;
                end
                next_state = IDLE;
            end
            WAIT_BRESP: begin
                stall_o = 1'b1;
                // m_axi_bready is hardwired to 1'b1 at the top of your module, 
                // so we just need to wait for the valid signal.
                
                if (m_axi_bvalid) begin
                    if (flush_active) next_state = WAIT;
                    else              next_state = SEND_ADDR_READ;
                end
            end
        endcase
    end

endmodule
`default_nettype wire
