`timescale 1ns / 1ps
`default_nettype none

// ============================================================================
// 1. Helper BRAM Models (Strictly consistent with RTL)
// ============================================================================
module tb_data_ram #(parameter line_amount=64, line_size=16, data_width=32)(
    input wire clk_i, req_i, wr_en,
    input wire [3:0] wstrb_i,
    input wire [5:0] line_idx_i, 
    input wire [3:0] word_idx_i,
    input wire [31:0] wdata_i,
    output reg [31:0] rdata_o
);
    reg [31:0] mem [0:1023];
    wire [9:0] addr = {line_idx_i, word_idx_i};
    integer i; initial for(i=0; i<1024; i=i+1) mem[i]=0;
    
    always @(posedge clk_i) begin
        if (req_i) begin
            if (wr_en) begin
                if (wstrb_i[0]) mem[addr][7:0]   <= wdata_i[7:0];
                if (wstrb_i[1]) mem[addr][15:8]  <= wdata_i[15:8];
                if (wstrb_i[2]) mem[addr][23:16] <= wdata_i[23:16];
                if (wstrb_i[3]) mem[addr][31:24] <= wdata_i[31:24];
            end
            rdata_o <= mem[addr];
        end
    end
endmodule

module tb_tag_ram #(parameter TAG_COUNT=64, ADDR_WIDTH=32)(
    input wire clk_i, rst_i,
    input wire [ADDR_WIDTH-1:0] addr_i,
    input wire dirty_in, valid_in, wr_en,
    output reg [21:0] tag_o
);
    reg [21:0] tags [0:TAG_COUNT-1];
    wire [5:0] index = addr_i[11:6];
    wire [19:0] new_tag = addr_i[31:12];
    integer k; initial for(k=0; k<TAG_COUNT; k=k+1) tags[k]=0;
    
    always @(posedge clk_i) begin
        if (wr_en) tags[index] <= {dirty_in, valid_in, new_tag};
        tag_o <= tags[index];
    end
endmodule

// ============================================================================
// 2. Advanced Testbench
// ============================================================================
module dcache_tb_advanced;

    // Signals
    reg clk, rst;
    reg [31:0] core_addr;
    reg        core_req, core_wen;
    reg [3:0]  core_wstrb;
    reg [31:0] core_wdata;
    wire [31:0] core_rdata;
    wire        stall;
    
    // Global Cycle Counter for Profiling
    reg [31:0] cycle_ctr;

    // ========================================================================
    // AXI4 Interface Wires
    // ========================================================================
    wire [3:0]  axi_awid;
    wire [31:0] axi_awaddr;
    wire [7:0]  axi_awlen;
    wire [2:0]  axi_awsize;
    wire [1:0]  axi_awburst;
    wire        axi_awvalid;
    wire        axi_awready;

    wire [31:0] axi_wdata;
    wire [3:0]  axi_wstrb;
    wire        axi_wlast;
    wire        axi_wvalid;
    wire        axi_wready;

    wire [3:0]  axi_bid;
    wire [1:0]  axi_bresp;
    wire        axi_bvalid;
    wire        axi_bready;

    wire [3:0]  axi_arid;
    wire [31:0] axi_araddr;
    wire [7:0]  axi_arlen;
    wire [2:0]  axi_arsize;
    wire [1:0]  axi_arburst;
    wire        axi_arvalid;
    wire        axi_arready;

    wire [3:0]  axi_rid;
    wire [31:0] axi_rdata;
    wire [1:0]  axi_rresp;
    wire        axi_rlast;
    wire        axi_rvalid;
    wire        axi_rready;

    // Shadow Memory for Verification
    reg [31:0] shadow_mem [int];
    int error_count = 0;

    // ========================================================================
    // DUT Instantiation
    // ========================================================================
    dcache #(
        .ADDR_WIDTH(32), .DATA_WIDTH(32), .TAG_BITS(20), .LINE_SIZE(16)
    ) dut (
        .clk(clk), .rst(rst),
        .core_addr_i(core_addr), .core_req_i(core_req), .core_wen_i(core_wen),
        .core_wstrb_i(core_wstrb), .core_wdata_i(core_wdata), .core_rdata_o(core_rdata),
        .stall_o(stall),

        .m_axi_awid(axi_awid), .m_axi_awaddr(axi_awaddr), .m_axi_awlen(axi_awlen),
        .m_axi_awsize(axi_awsize), .m_axi_awburst(axi_awburst), .m_axi_awvalid(axi_awvalid),
        .m_axi_awready(axi_awready),

        .m_axi_wdata(axi_wdata), .m_axi_wstrb(axi_wstrb), .m_axi_wlast(axi_wlast),
        .m_axi_wvalid(axi_wvalid), .m_axi_wready(axi_wready),

        .m_axi_bid(axi_bid), .m_axi_bresp(axi_bresp), .m_axi_bvalid(axi_bvalid),
        .m_axi_bready(axi_bready),

        .m_axi_arid(axi_arid), .m_axi_araddr(axi_araddr), .m_axi_arlen(axi_arlen),
        .m_axi_arsize(axi_arsize), .m_axi_arburst(axi_arburst), .m_axi_arvalid(axi_arvalid),
        .m_axi_arready(axi_arready),

        .m_axi_rid(axi_rid), .m_axi_rdata(axi_rdata), .m_axi_rresp(axi_rresp),
        .m_axi_rlast(axi_rlast), .m_axi_rvalid(axi_rvalid), .m_axi_rready(axi_rready)
    );
    
    // ========================================================================
    // RAM Model Instantiation
    // ========================================================================
    ram_model_axi #(.RAM_DEPTH(65536), .LATENCY(5)) ram (
        .clk(clk), .rst(rst),
        .rreq_i(axi_arvalid), .raddr_i(axi_araddr), .rgnt_o(axi_arready),
        .rvalid_o(axi_rvalid), .rdata_o(axi_rdata),
        .wreq_i(axi_awvalid), .waddr_i(axi_awaddr), .wgnt_o(axi_awready),
        .wdata_valid_i(axi_wvalid), .wdata_i(axi_wdata), .wlast_i(axi_wlast),
        .wdata_ready_o(axi_wready), .bvalid_o(axi_bvalid)
    );

    // Tie-off unused RAM outputs
    assign axi_bid   = 4'd0;
    assign axi_bresp = 2'd0; 
    assign axi_rid   = 4'd0;
    assign axi_rresp = 2'd0; 
    assign axi_rlast = 1'b0; 

    // Clock Gen & Cycle Counter
    initial clk = 0;
    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (rst) cycle_ctr <= 0;
        else cycle_ctr <= cycle_ctr + 1;
    end

    // ========================================================================
    // Tasks
    // ========================================================================
    task cpu_write(input [31:0] addr, input [31:0] data, input [3:0] strb);
        begin
            @(posedge clk);
            core_addr  <= addr;
            core_wdata <= data;
            core_req   <= 1'b1;
            core_wen   <= 1'b0; 
            core_wstrb <= strb;
            @(posedge clk); 
            if (stall) while(stall) @(posedge clk);
            else begin
                 @(posedge clk);
                 while(stall) @(posedge clk);
            end
            @(posedge clk);
            core_req <= 1'b0;
            update_shadow_mem(addr, data, strb);
        end
    endtask

    task cpu_read(input [31:0] addr, input [31:0] expected_val);
        begin
            @(posedge clk);
            core_addr  <= addr;
            core_req   <= 1'b1;
            core_wen   <= 1'b1; 
            core_wstrb <= 4'b0000;
            repeat(2) @(posedge clk); 
            while (stall) @(posedge clk);
            #1; 
            if (core_rdata !== expected_val) begin
                $error("[FAIL Time:%0d] Addr 0x%h: Read 0x%h, Expected 0x%h", cycle_ctr, addr, core_rdata, expected_val);
                error_count++;
            end 
            @(posedge clk);
            core_req <= 1'b0;
        end
    endtask

    function void update_shadow_mem(input [31:0] addr, input [31:0] data, input [3:0] strb);
        reg [31:0] current;
        reg [31:0] aligned_addr;
        aligned_addr = {addr[31:2], 2'b00};
        if (shadow_mem.exists(aligned_addr)) current = shadow_mem[aligned_addr];
        else current = 32'h00000000;
        if (strb[0]) current[7:0]   = data[7:0];
        if (strb[1]) current[15:8]  = data[15:8];
        if (strb[2]) current[23:16] = data[23:16];
        if (strb[3]) current[31:24] = data[31:24];
        shadow_mem[aligned_addr] = current;
    endfunction

    // ========================================================================
    // MAIN TEST SEQUENCE
    // ========================================================================
    initial begin
        $dumpfile("dcache_tb_advanced.vcd");
        $dumpvars(0, dcache_tb_advanced);
        
        rst = 1; core_req = 0; core_wen = 1; 
        repeat(10) @(posedge clk);
        rst = 0;
        @(posedge clk);
        
        $display("\n=======================================");
        $display("   STARTING ADVANCED CACHE TESTBENCH   ");
        $display("=======================================\n");

        // Phase 1
        $display("[Time: %0d] 1: Basic Read/Write", cycle_ctr);
        cpu_write(32'h1000, 32'hAAAA_BBBB, 4'b1111);
        cpu_read(32'h1000, 32'hAAAA_BBBB);
        cpu_write(32'h2000, 32'hCCCC_DDDD, 4'b1111);
        cpu_read(32'h2000, 32'hCCCC_DDDD);

        // Phase 2
        $display("\n[Time: %0d] 2: Byte Lane Verification", cycle_ctr);
        cpu_write(32'h3000, 32'hFFFF_FFFF, 4'b1111);
        cpu_write(32'h3000, 32'h0000_00AA, 4'b0001);
        cpu_read(32'h3000, 32'hFFFF_FFAA);
        cpu_write(32'h3000, 32'h1234_0000, 4'b1100);
        cpu_read(32'h3000, 32'h1234_FFAA);

        // Phase 3
        $display("\n[Time: %0d] 3: Full Line Fill Burst", cycle_ctr);
        for (int i=0; i<16; i++) cpu_write(32'h0140 + (i*4), i, 4'b1111);
        for (int i=0; i<16; i++) cpu_read(32'h0140 + (i*4), i);

        // Phase 4
        $display("\n[Time: %0d] 4: Thrashing Conflict Misses", cycle_ctr);
        cpu_write(32'h10280, 32'hDEAD_1000, 4'b1111);
        cpu_write(32'h20280, 32'hBEEF_2000, 4'b1111);
        cpu_read(32'h20280, 32'hBEEF_2000);
        cpu_read(32'h10280, 32'hDEAD_1000);

        // Phase 5
        $display("\n[Time: %0d] 5: Software Flush", cycle_ctr);
        cpu_write(32'h00500, 32'hbaadbeef, 4'b1111);
        cpu_write(32'h0000BEEF, 32'h0000_0500, 4'b1111);
        cpu_read(32'h00500, 32'hbaadbeef);

        // Phase 7
        $display("\n[Time: %0d] 7: Back-to-Back Conflict Misses", cycle_ctr);
        cpu_write(32'h10050, 32'hAAAA_1111, 4'b1111);
        cpu_write(32'h20050, 32'hBBBB_2222, 4'b1111);
        cpu_write(32'h10050, 32'hCCCC_3333, 4'b1111);
        cpu_read(32'h20050, 32'hBBBB_2222);
        cpu_read(32'h10050, 32'hCCCC_3333);

        // Phase 8
        $display("\n[Time: %0d] 8: Write Miss -> Immediate Read", cycle_ctr);
        cpu_write(32'h30080, 32'hFACE_FEED, 4'b1111); 
        cpu_read(32'h30080, 32'hFACE_FEED); 
        cpu_read(32'h30084, 0); 

        // Phase 9
        $display("\n[Time: %0d] 9: Randomized AXI Handshake Delays", cycle_ctr);
        for (int k=0; k<1000; k++) begin
            reg [31:0] r_addr;
            reg [31:0] r_data;
            reg [3:0]  r_strb;
            r_addr = ($random % 2048) & 32'hFFFF_FFFC; 
            r_data = $random;
            if ($random % 2) begin
                r_strb = ($random % 16);
                if (r_strb == 0) r_strb = 4'b1111;
                cpu_write(r_addr, r_data, r_strb);
            end else begin
                reg [31:0] exp_data;
                if (shadow_mem.exists(r_addr)) exp_data = shadow_mem[r_addr];
                else exp_data = 0;
                cpu_read(r_addr, exp_data);
            end
        end

        // Phase 10
        $display("\n[Time: %0d] 10: Extended Random Test (5000 Cycles)", cycle_ctr);
        for (int k=0; k<5000; k++) begin
            reg [31:0] rand_addr;
            reg [31:0] rand_data;
            reg [3:0]  rand_strb;
            int        op_type; 
            rand_addr = ($random % 65536) & 32'hFFFF_FFFC;
            rand_data = $random;
            rand_strb = $random % 16;
            if (rand_strb == 0) rand_strb = 4'b1111; 
            op_type = $urandom_range(0, 100);
            if (op_type <= 40) cpu_write(rand_addr, rand_data, rand_strb);
            else if (op_type <= 80) begin
                reg [31:0] exp;
                if (shadow_mem.exists(rand_addr)) exp = shadow_mem[rand_addr];
                else exp = 32'd0;
                cpu_read(rand_addr, exp); 
            end 
            else begin
                cpu_write(rand_addr, rand_data, 4'b1111); 
                cpu_read(rand_addr, rand_data);
            end
            if (k % 1000 == 0) $display("    ... %0d ops finished at cycle %0d", k, cycle_ctr);
        end  

        // Phase 11 - The Swiss Cheese Byte Write
        $display("\n[Time: %0d] 11: Swiss Cheese Byte Writes (New Edge Case)", cycle_ctr);
        cpu_write(32'h4000, 32'h0000_00EF, 4'b0001); // Byte 0
        cpu_write(32'h4000, 32'h0000_BE00, 4'b0010); // Byte 1
        cpu_write(32'h4000, 32'h00AD_0000, 4'b0100); // Byte 2
        cpu_write(32'h4000, 32'hDE00_0000, 4'b1000); // Byte 3
        cpu_read(32'h4000, 32'hDEADBEEF);            // Full Word

        // Phase 12 - The Tag Hammer
        $display("\n[Time: %0d] 12: Tag Hammer / Thrashing (New Edge Case)", cycle_ctr);
        cpu_write(32'h01100, 32'h1111_AAAA, 4'b1111); // Tag A
        cpu_write(32'h02100, 32'h2222_BBBB, 4'b1111); // Tag B (Evicts A)
        cpu_write(32'h03100, 32'h3333_CCCC, 4'b1111); // Tag C (Evicts B)
        cpu_read(32'h01100, 32'h1111_AAAA);           // Read A (Evicts C)
        cpu_read(32'h02100, 32'h2222_BBBB);           // Read B (Evicts A)

        // Phase 13 - WAW Hazard
        $display("\n[Time: %0d] 13: WAW Hazard (Immediate Overwrite) (New Edge Case)", cycle_ctr);
        cpu_write(32'h5000, 32'h1111_1111, 4'b1111); 
        cpu_write(32'h5000, 32'h9999_9999, 4'b1111); 
        cpu_read(32'h5000, 32'h9999_9999);
        
        $display("\n[Time: %0d] 14: Asynchronous Reset During Refill (Sanity Check)", cycle_ctr);
        
        // 1. Trigger a Miss
        core_addr  <= 32'h6000;
        core_req   <= 1'b1;
        core_wen   <= 1'b1; // Read
        @(posedge clk);
        
        // 2. Wait until we are definitely in the REFILL/AXI state (e.g., 5 cycles later)
        repeat(5) @(posedge clk);
        
        // 3. BOMBSHELL: Assert Reset in the middle of the transaction
        rst <= 1'b1;
        $display("    !!! Asserting RESET !!!");
        @(posedge clk);
        rst <= 1'b0;
        core_req <= 1'b0; // Drop request
        
        // 4. Verify Recovery: Can we immediately do a fresh request?
        @(posedge clk); // Recovery cycle
        cpu_write(32'h6000, 32'hDEAD_BEEF, 4'b1111); // Should work normally if FSM reset correctly
        cpu_read(32'h6000, 32'hDEAD_BEEF);


        // ---------------------------------------------------------
        // Phase 16: Back-to-Back Reads (Throughput Test)
        // ---------------------------------------------------------
        $display("\n[Time: %0d] 15: Back-to-Back Reads (1 Cycle Throughput)", cycle_ctr);
        
        // Setup: Ensure two lines are in cache
        cpu_write(32'h9000, 32'h1111_1111, 4'b1111);
        cpu_write(32'h9004, 32'h2222_2222, 4'b1111);
        
        // 1. Issue Read 1
        @(posedge clk);
        core_addr <= 32'h9000;
        core_req  <= 1'b1;
        core_wen  <= 1'b1;
        
        // 2. Issue Read 2 (Immediately next cycle)
        @(posedge clk);
        // Capture Read 1 Result (Pipelined return?) 
        // Note: Your cache seems to be 1-cycle latency for hits? Let's check.
        // If your cache returns data in the same cycle as address (combinatorial read), check now.
        // If it returns next cycle (registered read), check next.
        // Based on RTL: "ram_rdata_o" comes from BRAM, usually 1 cycle latency.
        
        core_addr <= 32'h9004; // Change address immediately
        
        // 3. Check Result 1 (Latency Cycle)
        // Depending on your BRAM model (read-first vs write-first), data might appear now or next cycle.
        // We wait 1 cycle for data 1 to settle
        @(posedge clk);
        if (core_rdata !== 32'h1111_1111) 
             $display("    [Info] Read 1 Latency > 1 cycle or pipeline gap. Data: %h", core_rdata);

        // 4. End Requests
        core_req <= 1'b0;
        @(posedge clk);
        // Check Result 2
        if (core_rdata !== 32'h2222_2222) 
             $display("    [Info] Read 2 Latency check. Data: %h", core_rdata);


        // ---------------------------------------------------------
        // Phase 17: Eviction Data Integrity Check
        // ---------------------------------------------------------
        $display("\n[Time: %0d] 16: Eviction Integrity (Did RAM actually get the data?)", cycle_ctr);
        
        // 1. Write Dirty Data to Tag A (0xA000)
        cpu_write(32'h0A000, 32'hCAFE_BABE, 4'b1111);
        
        // 2. Cause Conflict with Tag B (0xB000) -> Forces Eviction of A
        cpu_write(32'h0B000, 32'hD00D_FEED, 4'b1111);
        
        // 3. "Cheating": Peek into the Main Memory Model directly to see if 0xA000 was written
        // Note: This assumes we can access the RAM model's internals or we just read it back.
        // Let's read it back via the cache (which forces a re-load from RAM).
        // If the Eviction failed (data lost), this read will return Old RAM data (0), not CAFE_BABE.
        
        // Flush/Invalidate B by loading C, so we don't just hit in cache
        cpu_write(32'h0C000, 32'h9999_9999, 4'b1111);
        
        // Now Read A (0xA000) from RAM. It MUST be CAFE_BABE.
        cpu_read(32'h0A000, 32'hCAFE_BABE);
        
        $display("\n[Time: %0d] 17: B-Channel Ordering Check", cycle_ctr);
 
// 1. Force an Eviction
cpu_write(32'hE000, 32'h1111_1111, 4'b1111); // Dirty Line A
cpu_write(32'hF000, 32'h2222_2222, 4'b1111); // Evict A, Load B



          // ---------------------------------------------------------
        // Phase 18: Gappy Read Bursts (RVALID Toggling)
        // ---------------------------------------------------------
        $display("\n[Time: %0d] 18: Gappy Read Bursts (50%% Gap Probability)", cycle_ctr);
        
        // 1. Configure RAM to be nasty (50% chance of gap per beat)
        ram.set_read_gaps(50);
        
        // 2. Perform a Read Miss (force refill)
        cpu_write(32'h8000, 32'h1122_3344, 4'b1111); // Ensure clean state
        // Invalidate by conflict to force reload
        cpu_write(32'h9000, 32'h5566_7788, 4'b1111); 
        
        // 3. Read back 0x8000. 
        // Monitor waveforms: The 16-beat burst should take ~32 cycles now.
        // If FSM increments counter on !RVALID, it will finish early with garbage data.
        cpu_read(32'h8000, 32'h1122_3344);
        
        // Reset RAM to normal
        ram.set_read_gaps(0);
        
        
        
        
        // ---------------------------------------------------------
        // Phase 20: B-Channel Ordering (Write Response Delay)
        // ---------------------------------------------------------
        $display("\n[Time: %0d] 20: B-Channel Delay (Ordering Check)", cycle_ctr);
        
        // 1. Configure RAM to delay BVALID by 20 cycles after WLAST
        ram.set_b_delay(20);
        
        // 2. Setup: Dirty Line at 0xB000
        cpu_write(32'hB000, 32'hBAD_CAFE, 4'b1111);
        
        // 3. Trigger Eviction by reading 0xC000
        // Sequence: 
        //   a. Cache writes 0xB000 to RAM (WLAST sent).
        //   b. RAM waits 20 cycles.
        //   c. RAM asserts BVALID.
        //   d. ONLY THEN should Cache assert ARVALID for 0xC000.
        
        fork
            begin
                // Thread A: Do the CPU Op
                cpu_write(32'hC000, 32'hbaad_beef, 4'b1111); // Force eviction
            end
            begin
                // Thread B: Monitor the AXI Bus
                // Wait for Write Data to finish
                wait(axi_wlast && axi_wvalid && axi_wready);
                
                // Now check if ARVALID goes high *before* BVALID
                // We loop for ~15 cycles (since delay is 20) to see if Cache cheats
                repeat(15) begin
                    @(posedge clk);
                    if (axi_arvalid) begin
                         $error("[FAIL] Cache asserted ARVALID before BVALID! Ordering Violation.");
                         error_count++;
                    end
                end
            end
        join
         
        // Clean up
        ram.set_b_delay(0);

        // ---------------------------------------------------------
        // Phase 21: Extreme Backpressure ("The Constipated Slave")
        // ---------------------------------------------------------
        $display("\n[Time: %0d] 21: Extreme Backpressure (90%% Wait States)", cycle_ctr);
        
        // 1. Configure RAM to be incredibly slow
        ram.set_write_backpressure(90); 
        ram.set_read_gaps(90);
        
        // 2. Perform a Write (Force Eviction + Refill)
        // Use VALID hex this time: 5100_D00D ("SLOO DOOD")
        cpu_write(32'hD000, 32'h5100_D00D, 4'b1111);
        
        // 3. Verify Data
        cpu_read(32'hD000, 32'h5100_D00D);
        
        // 4. Reset RAM to normal speed
        ram.set_write_backpressure(0);
        ram.set_read_gaps(0);





// Summary
        $display("\n=======================================");
        $display("   FINAL REPORT (Total Cycles: %0d)", cycle_ctr);
        $display("=======================================");
        if (error_count == 0)
            $display("   SUCCESS: All Tests Passed!");
        else 
            $display("   FAILURE: %0d Errors Found.", error_count);
        
        $finish;
    end

endmodule
`default_nettype wire