`timescale 1ns / 1ps
`default_nettype none

// ============================================================================
// 1. Helper BRAM Models (Keep these strictly consistent with RTL)
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
    
    // AXI Interface
    wire        axi_rreq, axi_rgnt, axi_rvalid;
    wire [31:0] axi_raddr, axi_rdata;
    wire        axi_wreq, axi_wdata_valid, axi_wlast, axi_wgnt, axi_wdata_ready, axi_bvalid;
    wire [31:0] axi_waddr, axi_wdata;

    // Shadow Memory for Verification (Associative Array)
    // Stores [Address] -> [Expected Data]
    reg [31:0] shadow_mem [int];
    int error_count = 0;

    // Instantiate DUT
    dcache #(
        .ADDR_WIDTH(32), .DATA_WIDTH(32), .TAG_BITS(20), .LINE_SIZE(16)
    ) dut (
        .clk(clk), .rst(rst),
        .core_addr_i(core_addr), .core_req_i(core_req), .core_wen_i(core_wen),
        .core_wstrb_i(core_wstrb), .core_wdata_i(core_wdata), .core_rdata_o(core_rdata),
        .stall_o(stall),
        .axi_rreq_o(axi_rreq), .axi_raddr_o(axi_raddr), .axi_rgnt_i(axi_rgnt),
        .axi_rvalid_i(axi_rvalid), .axi_rdata_i(axi_rdata),
        .axi_wreq_o(axi_wreq), .axi_wdata_valid_o(axi_wdata_valid), .axi_waddr_o(axi_waddr),
        .axi_wdata_o(axi_wdata), .axi_wlast_o(axi_wlast), .axi_wgnt_i(axi_wgnt),
        .axi_wdata_ready_i(axi_wdata_ready), .axi_bvalid_i(axi_bvalid)
    );
    
    // Instantiate Main Memory Model
    ram_model_axi #(.RAM_DEPTH(65536), .LATENCY(5)) ram (
        .clk(clk), .rst(rst),
        .rreq_i(axi_rreq), .raddr_i(axi_raddr), .rgnt_o(axi_rgnt),
        .rvalid_o(axi_rvalid), .rdata_o(axi_rdata),
        .wreq_i(axi_wreq), .waddr_i(axi_waddr), .wgnt_o(axi_wgnt),
        .wdata_valid_i(axi_wdata_valid), .wdata_i(axi_wdata), .wlast_i(axi_wlast),
        .wdata_ready_o(axi_wdata_ready), .bvalid_o(axi_bvalid)
    );

    // Clock Gen
    initial clk = 0;
    always #5 clk = ~clk;

    // ========================================================================
    // Tasks
    // ========================================================================

    // Task: CPU Write with Strobe Support
    task cpu_write(input [31:0] addr, input [31:0] data, input [3:0] strb);
        begin
            @(posedge clk);
            core_addr  <= addr;
            core_wdata <= data;
            core_req   <= 1'b1;
            core_wen   <= 1'b0; 
            core_wstrb <= strb;

            // Wait 2 cycles for Pipeline
            @(posedge clk); 
            
            // Handle Stalls (Miss or Refill)
            if (stall) while(stall) @(posedge clk);
            else begin
                 // Check if stall happens late (Pipeline Fill)
                 @(posedge clk);
                 while(stall) @(posedge clk);
            end

            // Extra cycle for RAM Write Commit
            @(posedge clk);
            core_req <= 1'b0;

            // Update Shadow Memory (Golden Model)
            update_shadow_mem(addr, data, strb);
        end
    endtask

    // Task: CPU Read and Compare against Expected
    task cpu_read(input [31:0] addr, input [31:0] expected_val);
        begin
            @(posedge clk);
            core_addr  <= addr;
            core_req   <= 1'b1;
            core_wen   <= 1'b1; // Read
            core_wstrb <= 4'b0000;
            
            // 2-Cycle Latency Wait
            repeat(2) @(posedge clk); 
            
            while (stall) @(posedge clk);
            
            #1; // Sampling Edge
            if (core_rdata !== expected_val) begin
                $error("[FAIL] Addr 0x%h: Read 0x%h, Expected 0x%h", addr, core_rdata, expected_val);
                error_count++;
            end 
            
            @(posedge clk);
            core_req <= 1'b0;
        end
    endtask

    // Helper: Update Shadow Memory based on Strobe
    function void update_shadow_mem(input [31:0] addr, input [31:0] data, input [3:0] strb);
        reg [31:0] current;
        // Align address to word boundary (mask lower 2 bits)
        reg [31:0] aligned_addr;
        aligned_addr = {addr[31:2], 2'b00};

        if (shadow_mem.exists(aligned_addr)) 
            current = shadow_mem[aligned_addr];
        else 
            current = 32'h00000000;

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
        
        // Initialize
        rst = 1; core_req = 0; core_wen = 1; 
        repeat(10) @(posedge clk);
        rst = 0;
        @(posedge clk);
        
        $display("\n=======================================");
        $display("   STARTING ADVANCED CACHE TESTBENCH   ");
        $display("=======================================\n");

        // ---------------------------------------------------------
        // Phase 1: Basic Word Access (Sanity Check)
        // ---------------------------------------------------------
        $display("1: basic read/write ---");
        cpu_write(32'h1000, 32'hAAAA_BBBB, 4'b1111);
        cpu_read(32'h1000, 32'hAAAA_BBBB);
        cpu_write(32'h2000, 32'hCCCC_DDDD, 4'b1111); // Different Line
        cpu_read(32'h2000, 32'hCCCC_DDDD);

        // ---------------------------------------------------------
        // Phase 2: Byte Lane (Sub-word) Writes
        // ---------------------------------------------------------
        $display("\n 2: byte lane verification ---");
        cpu_write(32'h3000, 32'hFFFF_FFFF, 4'b1111);
        cpu_write(32'h3000, 32'h0000_00AA, 4'b0001);
        cpu_read(32'h3000, 32'hFFFF_FFAA);
        cpu_write(32'h3000, 32'h1234_0000, 4'b1100);
        cpu_read(32'h3000, 32'h1234_FFAA);

        // ---------------------------------------------------------
        // Phase 3: Cache Line Boundary Fill
        // ---------------------------------------------------------
        $display("\n3: full line fill burst check ---");
        for (int i=0; i<16; i++) begin
            cpu_write(32'h0140 + (i*4), i, 4'b1111);
        end
        for (int i=0; i<16; i++) begin
            cpu_read(32'h0140 + (i*4), i);
        end

        // ---------------------------------------------------------
        // Phase 4: Thrashing & Eviction (Conflict Misses)
        // ---------------------------------------------------------
        $display("\n--- 4: thrashing conflict misses");
        cpu_write(32'h10280, 32'hDEAD_1000, 4'b1111); // Load A (Dirty)
        cpu_write(32'h20280, 32'hBEEF_2000, 4'b1111); // Load B (Evicts A)
        cpu_read(32'h20280, 32'hBEEF_2000);           // Read B (Hit)
        cpu_read(32'h10280, 32'hDEAD_1000);           // Read A (Evicts B)

        // ---------------------------------------------------------
        // Phase 5: Flush Operation
        // ---------------------------------------------------------
        $display("\n 5: Software Flush ");
        cpu_write(32'h00500, 32'hbaadbeef, 4'b1111);
        $display(" FLUSH command...");
        cpu_write(32'h0000BEEF, 32'h0000_0500, 4'b1111);
        cpu_read(32'h00500, 32'hbaadbeef);
        $display("\n 7: Back-to-Back Conflict Misses ---");
        // We use Index 5 (Addr 0x50).
        // Tag A (0x10) -> Addr 0x10050
        // Tag B (0x20) -> Addr 0x20050
        
        cpu_write(32'h10050, 32'hAAAA_1111, 4'b1111); // Miss (Alloc A)
        cpu_write(32'h20050, 32'hBBBB_2222, 4'b1111); // Miss (Evict A, Alloc B)
        cpu_write(32'h10050, 32'hCCCC_3333, 4'b1111); // Miss (Evict B, Alloc A)
        
        // Verify data persisted through the evictions
        cpu_read(32'h20050, 32'hBBBB_2222);           // Miss (Evict A, Reload B)
        cpu_read(32'h10050, 32'hCCCC_3333);           // Miss (Evict B, Reload A)


        // ---------------------------------------------------------
        // Phase 8: Write Miss Followed by Read to Same Line
        // ---------------------------------------------------------
        $display("\n 8: Write Miss -> Immediate Read (Forwarding Check) ---");
        // This ensures the cache doesn't stall indefinitely or return old data
        // when a read hits the line currently being refilled/written.
        
        // 1. Trigger Write Miss
        cpu_write(32'h30080, 32'hFACE_FEED, 4'b1111); 
        
        // 2. Immediately Read same address (Should Hit or Forward)
        cpu_read(32'h30080, 32'hFACE_FEED); 

        // 3. Read neighbor word in same line (Should Hit)
        cpu_read(32'h30084, 0); // Expect 0 (uninitialized neighbor)


        // ---------------------------------------------------------
        // Phase 9: Randomized AXI Handshake Delays
        // ---------------------------------------------------------
        // Note: This requires the updated ram_model_axi below to work effectively.
        $display("\n 9: Randomized AXI Handshake Delays (Stress Test) ---");
        
        // We perform random ops while the RAM model (updated below) 
        // injects random waits on Ready/Valid signals.
        for (int k=0; k<1000; k++) begin
            reg [31:0] r_addr;
            reg [31:0] r_data;
            reg [3:0]  r_strb;
            
            // Focus on a small memory range to force collisions/evictions
            r_addr = ($random % 2048) & 32'hFFFF_FFFC; 
            r_data = $random;
            
            if ($random % 2) begin
                // Write
                r_strb = ($random % 16);
                if (r_strb == 0) r_strb = 4'b1111;
                cpu_write(r_addr, r_data, r_strb);
            end else begin
                // Read
                reg [31:0] exp_data;
                if (shadow_mem.exists(r_addr)) exp_data = shadow_mem[r_addr];
                else exp_data = 0;
                cpu_read(r_addr, exp_data);
            end
            
            if (k % 200 == 0) $display("    ... completed %0d randomized delay cycles", k);
        end
        

        // ---------------------------------------------------------
        // Phase 6: Randomized Stress Test (EXTENDED)
        // ---------------------------------------------------------
        $display("\n 10: Extended Random Test (5000 Cycles) with RAW Hazards ");
        
        for (int k=0; k<5000; k++) begin
            reg [31:0] rand_addr;
            reg [31:0] rand_data;
            reg [3:0]  rand_strb;
            int        op_type; 
            
            // Constrain address to mapped RAM range (0 to 65536) to prevent bounds error
            rand_addr = ($random % 65536) & 32'hFFFF_FFFC;
            rand_data = $random;
            
            // Random Byte Enables
            rand_strb = $random % 16;
            if (rand_strb == 0) rand_strb = 4'b1111; 

            // Operation Type: 0-40=Write, 41-80=Read, 81-100=RAW Hazard
            op_type = $urandom_range(0, 100);

            if (op_type <= 40) begin
                // --- WRITE ---
                cpu_write(rand_addr, rand_data, rand_strb);
            end 
            else if (op_type <= 80) begin
                // --- READ ---
                reg [31:0] exp;
                if (shadow_mem.exists(rand_addr)) 
                    exp = shadow_mem[rand_addr];
                else 
                    exp = 32'd0; // Uninitialized memory is 0
                
                cpu_read(rand_addr, exp); 
            end 
            else begin
                // --- RAW HAZARD (Write -> Immediate Read) ---
                // This forces the cache to use forwarding or handle stall logic cleanly
                cpu_write(rand_addr, rand_data, 4'b1111); 
                cpu_read(rand_addr, rand_data);
            end
            
            if (k % 500 == 0) $display("%0d random ops finished...", k);
        end  

        // ---------------------------------------------------------
        // Summary
        // --------------------------------------------------------- 
        if (error_count == 0)
            $display(" \n  SUCCESS: All Tests Passed!           ");
        else 
            $display("   FAILURE: %0d Errors Found.           ", error_count);
        
        $finish;
    end

endmodule
`default_nettype wire