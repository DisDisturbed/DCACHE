`timescale 1ns / 1ps

module ram_model_axi #(
    parameter RAM_DEPTH = 65536, 
    parameter LATENCY   = 5
)(
    input  wire        clk,
    input  wire        rst,

    // Read Channel
    input  wire        rreq_i,
    input  wire [31:0] raddr_i,
    output reg         rgnt_o,
    output reg         rvalid_o,
    output reg  [31:0] rdata_o,

    // Write Channel
    input  wire        wreq_i,
    input  wire [31:0] waddr_i,
    output reg         wgnt_o,
    input  wire        wdata_valid_i,
    input  wire [31:0] wdata_i,
    input  wire        wlast_i,
    output reg         wdata_ready_o,
    output reg         bvalid_o
);

    reg [31:0] mem [0:RAM_DEPTH-1];
    integer i;
    initial for (i = 0; i < RAM_DEPTH; i = i + 1) mem[i] = 0;

    // Internal Signals
    reg [31:0] word_addr_read;
    reg [31:0] word_addr_write;
    
    // Delays
    integer r_delay_cnt;
    integer w_delay_cnt;
    reg     rand_ready;

    // ================= READ FSM =================
    localparam R_IDLE  = 2'd0; 
    localparam R_WAIT  = 2'd1; 
    localparam R_BURST = 2'd2;

    reg [1:0]  r_state;
    reg [3:0]  r_lat_cnt;
    reg [4:0]  r_burst_cnt;
    reg [31:0] r_captured_addr;

    always @(posedge clk) begin
        if (rst) begin
            r_state <= R_IDLE;
            rgnt_o <= 0;
            rvalid_o <= 0;
            r_delay_cnt <= 0;
        end else begin
            case (r_state) 
                R_IDLE: begin
                    rvalid_o <= 0;
                    r_burst_cnt <= 0;
                    
                    if (rreq_i) begin
                        // If delay counter is 0, we can proceed. 
                        // If not, we wait and decrement.
                        if (r_delay_cnt > 0) begin
                            r_delay_cnt <= r_delay_cnt - 1;
                            rgnt_o <= 0;
                        end else begin
                            // Grant Access
                            rgnt_o <= 1;
                            r_captured_addr <= raddr_i;
                            r_state <= R_WAIT;
                            r_lat_cnt <= 0;
                            // Set a random delay for the *latency* phase
                            r_delay_cnt <= $urandom_range(0, 4); 
                        end
                    end else begin
                        rgnt_o <= 0;
                        // Reload random delay for the *next* request grant
                        r_delay_cnt <= $urandom_range(0, 4); 
                    end
                end
                
                R_WAIT: begin
                    rgnt_o <= 0;
                    // Wait for LATENCY + Random Jitter
                    if (r_lat_cnt < (LATENCY + r_delay_cnt)) 
                        r_lat_cnt <= r_lat_cnt + 1;
                    else 
                        r_state <= R_BURST;
                end

                R_BURST: begin
                    // Randomly toggle Valid to simulate bus jitter
                    // AXI Spec: If Valid drops, we must NOT advance data index
                    if (($urandom_range(0, 10) > 2)) begin
                        rvalid_o <= 1;
                        word_addr_read = (r_captured_addr[31:2]) + r_burst_cnt;

                        if (word_addr_read < RAM_DEPTH)
                            rdata_o <= mem[word_addr_read];
                        else
                            rdata_o <= 32'd0;

                        r_burst_cnt <= r_burst_cnt + 1;
                        if (r_burst_cnt == 15) begin
                            r_state <= R_IDLE;
                            // Set random delay for next IDLE phase
                            r_delay_cnt <= $urandom_range(0, 4);
                        end
                    end else begin
                        rvalid_o <= 0; // Bubble
                    end
                end
            endcase
        end
    end

    // ================= WRITE FSM =================
    localparam W_IDLE  = 2'd0;
    localparam W_BURST = 2'd1;
    localparam W_RESP  = 2'd2;

    reg [1:0]  w_state;
    reg [31:0] w_captured_addr;
    reg [4:0]  w_burst_cnt;

    always @(posedge clk) begin
        if (rst) begin
            w_state <= W_IDLE;
            wgnt_o <= 0;
            wdata_ready_o <= 0;
            bvalid_o <= 0;
            w_delay_cnt <= 0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    bvalid_o <= 0;
                    if (wreq_i) begin
                        if (w_delay_cnt > 0) begin
                            w_delay_cnt <= w_delay_cnt - 1;
                            wgnt_o <= 0;
                        end else begin
                            wgnt_o <= 1;
                            w_captured_addr <= waddr_i;
                            w_state <= W_BURST;
                            w_burst_cnt <= 0;
                            // Initialize random delay for the Response phase
                            w_delay_cnt <= $urandom_range(2, 10);
                        end
                    end else begin
                        wgnt_o <= 0;
                        // Reload random delay for next request
                        w_delay_cnt <= $urandom_range(0, 5);
                    end
                end

                W_BURST: begin
                    wgnt_o <= 0;
                    
                    // Random backpressure
                    rand_ready = ($urandom_range(0, 10) > 2); 
                    wdata_ready_o <= rand_ready;

                    // Handshake: Master Valid AND Slave Ready (from previous cycle reg)
                    if (wdata_valid_i && wdata_ready_o) begin
                        
                        word_addr_write = (w_captured_addr[31:2]) + w_burst_cnt;
                        if (word_addr_write < RAM_DEPTH)
                            mem[word_addr_write] <= wdata_i;
                        
                        w_burst_cnt <= w_burst_cnt + 1;
                        
                        if (wlast_i || w_burst_cnt == 15) begin
                            // CRITICAL FIX: Stop accepting data immediately
                            wdata_ready_o <= 0; 
                            w_state <= W_RESP; 
                        end
                    end
                end

                W_RESP: begin
                    // Handle the Response Delay HERE, not in BURST
                    wdata_ready_o <= 0; // Ensure we don't accept garbage
                    if (w_delay_cnt > 0) begin
                        w_delay_cnt <= w_delay_cnt - 1;
                        bvalid_o <= 0;
                    end else begin
                        bvalid_o <= 1;
                        // Only go to IDLE if master accepted the response?
                        // For this model, we pulse BVALID for 1 cycle.
                        w_state <= W_IDLE;
                        w_delay_cnt <= $urandom_range(0,5); // Reset for next IDLE
                    end
                end
            endcase
        end
    end

endmodule