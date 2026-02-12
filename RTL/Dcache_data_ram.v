`timescale 1ns / 1ps
`default_nettype none
//simplest possible ram module that support strb
module dcache_data_ram #(
    parameter line_amount = 64,
    parameter line_size   = 16,
    parameter data_width  = 32
)(
    input  wire        clk_i,
    input  wire        req_i,
    input  wire [5:0]  line_idx_i,
    input  wire [3:0]  word_idx_i,
    input  wire [31:0] wdata_i,
    input  wire        wr_en,
    input  wire [3:0]  wstrb_i,  
    output wire [31:0] rdata_o
);

    localparam TOTAL_DEPTH = line_amount * line_size;  

    reg [31:0] data_ram [0:TOTAL_DEPTH-1];
    reg [31:0] data_o_reg;
    
    wire [9:0] ram_addr;
    assign ram_addr = (line_idx_i * line_size) + word_idx_i;

    always @(posedge clk_i) begin
        if (wr_en) begin
            if (wstrb_i[0]) data_ram[ram_addr][7:0]   <= wdata_i[7:0];
            if (wstrb_i[1]) data_ram[ram_addr][15:8]  <= wdata_i[15:8];
            if (wstrb_i[2]) data_ram[ram_addr][23:16] <= wdata_i[23:16];
            if (wstrb_i[3]) data_ram[ram_addr][31:24] <= wdata_i[31:24];
        end
        if (req_i)
            data_o_reg <= data_ram[ram_addr];
    end

    assign rdata_o = data_o_reg;

endmodule

`default_nettype wire