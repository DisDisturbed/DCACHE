module dcache_tag_ram #(
    parameter TAG_COUNT = 64,
    parameter ADDR_WIDTH = 32
)(
    input wire clk_i,
    input wire rst_i,
    input wire [ADDR_WIDTH-1:0] addr_i,
    input wire dirty_in,
    input wire valid_in,
    input wire wr_en,
    output reg [21:0] tag_o // dirty, valid, tag
);
    // 22 bits: [21]=Dirty, [20]=Valid, [19:0]=Tag
    reg [21:0] tags [0:TAG_COUNT-1];
    
    wire [5:0] index = addr_i[11:6];
    wire [19:0] new_tag = addr_i[31:12];

    integer i;
    initial begin
        for(i=0; i<TAG_COUNT; i=i+1) tags[i] = 0;
    end

    always @(posedge clk_i) begin
        if (wr_en) begin
            tags[index] <= {dirty_in, valid_in, new_tag};
        end
        tag_o <= tags[index]; // Synchronous Read 
    end
endmodule