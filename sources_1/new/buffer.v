`timescale 1ns / 1ps

module buffer(
    input  wire [31:0] i_data,
    input  wire        clk,
    output reg  [31:0] o_data);

 
    always @(posedge clk) begin
    o_data <= i_data;
    end

endmodule
