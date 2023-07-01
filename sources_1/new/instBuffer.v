`timescale 1ns / 1ps

module instBuffer(
    input  wire [31:0] i_data,
    input  wire        clk,
    input  wire        IRWrite,
    output reg  [31:0] o_data);

 
    always @(posedge clk) begin
    if (IRWrite)
        o_data <= i_data;
    else
        o_data <= o_data;
    end

endmodule
