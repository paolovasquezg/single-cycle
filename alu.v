`timescale 1ns / 1ps
module alu(SrcA, SrcB, ALUControl, ALUResult, ALUFlags);
   
    input [31:0] SrcA;
    input [31:0] SrcB;
    input [2:0] ALUControl;
    output wire [3:0] ALUFlags;
    
    output reg [31:0] ALUResult;
    wire neg, zero, carry, overflow;
    wire [3:0] condinvb;
    wire [32:0] sum;
    wire clk_en;

    parameter ARITHMETIC = 3'b00?;
    parameter AND = 3'b010;
    parameter OR =  3'b011;
    parameter XOR =  3'b100;
    

    assign sum = SrcA + (ALUControl[1] ? (~SrcB + 1) : SrcB);
    
    always @(*) begin
        casex (ALUControl)
            ARITHMETIC:
                begin
                    ALUResult = sum;
                end
            AND:
                begin
                    ALUResult = SrcA & SrcB;    
                end
            OR:
                begin
                    ALUResult = SrcA | SrcB;
                end 
            XOR:
                begin
                    ALUResult = SrcA ^ SrcB;
                end
        endcase
    end

    assign neg = ALUResult[3];
    assign zero = (ALUResult == 0);
    assign carry = (ALUControl[2] & sum[32]);
    assign overflow = (~(ALUControl[1] ^ SrcA[31] ^ SrcB[31])) & (SrcA[31] ^ sum[31]) & (~ALUControl[2]);
    assign ALUFlags = {neg, zero, carry, overflow};
   
endmodule
