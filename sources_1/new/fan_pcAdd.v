`timescale 1ns / 1ps
module pcAdd(
        input clk,               //ʱ��
        input PCWrite,
        input zero,   //��alu�����ı�־
        input Branch,   //��֧�ź�
        input PCsrc,  //jalֱ����ת�ź�
        input PCsrc2 , //jalrֱ����ת�ź�
        input [31:0] extendOffset,  //��չ�õ�offset
//        input reg[31:0] curPC,  //��ǰָ��ĵ�ַ
        output [31:0] PCout  //��ָ���ַ
    );
    
    reg[31:0] nextPC;
    
    initial begin
        nextPC <= 32'h80000000; // riscv-tests ָ���ַ��h80000000��ʼ
    end
    
    
    
    always@(posedge clk)
    begin
        $display("PCsrc: %d��PCsrc2��%d, zero && Branch:%d",PCsrc,PCsrc2,zero && Branch);
        $display("nextPC: %d, extendOffset:%d,PCsrc:%d",nextPC,extendOffset,PCsrc);
        if(PCsrc && PCWrite) // jal
            begin 
                nextPC = nextPC + extendOffset;
            end
        else if(PCsrc2 && PCWrite) // jalr
            begin 
                nextPC = extendOffset;
            end
        else if(zero && Branch && PCWrite) // B��
            begin 
                nextPC = nextPC + extendOffset;
            end
        else if(PCWrite)
            begin
                nextPC = nextPC + 4;
            end
    end
    
    assign PCout = nextPC;
endmodule
