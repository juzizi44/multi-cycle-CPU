`timescale 1ns / 1ps
`define clock_period 20

module tb_decoder(
    );
    reg  w_PC; //pc��ֵ
    reg [31:0] inst; //ָ���ֵ
    reg [31:0] rs1Value; //��ַrs1������
    reg [31:0] rs2Value; //��ַrs2������

    wire [ 4:0]  rs1Addr;  //��ַrs1
    wire [ 4:0]  rs2Addr;  //��ַrs2
    wire [ 4:0]  rdAddr;  //��ַrd

    wire [31:0] aluOperand1;       // alu������1
    wire [31:0] aluOperand2;       // alu������2
    wire [31:0]  extendOffset; //��תָ�����չƫ��

    wire [16:0]  aluControl;  //alu�����ź�
    wire         MemtoReg ; //��������ݴ洢���ж��Ķ��źţ���Ϊ0�൱�ڶ�·ѡ����������alu�Ľ����
    wire         MemWrite ; //����洢��д�ź�  //��һ����Ҫ����������
    wire         RegWrite ;  //����Ĵ���д�ź�
    wire         Branch ;  //������ת��֧�ź�
    wire         w_loadStoreWidth_2  ; //��д��ȱ�־
    wire         w_loadSign_1 ; //Load��չ��ʽ
    wire         w_PCsrc_1;  //ֱ����ת�ź�
    wire         w_PCsrc2_1;  //ֱ����ת�ź�
    
    m_decode test(
        w_PC, //pc��ֵ
        inst,
        rs1Value, //��ַrs1������
        rs2Value, //��ַrs2������
        rs1Addr,
        rs2Addr,
        rdAddr,  //��ַrd
        
        aluOperand1,       // alu������1
        aluOperand2,       // alu������2
        extendOffset, //��תָ�����չƫ��
        
        aluControl,  //alu�����ź�
        MemtoReg , //��������ݴ洢���ж��Ķ��źţ���Ϊ0�൱�ڶ�·ѡ����������alu�Ľ����
        MemWrite , //����洢��д�ź�  //��һ����Ҫ����������
        RegWrite ,  //����Ĵ���д�ź�
        Branch ,  //������ת��֧�ź�
        w_loadStoreWidth_2  , //��д��ȱ�־
        w_loadSign_1 , //Load��չ��ʽ
        w_PCsrc_1,  //ֱ����ת�ź�
        w_PCsrc2_1  //delaySymbol 
        );

    initial begin
        #20
        inst = 32'b00000000010100000000000010010011;
    end

endmodule