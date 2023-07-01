`timescale 1ns / 1ps
`define clock_period 20

module tb_dataMem(
    );
    
    reg MemToReg; //ѡ������������ALU�������ݴ洢��
    reg MemWrite; //д�����źţ�Ϊ1ʱд
    reg [1:0]loadStoreWidth ;
    reg w_loadSign_1;
    reg CLK;
    reg [31:0] memAddr;  //�����ĵ�ַ
    reg [31:0] writeData;
    wire [31:0] writeBackData;  //����ѡ��������������

    
    DataMEM test(
           MemToReg,MemWrite,loadStoreWidth,w_loadSign_1,CLK,memAddr,writeData,writeBackData
        );
      
      initial CLK =1'b1;
      always#(`clock_period/2) CLK=~CLK;  

initial begin
    #20   
   // writeBackData=memAddr=32'h13 �ֽ�
    MemToReg = 1'b0;
    MemWrite = 1'b1;
    memAddr = 32'h13;
    writeData = 32'ha1;     //1010 0001
    loadStoreWidth =2'b00;

    #20
//��    writeBackData=1010 0001  �ֽ�
    MemToReg = 1'b1;
    MemWrite = 1'b0;
    memAddr = 32'h13;
    loadStoreWidth =2'b00;
    w_loadSign_1 =1'b0;

    #20   
   // writeBackData=memAddr=32'h11 ���ֳ�
    MemToReg = 1'b0;
    MemWrite = 1'b1;
    memAddr = 32'h11;
    writeData = 32'ha1a1;     //1010 0001 1010 0001
    loadStoreWidth =2'b01;

    #20
//��  writeBackData=1010 0001 1010 0001 ��ǰ�油1��
    MemToReg = 1'b1;
    MemWrite = 1'b0;
    memAddr = 32'h11;
    loadStoreWidth =2'b01;
    w_loadSign_1 =1'b1;

    #20   
   // writeBackData=memAddr=32'h12
    MemToReg = 1'b0;
    MemWrite = 1'b1;
    memAddr = 32'h12;
    writeData = 32'ha1a1a1;    
    loadStoreWidth =2'b11;

    #20
//��   writeBackData=32'ha1a1a1
    MemToReg = 1'b1;
    MemWrite = 1'b0;
    memAddr = 32'h12;
    loadStoreWidth =2'b11;
    w_loadSign_1 =1'b0;
end

endmodule