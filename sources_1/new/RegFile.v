`timescale 1ns / 1ps

module RegFile (
        input clk,
        input [4:0] readReg1,  //rs�Ĵ�����ַ(��ȡ)
        input [4:0] readReg2,  //rt�Ĵ�����ַ(��ȡ)
        input [4:0] writeReg,  //д��ĵ�ַ
        input [31:0] writeData,  //д�������
        input isWreg,  //WE��дʹ���źţ�Ϊ1ʱ����ʱ�ӱ��ش���д��
        output reg[31:0] readData1,  //rs����������
        output reg[31:0] readData2  //rt����������

    );
    reg[31:0] regF[0:31];//32���Ĵ���

//    initial begin
//        readData1 <= 0;
//        readData2 <= 0;
//    end

    integer i;
    initial begin
        for (i = 0; i < 32; i = i+ 1) regF[i] <= 0;  
    end

//    always@(readReg1 or readReg2) //clk
    always@(*) //��
    begin
        readData1 <= regF[readReg1];
        readData2 <= regF[readReg2];
        //$display("regfile %d %d\n", readReg1, readReg2);
    end

    always@(posedge clk)
    begin
        regF[writeReg] = (isWreg && writeReg) ? writeData : regF[writeReg];
        //$0��Ϊ0������д��Ĵ����ĵ�ַ����Ϊ0
//        if(isWreg)
//            regF[writeReg] <= writeData;
//        else
//            regF[writeReg] <= regF[writeReg];
       // $display("reg1: %d reg2: %d",regF[1], regF[2]);
    end
    
    always@(posedge clk)
    begin
//    $display("r1val: %d, r2val: %d, r3val: %d, r4val: %d",regF[1], regF[2], regF[3],regF[4]);
    $display("ra: %d, sp: %d, gp: %d, tp: %d",regF[1], regF[2], regF[3],regF[4]);
    $display("t0: %d, t1: %d, t2: %d, s0/fp: %d",regF[5], regF[6], regF[7],regF[8]);
    $display("s1: %d, a0: %d, a1: %d, a2: %d",regF[9], regF[10], regF[11],regF[12]);
    $display("a3: %d, a4: %d, a5: %d, a6: %d",regF[13], regF[14], regF[15],regF[16]);
    $display("a7: %d, s2: %d, s3: %d, s4: %d",regF[17], regF[18], regF[19],regF[20]);
    $display("s5: %d, s6: %d, s7: %d, s8: %d",regF[21], regF[22], regF[23],regF[24]);
    $display("s9: %d, s10: %d, s11: %d, t3: %d",regF[25], regF[26], regF[27],regF[28]);
    $display("t4: %d, t5: %d, t6: %d",regF[29], regF[30], regF[31]);
    end
    
endmodule