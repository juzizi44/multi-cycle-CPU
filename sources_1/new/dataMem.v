`timescale 1ns / 1ps


module DataMEM(
        input MemWrite, //д�����źţ�Ϊ1ʱд
        input[1:0] loadStoreWidth , //��д��ȱ�־
        input loadSign , //Load��չ��ʽ 1 �з��ţ� 0 �޷���
        input clk,
        input [31:0] memAddr,  //�����ĵ�ַ 
        input [31:0] writeData,  //rs2Value
        output reg[31:0] writeBackData  //����ѡ��������������
    );
    
//    initial begin 
//        writeBackData <= 32'b0;
//    end
    
    reg [7:0] ram [0:1023];     // �洢�����������reg���ͣ�1024��8λ�洢��
    
    reg [31:0] out;
    
    always@(posedge clk) 
    begin
        $display("writeData: %d, memAddr:%d, MemWrite: %d,writeBackData:%d",writeData, memAddr,MemWrite,writeBackData);
        if(MemWrite)  //д
            begin
                case(loadStoreWidth )
                    2'b00: begin //���ֽ�
                        ram[memAddr] <= writeData[7:0];  
                        end
                    2'b01: begin //�����
                        ram[memAddr] <= writeData[7:0];  
                        ram[memAddr + 1] <= writeData[15:8]; 
                        end
                    2'b11: begin //����
                        ram[memAddr] <= writeData[7:0];  
                        ram[memAddr + 1] <= writeData[15:8];     
                        ram[memAddr + 2] <= writeData[23:16];
                        ram[memAddr + 3] <= writeData[31:24];  
                        end
                    default:;
                    endcase
             end
        else
            ram[memAddr] <= ram[memAddr];
    end
    
    always@(*)
        begin
            out = {ram[memAddr + 3],ram[memAddr + 2],ram[memAddr + 1],ram[memAddr ]};

            case(loadStoreWidth )
                2'b00: begin //ȡ�ֽ�
                    writeBackData = loadSign ? {{24{out[7]}},out[7:0]} : {{24{1'b0}},out[7:0]};
                    end
                    
                2'b01: begin //ȡ����
                    writeBackData = loadSign ? {{16{out[15]}},out[15:0]} : {{16{1'b0}},out[15:0]};
                    end
                    
                2'b11: begin //ȡ��
                    writeBackData = out;
                    end
                default:
                    writeBackData = out;
            endcase
            // writeBackData = MemToReg ? outdata : memAddr;
        end
  
    
endmodule


