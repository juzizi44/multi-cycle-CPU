`timescale 1ns / 1ps


module DataMEM(
        input MemWrite, //写控制信号，为1时写
        input[1:0] loadStoreWidth , //读写宽度标志
        input loadSign , //Load拓展方式 1 有符号； 0 无符号
        input clk,
        input [31:0] memAddr,  //操作的地址 
        input [31:0] writeData,  //rs2Value
        output reg[31:0] writeBackData  //数据选择器最终输出结果
    );
    
//    initial begin 
//        writeBackData <= 32'b0;
//    end
    
    reg [7:0] ram [0:1023];     // 存储器定义必须用reg类型，1024个8位存储器
    
    reg [31:0] out;
    
    always@(posedge clk) 
    begin
        $display("writeData: %d, memAddr:%d, MemWrite: %d,writeBackData:%d",writeData, memAddr,MemWrite,writeBackData);
        if(MemWrite)  //写
            begin
                case(loadStoreWidth )
                    2'b00: begin //存字节
                        ram[memAddr] <= writeData[7:0];  
                        end
                    2'b01: begin //存半字
                        ram[memAddr] <= writeData[7:0];  
                        ram[memAddr + 1] <= writeData[15:8]; 
                        end
                    2'b11: begin //存字
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
                2'b00: begin //取字节
                    writeBackData = loadSign ? {{24{out[7]}},out[7:0]} : {{24{1'b0}},out[7:0]};
                    end
                    
                2'b01: begin //取半字
                    writeBackData = loadSign ? {{16{out[15]}},out[15:0]} : {{16{1'b0}},out[15:0]};
                    end
                    
                2'b11: begin //取字
                    writeBackData = out;
                    end
                default:
                    writeBackData = out;
            endcase
            // writeBackData = MemToReg ? outdata : memAddr;
        end
  
    
endmodule


