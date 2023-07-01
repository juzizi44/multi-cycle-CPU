module ALU (
    input wire [16:0] aluControl,
    input wire [31:0] aluData1,     //��һ��������
    input wire [31:0] aluData2,     //�ڶ���������
    output reg [31:0] aluResult ,  //���صĽ��
    output reg zero //������ת��־
);
    
    wire [31:0] addSubResult;  //�Ӽ�
    reg [31:0] sltResult;  //С����λ����
    wire [31:0] sltuResult;  //�޷���С����λ����
    wire [31:0] andResult;  //��
    wire [31:0] orResult;  //��
    wire [31:0] xorResult;  //���
    wire [31:0] sllResult;  //�߼�����
    wire [31:0] srlResult;  // �߼����Ʋ���
    wire [31:0] sraResult;  // �������Ʋ���
    wire [31:0] beqResult;  //���ʱ��֧
    wire [31:0] bneResult;  //�����ʱ��֧
    wire [31:0] bltResult;  //С��ʱ��֧
    wire [31:0] bgeResult;  //���ڵ���ʱ��֧
    wire [31:0] bltuResult;  //�޷���С��ʱ��֧
    wire [31:0] bgeuResult;  //�޷��Ŵ��ڵ���ʱ��֧
    wire [31:0] luiResult;  // ��λ����������

    wire c_add;
    assign c_add = aluControl[0];


    wire [31:0] adderData1;  // �ӷ�������1
    wire [31:0] adderData2;  // �ӷ�������2
    wire        adderCin;     // �ӷ���λ����
    wire[32:0]  adderResult;   // �ӷ����  ע����33λ
    wire        adderCout;      // �ӷ���λ���
    

    assign adderData1 = aluData1;
    assign adderData2 = c_add ? aluData2 : ~aluData2;  //���Ǽӷ� Ĭ������



    assign adderCin = ~c_add;  // �ӷ���λ����
    assign adderResult = adderData1 + adderData2 + adderCin;  
    assign adderCout = adderResult[32];  //�ӷ���λ���������ĵ�33λ��
    assign addSubResult = adderResult[31:0];  //���ս���ǵ�32λ


//    assign sltResult[31:1] = 31'd0;
//    assign sltResult[0] = ((aluData1[31]==0 && aluData2[31]==1) || adderResult[31]==0) ? 0 : 1;

    always @(*) begin
        sltResult[31:1] <= 31'd0;
        if (aluData1[31]==0 && aluData2[31]==1) 
            sltResult[0] <= 0;
        else if (aluData1[31]==1 && aluData2[31]==0)
            sltResult[0] <= 1;
        else if (adderResult[31]==1)
            sltResult[0] <= 1;
        else if (adderResult[31]==0)
            sltResult[0] <= 0;
        else
            sltResult[0] <= 0;
    end

    assign sltuResult  = {31'd0,~adderCout};  //�޷�������С����λ




    assign sllResult = ($unsigned(aluData1)) << aluData2[4:0]; //x[rs2]�ĵ� 5 λ�����ƶ�λ�������λ�򱻺���
    assign srlResult = ($unsigned(aluData1)) >> aluData2[4:0];
    assign sraResult = (($signed(aluData1)) >>> aluData2[4:0]);

   

    assign andResult = aluData1 & aluData2;
    assign orResult = aluData1 | aluData2;
    assign xorResult = aluData1 ^ aluData2;
    assign orResult = aluData1 | aluData2;
    assign luiResult  =  aluData2;

    

    assign beqResult=(($signed(aluData1))== ($signed(aluData2))? {31{1'b1}}: {32{1'b0}}); 
    assign bneResult=(($signed(aluData1))!= ($signed(aluData2))? {31{1'b1}}: {32{1'b0}});   
    assign bltResult=(($signed(aluData1)) < ($signed(aluData2))? {31{1'b1}}: {32{1'b0}});
    assign bgeResult=(($signed(aluData1))>= ($signed(aluData2))? {31{1'b1}}: {32{1'b0}});   
    assign bltuResult=(($unsigned(aluData1)) < ($unsigned(aluData2))?  {32{1'b1}}: {32{1'b0}});
    assign bgeuResult=(($unsigned(aluData1)) >= ($unsigned(aluData2))?  {32{1'b1}}: {32{1'b0}});



    always @(*) begin
        aluResult = 0; zero = 0;
        case(aluControl)
            17'b00000000000000001 : aluResult = addSubResult;
            17'b00000000000000010 : aluResult = addSubResult;
            17'b00000000000000100 : aluResult = sltResult;
            17'b00000000000001000 : aluResult = sltuResult;
            17'b00000000000010000 : aluResult = andResult;
            17'b00000000000100000 : aluResult = orResult;
            17'b00000000001000000 : aluResult = xorResult;
            17'b00000000010000000 : aluResult = sllResult;
            17'b00000000100000000 : aluResult = srlResult;
            17'b00000001000000000 : aluResult = sraResult;
            17'b00000010000000000 : zero = beqResult[0];
            17'b00000100000000000 : zero = bneResult[0]; 
            17'b00001000000000000 : zero = bltResult[0]; 
            17'b00010000000000000 : zero = bgeResult[0]; 
            17'b00100000000000000 : zero = bltuResult[0]; 
            17'b01000000000000000 : zero = bgeuResult[0]; 
            17'b10000000000000000 : aluResult = luiResult; 
        endcase
    end
    
endmodule