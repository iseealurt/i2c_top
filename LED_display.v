module LED_display(
input wire sys_clk,
input wire sys_rst,
input wire [23:0] DATA,
output reg [5:0] sel,
output reg [7:0] seg
);
parameter shift_amount = 4'd8;
parameter   SEG_0 = 8'b1100_0000, SEG_1 = 8'b1111_1001, //c0 f9
			SEG_2 = 8'b1010_0100, SEG_3 = 8'b1011_0000, //a4 b0
			SEG_4 = 8'b1001_1001, SEG_5 = 8'b1001_0010, //99 92
		    SEG_6 = 8'b1000_0010, SEG_7 = 8'b1111_1000, //82 f8
		    SEG_8 = 8'b1000_0000, SEG_9 = 8'b1001_0000, //80 90
		    SEG_A = 8'b1000_1000, SEG_B = 8'b1000_0011, //88 83
		    SEG_C = 8'b1100_0110, SEG_D = 8'b1010_0001, //c6 a1
		    SEG_E = 8'b1000_0110, SEG_F = 8'b1000_1110; //86 8e
reg [2:0] cnt_6;
reg [5:0] cnt_56; 
reg [23:0] DATA_temp;
reg stcp;
// 数据储存 当数据发生变动时读取新数据到低8位，并把旧数据左移8位
always@(posedge sys_clk or negedge sys_rst) begin
	if(!sys_rst) begin 
		DATA_temp <= 24'd0;
	end
   else begin 
		DATA_temp <= DATA;
	end
end
// 56分频计数器，用于与数码管控制模块时序同步
always@(posedge sys_clk or negedge sys_rst) begin
	if (!sys_rst) begin cnt_56 <= 6'd0;end
	else begin
        if(cnt_56 == 6'd55) begin cnt_56 <= 4'd0; end
		else begin cnt_56 <= cnt_56 + 6'd1; end
	end
end
// 传输数据时与数码管控制模块的时序同步
always@(posedge sys_clk or negedge sys_rst) begin
	if(sys_rst == 1'b0) begin
		stcp <= 1'b0;
	end
    else if(cnt_56 ==6'd55) begin
		stcp <= 1'b1;
	end
	else begin 
		stcp <= 1'b0;
	end
end
// 向数码管传输数据时选择位数
always@(posedge stcp or negedge sys_rst) begin
	if (sys_rst == 1'b0) begin 
		cnt_6 <= 3'd0;
	end
	else begin
		if (cnt_6 == 3'd5) begin cnt_6 <= 3'd0; end
		else begin cnt_6 <= cnt_6 + 3'd1; end
	end
end
// 向数码管传输数据
always@(posedge sys_clk or negedge sys_rst ) begin
	if (sys_rst == 1'b0) begin 
		sel <= 6'd0;
		seg <= 8'd0;
	end
	else begin
		case (cnt_6)
			6'd0:begin sel <= 6'b000001;end
			6'd1:begin sel <= 6'b000010;end
			6'd2:begin sel <= 6'b000100;end
			6'd3:begin sel <= 6'b001000;end
			6'd4:begin sel <= 6'b010000;end
			6'd5:begin sel <= 6'b100000;end
			default:begin sel <= 6'd0;end
		endcase
        case (DATA_temp[(cnt_6 * 4) +: 4])
			4'b0000:begin seg <= SEG_0;end
			4'b0001:begin seg <= SEG_1;end
			4'b0010:begin seg <= SEG_2;end
			4'b0011:begin seg <= SEG_3;end
			4'b0100:begin seg <= SEG_4;end
			4'b0101:begin seg <= SEG_5;end
			4'b0110:begin seg <= SEG_6;end
			4'b0111:begin seg <= SEG_7;end
			4'b1000:begin seg <= SEG_8;end
			4'b1001:begin seg <= SEG_9;end
			4'b1010:begin seg <= SEG_A;end
			4'b1011:begin seg <= SEG_B;end
			4'b1100:begin seg <= SEG_C;end
			4'b1101:begin seg <= SEG_D;end
			4'b1110:begin seg <= SEG_E;end
			4'b1111:begin seg <= SEG_F;end
			default:begin seg <= SEG_0;end
		endcase
	end
end
endmodule