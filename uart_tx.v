module uart_tx(
	input wire sys_clk,
	input wire sys_rst_n,
	input wire [7:0] fifo_input_data,
	input wire fifo_input_data_en,
	input wire [19:0] bode_rate_set,
	output reg fifo_rd_req,
	output reg TX
);	
	wire [27:0] cnt_value_set;
	parameter sys_clk_frequency = 28'd50_000_000; 
	reg [19:0] period_cnt;
	reg [7:0] fifo_input_data_reg;
	reg [3:0] bit_cnt;
	reg cnt_flag;
	
	assign cnt_value_set = (sys_clk_frequency / bode_rate_set) - 1;
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			cnt_flag <= 1'b0;
		end
		else begin 
			if (fifo_input_data_en && !cnt_flag) begin cnt_flag <= 1'b1;end
			else if (cnt_flag && period_cnt == cnt_value_set && bit_cnt == 4'd10) begin 
				cnt_flag <= 1'b0;
			end
		end
	end
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			period_cnt <= 20'd0;
		end
		else begin 
			if(cnt_flag) begin period_cnt <= period_cnt == cnt_value_set ? 20'd0 : period_cnt + 20'd1;end
			else begin period_cnt <= 20'd0; end
		end
	end
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			bit_cnt <= 4'd0;
		end
		else begin 
			if(cnt_flag) begin
				bit_cnt <= period_cnt == cnt_value_set ? bit_cnt+4'd1 : bit_cnt;
			end
			else begin bit_cnt <= 4'd0; end
		end
	end
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			fifo_rd_req <= 1'b0;
		end
		else begin 
			if (cnt_flag) begin 
				if(bit_cnt == 4'd0 && period_cnt == 20'd2) begin
					fifo_rd_req <= 1'b1;
				end
				else begin fifo_rd_req <= 1'b0; end
			end
			else begin fifo_rd_req <= 1'b0; end
		end
	end
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			fifo_input_data_reg <= 8'd0;
		end
		else begin 
			if (cnt_flag) begin 
				if(bit_cnt == 4'd0 && period_cnt == 20'd1) begin
					fifo_input_data_reg <= fifo_input_data;
				end
				else begin fifo_input_data_reg <= fifo_input_data_reg; end
			end
			else begin fifo_input_data_reg <= 8'd0; end
		end
	end
	always@(*) begin
		if (!sys_rst_n) begin
			TX = 1'b1;
		end
		else begin		
			case (bit_cnt) 
			4'd0:begin TX = 1'b1;end
			4'd1:begin TX = 1'b0;end
			4'd2:begin TX = fifo_input_data_reg[0];end
			4'd3:begin TX = fifo_input_data_reg[1];end
			4'd4:begin TX = fifo_input_data_reg[2];end
			4'd5:begin TX = fifo_input_data_reg[3];end
			4'd6:begin TX = fifo_input_data_reg[4];end
			4'd7:begin TX = fifo_input_data_reg[5];end
			4'd8:begin TX = fifo_input_data_reg[6];end
			4'd9:begin TX = fifo_input_data_reg[7];end
			4'd10:begin TX = 1'b1;end
			default:begin TX = 1'b1;end
			endcase
		end
	end
endmodule