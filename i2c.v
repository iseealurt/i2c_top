module i2c(
	input wire sys_clk,
	input wire sys_rst_n,
	input wire output_test_key,
	input wire input_test_key,
	input wire com_rate_set_key,
	output wire i2c_scl,
	output wire scl_osc,
	inout wire sda_osc,
	output wire stcp, //数据存储器时钟
	output wire shcp, //移位寄存器时钟
	output wire ds, //串行数据输入
	output wire oe, //使能信号，低有效
	inout wire i2c_sda,        
	output wire uart_tx,
	output wire [3:0] led
);
	parameter idle = 2'd0;
	parameter read = 2'd1;
	parameter write = 2'd2;
	parameter done = 2'd3;
	                  
	parameter eprom_address = 7'b1010011;
	parameter ad_da_chip_address = 7'b1001000;
	parameter eprom_byte_address= 16'h00_5A;
	parameter ad_da_byte_address= 16'h41_00;
	wire fifo_not_empty_flag;
	wire input_test_key_posedge;
	wire input_test_key_reg;
	wire output_test_key_posedge;
	wire output_test_key_reg;
	wire com_rate_set_key_posedge;
	wire i2c_data_output_en;
	wire [5:0] sel;
	wire [7:0] seg;
	wire [7:0] fifo_q;
	wire [23:0] i2c_output_data;
	wire [7:0] fifo_use_dw;
	wire i2c_fifo_rd_req;
	wire [3:0] status_machine_led;
	reg i2c_start_en;
	reg fifo_wr_req;
	reg [3:0] word_cnt;
	reg [1:0] top_stauts_machine;
	reg [7:0] fifo_input_data;
	reg [23:0] i2c_output_data_reg;
	reg [7:0] i2c_device_address_ctrl_word;
	reg [15:0] i2c_byte_address;
	reg [7:0] read_byte_num_reg;
	reg [15:0] time_cnt_1ms;
	reg i2c_output_ack;
	reg com_rate_set;
	
	assign sda_osc = i2c_sda;
	assign scl_osc = i2c_scl;
	assign led  = top_stauts_machine; 
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if(!sys_rst_n) begin
			time_cnt_1ms <= 16'd0;
		end
		else begin 
			time_cnt_1ms <= time_cnt_1ms == 16'd9999 ? 16'd0 : time_cnt_1ms + 16'd1;
		end
	end
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			top_stauts_machine <= idle;
		end
		else begin
			if (top_stauts_machine == idle) begin 
				if (input_test_key_posedge) top_stauts_machine <= write;
				else if (!output_test_key_reg &&  time_cnt_1ms == 16'd2499) top_stauts_machine <= read;
			end
			else if (top_stauts_machine == read) begin 
				//1 i2c_device_address_ctrl_word;i2c_byte_address;
				//2 fifo_input_data
				//3 fifo_wr_req->1 
				//4 fifo_wr_req->0
				//5 i2c_start_en->1
				//6 i2c_start_en->0
				//7 top_stauts_machine->done
				if (word_cnt == 4'd7) begin top_stauts_machine <= done;end
				else begin top_stauts_machine <= top_stauts_machine; end
			end
			else if (top_stauts_machine == write) begin
				//1 i2c_device_address_ctrl_word;i2c_byte_address;
				//2 fifo_input_data
				//3 fifo_wr_req->1 
				//4 fifo_wr_req->0
				//5 fifo_input_data
				//6 fifo_wr_req->1 
				//7 fifo_wr_req->0
				//8 fifo_input_data
				//9 fifo_wr_req->1 
				//19 fifo_wr_req->0
				//11 i2c_start_en->1
				//12 i2c_start_en->0
				//13 top_stauts_machine->done
				if (word_cnt == 4'd13) begin top_stauts_machine <= done;end
				else begin top_stauts_machine <= top_stauts_machine;end
			end
			else if (top_stauts_machine == done ) begin 
				if (word_cnt == 4'd4 && read_byte_num_reg == 8'd0) top_stauts_machine <= idle;
			end
		end
	end
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			read_byte_num_reg  <= 8'd0;
		end
		else begin
			if (top_stauts_machine == read && word_cnt == 4'd3) begin read_byte_num_reg <= fifo_input_data + 8'd1;end
			else if (word_cnt == 4'd3 && i2c_data_output_en && top_stauts_machine == done) begin 
				read_byte_num_reg <= read_byte_num_reg > 8'd0 ? read_byte_num_reg - 8'd1 : 8'd0;
			end
		end
	end
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			word_cnt <= 4'd0;
		end
		else begin 
			if (top_stauts_machine == read || top_stauts_machine == write) begin
				word_cnt <= word_cnt + 4'd1;
			end
			else if(top_stauts_machine == done && i2c_data_output_en) begin
				word_cnt <= word_cnt + 4'd1;
			end
			else begin word_cnt <= 4'd0; end
		end
	end
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) fifo_wr_req <= 1'b0;
		else 
			if(top_stauts_machine == read) begin
				if (word_cnt == 4'd3) fifo_wr_req <= 1'b1;
				else fifo_wr_req <= 1'b0;
			end
			else if (top_stauts_machine == write) begin
				if (word_cnt == 4'd3 || word_cnt == 4'd6 || word_cnt == 4'd9 )fifo_wr_req <= 1'b1;
				else fifo_wr_req <= 1'b0;
			end
			
	end
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) fifo_input_data <= 8'd0;
		else 
			if (word_cnt == 4'd2 || word_cnt == 4'd5 || word_cnt == 4'd8) fifo_input_data <= 8'd1;
			else fifo_input_data <= fifo_input_data;
	end
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			i2c_device_address_ctrl_word <= 8'd0;
			i2c_byte_address <= 16'd0;
		end
		else begin
			if (word_cnt == 4'd1 ) begin
				if (top_stauts_machine == read) begin
					i2c_device_address_ctrl_word <= {ad_da_chip_address,1'b1};
					i2c_byte_address <= ad_da_byte_address;
				end
				else if(top_stauts_machine == write) begin
					i2c_device_address_ctrl_word <= {eprom_address,1'b0};
					i2c_byte_address <= eprom_byte_address;
				end
			end
			else begin
				fifo_input_data <= fifo_input_data;
				i2c_byte_address <= i2c_byte_address;
			end
		end
	end
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) i2c_start_en <= 1'b0;
		else 
			if (top_stauts_machine == read) 
				if (word_cnt == 4'd5) i2c_start_en <= 1'b1;
				else i2c_start_en <= 1'b0;
			else if(top_stauts_machine == write)
				if (word_cnt == 4'd11) i2c_start_en <= 1'b1;
				else i2c_start_en <= 1'b0;
			else i2c_start_en <= 1'b0;
	end
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin 
			i2c_output_ack <= 1'b0;
			i2c_output_data_reg <= 23'd0; 
		end
		else begin
			if (i2c_data_output_en) begin
				if(word_cnt == 4'd2 && read_byte_num_reg > 8'd1) i2c_output_data_reg <= i2c_output_data;
				else if (word_cnt == 4'd3) i2c_output_ack <= 1'b1;
				else if (word_cnt == 4'd4) i2c_output_ack <= 1'b0;
			end
			else begin
				i2c_output_ack <= 1'b0;
				i2c_output_data_reg <= i2c_output_data_reg;
			end
		end
	end
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) com_rate_set <=1'b0;
		else 
			if (com_rate_set_key_posedge) com_rate_set <= ~com_rate_set;
			else com_rate_set <= com_rate_set;
	end
	key_filter input_test_key_inst(
	.sys_clk(sys_clk),
	.sys_rst(sys_rst_n),
	.key_input(input_test_key),
	.key_reg(input_test_key_reg),
	.key_posedge(input_test_key_posedge),
	.key_negedge()
);
	key_filter output_test_key_inst(
	.sys_clk(sys_clk),
	.sys_rst(sys_rst_n),
	.key_input(output_test_key),
	.key_reg(output_test_key_reg),
	.key_posedge(output_test_key_posedge),
	.key_negedge()
);
	key_filter com_rate_set_key_inst(
	.sys_clk(sys_clk),
	.sys_rst(sys_rst_n),
	.key_input(com_rate_set_key),
	.key_reg(),
	.key_posedge(com_rate_set_key_posedge),
	.key_negedge()
);
	i2c_top i2c_module(
	.sys_clk(sys_clk),
	.sys_rst_n(sys_rst_n),
	.data_input({i2c_byte_address,fifo_q}), 
	.low_8_bit_device_address_crtl_word(i2c_device_address_ctrl_word), 
	.high_8_bit_device_address(8'd0), 
	.communicate_en(i2c_start_en), 
	.double_address_en(1'b0), 
	.output_data_ack(i2c_output_ack), 
	.com_rate_set(com_rate_set), 
	.data_input_en(!fifo_not_empty_flag), 
	.i2c_scl(i2c_scl), 
	.data_output(i2c_output_data),  
	.data_output_en(i2c_data_output_en),
	.fifo_rd_req(i2c_fifo_rd_req),
	.status_machine_led(status_machine_led),
	.i2c_sda(i2c_sda)
	
);
	hc595_ctrl hc595_ctrl_isnt
(
	.sys_clk(sys_clk), //系统时钟，频率50MHz
	.sys_rst_n(sys_rst_n), //复位信号，低有效
	.sel(sel), //数码管位选信号
	.seg(seg), //数码管段选信号
	.stcp(stcp), //数据存储器时钟
	.shcp(shcp), //移位寄存器时钟
	.ds(ds), //串行数据输入
	.oe(oe)//使能信号，低有效
);
	LED_display LED_display_inst(
	.sys_clk(sys_clk),
	.sys_rst(sys_rst_n),
	.DATA({i2c_output_data_reg[15:0],8'd0}),
	.sel(sel),
	.seg(seg)
);
	fifo fifo_inst(
	.aclr(~sys_rst_n),
	.clock(sys_clk),
	.data(fifo_input_data),
	.rdreq(i2c_fifo_rd_req),
	.wrreq(fifo_wr_req),
	.empty(fifo_not_empty_flag),
	.full(),
	.q(fifo_q),
	.usedw(fifo_use_dw)
);
	uart_tx(
	.sys_clk(sys_clk),
	.sys_rst_n(sys_rst_n),
	.fifo_input_data(i2c_output_data[7:0]),
	.fifo_input_data_en(i2c_data_output_en),
	.bode_rate_set(115200),
	.fifo_rd_req(),
	.TX(uart_tx)
);	
endmodule