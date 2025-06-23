/*
	20250610
		scl波形产生
		单双设备地址写入
		双地址字节写入
		标准/快速模式切换
		设备单双字节地址切换
		fifo数据连续读取、
		sda数据输出、
		从sda线上读取数据
		向cpu输出数据
		cpu响应等待
		根据fifo数据，连续读取字节(fifo内给出需要读取的字节数，自动读取并输出)
		从机响应等待/报错
*/
module i2c_top(
	input wire sys_clk,
	input wire sys_rst_n,
	input wire [23:0] data_input, //输入数据，前两个字节是字节地址，第三个字节与fifo相连，进行连续读写，
	input wire [7:0] low_8_bit_device_address_crtl_word, //低7位设备地址和读写控制字
	input wire [7:0] high_8_bit_device_address, //高8位地址
	input wire communicate_en, //传输/接收使能，要求至少持续1个sys_clk周期
	input wire double_address_en, //高8位设备地址使能，与transmit_en同步进入高电平，持续2个sys_clk周期
	input wire output_data_ack, 
	//输出数据握手信号，由高电平变为低电平表示cpu接收到数据
	//由高电平变为低电平后才能继续下一次传输或接收数据，否则连续读/写
	input wire com_rate_set, //传输速率设置，高电平400kbps，低电平100kbps
	input wire data_input_en, //输入fifo的数据有效指示
	output wire [3:0] status_machine_led,
	output wire error_en,
	output reg i2c_scl,  //i2c的同步时钟
	output wire [23:0] data_output,  //输出数据(前两个字节是字节地址，第三个字节为读取到的数据)
	output reg data_output_en, //任务完成信号，高电平有效，持续直至主机返回握手信号
	output reg fifo_rd_req,
	inout wire i2c_sda
);
	parameter status_machine_idle = 4'd0;//闲置
	parameter status_machine_start = 4'd1;//启动状态
	parameter status_machine_send_device_address = 4'd2;//发送设备地址
	parameter status_machine_wait_device_ack = 4'd3;//等待设备响应
	parameter status_machine_address_low_byte_send = 4'd4; //发送低八位字节地址
	parameter status_machine_wait_address_low_byte_ack = 4'd5; //等待设备响应
	parameter status_machine_address_high_byte_send = 4'd6; //发送高八位字节地址
	parameter status_machine_wait_address_high_byte_ack = 4'd7; //等待设备响应
	parameter status_machine_send_data = 4'd8; //写入数据
	parameter status_machine_wait_send_data_ack = 4'd9; //等待设备响应
	parameter status_machine_read_data = 4'd10; //读取数据
	parameter status_machine_read_data_ack = 4'd11; //向从设备响应
	parameter status_machine_mission_end = 4'd12; //任务结束
	parameter status_machine_wait_end_ack = 4'd13; //等待CPU响应
	//状态机参数
	parameter bit_cnt_value_100k = 499;
	parameter bit_cnt_value_400k = 124;
	reg data_output_finished_reg;
	reg data_input_en_reg;
	reg sda_reg;
	reg sda_flag;
	reg rd_flag;
	reg ctrl_word_en;
	reg rd_wr_ctrl_word_reg;
	reg error_flag;
	reg [1:0] cnt_flag;//计数标识符，用于判断是否是双字节的设备地址
	reg [3:0] status_machine;
	reg [8:0] cnt_value_reg;
	reg [4:0] byte_cnt;
	reg [8:0] bit_cnt;
	reg [15:0] address_ctrl_word_reg;
	reg [23:0] data_input_reg;
	reg [23:0] data_output_reg;
	reg [7:0] data_read_numbers_reg; //连续读取数据个数的寄存器
	assign i2c_sda = sda_flag ? sda_reg : 1'bz;
	assign data_output = data_output_en ? data_output_reg : 24'd0;
	assign status_machine_led = status_machine;
	assign error_en = error_flag;
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin cnt_flag <= 2'b00; end
		else begin 
			if(communicate_en && !double_address_en && cnt_flag == 2'd0) cnt_flag <= 2'd1;
			else if (communicate_en && double_address_en && cnt_flag == 2'd0) cnt_flag <= 2'd2;
			else if(cnt_flag == 2'd1 || cnt_flag == 2'd2) begin
				if (status_machine == status_machine_wait_end_ack) cnt_flag <= 2'd0;
				else if(error_flag) cnt_flag <= 2'd0;
			//等待状态机恢复到等待结束信号的状态
			end
		end
 	end
	//计数标识符设置
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin 
			status_machine <= status_machine_idle;
			cnt_value_reg <= 0;
			rd_wr_ctrl_word_reg <= 0;
			rd_flag <= 1'b0;
			fifo_rd_req <= 1'b0;
			data_input_reg <= 24'd0;
			address_ctrl_word_reg <= 0;
			data_read_numbers_reg <= 0;
			data_input_en_reg <= 1'b0;
			ctrl_word_en <= 1'b0;
		end
		else begin 
			if(status_machine == status_machine_idle) begin 
				//if (error_flag) begin status_machine <= status_machine_idle; end
				if (cnt_flag != 2'd0)
				//!error_flag
				//) 
				begin
					status_machine <= status_machine_start;
					ctrl_word_en <= 1'b0;
				end
			end
			//满足条件进入启动状态
			else if (status_machine == status_machine_start) begin 
				if (!data_input_en_reg) begin 
					if (data_input_en) begin 
						data_input_en_reg <= 1'b1;
						rd_flag <= 1'b0;
						cnt_value_reg <= com_rate_set ? bit_cnt_value_400k : bit_cnt_value_100k;
						address_ctrl_word_reg <= {high_8_bit_device_address,low_8_bit_device_address_crtl_word};
						rd_wr_ctrl_word_reg <= low_8_bit_device_address_crtl_word[0];
						ctrl_word_en <= 1'b1;
						fifo_rd_req <= 1'b0;
					end
					else begin 
						status_machine <= status_machine_wait_end_ack; 
					end
				end
				else begin 
					if (ctrl_word_en) begin
						ctrl_word_en <= 1'b0;
						if (rd_wr_ctrl_word_reg) fifo_rd_req <= 1'b1;
						else fifo_rd_req <= 1'b0;
					end
					else begin 
						status_machine <= status_machine_send_device_address;
						data_input_reg <= data_input;	
						ctrl_word_en <= 1'b0;
						data_input_en_reg <= 1'b0;
						fifo_rd_req <= 1'b0;
					end
				end
			end
			//启动状态：缓存设备地址、读写控制字、初始化读取标识符
			else if (status_machine == status_machine_send_device_address) begin 
				if (rd_wr_ctrl_word_reg) begin data_read_numbers_reg <= data_input[7:0]; end
				if (cnt_flag == 2'd1) 
					if (bit_cnt == cnt_value_reg && byte_cnt == 5'd9) status_machine <= status_machine_wait_device_ack;
					else status_machine <= status_machine_send_device_address;
				else if (cnt_flag == 2'd2)
					if (bit_cnt == cnt_value_reg && byte_cnt == 5'd17) status_machine <= status_machine_wait_device_ack;
					else status_machine <= status_machine_send_device_address;
				else status_machine <= status_machine_send_device_address;
			end
			//发送设备地址，达到对应计数值后进入等待响应状态
			else if (status_machine == status_machine_wait_device_ack) begin 
				if(!error_flag && bit_cnt == cnt_value_reg) begin
					if (!rd_flag) status_machine <= status_machine_address_high_byte_send;
					else status_machine <= status_machine_read_data;
				end
				else if (error_flag &&  bit_cnt == cnt_value_reg) begin
					status_machine <= status_machine_idle;
				end
			end 
			//如果响应成功则根据标志符选择进入发送字节地址状态还是数据读取状态
			else if (status_machine == status_machine_address_high_byte_send) begin 
				if (byte_cnt == 5'd7 && bit_cnt == cnt_value_reg) status_machine <= status_machine_wait_address_high_byte_ack;
			end
			//发送高八位字节地址，达到对应计数值后进入等待响应状态
			else if (status_machine == status_machine_address_low_byte_send) begin 
				if (byte_cnt == 5'd7 && bit_cnt == cnt_value_reg) status_machine <= status_machine_wait_address_low_byte_ack;
			end
			//发送低八位字节地址，达到对应计数值后进入等待响应状态
			else if (status_machine == status_machine_wait_address_high_byte_ack) begin 
				if (!error_flag && bit_cnt == cnt_value_reg) begin
					status_machine <= status_machine_address_low_byte_send;
				end
				else if (error_flag) begin
					status_machine <= status_machine_idle;
				end
			end 
			//等待响应成功，如果响应成功则继续发送低八位地址
			else if (status_machine ==  status_machine_wait_address_low_byte_ack) begin 
				if (rd_wr_ctrl_word_reg) begin
					if (!error_flag && bit_cnt == cnt_value_reg) begin
						status_machine <= status_machine_mission_end;
					end
					else if (error_flag) begin
					status_machine <= status_machine_idle;
					end
				end
				else begin
					if (!error_flag && bit_cnt == cnt_value_reg) begin
						status_machine <= status_machine_send_data;
					end
					else if (error_flag) begin
					status_machine <= status_machine_idle;
					end
				end
			end
			//等待响应成功，如果响应成功则根据读写控制字决定读写状态跳变
			else if (status_machine == status_machine_read_data) begin
				if (data_read_numbers_reg > 0) begin
					if (byte_cnt == 5'd7 && bit_cnt == cnt_value_reg) status_machine <= status_machine_read_data_ack;
				end
				else begin
					status_machine <= status_machine_mission_end;
				end
			end
			//读取状态进行中间时刻采样
			else if (status_machine == status_machine_read_data_ack) begin 
				if (byte_cnt == 0 && bit_cnt == cnt_value_reg) begin
					if (data_read_numbers_reg > 0 ) begin
						data_read_numbers_reg <= data_read_numbers_reg - 1;
						status_machine <= status_machine_read_data;
					end
					else begin status_machine <= status_machine_mission_end; end
				end
				
			end
			/*
				单个字节读取完成后向主机发送数据和响应从机，
				然后根据communicate_en的高低电平决定是否继续进行读取响应从机，
				并根据communicate_en的高低电平决定是否继续进行读取
			*/
			else if (status_machine == status_machine_send_data) begin 
				if (byte_cnt == 5'd0 && bit_cnt == 0) begin 
					if (data_input_en && !fifo_rd_req ) fifo_rd_req <= 1'b1;
				end
				else if (fifo_rd_req == 1'b1) begin
					fifo_rd_req <= 1'b0;
				end
				else if (byte_cnt == 5'd0 && bit_cnt == 2) begin
					data_input_reg <= data_input;
					data_input_en_reg <= 1'b1;
				end
				else if (byte_cnt == 5'd7 && bit_cnt == cnt_value_reg) begin 
					status_machine <= status_machine_wait_send_data_ack;
					data_input_en_reg <= 1'b0;
				end	
				/*
				else if (fifo_rd_req == 1'b1) begin 
					fifo_rd_req <= 1'b0; 
					data_input_reg <= data_input;
					data_input_en_reg <= 1'b1;
				end
				else if (byte_cnt == 5'd7 && bit_cnt == cnt_value_reg) begin 
					status_machine <= status_machine_wait_send_data_ack;
					data_input_en_reg <= 1'b0;
				end	
				end
				else begin 
					if (!data_input_en_reg) status_machine <= status_machine_mission_end; 
					else begin
					end
				end
				*/
			end
			//发送数据，达到对应计数值后进入等待响应状态
			else if (status_machine == status_machine_wait_send_data_ack) begin 
				if (!error_flag && bit_cnt == cnt_value_reg) begin 
					if(!data_input_en) status_machine <= status_machine_mission_end;
					else status_machine <= status_machine_send_data;
				end
				else if (error_flag) begin
					status_machine <= status_machine_idle;
				end
			end 
			//接收到来自从机的响应，并根据fifo内是否还有数据继续发送
			else if (status_machine == status_machine_mission_end) begin 
				if (rd_wr_ctrl_word_reg) begin
					if (byte_cnt == 5'd0 && bit_cnt == cnt_value_reg) begin 
						if (!rd_flag) begin 
							rd_flag <= 1'b1;
							status_machine <= status_machine_send_device_address;
						end
						else begin 
							rd_flag <= 1'b0; 
							status_machine <= status_machine_wait_end_ack;
						end
					end 
				end //读取状态
				else begin
					if (byte_cnt == 5'd0 && bit_cnt == cnt_value_reg) begin
						status_machine <= status_machine_wait_end_ack;
					end
				end
			end
			//如果CPU停止读取/fifo内没有数据，则进入等待cpu响应状态
			else if (status_machine == status_machine_wait_end_ack) begin 
			//若cpu返回响应，则回到闲置状态
				if (output_data_ack) status_machine <= status_machine_idle;
				else status_machine <= status_machine_wait_end_ack;
			end
		end
	end
	//状态机跳变设置
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin 
			bit_cnt <= 9'd0;
		end
		else begin 
			if (status_machine == status_machine_send_device_address) 
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_address_high_byte_send) 
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_address_low_byte_send) 
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_read_data) 
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_send_data) 
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_mission_end)
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_wait_address_high_byte_ack)
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_wait_address_low_byte_ack)
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_wait_device_ack)
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_wait_send_data_ack)
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else if (status_machine == status_machine_read_data_ack) 
				bit_cnt <= bit_cnt == cnt_value_reg ? 9'd0 : bit_cnt + 9'd1;
			else bit_cnt <= 9'd0;
		end
	end
	//单字计数器
	
	always@(posedge sys_clk or negedge sys_rst_n) begin 
		if (!sys_rst_n) begin byte_cnt <= 5'd0; end
		else begin 
		
			if (status_machine == status_machine_send_device_address) begin
				if (cnt_flag == 2'd2) begin
					if (byte_cnt == 5'd17 && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
					else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
				end
				else if (cnt_flag == 2'd1) begin
					if (byte_cnt == 5'd9 && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
					else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
				end
				else byte_cnt <= 5'd0;
			end
			
			else if (status_machine == status_machine_address_high_byte_send) begin
				if (byte_cnt == 5'd7 && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
			end
			
			else if (status_machine == status_machine_address_low_byte_send) begin
				if (byte_cnt == 5'd7 && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
			end
			
			else if (status_machine == status_machine_read_data) begin
				if (byte_cnt == 5'd7 && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
			end
			
			else if (status_machine == status_machine_send_data) begin
				if (byte_cnt == 5'd7 && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
			end
			
			else if (status_machine == status_machine_mission_end) begin
				if (byte_cnt == 5'd0 && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= 5'd0;
			end
			
			else if (status_machine == status_machine_wait_address_high_byte_ack) begin 
				if (!error_flag && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
			end
			
			else if (status_machine == status_machine_wait_address_low_byte_ack) begin
				if (!error_flag && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
			end
			
			else if (status_machine == status_machine_wait_device_ack) begin 
				if (!error_flag && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
			end
			
			else if (status_machine == status_machine_wait_send_data_ack) begin 
				if (!error_flag && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
			end
			
			else if (status_machine == status_machine_read_data_ack) begin 
				if (!error_flag && bit_cnt == cnt_value_reg) byte_cnt <= 5'd0;
				else byte_cnt <= bit_cnt == cnt_value_reg ? byte_cnt + 4'd1 : byte_cnt;
			end
			
			else begin byte_cnt <= 4'd0; end
		end
	end
	//字节计数器
	
	always@(posedge sys_clk or negedge sys_rst_n) begin 
		if (!sys_rst_n) begin i2c_scl <= 1'b1; end
		else begin 
			if (status_machine == status_machine_send_device_address) begin
				if(cnt_flag == 2'd1) begin 
					if (byte_cnt == 5'd0) i2c_scl <= 1'b1;
					else if (byte_cnt == 5'd1) begin
						i2c_scl <= 1'b0;
					end
					else if (byte_cnt == 5'd9) begin
						if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
						else i2c_scl <= 1'b1;
					end
					else  begin 
						if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
						else i2c_scl <= 1'b1;
					end
				end
				else if (cnt_flag == 2'd2) begin 
					if (byte_cnt == 5'd0) i2c_scl <= 1'b1;
					else if (byte_cnt == 5'd1)  begin
						i2c_scl <= 1'b0;
					end
					else if (byte_cnt == 5'd17) begin 
						if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
						else i2c_scl <= 1'b1;
					end
					else begin
						if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
						else i2c_scl <= 1'b1;
					end
				end
				else i2c_scl <= 1'b0;
			end
			
			else if (status_machine == status_machine_address_high_byte_send) begin
				if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
				else i2c_scl <= 1'b1;
			end
			
			else if (status_machine == status_machine_address_low_byte_send) begin
				if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
				else i2c_scl <= 1'b1;
			end
			
			else if (status_machine == status_machine_read_data) begin 
				if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
				else i2c_scl <= 1'b1;
			end
			else if (status_machine == status_machine_send_data) begin 	
				if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
				else i2c_scl <= 1'b1;
			end
			else if (status_machine == status_machine_wait_address_high_byte_ack) begin 
				if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
				else i2c_scl <= 1'b1;
			end
			
			else if (status_machine == status_machine_wait_address_low_byte_ack) begin
				if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
				else i2c_scl <= 1'b1;
			end
			
			else if (status_machine == status_machine_wait_device_ack) begin 
				if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
				else i2c_scl <= 1'b1;
			end       
			else if (status_machine == status_machine_mission_end) begin 
				if (byte_cnt == 5'd0)  begin 
					if (bit_cnt <=cnt_value_reg/4) i2c_scl <= 1'b0;
					else i2c_scl <= 1'b1;
				end
				else 
					i2c_scl <= 1'b1;
			end
			else if (status_machine == status_machine_wait_send_data_ack) begin 
				if (byte_cnt == 5'd0)  begin 
					if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
					else i2c_scl <= 1'b1;
				end
				else 
					i2c_scl <= 1'b1;
			end
			else if (status_machine == status_machine_read_data_ack) begin 
				if (byte_cnt == 5'd0)  begin 
					if (bit_cnt <=cnt_value_reg/4 || bit_cnt >=cnt_value_reg*3/4) i2c_scl <= 1'b0;
					else i2c_scl <= 1'b1;
				end
				else 
					i2c_scl <= 1'b1;
			end
			else i2c_scl <= 1'b1;
		end
	end
	//scl生成器
	
	always@(*) begin
		if (!sys_rst_n) begin sda_flag = 1'b0; end
		else begin
			case (status_machine)
				status_machine_idle:sda_flag = 1'b1;
				status_machine_address_high_byte_send:sda_flag = 1'b1;
				status_machine_address_low_byte_send:sda_flag = 1'b1;
				status_machine_mission_end:sda_flag = 1'b1;
				status_machine_read_data:sda_flag = 1'b0;
				status_machine_read_data_ack:sda_flag = 1'b1;
				status_machine_send_data:sda_flag = 1'b1;
				status_machine_send_device_address:sda_flag = 1'b1;
				status_machine_start:sda_flag = 1'b1;
				status_machine_wait_address_high_byte_ack:sda_flag = 1'b0;
				status_machine_wait_address_low_byte_ack:sda_flag = 1'b0;
				status_machine_wait_device_ack:sda_flag = 1'b0;
				status_machine_wait_end_ack:sda_flag = 1'b1;
				status_machine_wait_send_data_ack:sda_flag = 1'b0;
				default:sda_flag = 1'b1;
			endcase
		end
	end
	//sda控制
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin 
			sda_reg <= 1'b1;
		end
		else begin
			case (status_machine)
				status_machine_idle:begin 
					sda_reg <= 1'b1;
				end
				status_machine_start:begin 
					sda_reg <= 1'b1;
				end
				status_machine_send_device_address:begin 
						if (cnt_flag == 2'd1) begin 
							case(byte_cnt) 
								5'd0:begin sda_reg <= (bit_cnt >= cnt_value_reg/2) ? 1'b0 : 1'b1;end
								5'd1:begin sda_reg <= 1'b0;end
								5'd2:begin sda_reg <= address_ctrl_word_reg[7];end
								5'd3:begin sda_reg <= address_ctrl_word_reg[6];end
								5'd4:begin sda_reg <= address_ctrl_word_reg[5];end
								5'd5:begin sda_reg <= address_ctrl_word_reg[4];end
								5'd6:begin sda_reg <= address_ctrl_word_reg[3];end
								5'd7:begin sda_reg <= address_ctrl_word_reg[2];end
								5'd8:begin sda_reg <= address_ctrl_word_reg[1];end
								5'd9:begin sda_reg <= rd_flag;end
							endcase
							/*
							
							if(rd_wr_ctrl_word_reg) begin
								if (byte_cnt == 0) begin
								sda_reg <= (bit_cnt >= cnt_value_reg/2) ? 1'b0 : 1'b1;
								end
								else if (byte_cnt >= 2 || byte_cnt <= 3) begin sda_reg <= address_ctrl_word_reg[9-byte_cnt];end
								else begin sda_reg <= 1'b0; end
							end
							else begin
								if (byte_cnt == 0) begin
								sda_reg <= (bit_cnt >= cnt_value_reg/2) ? 1'b0 : 1'b1;
								end
								else if (byte_cnt >= 2 || byte_cnt <= 8) begin sda_reg <= address_ctrl_word_reg[9-byte_cnt];end
								else if (byte_cnt == 9 ) begin sda_reg <= 1'b0; end
							end
							*/
						end
						else if(cnt_flag == 2'd2) begin 
							case(byte_cnt) 
								5'd0:begin sda_reg <= (bit_cnt >= cnt_value_reg/2) ? 1'b0 : 1'b1;end
								5'd1:begin sda_reg <= 1'b0;end
								5'd2:begin sda_reg <= address_ctrl_word_reg[15];end
								5'd3:begin sda_reg <= address_ctrl_word_reg[14];end
								5'd4:begin sda_reg <= address_ctrl_word_reg[13];end
								5'd5:begin sda_reg <= address_ctrl_word_reg[12];end
								5'd6:begin sda_reg <= address_ctrl_word_reg[11];end
								5'd7:begin sda_reg <= address_ctrl_word_reg[10];end
								5'd8:begin sda_reg <= address_ctrl_word_reg[9];end
								5'd9:begin sda_reg <= address_ctrl_word_reg[8];end
								5'd10:begin sda_reg <= address_ctrl_word_reg[7];end
								5'd11:begin sda_reg <= address_ctrl_word_reg[6];end
								5'd12:begin sda_reg <= address_ctrl_word_reg[5];end
								5'd13:begin sda_reg <= address_ctrl_word_reg[4];end
								5'd14:begin sda_reg <= address_ctrl_word_reg[3];end
								5'd15:begin sda_reg <= address_ctrl_word_reg[2];end
								5'd16:begin sda_reg <= address_ctrl_word_reg[1];end
								5'd17:begin sda_reg <= rd_flag;end
							endcase
						end
				end
				status_machine_wait_device_ack:begin 
					sda_reg <= 1'bz;
				end
				status_machine_address_high_byte_send:begin 
					if (bit_cnt == 2) begin
						sda_reg <= data_input_reg[23-byte_cnt];
					end
				end
				status_machine_wait_address_high_byte_ack:begin 
					sda_reg <= 1'bz;
				end
				status_machine_address_low_byte_send:begin 
					if (bit_cnt == 2) begin
						sda_reg <= data_input_reg[15-byte_cnt];
					end
				end
				status_machine_wait_address_low_byte_ack:begin 
					sda_reg <= 1'b1;
				end
				status_machine_read_data:begin 
					sda_reg <= 1'b1;
				end
				status_machine_read_data_ack:begin 
					sda_reg <= 1'b1;
				end
				status_machine_send_data:begin 
					if (bit_cnt == 3) begin
						sda_reg <= data_input_reg[7-byte_cnt];
					end
				end
				status_machine_wait_send_data_ack:begin 
					sda_reg <= 1'b1;
				end
				status_machine_mission_end:begin 
					if (bit_cnt <= cnt_value_reg/2) sda_reg<=1'b0;
					else sda_reg <= 1'b1;
				end
				status_machine_wait_end_ack:begin 
					sda_reg <= 1'b1;
				end
				default:sda_reg <= 1'b1;
			endcase
		end
	end
	//sda波形生成
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin 
			data_output_reg <= 24'd0;
			data_output_en <= 1'b0;
			data_output_finished_reg <= 1'b0;
		end
		else begin 
			if (status_machine == status_machine_read_data) begin
				data_output_finished_reg <= 1'b0;
				if (bit_cnt == cnt_value_reg/2) begin 
					data_output_reg[7-byte_cnt] <= i2c_sda;
					data_output_en <= 1'b0;
				end
				else if (bit_cnt == cnt_value_reg && byte_cnt == 7) begin
					data_output_reg[23:8] <= data_input_reg[23:8];
					data_output_en <= 1'b0;
				end
				else begin 
					data_output_reg <= data_output_reg;
					data_output_en <= data_output_en;
				end
			end
			else if (status_machine == status_machine_read_data_ack) begin 
				if (!data_output_en && !output_data_ack) begin 
					if (!data_output_finished_reg ) begin 
						data_output_en <= 1'b1;
						data_output_finished_reg <= 1'b1;
					end
				end
				else if (data_output_en && output_data_ack) begin
					data_output_en <= 1'b0; 
					data_output_reg <= 24'd0;
				end
				else begin 
					data_output_en <= data_output_en;
					data_output_reg <= data_output_reg;
				end
			end
			else if (status_machine == status_machine_wait_end_ack) begin
				if (!data_output_en && !output_data_ack) begin 
					if (!data_output_finished_reg ) begin 
						data_output_en <= 1'b1;
						data_output_finished_reg <= 1'b1;
					end
				end
				else if (data_output_en && output_data_ack) begin
					data_output_en <= 1'b0; 
					data_output_reg <= 24'd0;
				end
				else begin 
					data_output_en <= data_output_en;
					data_output_reg <= data_output_reg;
				end
			end
			else begin 
				data_output_reg <= 24'd0;
				data_output_en <= 1'b0;
				data_output_finished_reg <= 1'b0;
			end
		end
	end
	//读取模式下从sda中读取数据（在scl的高电平中间时刻进行采样）
	//兼顾任务完成标识
	
	always@(posedge sys_clk or negedge sys_rst_n) begin
		if (!sys_rst_n) begin
			error_flag <= 1'b0;
		end
		else begin
		if (communicate_en) begin error_flag <= 1'b0;end
		else begin
				case (status_machine)
					status_machine_wait_device_ack:begin 
						if (i2c_scl) begin
							error_flag <= i2c_sda;
						end
						else begin
							error_flag <= error_flag;
						end
					end
					status_machine_wait_address_high_byte_ack:begin 
						if (i2c_scl) begin
							error_flag <= i2c_sda;
						end
						else begin
							error_flag <= error_flag;
						end
					end
					status_machine_wait_address_low_byte_ack:begin 
						if (i2c_scl) begin
							error_flag <= i2c_sda;
						end
						else begin
							error_flag <= error_flag;
						end
					end
					status_machine_wait_send_data_ack:begin 
						if (i2c_scl) begin
							error_flag <= i2c_sda;
						end
						else begin
							error_flag <= error_flag;
						end
					end
					default:error_flag <= error_flag;
				endcase
			end
		end
	end
	//从机响应等待
endmodule
