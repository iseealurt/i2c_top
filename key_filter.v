module key_filter(

input wire sys_clk,
input wire sys_rst,
input wire key_input,

output reg key_reg,
output wire key_posedge,
output wire key_negedge
);

reg [15:0] cnt_5000;
reg key_edge_reg;

assign key_posedge = {key_edge_reg,key_reg} == 2'b01 ? 1'b1 : 1'b0;
assign key_negedge = {key_edge_reg,key_reg} == 2'b10 ? 1'b1 : 1'b0;

always@(posedge sys_clk or negedge sys_rst) begin
	if (!sys_rst) key_edge_reg <= 1'b0;
	else 
		key_edge_reg <= key_reg;
end

always@(posedge sys_clk or negedge sys_rst) begin
	if (!sys_rst) 
		cnt_5000 <= 16'd0;
	else 
		cnt_5000 <= cnt_5000 == 16'd9999 ? 13'd0 : cnt_5000 + 13'd1;
end


always@(posedge sys_clk or negedge sys_rst) begin
	if (!sys_rst) 
		key_reg <= 1'b0;
	else 
		if (cnt_5000 == 16'd9999)
			key_reg <= key_input;
end 

endmodule
