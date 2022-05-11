module multiplier_layer #(parameter bits) (
	input signed [bits-1:0] last,
	input signed [bits-1:0] B,
	input A_bit,
	output out_bit,
	output signed [bits-1:0] result
	);
	
	logic C;
	logic signed [bits-1:0] sum;
	logic signed [bits-1:0] transfer;

	bitAnd #(.bits(bits)) bitAnd(.X(B), .Y(A_bit), .Out(transfer));
	adder #(.bits(bits)) adder(.A(last),.B(transfer),.Sum(sum),.Cout(C));
	
	assign result[bits-2:0] = sum[bits-1:1];	
	assign result[bits-1] = C;
	assign out_bit = sum[0];
endmodule
