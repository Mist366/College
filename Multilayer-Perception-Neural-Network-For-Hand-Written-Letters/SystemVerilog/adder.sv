module adder #(parameter bits)(
	input signed [bits-1:0] A,
	input signed [bits-1:0] B,
	input Cin=0,
	output signed [bits-1:0] Sum,
	output Cout
	);
	
	reg signed [bits:0] C_;
	reg signed [bits-1:0] buffered_Sum;
	assign C_[0] = Cin;

	genvar i;
	generate
		for(i = 0; i < bits; i=i+1) begin : adders
			fullAdder fullAdder(.A(A[i]), .B(B[i]), .Cin(C_[i]), .Cout(C_[i+1]), .Sum(buffered_Sum[i]));
		end
	endgenerate
	
	assign Cout = C_[bits];
	assign Sum = buffered_Sum;
endmodule
