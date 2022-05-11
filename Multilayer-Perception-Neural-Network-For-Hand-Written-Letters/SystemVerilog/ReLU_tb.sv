module ReLU_tb;
	reg signed [15:0] positive;
	reg signed [15:0] negative;

	reg signed [15:0] value;
	reg signed [15:0] out;

	ReLU #(.bits(16)) ReLU(.in(value), .out(out));

	assign positive = 8;
	assign negative = -8;

	initial begin
		assign value = negative;
		#10
		assign value = positive;

	end
endmodule
