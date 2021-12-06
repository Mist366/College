module multiplier_layer_tb;
	logic signed [15:0] last;
	logic signed [15:0] B;
	logic A_bit;

	logic out;
	logic signed [15:0] result;

	multiplier_layer #(.bits(16)) multiplier_layer(.last(last), .B(B), .A_bit(A_bit), .out_bit(out), .result(result));

	initial begin
		assign last = 'h70F0;
		assign B = 'h5555;
		assign A_bit = 1;
	end
endmodule
