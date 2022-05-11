module bitAnd_tb;
	logic bits;
	logic [15:0] word;
	logic [15:0] out;

	bitAnd #(.bits(16)) bitAnd(.X(word),.Y(bits),.Out(out));

	initial begin
		assign word = 10;
		assign bits = 1;
		#10
		assign bits = 0;
		#10
		assign bits = 1;
	end
endmodule
