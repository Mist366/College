module ReLU #(parameter bits)(input signed [bits-1:0] in,
	output signed [bits-1:0] out);
	
	assign out = (in>0) ? in : 0;
endmodule
