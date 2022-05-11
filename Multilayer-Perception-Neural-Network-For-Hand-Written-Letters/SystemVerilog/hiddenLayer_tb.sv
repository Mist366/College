module hiddenLayer_tb;
	reg signed [15:0] picture1 [0:783];

	reg signed [15:0] out [0:49];
	reg signed [15:0] picture [0:783];
	reg ready;
	reg clock;
	reg start;
	
	reg signed [15:0] hidden_weights [0:39199];
	reg signed [15:0] hidden_biases [0:49];

	layer #(.bits(16), .fractional_bits(11), .in_size(784), .out_size(50)) hiddenLayer(
				.start(start), 
				.clock(clock), 
				.in(picture),
				.biases(hidden_biases),
				.weights(hidden_weights),  
				.out(out), 
				.ready(ready));

	initial begin
		ready = 0;
		clock = 0;
		start = 0;
		$readmemh("picture_1_16.mem", picture1);
		$readmemh("hidden_weight_16.mem", hidden_weights);	
		$readmemh("hidden_bias_16.mem", hidden_biases);
		assign picture = picture1;
		#10
		start = 1;
		while (~ready) begin
			#1 clock = ~clock;
		end
	end
endmodule
