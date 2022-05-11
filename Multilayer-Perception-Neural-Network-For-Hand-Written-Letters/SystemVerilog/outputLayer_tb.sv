module outputLayer_tb;
	reg signed [15:0] test1 [0:49];
	for (genvar i = 0; i<50; i=i+1) assign test1[i] = 'h100;

	reg signed [15:0] out [0:9];
	reg signed [15:0] test [0:49];
	reg ready;
	reg clock;
	reg start;
	
	reg signed [15:0] output_weights [0:499];
	reg signed [15:0] output_biases [0:9];

	layer #(.bits(16), .fractional_bits(11), .in_size(50), .out_size(10)) outputLayer(
				.start(start), 
				.clock(clock), 
				.in(test),
				.biases(output_biases),
				.weights(output_weights),  
				.out(out), 
				.ready(ready));

	initial begin
		ready = 0;
		clock = 0;
		start = 0;
		$readmemh("output_weight_16.mem", output_weights);	
		$readmemh("output_bias_16.mem", output_biases);
		assign test = test1;
		#10
		start = 1;
		while (~ready) begin
			#1 clock = ~clock;
		end
	end
endmodule
