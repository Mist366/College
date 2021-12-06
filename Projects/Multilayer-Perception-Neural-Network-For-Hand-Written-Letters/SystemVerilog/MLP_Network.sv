module MLP_Network #(parameter bits, fractional_bits) (
	input start,
	input clock, 
	input signed [bits-1:0] picture [0:783], 
	output ready, 
	output signed [bits-1:0] results [0:9]
	);

	reg signed [bits-1:0] transfer [0:49];
	reg hiddenReady;
	reg outputReady;

	reg signed [bits-1:0] hidden_weights [0:39199];
	reg signed [bits-1:0] hidden_biases [0:49];
	reg signed [bits-1:0] out_weights [0:499];
	reg signed [bits-1:0] out_biases [0:9];

	initial begin
		$readmemh($sformatf("hidden_weight_%0d.mem",bits), hidden_weights);	
		$readmemh($sformatf("hidden_bias_%0d.mem",bits), hidden_biases);
		$readmemh($sformatf("output_weight_%0d.mem",bits), out_weights);	
		$readmemh($sformatf("output_bias_%0d.mem",bits), out_biases);
	end

	layer #(.bits(bits), .fractional_bits(fractional_bits), .in_size(784), .out_size(50)) hiddenLayer(
				.start(start), 
				.clock(clock), 
				.in(picture),
				.biases(hidden_biases),
				.weights(hidden_weights),  
				.out(transfer), 
				.ready(hiddenReady));
	layer #(.bits(bits), .fractional_bits(fractional_bits), .in_size(50), .out_size(10)) outputLayer(
				.start(hiddenReady), 
				.clock(clock), 
				.in(transfer), 
				.biases(out_biases),
				.weights(out_weights), 
				.out(results), 
				.ready(outputReady));
	assign ready = outputReady;
endmodule
