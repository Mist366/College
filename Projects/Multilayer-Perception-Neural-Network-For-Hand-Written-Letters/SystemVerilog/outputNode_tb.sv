
module outputNode_tb;
	

	reg signed [15:0] test_memory [0:49];
	for (genvar i = 0; i<50; i=i+1) assign test_memory[i] = 'h100;

	
	reg signed [15:0] in [0:49];	reg signed [15:0] out;

	reg signed [15:0] weights [0:499];
	reg signed [15:0] bias [0:9];
	reg signed [15:0] weight [0:49];
	reg signed [15:0] bia;
	
	reg clk;
	reg start;
	reg ready;

	node #(.bits(16), .fractional_bits(11), .in_size(50)) outputNode (
		.start(start),
		.clock(clk),
		.in(in),
		.ready(ready),
		.out(out),
		.bias(bia),
		.weights(weight)
		);

	initial begin
		clk = 0;
		start = 0;
		ready = 0;
		$readmemh("output_weight_16.mem", weights);	
		$readmemh("output_bias_16.mem", bias);
		assign in = test_memory;
		assign weight = weights[0:49];
		assign bia = bias[0];
		#10
		start = 1;
		while (~ready) begin
			#1 clk = ~clk;
		end
    	end
endmodule