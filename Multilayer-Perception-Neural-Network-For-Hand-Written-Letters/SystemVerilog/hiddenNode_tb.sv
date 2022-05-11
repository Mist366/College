module hiddenNode_tb;
	
	reg signed [15:0] test_memory [0:783];
	reg signed [15:0] in [0:783];
	reg signed [15:0] out;

	reg signed [15:0] weights [0:39199];
	reg signed [15:0] bias [0:49];
	reg signed [15:0] weight [0:783];
	reg signed [15:0] bia;
	
	reg clk;
	reg start;
	reg ready;

	node #(.bits(16), .fractional_bits(11), .in_size(784)) hiddenNode (
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
		$readmemh("picture_1_16.mem", test_memory);
		$readmemh("hidden_weight_16.mem", weights);	
		$readmemh("hidden_bias_16.mem", bias);
		assign in = test_memory;
		assign weight = weights[0:783];
		assign bia = bias[0];
		#10
		start = 1;
		while (~ready) begin
			#1 clk = ~clk;
		end
    end
endmodule