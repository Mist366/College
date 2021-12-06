module layer #(parameter bits, fractional_bits, in_size, out_size)(
	input signed [bits-1:0] in [0:in_size-1], 
	input clock,
	input start,
	input signed [bits-1:0] weights [0:in_size*out_size-1],
	input signed [bits-1:0] biases [0:out_size-1],
	output reg signed [bits-1:0] out [0:out_size-1],
	output ready
	);

	reg signed [bits-1:0] to_activation [0:out_size-1];
	reg [out_size-1:0] allReady;
	
	genvar i;
	generate
		for (i = 0; i<out_size; i=i+1) begin : output_layer
			node #(.bits(bits), .fractional_bits(fractional_bits), .in_size(in_size)) node(
					.start(start), 
					.clock(clock), 
					.in(in), 
					.ready(allReady[i]), 
					.out(to_activation[i]), 
					.bias(biases[i]), 
					.weights(weights[in_size*i:in_size-1+in_size*i]));
			ReLU #(.bits(bits)) ReLU(.in(to_activation[i]), .out(out[i]));	
		end
	endgenerate
	assign ready = &allReady;
endmodule
