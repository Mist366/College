module node #(parameter bits, fractional_bits, in_size)(
	input signed [bits-1:0] in [0:in_size-1],
	input signed [bits-1:0] bias,
	input signed [bits-1:0] weights [0:in_size-1],
	input clock,
	input start,
	output reg ready,
	output reg signed [bits-1:0] out
	);
	
	reg signed [bits-1:0] sum;
	reg signed [bits-1:0] sumIn;
	reg signed [bits-1:0] sumOut;
	reg [bits-1:0] count;
	reg [bits-1:0] countOut;
	reg signed [bits-1:0] weight_var;
	reg signed [bits-1:0] weighted_var;
	reg signed [bits-1:0] input_var;
	reg triggered;
	reg signed [bits-1:0] one;
	
	multiplier #(.bits(bits), .fractional_bits(fractional_bits)) multiplier_instance (
			.A(input_var),
			.B(weight_var),
			.Product(weighted_var)
			);
	adder #(.bits(bits)) adder(.A(sumIn), .B(weighted_var), .Sum(sum));
	adder #(.bits(bits)) counter(.A(count), .B(one), .Sum(countOut));
	adder #(.bits(bits)) addBias(.A(sumOut), .B(bias), .Sum(out));
	assign one = 1;

	initial begin
		count = in_size;
		sumIn = 0;
		triggered = 1;
	end

	always @(posedge clock or posedge start or negedge start) begin
		if (count < in_size) begin
			ready <= 0;
			input_var <= in[count];
			weight_var <= weights[count];
			sumIn <= sum;
			count <= countOut;
			triggered <= 0;
		end
		else if (~triggered) begin
			sumOut <= sum;
			ready <= 1;
			triggered <= 1;
		end 
		else if ((count >= in_size) && (start)) begin
			count <= 0;
			sum <= 0;
			sumIn <= 0;
		end
		else begin
			ready <= 0;
		end
	end
endmodule
