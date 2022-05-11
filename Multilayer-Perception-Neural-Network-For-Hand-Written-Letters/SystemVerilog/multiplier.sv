module multiplier #(parameter bits, fractional_bits=0)(
	input signed [bits-1:0] A,
	input signed [bits-1:0] B,
	output signed [bits-1:0] Product
	);

	logic signed [bits-1:0] A_logic;
	logic signed [bits-1:0] B_logic;
	logic signed [bits*2-1:0] temp;
	logic signed [bits*2-1:0] shift;
	logic signed [bits-1:0] previous [0:bits-1];
	logic signed [bits-1:0] setup;
	
	assign sign = (A<0 || B<0) && ~(A<0 && B<0);
	assign A_logic = (A>0) ? A : -A;
	assign B_logic = (B>0) ? B : -B;

	bitAnd #(.bits(bits)) bitAnd(.X(B_logic),.Y(A_logic[0]), .Out(setup));
	assign temp[0] = setup[0];
	assign previous[0] = setup>>1 ;

	genvar i;
	generate
		for(i = 1; i<bits; i=i+1) begin : multiplier
			multiplier_layer #(.bits(bits)) multiplier_layer(.B(B_logic), .A_bit(A_logic[i]), .last(previous[i-1]),.result(previous[i]), .out_bit(temp[i]));
		end
	endgenerate

	assign temp[bits*2-1:bits] = previous[bits-1];
	assign shift = (sign == 0) ? temp : -temp;
	assign Product = shift>>>fractional_bits;

endmodule
