module bitAnd #(parameter bits) (
	input Y,
	input [bits-1:0] X,
	output [bits-1:0] Out
	);
	
	logic [bits-1:0] temp;

	genvar i;
	generate
		for(i = 0; i<bits; i=i+1) begin : ands
			assign temp[i] = Y&X[i];
		end
	endgenerate
 
	assign Out = temp;
endmodule
