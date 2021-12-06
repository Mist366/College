module adder_tb;
	logic [15:0] A;
	logic [15:0] B;
	logic [15:0] Sum;

	adder #(.bits(16)) adder(.A(A), .B(B), .Sum(Sum));

	initial 
		begin
		assign A = 'hFF00;
		assign B = 0;
		#10
		assign A = 0;
		assign B = 'hFF00;
		#10
		assign A = 'hFF00;
		assign B = 'h00FF;
		#10
		assign A = 250;
		assign B = 1;
		#10
		assign A = 1;
		assign B = 250;
		#10
		assign A = 250;
		assign B = 250;
		#10
		assign A = 'h70F0;
		assign B = 'h5555;
		end

endmodule
