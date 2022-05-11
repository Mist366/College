module fullAdder_tb;
	logic A;
	logic B;
	logic Cin;
	logic Cout;
	logic Sum;

	fullAdder fullAdder(.A(A), .B(B), .Cin(Cin), .Cout(Cout), .Sum(Sum));

	initial 
		begin
		assign A = 0;
		assign B = 0;
		assign Cin = 0;
		#10
		assign A = 1;
		assign B = 0;
		assign Cin = 0;
		#10
		assign A = 0;
		assign B = 1;
		assign Cin = 0;
		#10
		assign A = 1;
		assign B = 1;
		assign Cin = 0;
		#10
		assign A = 0;
		assign B = 0;
		assign Cin = 1;
		#10
		assign A = 1;
		assign B = 0;
		assign Cin = 1;
		#10
		assign A = 0;
		assign B = 1;
		assign Cin = 1;
		#10
		assign A = 1;
		assign B = 1;
		assign Cin = 1;
		end

endmodule
