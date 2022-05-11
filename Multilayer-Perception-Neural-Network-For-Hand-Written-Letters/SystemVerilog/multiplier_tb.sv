module multiplier_tb;
	logic signed [15:0] A;
	logic signed [15:0] B;
	logic signed [15:0] Product;
	logic signed [31:0] shift;
	logic signed [15:0] Actual;

	multiplier #(.bits(16), .fractional_bits(11)) multiplier(.A(A), .B(B), .Product(Product));

	initial 
		begin
		
		assign A = 65279; //254
		assign B = 128;	  //.2
		assign shift = A*B;
		assign Actual = shift>>>11;
		#10		  //answer should be 32640 in decimal
		assign A = 128;	  //.2
		assign B = 65280; //255
		assign shift = A*B;
		assign Actual = shift>>>11;
		#10		  //answer should be 32640 in decimal
		assign A = 5000; 
		assign B = -4;	
		assign shift = A*B;
		assign Actual = shift>>>11;  
		#10		  
		assign A = -4;	  
		assign B = 5000; 
		assign shift = A*B;
		assign Actual = shift>>>11;
		#10		  
		assign A = 16;
		assign B = 0;
		assign shift = A*B;
		assign Actual = shift>>>11;
		#10
		assign A = 0;
		assign B = 16;
		assign shift = A*B;
		assign Actual = shift>>>11;
		#10
		assign A = 65280;
		assign B = -256;
		assign shift = A*B;
		assign Actual = shift>>>11;
		#10
		assign A = 250;
		assign B = 250;
		assign shift = A*B;
		assign Actual = shift>>>11;
	end

endmodule
