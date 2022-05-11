module fullAdder(input A, input B, input Cin, output Cout, output Sum);
	assign Sum = Cin^(A^B);
	assign Cout = (A&B)|((A^B)&Cin);
endmodule
