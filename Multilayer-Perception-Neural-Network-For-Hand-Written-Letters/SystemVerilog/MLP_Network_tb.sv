module MLP_Network_tb;
	reg signed [15:0] picture1 [0:783];
	reg signed [15:0] picture2 [0:783];
	reg signed [15:0] picture3 [0:783];
	reg signed [15:0] picture4 [0:783];
	reg signed [15:0] picture5 [0:783];
	reg signed [15:0] picture6 [0:783];
	reg signed [15:0] picture7 [0:783];
	reg signed [15:0] picture8 [0:783];
	reg signed [15:0] picture9 [0:783];
	reg signed [15:0] picture10 [0:783];

	reg signed [15:0] out [0:9];
	reg signed [15:0] picture [0:783];
	reg clock;
	reg start;
	reg ready;
	
	MLP_Network #(.bits(16), .fractional_bits(11)) Network(
		.start(start), 
		.clock(clock), 
		.picture(picture), 
		.results(out), 
		.ready(ready)
		);

	initial begin
		clock = 0;
		start = 0;
		ready = 0;
		$readmemh("picture_1_16.mem" , picture1);
		$readmemh("picture_2_16.mem" , picture2);
		$readmemh("picture_3_16.mem" , picture3);
		$readmemh("picture_4_16.mem" , picture4);
		$readmemh("picture_5_16.mem" , picture5);
		$readmemh("picture_6_16.mem" , picture6);
		$readmemh("picture_7_16.mem" , picture7);
		$readmemh("picture_8_16.mem" , picture8);
		$readmemh("picture_9_16.mem" , picture9);
		$readmemh("picture_10_16.mem" , picture10);
		
		assign picture = picture1;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end
		
		assign picture = picture2;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end
		
		assign picture = picture3;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end
		
		assign picture = picture4;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end
		
		assign picture = picture5;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end
		
		assign picture = picture6;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end
		
		assign picture = picture7;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end
		
		assign picture = picture8;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end

		assign picture = picture9;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end
		
		assign picture = picture10;
		#5
		start = 1;
		#5
		start = 0;
		#1 clock = ~clock;
		#1 clock = ~clock;
		while(~ready) begin
			#1 clock = ~clock;
		end
	end
endmodule
