// Part 2 skeleton
`timescale 1ns / 1ns

module dummy_tennis
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
        KEY,
        SW,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		HEX5,
		HEX4,
		HEX3,
		HEX2,
		HEX1,
		HEX0,
		CLOCK2_50,
		FPGA_I2C_SCLK, 
		FPGA_I2C_SDAT, 
		AUD_XCK,
		AUD_DACLRCK, 
		AUD_ADCLRCK,
		AUD_BCLK, 
		AUD_ADCDAT, 
		AUD_DACDAT
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;

	// Declare your inputs and outputs here
	// Do not change the following outputs
	output		VGA_CLK;   				//	VGA Clock
	output		VGA_HS;					//	VGA H_SYNC
	output		VGA_VS;					//	VGA V_SYNC
	output		VGA_BLANK;				//	VGA BLANK
	output		VGA_SYNC;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	output	[6:0]HEX5;
	output	[6:0]HEX4;
	output	[6:0]HEX3;
	output	[6:0]HEX2;
	output	[6:0]HEX1;
	output	[6:0]HEX0;
	
	// sounds
	input CLOCK2_50;
	// I2C Audio/Video config interface
	output FPGA_I2C_SCLK;
	inout FPGA_I2C_SDAT;
	// Audio CODEC
	output AUD_XCK;
	input AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK;
	input AUD_ADCDAT;
	output AUD_DACDAT;
	wire noise;
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;	
	wire [6:0]p1_out;
	wire [6:0]p2_out;
	wire [6:0]p1_in;
	wire [6:0]p2_in;
	wire [14:0]ball_out;
	wire [14:0]ball_in;
	wire [1:0]direction_out;
	wire [1:0]direction_in;
	wire [4:0]speed_out;
	wire [4:0]speed_in;
	wire [2:0]select_op;
	wire [1:0]win;
	wire done, resetc, writeEn;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(KEY[0]),
			.clock(CLOCK_50),
			.colour(colour[2:0]),
			.x(x[7:0]),
			.y(y[6:0]),
			.plot(1),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK),
			.VGA_SYNC(VGA_SYNC),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
	assign HEX4[6:0] = 7'b1111111;
	
	ball_next_logic bl(
	.ball_in(ball_in[14:0]),
	.p1(p1_out[6:0]),
	.p2(p2_out[6:0]),
	.direction_in(direction_in[1:0]),
	.speed_in(speed_in[4:0]),
	.ball_out(ball_out[14:0]),
	.direction_out(direction_out[1:0]),
	.speed_out(speed_out[4:0]),
	.win(win[1:0]),
	.noise(noise)
	);
	
	ball_reg br(
	.writeEn(writeEn),
	.ball_in(ball_out[14:0]),
	.direction_in(direction_out[1:0]),
	.speed_in(speed_out[4:0]),
	.ball_out(ball_in[14:0]),
	.direction_out(direction_in[1:0]),
	.speed_out(speed_in[4:0]),
	.resetn(KEY[0]),
	.clk(CLOCK_50)
	);
	
	player_control p1(
	.SW(SW[9]),
	.position(p1_in[6:0]),
	.p_out(p1_out[6:0])
	);
	
	player_control p2(
	.SW(SW[0]),
	.position(p2_in[6:0]),
	.p_out(p2_out[6:0])
	);
	
	player_reg p1r(
	.pos_in(p1_out[6:0]),
	.pos_out(p1_in[6:0]),
	.writeEn(writeEn),
	.resetn(KEY[0]),
	.clk(CLOCK_50)
	);
	
	player_reg p2r(
	.pos_in(p2_out[6:0]),
	.pos_out(p2_in[6:0]),
	.writeEn(writeEn),
	.resetn(KEY[0]),
	.clk(CLOCK_50)
	);
	
	display_control dc(
	.colour(colour),
	.select_op(select_op[2:0]),
	.resetc(resetc),
	.done(done),
	.clean_enable(writeEn),
	.resetn(KEY[0]),
	.clk(CLOCK_50)
	);
	
	display_module dm(
	.xout(x),
	.yout(y),
	.done(done),
	.p1_y(p1_out[6:0]),
	.p2_y(p2_out[6:0]),
	.ball_p(ball_out[14:0]),
	.select_op(select_op[2:0]),
	.resetc(resetc),
	.clk(CLOCK_50)
	);
	
	frame fr(
	.enable(writeEn), 
	.resetn(KEY[0]), 
	.clk(CLOCK_50),
	.pause(SW[5])
	);
	
	score_display sd(
	.HEX5(HEX5[6:0]), 
	.HEX3(HEX3[6:0]), 
	.HEX2(HEX2[6:0]), 
	.HEX1(HEX1[6:0]), 
	.HEX0(HEX0[6:0]), 
	.score_in(win[1:0]), 
	.resetn(KEY[0]), 
	.clk(CLOCK_50),
	.writeEn(writeEn)
	);
	
	sound_mod sm(
	.CLOCK_50(CLOCK_50), 
	.CLOCK2_50(CLOCK2_50), 
	.KEY({noise, KEY[0]}), 
	.FPGA_I2C_SCLK(FPGA_I2C_SCLK), 
	.FPGA_I2C_SDAT(FPGA_I2C_SDAT), 
	.AUD_XCK(AUD_XCK), 
	.AUD_DACLRCK(AUD_DACLRCK), 
	.AUD_ADCLRCK(AUD_ADCLRCK), 
	.AUD_BCLK(AUD_BCLK), 
	.AUD_ADCDAT(AUD_ADCDAT), 
	.AUD_DACDAT(AUD_DACDAT)
	);
endmodule


module ball_next_logic(
	ball_in,
	p1,
	p2,
	direction_in,
	speed_in,
	ball_out,
	direction_out,
	speed_out,
	win,
	noise
	);

	input [14:0]ball_in;
	input [6:0]p1;
	input [6:0]p2;
	input [1:0]direction_in;
	input [4:0]speed_in;
	output reg [1:0]direction_out;
	output reg [4:0]speed_out;
	output reg [14:0]ball_out;
	reg [6:0] racket;
	output reg [1:0]win;
	output reg noise;
	
	 
	always@(*) begin
	racket = 7'd15 + p1[6:0] - ball_in[6:0];
	win[1:0] = 2'b00;
	noise = 1'b0;
	if (~direction_in[0]) begin
	    if ((ball_in[6:0] + speed_in[4:3]) >= 7'd118) begin
		ball_out[6:0] = 7'd118;
		direction_out[0] = 1;
		speed_out[4:0] = speed_in[4:0];
		noise = 1'b1;
	    end
	    else begin 
		ball_out[6:0] = ball_in[6:0] + speed_in[4:3];
		if (speed_in < 5'b11111)
		    speed_out[4:0] = speed_in[4:0] + 1;
		else
		    speed_out[4:0] = speed_in[4:0];
		direction_out[0] = direction_in[0];
	    end
	end
	else begin
	    if (speed_in[4:0] == 5'd0) begin
		ball_out[6:0] = ball_in[6:0];
		direction_out[0] = 0;
		speed_out[4:0] = 5'd1;
	    end
	    else begin
		ball_out[6:0] = ball_in[6:0] - speed_in[4:3];
		direction_out[0] = direction_in[0];
		speed_out[4:0] = speed_in[4:0] - 1;
	    end
	end
	
	if (ball_in[14:7] == 8'd2 ) begin
		noise = 1'b1;
	   if (ball_in[6:0] >= p1[6:0] && ball_in[6:0] <= 8'd15 + p1[6:0]) begin
			ball_out[14:7] = 8'd3;
			direction_out[1] = 1;
			if (direction_out[0]) begin
				if (speed_in[4:0] + racket[4:0] < 5'b11111)
					speed_out[4:0] = racket[4:0] + speed_in[4:0];
				else
					speed_out[4:0] = 5'b11111;
			end else begin
				if (speed_in[4:0] > racket[4:0]) begin
					speed_out[4:0] = speed_in[4:0] - racket[4:0];
				end
				else begin
					speed_out[4:0] = racket[4:0] - speed_in[4:0];
					direction_out[0] = 1;
				end
			end
		end else begin
			ball_out[14:7] = 8'd1;
			direction_out[1] = direction_in[1];
		end
	end else if (ball_in[14:7] == 8'd158 ) begin
		noise = 1'b1;
		if (ball_in[6:0] >= p2[6:0] && ball_in[6:0] <= 8'd15 + p2[6:0]) begin
			ball_out[14:7] = 8'd157;
			direction_out[1] = 0;
		end else begin
		ball_out[14:7] = 8'd159;
		direction_out[1] = direction_in[1];
	end
	end else if(ball_in[14:7] == 8'd159) begin
		noise = 1'b1;
	   ball_out[6:0] = 7'd50;
		ball_out[14:7] = 8'd80;
		direction_out[1:0] =  {1'b0, ball_in[3]};
		win[1:0] = 2'b10;
		speed_out[4:0] = ball_in[4:0];
	end else if(ball_in[14:7] == 8'd1) begin
		noise = 1'b1;
	   ball_out[6:0] = 7'd50;
		ball_out[14:7] = 8'd80;
		direction_out[1:0] = {1'b1, ball_in[3]};
		win[1:0] = 2'b11;
		speed_out[4:0] = ball_in[4:0];
	end else if(ball_in[14:7] == 8'd80 && ball_in[6:0] >= 7'd104) begin
		noise = 1'b1;
		if (direction_out[1]) begin
			ball_out[6:0] = 7'd50;
			ball_out[14:7] = 8'd80;
			direction_out[1:0] = {1'b1, ball_in[3]};
			win[1:0] = 2'b11;
			speed_out[4:0] = ball_in[4:0];
		end else begin
			ball_out[6:0] = 7'd50;
			ball_out[14:7] = 8'd80;
			direction_out[1:0] =  {1'b0, ball_in[3]};
			win[1:0] = 2'b10;
			speed_out[4:0] = ball_in[4:0];
		end
	end else begin 
	   if (direction_in[1]) begin
		ball_out[14:7] = ball_in[14:7] + 1;
		direction_out[1] = direction_in[1]; 
		end else begin
		ball_out[14:7] = ball_in[14:7] - 1;
		direction_out[1] = direction_in[1];
		end
	end
	end
endmodule


module ball_reg(
	writeEn,
	ball_in,
	direction_in,
	speed_in,
	ball_out,
	direction_out,
	speed_out,
	clk,
	resetn
	);

	output reg [14:0]ball_out;
	output reg [1:0]direction_out;
	output reg [4:0]speed_out;
	input [14:0]ball_in;
	input [1:0]direction_in;
	input [4:0]speed_in;
	input clk;
	input writeEn;
	input resetn;
	
	always@(posedge clk) begin 
		if (~resetn) begin
			 ball_out <= {8'd80,7'd60};
			 direction_out <= 2'b11;
			 speed_out <= 5'b01000;
			 end
	  else if (writeEn) begin
		  ball_out <= ball_in;
		  direction_out <= direction_in;
		  speed_out <= speed_in;
	  end 
	end
	
endmodule


module player_control(
	SW,
	position,
	p_out
	);

	output reg [6:0]p_out;
	input SW;
	input [6:0]position;


    // Next state logic aka our state table
    always@(*) begin 
		if (SW) begin
			if (position == 7'd0)
				p_out <= position;
			else
				p_out <= position - 1;
		end 
		else begin
	   if (position == 7'd101)
			p_out <= position;
	   else
			p_out <= position + 1;
		end
    end
endmodule

module player_reg(
	pos_in,
	pos_out,
	writeEn,
	clk,
	resetn
	);

	output reg [6:0]pos_out;
	input writeEn;
	input clk;
	input [6:0]pos_in;
	input resetn;


    // Next state logic aka our state table
    always@(posedge clk) begin
	 	if (~resetn) 
			pos_out <= 7'd60;
		else if (writeEn) 
			pos_out <= pos_in;
    end
    
endmodule

module display_module(xout, yout, done, p1_y, p2_y, ball_p, select_op, resetc, clk);

output reg [7:0]xout;
output reg [6:0]yout;
output reg done;

input [6:0]p1_y;
input [6:0]p2_y;
input [14:0]ball_p;
input [2:0]select_op;
input resetc;
input clk;

reg [6:0]local_p1_y;
reg [6:0]local_p2_y;
reg [6:0]local_m_y;
reg [14:0]local_ball_p;

reg [1:0]counter;

// counter to clean the screen
always @(posedge clk) begin
	if (~resetc) begin
		xout <= 8'd0;
		yout <= 7'd0;
		done <= 1'b0;
		local_p1_y <= p1_y;
		local_p2_y <= p2_y;
		local_m_y <= 7'd104;
		local_ball_p <= ball_p;
		counter <= 0;
	end
	else if (select_op == 3'b000) begin
		if (yout < 7'd120)
		  	begin
			    if (xout < 8'd160)
			    	xout <= xout + 1'd1;
			    else begin
			     	xout <= 8'd0;
			     	yout <= yout + 1'd1;
			    end
			end
		else
			done <= 1'b1; // finish counting
	end
	else if (select_op == 3'b001) begin // draw p1 bar
		xout <= 8'd0;
		if (yout < p1_y + 7'd18)
		  	begin
		  		yout <= local_p1_y;
			    local_p1_y <= local_p1_y + 1'd1;
			end
		else
			done <= 1'b1; // finish counting
	end
	else if (select_op == 3'b010) begin // draw p2 bar
		xout <= 8'd158;
		if (yout < p2_y + 7'd18)
		  	begin
		  		yout <= local_p2_y;
			    local_p2_y <= local_p2_y + 1'd1;
			end
		else
			done <= 1'b1; // finish counting
	end
	else if (select_op == 3'b011) begin // draw ball
		if(counter < 2) begin
			xout <= local_ball_p[14:7];
			yout <= local_ball_p[6:0];
			counter <= counter + 1;
		end
		else 
			done <= 1'b1; // finish counting
	end
	else if (select_op == 3'b100) begin // draw middle bar
		xout <= 8'd81;
		if (yout < 7'd119)
		  	begin
		  		yout <= local_m_y;
			    local_m_y <= local_m_y + 1'd1;
			end
		else
			done <= 1'b1; // finish counting
	end
	else if (select_op == 3'b101) begin // draw bottom bar
		xout <= 8'd0;
		yout <= 7'd119;
		if (xout < 8'd158)
		  	begin
			     xout <= xout + 1'd1;
			end
		else
			done <= 1'b1; // finish counting
	end
end

endmodule

module display_control(
	colour,
	select_op,
	resetc,
	done,
	clean_enable,
	resetn,
	clk
	);
	
	output reg [2:0]select_op;
	output reg [2:0]colour;
	output reg resetc;

	input done; // signal to move to the next state
	input clean_enable;
	input resetn;
	input clk;

	reg [4:0] current_state, next_state;   // 2 states require 1 bits

	
    localparam [4:0]  S_PREPARE_CLEAN           = 5'd0,
   			S_CLEAN                   = 5'd1,
    			S_PREPARE_DRAW_P1_BAR     = 5'd2,
   			S_DRAW_P1_BAR             = 5'd3,
    			S_PREPARE_DRAW_P2_BAR     = 5'd4,
    			S_DRAW_P2_BAR             = 5'd5,
				S_PREPARE_DRAW_BALL       = 5'd6,
    			S_DRAW_BALL               = 5'd7,
    			S_PREPARE_DRAW_MIDDLE_BAR     = 5'd8,
    			S_DRAW_MIDDLE_BAR             = 5'd9,
    			S_PREPARE_DRAW_BOTTOM_BAR     = 5'd10,
    			S_DRAW_BOTTOM_BAR             = 5'd11;

    // Next state logic aka our state table
    always@(*) 
	 begin: state_table
        case (current_state)
        		S_PREPARE_CLEAN:next_state = S_CLEAN;
        		S_CLEAN: next_state = done ? S_PREPARE_DRAW_P1_BAR : S_CLEAN;
        		S_PREPARE_DRAW_P1_BAR: next_state = S_DRAW_P1_BAR;
        		S_DRAW_P1_BAR: next_state = done ? S_PREPARE_DRAW_P2_BAR : S_DRAW_P1_BAR;
        		S_PREPARE_DRAW_P2_BAR: next_state = S_DRAW_P2_BAR;
        		S_DRAW_P2_BAR: next_state = done ? S_PREPARE_DRAW_MIDDLE_BAR : S_DRAW_P2_BAR;
				S_PREPARE_DRAW_MIDDLE_BAR: next_state = S_DRAW_MIDDLE_BAR;
        		S_DRAW_MIDDLE_BAR: next_state = done ? S_PREPARE_DRAW_BOTTOM_BAR : S_DRAW_MIDDLE_BAR;
				S_PREPARE_DRAW_BOTTOM_BAR: next_state = S_DRAW_BOTTOM_BAR;
        		S_DRAW_BOTTOM_BAR: next_state = done ? S_PREPARE_DRAW_BALL : S_DRAW_BOTTOM_BAR;
				S_PREPARE_DRAW_BALL: next_state = S_DRAW_BALL;
				S_DRAW_BALL: next_state = clean_enable ? S_PREPARE_CLEAN : S_DRAW_BALL;
            default:     next_state = S_PREPARE_CLEAN;
        endcase
	end
	
	always@(*) 
	begin: enable_signals

		case (current_state)
			S_PREPARE_CLEAN:begin
				resetc = 1'b0;
			end
			S_CLEAN:begin
				resetc = 1'b1;
				select_op = 3'd0; // select clean counter
				colour = 3'b000; // draw black
			end
			S_PREPARE_DRAW_P1_BAR:begin
				resetc = 1'b0;
			end
			S_DRAW_P1_BAR:begin
				resetc = 1'b1;
				select_op = 3'd1; // select P1 bar counter
				colour = 3'b100; // draw white
			end
			S_PREPARE_DRAW_P2_BAR:begin
				resetc = 1'b0;
			end
			S_DRAW_P2_BAR:begin
				resetc = 1'b1;
				select_op = 3'd2; // select p2 bar counter
				colour = 3'b001; // draw white
			end
			S_PREPARE_DRAW_BALL:begin
				resetc = 1'b0;
			end
			S_DRAW_BALL:begin
				resetc = 1'b1;
				select_op = 3'd3; // select ball
				colour = 3'b110; // draw white
			end
			S_PREPARE_DRAW_MIDDLE_BAR:begin
				resetc = 1'b0;
			end
			S_DRAW_MIDDLE_BAR:begin
				resetc = 1'b1;
				select_op = 3'd4; // select middle bar counter
				colour = 3'b101; // draw white
			end
			S_PREPARE_DRAW_BOTTOM_BAR:begin
				resetc = 1'b0;
			end
			S_DRAW_BOTTOM_BAR:begin
				resetc = 1'b1;
				select_op = 3'd5; // select bottom bar counter
				colour = 3'b111; // draw white
			end
		endcase
	end

	// FSM state register/FF
	always @(posedge clk, negedge resetn) begin
		if (~resetn) 
			current_state <= S_PREPARE_CLEAN;
		else
			current_state <= next_state;
	end

endmodule

module frame(enable, resetn, clk, pause);

	output enable;

	input resetn;
	input clk;
	input pause;

	reg [19:0]rate_counter;  //1100101101110100

	always@(posedge clk) begin
		if (~resetn)
			rate_counter <= 20'b11001011011101000000;
		else if (rate_counter == 0)
			rate_counter <= 20'b11001011011101000000;
		else
			if (~pause)
				rate_counter <= rate_counter - 1'b1;
	end

	assign enable = (rate_counter == 0) ? 1:0;

endmodule

module score_display(HEX5, HEX3, HEX2, HEX1, HEX0, score_in, resetn, clk, writeEn);

// display winner
output [6:0]HEX5;

// p2 score
output [6:0]HEX3;
output [6:0]HEX2;

// p1 score
output [6:0]HEX1;
output [6:0]HEX0;

input [1:0]score_in;
input resetn;
input clk;
input writeEn;

wire [1:0]winner;
wire [3:0]p1_score;
wire [3:0]p2_score;

	score_counter sc(.winner(winner), .p1_score(p1_score[3:0]), .p2_score(p2_score[3:0]), .score_in(score_in[1:0]), .resetn(resetn), .clk(clk), .writeEn(writeEn));

	decoder_lower h5(.ss_out(HEX5[6:0]), .bcd_in({2'b00,winner})); // winner

	//scores
	decoder_upper h3(.ss_out(HEX3[6:0]), .bcd_in(p1_score));
	decoder_lower h2(.ss_out(HEX2[6:0]), .bcd_in(p1_score));
	
	decoder_upper h1(.ss_out(HEX1[6:0]), .bcd_in(p2_score));
	decoder_lower h0(.ss_out(HEX0[6:0]), .bcd_in(p2_score));

endmodule



module score_counter(winner, p1_score, p2_score, score_in, resetn, clk, writeEn);


output reg [1:0]winner; // p2 wins 10 , p1 wins 01
output reg [3:0]p1_score;
output reg [3:0]p2_score;

input [1:0]score_in;
input resetn;
input clk;
input writeEn;

always @(posedge clk) begin
		if (~resetn) begin
			p1_score[3:0] <= 4'd0;
			p2_score[3:0] <= 4'd0;
			winner[1:0] <= 2'b00;
		end
		else if (writeEn) begin
			if (p1_score == 4'd11) begin
				winner[1:0] <= 2'b01;
				p1_score <= 0;
				p2_score <= 0;
			end
			else if (p2_score == 4'd11) begin
				winner[1:0] <= 2'b10;
				p1_score <= 0;
				p2_score <= 0;
			end
			if (score_in == 2'b10)
				p1_score[3:0] <= p1_score[3:0] + 4'd1;
			else if (score_in == 2'b11)
				p2_score[3:0] <= p2_score[3:0] + 4'd1;
		end
	end

	endmodule

module decoder_lower(ss_out, bcd_in);
	 output reg [6:0] ss_out;
	 input [3:0] bcd_in;
	 always @(bcd_in)
	 case (bcd_in)
	 4'd0: ss_out = 7'b1000000;
	 4'd1: ss_out = 7'b1111001;
	 4'd2: ss_out = 7'b0100100;
	 4'd3: ss_out = 7'b0110000;
	 4'd4: ss_out = 7'b0011001;
	 4'd5: ss_out = 7'b0010010;
	 4'd6: ss_out = 7'b0000010;
	 4'd7: ss_out = 7'b1111000;
	 4'd8: ss_out = 7'b0000000;
	 4'd9: ss_out = 7'b0011000;
	 4'd10: ss_out = 7'b1000000;
	 4'd11: ss_out = 7'b1111001;	 
	 default: ss_out = 7'b1000000;
	 endcase
endmodule

module decoder_upper(ss_out, bcd_in);
	 output reg [6:0] ss_out;
	 input [3:0] bcd_in;
	 always @(bcd_in)
	 case (bcd_in)
	 4'd0: ss_out = 7'b1000000;
	 4'd1: ss_out = 7'b1000000;
	 4'd2: ss_out = 7'b1000000;
	 4'd3: ss_out = 7'b1000000;
	 4'd4: ss_out = 7'b1000000;
	 4'd5: ss_out = 7'b1000000;
	 4'd6: ss_out = 7'b1000000;
	 4'd7: ss_out = 7'b1000000;
	 4'd8: ss_out = 7'b1000000;
	 4'd9: ss_out = 7'b1000000;
	 4'd10: ss_out = 7'b1111001;
	 4'd11: ss_out = 7'b1111001;	 
	 default: ss_out = 7'b1000000;
	 endcase
endmodule
