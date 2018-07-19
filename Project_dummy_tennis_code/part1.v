





module sound_mod (CLOCK_50, CLOCK2_50, KEY, FPGA_I2C_SCLK, FPGA_I2C_SDAT, AUD_XCK, 
		        AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK, AUD_ADCDAT, AUD_DACDAT);

	input CLOCK_50, CLOCK2_50;
	input [1:0] KEY;
	// I2C Audio/Video config interface
	output FPGA_I2C_SCLK;
	inout FPGA_I2C_SDAT;
	// Audio CODEC
	output AUD_XCK;
	input AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK;
	input AUD_ADCDAT;
	output AUD_DACDAT;
	
	// Local wires.
	wire read_ready, write_ready, read, write;
	wire [23:0] readdata_left, readdata_right;
	wire [23:0] writedata_left, writedata_right;
	wire reset = ~KEY[0];

	/////////////////////////////////
	// Your code goes here 
	
		sound_control sc(
		.write_ready(write_ready), 
			.play(KEY[1]), 
		.reset(reset),
		.clk(CLOCK_50), 
		.writedata_left(writedata_left), 
		.writedata_right(writedata_right), 
		.write(write));
	/////////////////////////////////
	
	//assign writedata_left = ... not shown
	//assign writedata_right = ... not shown
	//assign read = ... not shown
	//assign write = ... not shown
	
/////////////////////////////////////////////////////////////////////////////////
// Audio CODEC interface. 
//
// The interface consists of the following wires:
// read_ready, write_ready - CODEC ready for read/write operation 
// readdata_left, readdata_right - left and right channel data from the CODEC
// read - send data from the CODEC (both channels)
// writedata_left, writedata_right - left and right channel data to the CODEC
// write - send data to the CODEC (both channels)
// AUD_* - should connect to top-level entity I/O of the same name.
//         These signals go directly to the Audio CODEC
// I2C_* - should connect to top-level entity I/O of the same name.
//         These signals go directly to the Audio/Video Config module
/////////////////////////////////////////////////////////////////////////////////
	clock_generator my_clock_gen(
		// inputs
		CLOCK2_50,
		reset,

		// outputs
		AUD_XCK
	);

	audio_and_video_config cfg(
		// Inputs
		CLOCK_50,
		reset,

		// Bidirectionals
		FPGA_I2C_SDAT,
		FPGA_I2C_SCLK
	);

	audio_codec codec(
		// Inputs
		CLOCK_50,
		reset,

		read,	write,
		writedata_left, writedata_right,

		AUD_ADCDAT,

		// Bidirectionals
		AUD_BCLK,
		AUD_ADCLRCK,
		AUD_DACLRCK,

		// Outputs
		read_ready, write_ready,
		readdata_left, readdata_right,
		AUD_DACDAT
	);
	


endmodule

module sound_control(write_ready, play, reset, clk, writedata_left, writedata_right, write);

input write_ready, play;
input reset;
input clk;

output reg [23:0]writedata_left;
output reg [23:0]writedata_right;
output write;

reg [8:0]counter;

always@(posedge clk) begin
	if(reset) begin
		counter <= 9'd0;
	end
	else if(play && write_ready) begin
		counter <= counter + 1'd1;
	end
	else if(counter == 9'd511) begin
		counter <= 0;
	
	end
end

reg current_state, next_state;

localparam write_256 = 1'b1,
				write_n256 = 1'b0;


always@(*) begin
	case(current_state)
		write_256: next_state = (counter == 9'd511)? write_n256 : write_256;
		write_n256: next_state = (counter == 9'd511)? write_256 : write_n256;
		default: next_state = write_256;
	endcase
end

always@(posedge clk, posedge reset) begin
	if(reset)
		current_state <= write_256;
	else
		current_state <= next_state;
end


always@(*) begin
	
	if (play) begin
		if (current_state == write_256) begin
			writedata_left = 24'b11111111111111111;
			writedata_right = 24'b11111111111111111;
		end
		else if (current_state == write_n256) begin
			writedata_left = -24'b11111111111111111;
			writedata_right = -24'b11111111111111111;
		end
	end
	else begin
		writedata_left = 0;
		writedata_right = 0;
	end
end


assign write = write_ready;

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module can create clock signals that have a frequency lower           *
 *  than those a PLL can generate.                                            *
 *                                                                            *
 * Revision: 1.1                                                              *
 *                                                                            *
 * Used in IP Cores:                                                          *
 *   Altera UP Avalon Audio and Video Config                                  *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_Slow_Clock_Generator (
	// Inputs
	clk,
	reset,
	
	enable_clk,

	// Bidirectionals

	// Outputs
	new_clk,

	rising_edge,
	falling_edge,

	middle_of_high_level,
	middle_of_low_level
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter COUNTER_BITS	= 10;
parameter COUNTER_INC	= 10'h001;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input				clk;
input				reset;

input				enable_clk;
	
// Bidirectionals

// Outputs
output	reg			new_clk;

output	reg			rising_edge;
output	reg			falling_edge;

output	reg			middle_of_high_level;
output	reg			middle_of_low_level;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires

// Internal Registers
reg			[COUNTER_BITS:1]	clk_counter;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		clk_counter	<= {COUNTER_BITS{1'b0}};
	else if (enable_clk == 1'b1)
		clk_counter	<= clk_counter + COUNTER_INC;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		new_clk	<= 1'b0;
	else
		new_clk	<= clk_counter[COUNTER_BITS];
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		rising_edge	<= 1'b0;
	else
		rising_edge	<= (clk_counter[COUNTER_BITS] ^ new_clk) & ~new_clk;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		falling_edge <= 1'b0;
	else
		falling_edge <= (clk_counter[COUNTER_BITS] ^ new_clk) & new_clk;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		middle_of_high_level <= 1'b0;
	else
		middle_of_high_level <= 
			clk_counter[COUNTER_BITS] & 
			~clk_counter[(COUNTER_BITS - 1)] &
			(&(clk_counter[(COUNTER_BITS - 2):1]));
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		middle_of_low_level <= 1'b0;
	else
		middle_of_low_level <= 
			~clk_counter[COUNTER_BITS] & 
			~clk_counter[(COUNTER_BITS - 1)] &
			(&(clk_counter[(COUNTER_BITS - 2):1]));
end



/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

// Output Assignments

// Internal Assignments

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module is a FIFO with same clock for both reads and writes.           *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_SYNC_FIFO (
	// Inputs
	clk,
	reset,

	write_en,
	write_data,

	read_en,
	
	// Bidirectionals

	// Outputs
	fifo_is_empty,
	fifo_is_full,
	words_used,

	read_data
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter	DATA_WIDTH	= 32;
parameter	DATA_DEPTH	= 128;
parameter	ADDR_WIDTH	= 7;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input				clk;
input				reset;

input				write_en;
input		[DATA_WIDTH:1]	write_data;

input				read_en;

// Bidirectionals

// Outputs
output				fifo_is_empty;
output				fifo_is_full;
output		[ADDR_WIDTH:1]	words_used;

output		[DATA_WIDTH:1]	read_data;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires

// Internal Registers

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/


/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/


scfifo	Sync_FIFO (
	// Inputs
	.clock			(clk),
	.sclr			(reset),

	.data			(write_data),
	.wrreq			(write_en),

	.rdreq			(read_en),

	// Bidirectionals

	// Outputs
	.empty			(fifo_is_empty),
	.full			(fifo_is_full),
	.usedw			(words_used),
	
	.q				(read_data)

	// Unused
	// synopsys translate_off
	,
	.aclr			(),
	.almost_empty	(),
	.almost_full	()
	// synopsys translate_on
);
defparam
	Sync_FIFO.add_ram_output_register	= "OFF",
	Sync_FIFO.intended_device_family	= "Cyclone II",
	Sync_FIFO.lpm_numwords				= DATA_DEPTH,
	Sync_FIFO.lpm_showahead				= "ON",
	Sync_FIFO.lpm_type					= "scfifo",
	Sync_FIFO.lpm_width					= DATA_WIDTH,
	Sync_FIFO.lpm_widthu				= ADDR_WIDTH,
	Sync_FIFO.overflow_checking			= "OFF",
	Sync_FIFO.underflow_checking		= "OFF",
	Sync_FIFO.use_eab					= "ON";

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module sends and receives data from the audio's and TV in's           *
 *  control registers for the chips on Altera's DE1 board. Plus, it can       *
 *  send and receive data from the TRDB_DC2 and TRDB_LCM add-on modules.      *
 *                                                                            *
 ******************************************************************************/

module audio_and_video_config (
	// Inputs
	clk,
	reset,

	// Bidirectionals
	I2C_SDAT,
	I2C_SCLK
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter I2C_BUS_MODE			= 1'b0;
parameter CFG_TYPE				= 8'h01;

parameter MIN_ROM_ADDRESS		= 6'h00;
parameter MAX_ROM_ADDRESS		= 6'h32;

parameter AUD_LINE_IN_LC		= 9'h01A;
parameter AUD_LINE_IN_RC		= 9'h01A;
parameter AUD_LINE_OUT_LC		= 9'h07B;
parameter AUD_LINE_OUT_RC		= 9'h07B;
parameter AUD_ADC_PATH			= 9'd149;
parameter AUD_DAC_PATH			= 9'h006;
parameter AUD_POWER				= 9'h000;
parameter AUD_DATA_FORMAT		= 9'd73;
parameter AUD_SAMPLE_CTRL		= 9'd0;
parameter AUD_SET_ACTIVE		= 9'h001;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

// Bidirectionals
inout				I2C_SDAT;					//	I2C Data
output			I2C_SCLK;				 	//	I2C Clock
/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/


/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				clk_400KHz;
wire				start_and_stop_en;
wire				change_output_bit_en;

wire				enable_clk;

wire				send_start_bit;
wire				send_stop_bit;

wire		[7:0]	auto_init_data;
wire				auto_init_transfer_data;
wire				auto_init_start_bit;
wire				auto_init_stop_bit;
wire				auto_init_complete;
wire				auto_init_error;

wire				transfer_data;
wire				transfer_complete;

wire				i2c_ack;
wire		[7:0]	i2c_received_data;

// Internal Registers
reg			[7:0]	data_to_transfer;
reg			[2:0]	num_bits_to_transfer;

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset)
	begin
		data_to_transfer		<= 8'h00;
		num_bits_to_transfer	<= 3'h0;
	end
	else
		if (auto_init_complete == 1'b0)
		begin
			data_to_transfer		<= auto_init_data;
			num_bits_to_transfer	<= 3'h7;
		end
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign transfer_data = auto_init_transfer_data;

assign send_start_bit = auto_init_start_bit;

assign send_stop_bit = auto_init_stop_bit;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Altera_UP_Slow_Clock_Generator Clock_Generator_400KHz (
	// Inputs
	.clk					(clk),
	.reset					(reset),

	.enable_clk				(enable_clk),
	
	// Bidirectionals

	// Outputs
	.new_clk				(clk_400KHz),

	.rising_edge			(),
	.falling_edge			(),

	.middle_of_high_level		(start_and_stop_en),
	.middle_of_low_level		(change_output_bit_en)
);
defparam
	Clock_Generator_400KHz.COUNTER_BITS	= 10, // 4, // 
	Clock_Generator_400KHz.COUNTER_INC	= 10'h001; // 4'h1; // 

Altera_UP_I2C_AV_Auto_Initialize Auto_Initialize (
	// Inputs
	.clk				(clk),
	.reset				(reset),

	.clear_error		(1'b1),

	.ack				(i2c_ack),
	.transfer_complete	(transfer_complete),

	// Bidirectionals

	// Outputs
	.data_out			(auto_init_data),
	.transfer_data		(auto_init_transfer_data),
	.send_start_bit		(auto_init_start_bit),
	.send_stop_bit		(auto_init_stop_bit),

	.auto_init_complete	(auto_init_complete),
	.auto_init_error	(auto_init_error)
);
defparam
	Auto_Initialize.MIN_ROM_ADDRESS	= MIN_ROM_ADDRESS,
	Auto_Initialize.MAX_ROM_ADDRESS	= MAX_ROM_ADDRESS,

	Auto_Initialize.AUD_LINE_IN_LC	= AUD_LINE_IN_LC,
	Auto_Initialize.AUD_LINE_IN_RC	= AUD_LINE_IN_RC,
	Auto_Initialize.AUD_LINE_OUT_LC	= AUD_LINE_OUT_LC,
	Auto_Initialize.AUD_LINE_OUT_RC	= AUD_LINE_OUT_RC,
	Auto_Initialize.AUD_ADC_PATH	= AUD_ADC_PATH,
	Auto_Initialize.AUD_DAC_PATH	= AUD_DAC_PATH,
	Auto_Initialize.AUD_POWER		= AUD_POWER,
	Auto_Initialize.AUD_DATA_FORMAT	= AUD_DATA_FORMAT,
	Auto_Initialize.AUD_SAMPLE_CTRL	= AUD_SAMPLE_CTRL,
	Auto_Initialize.AUD_SET_ACTIVE	= AUD_SET_ACTIVE;

Altera_UP_I2C I2C_Controller (
	// Inputs
	.clk					(clk),
	.reset					(reset),

	.clear_ack				(1'b1),

	.clk_400KHz				(clk_400KHz),
	.start_and_stop_en		(start_and_stop_en),
	.change_output_bit_en	(change_output_bit_en),

	.send_start_bit			(send_start_bit),
	.send_stop_bit			(send_stop_bit),

	.data_in				(data_to_transfer),
	.transfer_data			(transfer_data),
	.read_byte				(1'b0),
	.num_bits_to_transfer	(num_bits_to_transfer),

	// Bidirectionals
	.i2c_sdata				(I2C_SDAT),

	// Outputs
	.i2c_sclk				(I2C_SCLK),
	.i2c_scen				(),

	.enable_clk				(enable_clk),

	.ack					(i2c_ack),
	.data_from_i2c			(i2c_received_data),
	.transfer_complete		(transfer_complete)
);
defparam
	I2C_Controller.I2C_BUS_MODE	= I2C_BUS_MODE;

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module reads and writes data to the Audio chip on Altera's DE1        *
 *  Development and Education Board. The audio chip must be in master mode    *
 *  and the digital format must be left justified.                            *
 *                                                                            *
 ******************************************************************************/

module audio_codec (
// Inputs
clk,
reset,

read,	write,
writedata_left, writedata_right,

AUD_ADCDAT,

// Bidirectionals
AUD_BCLK,
AUD_ADCLRCK,
AUD_DACLRCK,

// Outputs
read_ready, write_ready,
readdata_left, readdata_right,
AUD_DACDAT
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter AUDIO_DATA_WIDTH	= 24;
parameter BIT_COUNTER_INIT	= 5'd23;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input					read;
input					write;
input	[AUDIO_DATA_WIDTH-1:0]	writedata_left;
input	[AUDIO_DATA_WIDTH-1:0]	writedata_right;

input				AUD_ADCDAT;
input				AUD_BCLK;
input				AUD_ADCLRCK;
input				AUD_DACLRCK;

// Outputs
output			read_ready, write_ready;
output	[AUDIO_DATA_WIDTH-1:0]	readdata_left;
output	[AUDIO_DATA_WIDTH-1:0]	readdata_right;


output			AUD_DACDAT;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire				bclk_rising_edge;
wire				bclk_falling_edge;

wire				adc_lrclk_rising_edge;
wire				adc_lrclk_falling_edge;

wire		[AUDIO_DATA_WIDTH:1] new_left_channel_audio;
wire		[AUDIO_DATA_WIDTH:1] new_right_channel_audio;

wire		[7:0]	left_channel_read_available;
wire		[7:0]	right_channel_read_available;
wire				dac_lrclk_rising_edge;
wire				dac_lrclk_falling_edge;

wire		[7:0]	left_channel_write_space;
wire		[7:0]	right_channel_write_space;

// Internal Registers
reg					done_adc_channel_sync;
reg					done_dac_channel_sync;

// State Machine Registers


/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @ (posedge clk)
begin
	if (reset == 1'b1)
		done_adc_channel_sync <= 1'b0;
	else if (adc_lrclk_rising_edge == 1'b1)
		done_adc_channel_sync <= 1'b1;
end

always @ (posedge clk)
begin
	if (reset == 1'b1)
		done_dac_channel_sync <= 1'b0;
	else if (dac_lrclk_falling_edge == 1'b1)
		done_dac_channel_sync <= 1'b1;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign read_ready = (left_channel_read_available != 8'd0) & (right_channel_read_available != 8'd0);
assign write_ready = (left_channel_write_space != 8'd0) & (right_channel_write_space != 8'd0);
assign readdata_left = new_left_channel_audio;
assign readdata_right = new_right_channel_audio;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Altera_UP_Clock_Edge Bit_Clock_Edges (
	// Inputs
	.clk			(clk),
	.reset		(reset),
	
	.test_clk		(AUD_BCLK),
	
	// Bidirectionals

	// Outputs
	.rising_edge	(bclk_rising_edge),
	.falling_edge	(bclk_falling_edge)
);

Altera_UP_Clock_Edge ADC_Left_Right_Clock_Edges (
	// Inputs
	.clk			(clk),
	.reset		(reset),
	
	.test_clk		(AUD_ADCLRCK),
	
	// Bidirectionals

	// Outputs
	.rising_edge	(adc_lrclk_rising_edge),
	.falling_edge	(adc_lrclk_falling_edge)
);

Altera_UP_Clock_Edge DAC_Left_Right_Clock_Edges (
	// Inputs
	.clk			(clk),
	.reset		(reset),
	
	.test_clk		(AUD_DACLRCK),
	
	// Bidirectionals

	// Outputs
	.rising_edge	(dac_lrclk_rising_edge),
	.falling_edge	(dac_lrclk_falling_edge)
);



Altera_UP_Audio_In_Deserializer Audio_In_Deserializer (
	// Inputs
	.clk						(clk),
	.reset					(reset),
	
	.bit_clk_rising_edge			(bclk_rising_edge),
	.bit_clk_falling_edge			(bclk_falling_edge),
	.left_right_clk_rising_edge		(adc_lrclk_rising_edge),
	.left_right_clk_falling_edge		(adc_lrclk_falling_edge),

	.done_channel_sync			(done_adc_channel_sync),

	.serial_audio_in_data			(AUD_ADCDAT),

	.read_left_audio_data_en		(read & (left_channel_read_available != 8'd0)),
	.read_right_audio_data_en		(read & (right_channel_read_available != 8'd0)),

	// Bidirectionals

	// Outputs
	.left_audio_fifo_read_space		(left_channel_read_available),
	.right_audio_fifo_read_space		(right_channel_read_available),

	.left_channel_data			(new_left_channel_audio),
	.right_channel_data			(new_right_channel_audio)
);
defparam
	Audio_In_Deserializer.AUDIO_DATA_WIDTH = AUDIO_DATA_WIDTH,
	Audio_In_Deserializer.BIT_COUNTER_INIT = BIT_COUNTER_INIT;


Altera_UP_Audio_Out_Serializer Audio_Out_Serializer (
	// Inputs
	.clk						(clk),
	.reset						(reset),
	
	.bit_clk_rising_edge			(bclk_rising_edge),
	.bit_clk_falling_edge			(bclk_falling_edge),
	.left_right_clk_rising_edge		(done_dac_channel_sync & dac_lrclk_rising_edge),
	.left_right_clk_falling_edge		(done_dac_channel_sync & dac_lrclk_falling_edge),
	
	.left_channel_data			(writedata_left),
	.left_channel_data_en			(write & (left_channel_write_space != 8'd0)),

	.right_channel_data			(writedata_right),
	.right_channel_data_en			(write & (right_channel_write_space != 8'd0)),
	
	// Bidirectionals

	// Outputs
	.left_channel_fifo_write_space	(left_channel_write_space),
	.right_channel_fifo_write_space	(right_channel_write_space),

	.serial_audio_out_data			(AUD_DACDAT)
);
defparam
	Audio_Out_Serializer.AUDIO_DATA_WIDTH = AUDIO_DATA_WIDTH;

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module sends and receives data to/from the DE1's audio and TV         *
 *  peripherals' control registers.                                           *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_I2C (
	// Inputs
	clk,
	reset,

	clear_ack,

	clk_400KHz,
	start_and_stop_en,
	change_output_bit_en,

	send_start_bit,
	send_stop_bit,

	data_in,
	transfer_data,
	read_byte,
	num_bits_to_transfer,

	// Bidirectionals
	i2c_sdata,

	// Outputs
	i2c_sclk,
	i2c_scen,

	enable_clk,

	ack,
	data_from_i2c,
	transfer_complete
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter I2C_BUS_MODE = 1'b0;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input				clear_ack;

input				clk_400KHz;
input				start_and_stop_en;
input				change_output_bit_en;

input				send_start_bit;
input				send_stop_bit;

input		[7:0]	data_in;
input				transfer_data;
input				read_byte;
input		[2:0]	num_bits_to_transfer;

// Bidirectionals
inout				i2c_sdata;					//	I2C Data

// Outputs
output				i2c_sclk;					//	I2C Clock
output	reg			i2c_scen;

output				enable_clk;

output	reg			ack;
output	reg	[7:0]	data_from_i2c;
output				transfer_complete;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/
// states
localparam	I2C_STATE_0_IDLE			= 3'h0,
			I2C_STATE_1_PRE_START		= 3'h1,
			I2C_STATE_2_START_BIT		= 3'h2,
			I2C_STATE_3_TRANSFER_BYTE	= 3'h3,
			I2C_STATE_4_TRANSFER_ACK	= 3'h4,
			I2C_STATE_5_STOP_BIT		= 3'h5,
			I2C_STATE_6_COMPLETE		= 3'h6;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires

// Internal Registers
reg			[2:0]	current_bit;
reg			[7:0]	current_byte;

// State Machine Registers
reg			[2:0]	ns_i2c_transceiver;
reg			[2:0]	s_i2c_transceiver;

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		s_i2c_transceiver <= I2C_STATE_0_IDLE;
	else
		s_i2c_transceiver <= ns_i2c_transceiver;
end

always @(*)
begin
	// Defaults
	ns_i2c_transceiver = I2C_STATE_0_IDLE;

    case (s_i2c_transceiver)
	I2C_STATE_0_IDLE:
		begin
			if ((send_start_bit == 1'b1) && (clk_400KHz == 1'b0))
				ns_i2c_transceiver = I2C_STATE_1_PRE_START;
			else if (send_start_bit == 1'b1)
				ns_i2c_transceiver = I2C_STATE_2_START_BIT;
			else if (send_stop_bit == 1'b1)
				ns_i2c_transceiver = I2C_STATE_5_STOP_BIT;
			else if (transfer_data == 1'b1)
				ns_i2c_transceiver = I2C_STATE_3_TRANSFER_BYTE;
			else
				ns_i2c_transceiver = I2C_STATE_0_IDLE;
		end
	I2C_STATE_1_PRE_START:
		begin
			if (start_and_stop_en == 1'b1)
				ns_i2c_transceiver = I2C_STATE_2_START_BIT;
			else
				ns_i2c_transceiver = I2C_STATE_1_PRE_START;
		end
	I2C_STATE_2_START_BIT:
		begin
			if (change_output_bit_en == 1'b1)
			begin
				if ((transfer_data == 1'b1) && (I2C_BUS_MODE == 1'b0))
					ns_i2c_transceiver = I2C_STATE_3_TRANSFER_BYTE;
				else
					ns_i2c_transceiver = I2C_STATE_6_COMPLETE;
			end
			else
				ns_i2c_transceiver = I2C_STATE_2_START_BIT;
		end
	I2C_STATE_3_TRANSFER_BYTE:
		begin
			if ((current_bit == 3'h0) && (change_output_bit_en == 1'b1))
			begin
				if ((I2C_BUS_MODE == 1'b0) || (num_bits_to_transfer == 3'h6))
					ns_i2c_transceiver = I2C_STATE_4_TRANSFER_ACK;
				else
					ns_i2c_transceiver = I2C_STATE_6_COMPLETE;
			end
			else
				ns_i2c_transceiver = I2C_STATE_3_TRANSFER_BYTE;
		end
	I2C_STATE_4_TRANSFER_ACK:
		begin
			if (change_output_bit_en == 1'b1)
				ns_i2c_transceiver = I2C_STATE_6_COMPLETE;
			else
				ns_i2c_transceiver = I2C_STATE_4_TRANSFER_ACK;
		end
	I2C_STATE_5_STOP_BIT:
		begin
			if (start_and_stop_en == 1'b1)
				ns_i2c_transceiver = I2C_STATE_6_COMPLETE;
			else
				ns_i2c_transceiver = I2C_STATE_5_STOP_BIT;
		end
	I2C_STATE_6_COMPLETE:
		begin
			if (transfer_data == 1'b0)
				ns_i2c_transceiver = I2C_STATE_0_IDLE;
			else
				ns_i2c_transceiver = I2C_STATE_6_COMPLETE;
		end
	default:
		begin
			ns_i2c_transceiver = I2C_STATE_0_IDLE;
		end
	endcase
end

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

// Output Registers
always @(posedge clk)
begin
	if (reset == 1'b1)
		i2c_scen <= 1'b1;
	else if (change_output_bit_en & (s_i2c_transceiver == I2C_STATE_2_START_BIT))
		i2c_scen <= 1'b0;
	else if (s_i2c_transceiver == I2C_STATE_5_STOP_BIT)
		i2c_scen <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		ack <= 1'b0;
	else if (clear_ack == 1'b1)
		ack <= 1'b0;
	else if (start_and_stop_en & (s_i2c_transceiver == I2C_STATE_4_TRANSFER_ACK))
		ack <= i2c_sdata ^ I2C_BUS_MODE;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		data_from_i2c <= 8'h00;
	else if (start_and_stop_en & (s_i2c_transceiver == I2C_STATE_3_TRANSFER_BYTE))
		data_from_i2c <= {data_from_i2c[6:0], i2c_sdata};
end


// Internal Registers
always @(posedge clk)
begin
	if (reset == 1'b1)
		current_bit	<= 3'h0;
	else if ((s_i2c_transceiver == I2C_STATE_3_TRANSFER_BYTE) && 
			(change_output_bit_en == 1'b1))
		current_bit <= current_bit - 3'h1;
	else if (s_i2c_transceiver != I2C_STATE_3_TRANSFER_BYTE)
		current_bit <= num_bits_to_transfer;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		current_byte <= 8'h00;
	else if ((s_i2c_transceiver == I2C_STATE_0_IDLE) || 
			 (s_i2c_transceiver == I2C_STATE_2_START_BIT))
		current_byte <= data_in;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign i2c_sclk		= (I2C_BUS_MODE == 1'b0) ? 
						clk_400KHz :
						((s_i2c_transceiver == I2C_STATE_3_TRANSFER_BYTE) |
						(s_i2c_transceiver == I2C_STATE_4_TRANSFER_ACK)) ?
							clk_400KHz :
							1'b0;

assign i2c_sdata	= 
	(s_i2c_transceiver == I2C_STATE_2_START_BIT) ? 1'b0 :
	(s_i2c_transceiver == I2C_STATE_5_STOP_BIT) ? 1'b0 :
	((s_i2c_transceiver == I2C_STATE_4_TRANSFER_ACK) & read_byte) ? 1'b0 :
	((s_i2c_transceiver == I2C_STATE_3_TRANSFER_BYTE) & ~read_byte) ? 
		current_byte[current_bit]
		: 1'bz;

assign enable_clk	= ~(s_i2c_transceiver == I2C_STATE_0_IDLE) &&
			~(s_i2c_transceiver == I2C_STATE_6_COMPLETE);

assign transfer_complete = 
		(s_i2c_transceiver == I2C_STATE_6_COMPLETE) ? 1'b1 : 1'b0;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module loads data into the Audio and Video chips' control             *
 *  registers after system reset.                                             *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_I2C_AV_Auto_Initialize (
	// Inputs
	clk,
	reset,

	clear_error,

	ack,
	transfer_complete,

	// Bidirectionals

	// Outputs
	data_out,
	transfer_data,
	send_start_bit,
	send_stop_bit,
	
	auto_init_complete,
	auto_init_error
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter MIN_ROM_ADDRESS	= 6'h00;
parameter MAX_ROM_ADDRESS	= 6'h32;

parameter AUD_LINE_IN_LC	= 9'h01A;
parameter AUD_LINE_IN_RC	= 9'h01A;
parameter AUD_LINE_OUT_LC	= 9'h07B;
parameter AUD_LINE_OUT_RC	= 9'h07B;
parameter AUD_ADC_PATH		= 9'h0F8;
parameter AUD_DAC_PATH		= 9'h006;
parameter AUD_POWER			= 9'h000;
parameter AUD_DATA_FORMAT	= 9'h001;
parameter AUD_SAMPLE_CTRL	= 9'h002;
parameter AUD_SET_ACTIVE	= 9'h001;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input				clear_error;

input				ack;
input				transfer_complete;

// Bidirectionals

// Outputs
output	reg	[7:0]	data_out;
output	reg			transfer_data;
output	reg			send_start_bit;
output	reg			send_stop_bit;


output				auto_init_complete;
output	reg			auto_init_error;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/
// States
localparam	AUTO_STATE_0_CHECK_STATUS		= 3'h0,
			AUTO_STATE_1_SEND_START_BIT		= 3'h1,
			AUTO_STATE_2_TRANSFER_BYTE_1	= 3'h2,
			AUTO_STATE_3_TRANSFER_BYTE_2	= 3'h3,
			AUTO_STATE_4_WAIT				= 3'h4,
			AUTO_STATE_5_SEND_STOP_BIT		= 3'h5,
			AUTO_STATE_6_INCREASE_COUNTER	= 3'h6,
			AUTO_STATE_7_DONE				= 3'h7;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				change_state;

wire				finished_auto_init;

// Internal Registers
reg			[5:0]	rom_address_counter;
reg			[25:0]	rom_data;

// State Machine Registers
reg			[2:0]	ns_i2c_auto_init;
reg			[2:0]	s_i2c_auto_init;

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		s_i2c_auto_init <= AUTO_STATE_0_CHECK_STATUS;
	else
		s_i2c_auto_init <= ns_i2c_auto_init;
end

always @(*)
begin
	// Defaults
	ns_i2c_auto_init = AUTO_STATE_0_CHECK_STATUS;

    case (s_i2c_auto_init)
	AUTO_STATE_0_CHECK_STATUS:
		begin
			if (finished_auto_init == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_7_DONE;
			else if (rom_data[25] == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_1_SEND_START_BIT;
			else
				ns_i2c_auto_init = AUTO_STATE_3_TRANSFER_BYTE_2;
		end
	AUTO_STATE_1_SEND_START_BIT:
		begin
			if (change_state == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_2_TRANSFER_BYTE_1;
			else
				ns_i2c_auto_init = AUTO_STATE_1_SEND_START_BIT;
		end
	AUTO_STATE_2_TRANSFER_BYTE_1:
		begin
			if (change_state == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_3_TRANSFER_BYTE_2;
			else
				ns_i2c_auto_init = AUTO_STATE_2_TRANSFER_BYTE_1;
		end
	AUTO_STATE_3_TRANSFER_BYTE_2:
		begin
			if ((change_state == 1'b1) && (rom_data[24] == 1'b1))
				ns_i2c_auto_init = AUTO_STATE_4_WAIT;
			else if (change_state == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_6_INCREASE_COUNTER;
			else
				ns_i2c_auto_init = AUTO_STATE_3_TRANSFER_BYTE_2;
		end
	AUTO_STATE_4_WAIT:
		begin
			if (transfer_complete == 1'b0)
				ns_i2c_auto_init = AUTO_STATE_5_SEND_STOP_BIT;
			else
				ns_i2c_auto_init = AUTO_STATE_4_WAIT;
		end
	AUTO_STATE_5_SEND_STOP_BIT:
		begin
			if (transfer_complete == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_6_INCREASE_COUNTER;
			else
				ns_i2c_auto_init = AUTO_STATE_5_SEND_STOP_BIT;
		end
	AUTO_STATE_6_INCREASE_COUNTER:
		begin
			ns_i2c_auto_init = AUTO_STATE_0_CHECK_STATUS;
		end
	AUTO_STATE_7_DONE:
		begin
			ns_i2c_auto_init = AUTO_STATE_7_DONE;
		end
	default:
		begin
			ns_i2c_auto_init = AUTO_STATE_0_CHECK_STATUS;
		end
	endcase
end

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

// Output Registers
always @(posedge clk)
begin
	if (reset == 1'b1)
		data_out <= 8'h00;
	else if (s_i2c_auto_init == AUTO_STATE_1_SEND_START_BIT)
		data_out <= rom_data[23:16];
	else if (s_i2c_auto_init == AUTO_STATE_0_CHECK_STATUS)
		data_out <= rom_data[15: 8];
	else if (s_i2c_auto_init == AUTO_STATE_2_TRANSFER_BYTE_1)
		data_out <= rom_data[15: 8];
	else if (s_i2c_auto_init == AUTO_STATE_3_TRANSFER_BYTE_2)
		data_out <= rom_data[ 7: 0];
end

always @(posedge clk)
begin
	if (reset == 1'b1) 
		transfer_data <= 1'b0;
	else if (transfer_complete == 1'b1)
		transfer_data <= 1'b0;
	else if (s_i2c_auto_init == AUTO_STATE_1_SEND_START_BIT)
		transfer_data <= 1'b1;
	else if (s_i2c_auto_init == AUTO_STATE_2_TRANSFER_BYTE_1)
		transfer_data <= 1'b1;
	else if (s_i2c_auto_init == AUTO_STATE_3_TRANSFER_BYTE_2)
		transfer_data <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		send_start_bit <= 1'b0;
	else if (transfer_complete == 1'b1)
		send_start_bit <= 1'b0;
	else if (s_i2c_auto_init == AUTO_STATE_1_SEND_START_BIT)
		send_start_bit <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		send_stop_bit <= 1'b0;
	else if (transfer_complete == 1'b1)
		send_stop_bit <= 1'b0;
	else if (s_i2c_auto_init == AUTO_STATE_5_SEND_STOP_BIT)
		send_stop_bit <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		auto_init_error <= 1'b0;
	else if (clear_error == 1'b1)
		auto_init_error <= 1'b0;
	else if ((s_i2c_auto_init == AUTO_STATE_6_INCREASE_COUNTER) & ack)
		auto_init_error <= 1'b1;
end

// Internal Registers
always @(posedge clk)
begin
	if (reset == 1'b1)
		rom_address_counter <= MIN_ROM_ADDRESS;
	else if (s_i2c_auto_init == AUTO_STATE_6_INCREASE_COUNTER)
		rom_address_counter <= rom_address_counter + 6'h01;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/
// Output Assignments
assign auto_init_complete = (s_i2c_auto_init == AUTO_STATE_7_DONE);

// Internals Assignments
assign change_state	= transfer_complete & transfer_data;

assign finished_auto_init = (rom_address_counter == MAX_ROM_ADDRESS);

always @(*)
begin
	case (rom_address_counter)
	//	Audio Config Data
	0		:	rom_data	<=	{10'h334, 7'h0, AUD_LINE_IN_LC};
	1		:	rom_data	<=	{10'h334, 7'h1, AUD_LINE_IN_RC};
	2		:	rom_data	<=	{10'h334, 7'h2, AUD_LINE_OUT_LC};
	3		:	rom_data	<=	{10'h334, 7'h3, AUD_LINE_OUT_RC};
	4		:	rom_data	<=	{10'h334, 7'h4, AUD_ADC_PATH};
	5		:	rom_data	<=	{10'h334, 7'h5, AUD_DAC_PATH};
	6		:	rom_data	<=	{10'h334, 7'h6, AUD_POWER};
	7		:	rom_data	<=	{10'h334, 7'h7, AUD_DATA_FORMAT};
	8		:	rom_data	<=	{10'h334, 7'h8, AUD_SAMPLE_CTRL};
	9		:	rom_data	<=	{10'h334, 7'h9, AUD_SET_ACTIVE};
	//	Video Config Data
	10		:	rom_data	<=	26'h3401500;
	11		:	rom_data	<=	26'h3401741;
	12		:	rom_data	<=	26'h3403a16;
	13		:	rom_data	<=	26'h3405004;
	14		:	rom_data	<=	26'h340c305;
	15		:	rom_data	<=	26'h340c480;
	16		:	rom_data	<=	26'h3400e80;
	17		:	rom_data	<=	26'h3405020;
	18		:	rom_data	<=	26'h3405218;
	19		:	rom_data	<=	26'h34058ed;
	20		:	rom_data	<=	26'h34077c5;
	21		:	rom_data	<=	26'h3407c93;
	22		:	rom_data	<=	26'h3407d00;
	23		:	rom_data	<=	26'h340d048;
	24		:	rom_data	<=	26'h340d5a0;
	25		:	rom_data	<=	26'h340d7ea;
	26		:	rom_data	<=	26'h340e43e;
	27		:	rom_data	<=	26'h340ea0f;
	28		:	rom_data	<=	26'h3403112;
	29		:	rom_data	<=	26'h3403281;
	30		:	rom_data	<=	26'h3403384;
	31		:	rom_data	<=	26'h34037A0;
	32		:	rom_data	<=	26'h340e580;
	33		:	rom_data	<=	26'h340e603;
	34		:	rom_data	<=	26'h340e785;
	35		:	rom_data	<=	26'h3405000;
	36		:	rom_data	<=	26'h3405100;
	37		:	rom_data	<=	26'h3400070;
	38		:	rom_data	<=	26'h3401010;
	39		:	rom_data	<=	26'h3400482;
	40		:	rom_data	<=	26'h3400860;
	41		:	rom_data	<=	26'h3400a18;
	42		:	rom_data	<=	26'h3401100;
	43		:	rom_data	<=	26'h3402b00;
	44		:	rom_data	<=	26'h3402c8c;
	45		:	rom_data	<=	26'h3402df2;
	46		:	rom_data	<=	26'h3402eee;
	47		:	rom_data	<=	26'h3402ff4;
	48		:	rom_data	<=	26'h34030d2;
	49		:	rom_data	<=	26'h3400e05;
	default	:	rom_data	<=	26'h1000000;
	endcase
end

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/


endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module loads data into the TRDB DC2 camera's control registers        *
 *  after system reset.                                                       *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_I2C_DC_Auto_Initialize (
	// Inputs
	clk,
	reset,

	clear_error,

	ack,
	transfer_complete,

	// Bidirectionals

	// Outputs
	data_out,
	transfer_data,
	send_start_bit,
	send_stop_bit,
	
	auto_init_complete,
	auto_init_error
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter DC_ROW_START			= 16'h000C;
parameter DC_COLUMN_START		= 16'h001E;
parameter DC_ROW_WIDTH			= 16'h0400;
parameter DC_COLUMN_WIDTH		= 16'h0500;
parameter DC_H_BLANK_B			= 16'h018C;
parameter DC_V_BLANK_B			= 16'h0032;
parameter DC_H_BLANK_A			= 16'h00C6;
parameter DC_V_BLANK_A			= 16'h0019;
parameter DC_SHUTTER_WIDTH		= 16'h0432;
parameter DC_ROW_SPEED			= 16'h0011;
parameter DC_EXTRA_DELAY		= 16'h0000;
parameter DC_SHUTTER_DELAY		= 16'h0000;
parameter DC_RESET				= 16'h0008;
parameter DC_FRAME_VALID		= 16'h0000;
parameter DC_READ_MODE_B		= 16'h0200;
parameter DC_READ_MODE_A		= 16'h040C;
parameter DC_DARK_COL_ROW		= 16'h0129;
parameter DC_FLASH				= 16'h0608;
parameter DC_GREEN_GAIN_1		= 16'h0020;
parameter DC_BLUE_GAIN			= 16'h0020;
parameter DC_RED_GAIN			= 16'h0020;
parameter DC_GREEN_GAIN_2		= 16'h0020;
parameter DC_GLOBAL_GAIN		= 16'h0020;
parameter DC_CONTEXT_CTRL		= 16'h000B;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input				clear_error;

input				ack;
input				transfer_complete;

// Bidirectionals

// Outputs
output	reg	[7:0]	data_out;
output	reg			transfer_data;
output	reg			send_start_bit;
output	reg			send_stop_bit;


output				auto_init_complete;
output	reg			auto_init_error;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/
// States
localparam	AUTO_STATE_0_CHECK_STATUS		= 4'h0,
			AUTO_STATE_1_SEND_START_BIT		= 4'h1,
			AUTO_STATE_2_TRANSFER_BYTE_0	= 4'h2,
			AUTO_STATE_3_TRANSFER_BYTE_1	= 4'h3,
			AUTO_STATE_4_TRANSFER_BYTE_2	= 4'h4,
			AUTO_STATE_5_WAIT				= 4'h5,
			AUTO_STATE_6_SEND_STOP_BIT		= 4'h6,
			AUTO_STATE_7_INCREASE_COUNTER	= 4'h7,
			AUTO_STATE_8_DONE				= 4'h8;

localparam	MIN_ROM_ADDRESS	= 5'h00;
localparam	MAX_ROM_ADDRESS	= 5'h18;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				change_state;

wire				finished_auto_init;

// Internal Registers
reg			[4:0]	rom_address_counter;
reg			[25:0]	rom_data;

// State Machine Registers
reg			[3:0]	ns_i2c_auto_init;
reg			[3:0]	s_i2c_auto_init;

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		s_i2c_auto_init <= AUTO_STATE_0_CHECK_STATUS;
	else
		s_i2c_auto_init <= ns_i2c_auto_init;
end

always @(*)
begin
	// Defaults
	ns_i2c_auto_init = AUTO_STATE_0_CHECK_STATUS;

    case (s_i2c_auto_init)
	AUTO_STATE_0_CHECK_STATUS:
		begin
			if (finished_auto_init == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_8_DONE;
			else if (rom_data[25] == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_1_SEND_START_BIT;
			else
				ns_i2c_auto_init = AUTO_STATE_3_TRANSFER_BYTE_1;
		end
	AUTO_STATE_1_SEND_START_BIT:
		begin
			if (change_state == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_2_TRANSFER_BYTE_0;
			else
				ns_i2c_auto_init = AUTO_STATE_1_SEND_START_BIT;
		end
	AUTO_STATE_2_TRANSFER_BYTE_0:
		begin
			if (change_state == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_3_TRANSFER_BYTE_1;
			else
				ns_i2c_auto_init = AUTO_STATE_2_TRANSFER_BYTE_0;
		end
	AUTO_STATE_3_TRANSFER_BYTE_1:
		begin
			if (change_state == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_4_TRANSFER_BYTE_2;
			else
				ns_i2c_auto_init = AUTO_STATE_3_TRANSFER_BYTE_1;
		end
	AUTO_STATE_4_TRANSFER_BYTE_2:
		begin
			if ((change_state == 1'b1) && (rom_data[24] == 1'b1))
				ns_i2c_auto_init = AUTO_STATE_5_WAIT;
			else if (change_state == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_7_INCREASE_COUNTER;
			else
				ns_i2c_auto_init = AUTO_STATE_4_TRANSFER_BYTE_2;
		end
	AUTO_STATE_5_WAIT:
		begin
			if (transfer_complete == 1'b0)
				ns_i2c_auto_init = AUTO_STATE_6_SEND_STOP_BIT;
			else
				ns_i2c_auto_init = AUTO_STATE_5_WAIT;
		end
	AUTO_STATE_6_SEND_STOP_BIT:
		begin
			if (transfer_complete == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_7_INCREASE_COUNTER;
			else
				ns_i2c_auto_init = AUTO_STATE_6_SEND_STOP_BIT;
		end
	AUTO_STATE_7_INCREASE_COUNTER:
		begin
			ns_i2c_auto_init = AUTO_STATE_0_CHECK_STATUS;
		end
	AUTO_STATE_8_DONE:
		begin
			ns_i2c_auto_init = AUTO_STATE_8_DONE;
		end
	default:
		begin
			ns_i2c_auto_init = AUTO_STATE_0_CHECK_STATUS;
		end
	endcase
end

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

// Output Registers
always @(posedge clk)
begin
	if (reset == 1'b1)
		data_out <= 8'h00;
	else if (s_i2c_auto_init == AUTO_STATE_1_SEND_START_BIT)
		data_out <= 8'hBA;
	else if (s_i2c_auto_init == AUTO_STATE_2_TRANSFER_BYTE_0)
		data_out <= rom_data[23:16];
	else if (s_i2c_auto_init == AUTO_STATE_0_CHECK_STATUS)
		data_out <= rom_data[15: 8];
	else if (s_i2c_auto_init == AUTO_STATE_3_TRANSFER_BYTE_1)
		data_out <= rom_data[15: 8];
	else if (s_i2c_auto_init == AUTO_STATE_4_TRANSFER_BYTE_2)
		data_out <= rom_data[ 7: 0];
end

always @(posedge clk)
begin
	if (reset == 1'b1) 
		transfer_data <= 1'b0;
	else if (transfer_complete == 1'b1)
		transfer_data <= 1'b0;
	else if (s_i2c_auto_init == AUTO_STATE_1_SEND_START_BIT)
		transfer_data <= 1'b1;
	else if (s_i2c_auto_init == AUTO_STATE_2_TRANSFER_BYTE_0)
		transfer_data <= 1'b1;
	else if (s_i2c_auto_init == AUTO_STATE_3_TRANSFER_BYTE_1)
		transfer_data <= 1'b1;
	else if (s_i2c_auto_init == AUTO_STATE_4_TRANSFER_BYTE_2)
		transfer_data <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		send_start_bit <= 1'b0;
	else if (transfer_complete == 1'b1)
		send_start_bit <= 1'b0;
	else if (s_i2c_auto_init == AUTO_STATE_1_SEND_START_BIT)
		send_start_bit <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		send_stop_bit <= 1'b0;
	else if (transfer_complete == 1'b1)
		send_stop_bit <= 1'b0;
	else if (s_i2c_auto_init == AUTO_STATE_6_SEND_STOP_BIT)
		send_stop_bit <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		auto_init_error <= 1'b0;
	else if (clear_error == 1'b1)
		auto_init_error <= 1'b0;
	else if ((s_i2c_auto_init == AUTO_STATE_7_INCREASE_COUNTER) & ack)
		auto_init_error <= 1'b1;
end

// Internal Registers
always @(posedge clk)
begin
	if (reset == 1'b1)
		rom_address_counter <= MIN_ROM_ADDRESS;
	else if (s_i2c_auto_init == AUTO_STATE_7_INCREASE_COUNTER)
		rom_address_counter <= rom_address_counter + 5'h01;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/
// Output Assignments
assign auto_init_complete = (s_i2c_auto_init == AUTO_STATE_8_DONE);


// Internals Assignments
assign change_state	= transfer_complete & transfer_data;

assign finished_auto_init = (rom_address_counter == MAX_ROM_ADDRESS);

always @(*)
begin
	case (rom_address_counter)
	0		:	rom_data	<=	{10'h201, DC_ROW_START};
	1		:	rom_data	<=	{10'h002, DC_COLUMN_START};
	2		:	rom_data	<=	{10'h003, DC_ROW_WIDTH};
	3		:	rom_data	<=	{10'h004, DC_COLUMN_WIDTH};
	4		:	rom_data	<=	{10'h005, DC_H_BLANK_B};
	5		:	rom_data	<=	{10'h006, DC_V_BLANK_B};
	6		:	rom_data	<=	{10'h007, DC_H_BLANK_A};
	7		:	rom_data	<=	{10'h008, DC_V_BLANK_A};
	8		:	rom_data	<=	{10'h009, DC_SHUTTER_WIDTH};
	9		:	rom_data	<=	{10'h00A, DC_ROW_SPEED};
	10		:	rom_data	<=	{10'h00B, DC_EXTRA_DELAY};
	11		:	rom_data	<=	{10'h00C, DC_SHUTTER_DELAY};
	12		:	rom_data	<=	{10'h10D, DC_RESET};
	13		:	rom_data	<=	{10'h21F, DC_FRAME_VALID};
	14		:	rom_data	<=	{10'h020, DC_READ_MODE_B};
	15		:	rom_data	<=	{10'h021, DC_READ_MODE_A};
	16		:	rom_data	<=	{10'h022, DC_DARK_COL_ROW};
	17		:	rom_data	<=	{10'h123, DC_FLASH};
	18		:	rom_data	<=	{10'h22B, DC_GREEN_GAIN_1};
	19		:	rom_data	<=	{10'h02C, DC_BLUE_GAIN};
	20		:	rom_data	<=	{10'h02D, DC_RED_GAIN};
	21		:	rom_data	<=	{10'h02E, DC_GREEN_GAIN_2};
	22		:	rom_data	<=	{10'h12F, DC_GLOBAL_GAIN};
	23		:	rom_data	<=	{10'h3C8, DC_CONTEXT_CTRL};
	default	:	rom_data	<=	26'h1000000;
	endcase
end

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/


endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module loads data into the TRDB LCM screen's control registers        *
 *  after system reset.                                                       *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_I2C_LCM_Auto_Initialize (
	// Inputs
	clk,
	reset,

	clear_error,

	ack,
	transfer_complete,

	// Bidirectionals

	// Outputs
	data_out,
	data_size,
	transfer_data,
	send_start_bit,
	send_stop_bit,
	
	auto_init_complete,
	auto_init_error
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter LCM_INPUT_FORMAT_UB			= 8'h00;
parameter LCM_INPUT_FORMAT_LB			= 8'h01;
parameter LCM_POWER						= 8'h3F;
parameter LCM_DIRECTION_AND_PHASE		= 8'h17;
parameter LCM_HORIZONTAL_START_POSITION	= 8'h18;
parameter LCM_VERTICAL_START_POSITION	= 8'h08;
parameter LCM_ENB_NEGATIVE_POSITION		= 8'h00;
parameter LCM_GAIN_OF_CONTRAST			= 8'h20;
parameter LCM_R_GAIN_OF_SUB_CONTRAST	= 8'h20;
parameter LCM_B_GAIN_OF_SUB_CONTRAST	= 8'h20;
parameter LCM_OFFSET_OF_BRIGHTNESS		= 8'h10;
parameter LCM_VCOM_HIGH_LEVEL			= 8'h3F;
parameter LCM_VCOM_LOW_LEVEL			= 8'h3F;
parameter LCM_PCD_HIGH_LEVEL			= 8'h2F;
parameter LCM_PCD_LOW_LEVEL				= 8'h2F;
parameter LCM_GAMMA_CORRECTION_0		= 8'h98;
parameter LCM_GAMMA_CORRECTION_1		= 8'h9A;
parameter LCM_GAMMA_CORRECTION_2		= 8'hA9;
parameter LCM_GAMMA_CORRECTION_3		= 8'h99;
parameter LCM_GAMMA_CORRECTION_4		= 8'h08;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input				clear_error;

input				ack;
input				transfer_complete;

// Bidirectionals

// Outputs
output	reg	[7:0]	data_out;
output	reg	[2:0]	data_size;
output	reg			transfer_data;
output	reg			send_start_bit;
output	reg			send_stop_bit;


output				auto_init_complete;
output	reg			auto_init_error;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/
// States
localparam	AUTO_STATE_0_CHECK_STATUS		= 3'h0,
			AUTO_STATE_1_SEND_START_BIT		= 3'h1,
			AUTO_STATE_2_TRANSFER_BYTE_0	= 3'h2,
			AUTO_STATE_3_TRANSFER_BYTE_1	= 3'h3,
			AUTO_STATE_4_WAIT				= 3'h4,
			AUTO_STATE_5_SEND_STOP_BIT		= 3'h5,
			AUTO_STATE_6_INCREASE_COUNTER	= 3'h6,
			AUTO_STATE_7_DONE				= 3'h7;

localparam	MIN_ROM_ADDRESS	= 5'h00;
localparam	MAX_ROM_ADDRESS	= 5'h14;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				change_state;

wire				finished_auto_init;

// Internal Registers
reg			[4:0]	rom_address_counter;
reg			[13:0]	rom_data;

// State Machine Registers
reg			[2:0]	ns_i2c_auto_init;
reg			[2:0]	s_i2c_auto_init;

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		s_i2c_auto_init <= AUTO_STATE_0_CHECK_STATUS;
	else
		s_i2c_auto_init <= ns_i2c_auto_init;
end

always @(*)
begin
	// Defaults
	ns_i2c_auto_init = AUTO_STATE_0_CHECK_STATUS;

    case (s_i2c_auto_init)
	AUTO_STATE_0_CHECK_STATUS:
		begin
			if (finished_auto_init == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_7_DONE;
			else
				ns_i2c_auto_init = AUTO_STATE_1_SEND_START_BIT;
		end
	AUTO_STATE_1_SEND_START_BIT:
		begin
			if (transfer_complete == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_2_TRANSFER_BYTE_0;
			else
				ns_i2c_auto_init = AUTO_STATE_1_SEND_START_BIT;
		end
	AUTO_STATE_2_TRANSFER_BYTE_0:
		begin
			if (change_state == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_3_TRANSFER_BYTE_1;
			else
				ns_i2c_auto_init = AUTO_STATE_2_TRANSFER_BYTE_0;
		end
	AUTO_STATE_3_TRANSFER_BYTE_1:
		begin
			if (change_state == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_4_WAIT;
			else
				ns_i2c_auto_init = AUTO_STATE_3_TRANSFER_BYTE_1;
		end
	AUTO_STATE_4_WAIT:
		begin
			if (transfer_complete == 1'b0)
				ns_i2c_auto_init = AUTO_STATE_5_SEND_STOP_BIT;
			else
				ns_i2c_auto_init = AUTO_STATE_4_WAIT;
		end
	AUTO_STATE_5_SEND_STOP_BIT:
		begin
			if (transfer_complete == 1'b1)
				ns_i2c_auto_init = AUTO_STATE_6_INCREASE_COUNTER;
			else
				ns_i2c_auto_init = AUTO_STATE_5_SEND_STOP_BIT;
		end
	AUTO_STATE_6_INCREASE_COUNTER:
		begin
			ns_i2c_auto_init = AUTO_STATE_0_CHECK_STATUS;
		end
	AUTO_STATE_7_DONE:
		begin
			ns_i2c_auto_init = AUTO_STATE_7_DONE;
		end
	default:
		begin
			ns_i2c_auto_init = AUTO_STATE_0_CHECK_STATUS;
		end
	endcase
end

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

// Output Registers
always @(posedge clk)
begin
	if (reset == 1'b1)
		data_out <= 8'h00;
	else if (s_i2c_auto_init == AUTO_STATE_1_SEND_START_BIT)
		data_out <= {1'b0, rom_data[13: 8], 1'b0};
	else if (s_i2c_auto_init == AUTO_STATE_3_TRANSFER_BYTE_1)
		data_out <= rom_data[ 7: 0];
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		data_size <= 3'h0;
	else if (s_i2c_auto_init == AUTO_STATE_1_SEND_START_BIT)
		data_size <= 3'h6;
	else if (s_i2c_auto_init == AUTO_STATE_3_TRANSFER_BYTE_1)
		data_size <= 3'h7;
end

always @(posedge clk)
begin
	if (reset == 1'b1) 
		transfer_data <= 1'b0;
	else if (transfer_complete == 1'b1)
		transfer_data <= 1'b0;
	else if (s_i2c_auto_init == AUTO_STATE_2_TRANSFER_BYTE_0)
		transfer_data <= 1'b1;
	else if (s_i2c_auto_init == AUTO_STATE_3_TRANSFER_BYTE_1)
		transfer_data <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		send_start_bit <= 1'b0;
	else if (transfer_complete == 1'b1)
		send_start_bit <= 1'b0;
	else if (s_i2c_auto_init == AUTO_STATE_1_SEND_START_BIT)
		send_start_bit <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		send_stop_bit <= 1'b0;
	else if (transfer_complete == 1'b1)
		send_stop_bit <= 1'b0;
	else if (s_i2c_auto_init == AUTO_STATE_5_SEND_STOP_BIT)
		send_stop_bit <= 1'b1;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		auto_init_error <= 1'b0;
	else if (clear_error == 1'b1)
		auto_init_error <= 1'b0;
	else if ((s_i2c_auto_init == AUTO_STATE_6_INCREASE_COUNTER) & ack)
		auto_init_error <= 1'b1;
end

// Internal Registers
always @(posedge clk)
begin
	if (reset == 1'b1)
		rom_address_counter <= MIN_ROM_ADDRESS;
	else if (s_i2c_auto_init == AUTO_STATE_6_INCREASE_COUNTER)
		rom_address_counter <= rom_address_counter + 5'h01;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/
// Output Assignments
assign auto_init_complete = (s_i2c_auto_init == AUTO_STATE_7_DONE);


// Internals Assignments
assign change_state	= transfer_complete & transfer_data;

assign finished_auto_init = (rom_address_counter == MAX_ROM_ADDRESS);

always @(*)
begin
	case (rom_address_counter)
	0		:	rom_data	<=	{6'h02, LCM_INPUT_FORMAT_UB};
	1		:	rom_data	<=	{6'h03, LCM_INPUT_FORMAT_LB};
	2		:	rom_data	<=	{6'h04, LCM_POWER};
	3		:	rom_data	<=	{6'h05, LCM_DIRECTION_AND_PHASE};
	4		:	rom_data	<=	{6'h06, LCM_HORIZONTAL_START_POSITION};
	5		:	rom_data	<=	{6'h07, LCM_VERTICAL_START_POSITION};
	6		:	rom_data	<=	{6'h08, LCM_ENB_NEGATIVE_POSITION};
	7		:	rom_data	<=	{6'h09, LCM_GAIN_OF_CONTRAST};
	8		:	rom_data	<=	{6'h0A, LCM_R_GAIN_OF_SUB_CONTRAST};
	9		:	rom_data	<=	{6'h0B, LCM_B_GAIN_OF_SUB_CONTRAST};
	10		:	rom_data	<=	{6'h0C, LCM_OFFSET_OF_BRIGHTNESS};
	11		:	rom_data	<=	{6'h10, LCM_VCOM_HIGH_LEVEL};
	12		:	rom_data	<=	{6'h11, LCM_VCOM_LOW_LEVEL};
	13		:	rom_data	<=	{6'h12, LCM_PCD_HIGH_LEVEL};
	14		:	rom_data	<=	{6'h13, LCM_PCD_LOW_LEVEL};
	15		:	rom_data	<=	{6'h14, LCM_GAMMA_CORRECTION_0};
	16		:	rom_data	<=	{6'h15, LCM_GAMMA_CORRECTION_1};
	17		:	rom_data	<=	{6'h16, LCM_GAMMA_CORRECTION_2};
	18		:	rom_data	<=	{6'h17, LCM_GAMMA_CORRECTION_3};
	19		:	rom_data	<=	{6'h18, LCM_GAMMA_CORRECTION_4};
	default	:	rom_data	<=	14'h0000;
	endcase
end

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/


endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 *   This module generates the clocks needed for the I/O devices on           *
 * Altera's DE1 and DE2 Boards.                                               *
 *                                                                            *
 ******************************************************************************/

module clock_generator (
	// inputs
	CLOCK2_50,
	reset,

	// outputs
	AUD_XCK
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter AUD_CLK_MULT	= 14;
parameter AUD_CLK_DIV	= 31;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				CLOCK2_50;
input				reset;

// Outputs
output				AUD_XCK;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/


/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				audio_clk_locked;

// Internal Registers

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/


/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

altpll DE_Clock_Generator_Audio (
	.inclk		({1'b0, CLOCK2_50}),
	.clk		(AUD_XCK),
	.locked		(audio_clk_locked),
	.activeclock	(),
	.areset		(reset),
	.clkbad		(),
	.clkena		({6{1'b1}}),
	.clkloss		(),
	.clkswitch		(1'b0),
	.enable0		(),
	.enable1		(),
	.extclk		(),
	.extclkena		({4{1'b1}}),
	.fbin			(1'b1),
	.pfdena		(1'b1),
	.pllena		(1'b1),
	.scanaclr		(1'b0),
	.scanclk		(1'b0),
	.scandata		(1'b0),
	.scandataout	(),
	.scandone		(),
	.scanread		(1'b0),
	.scanwrite		(1'b0),
	.sclkout0		(),
	.sclkout1		()
);
defparam
	DE_Clock_Generator_Audio.clk0_divide_by			= AUD_CLK_DIV,
	DE_Clock_Generator_Audio.clk0_duty_cycle			= 50,
	DE_Clock_Generator_Audio.clk0_multiply_by			= AUD_CLK_MULT,
	DE_Clock_Generator_Audio.clk0_phase_shift			= "0",
	DE_Clock_Generator_Audio.compensate_clock			= "CLK0",
	DE_Clock_Generator_Audio.gate_lock_signal			= "NO",
	DE_Clock_Generator_Audio.inclk0_input_frequency		= 37037,
	DE_Clock_Generator_Audio.intended_device_family		= "Cyclone II",
	DE_Clock_Generator_Audio.invalid_lock_multiplier	= 5,
	DE_Clock_Generator_Audio.lpm_type				= "altpll",
	DE_Clock_Generator_Audio.operation_mode			= "NORMAL",
	DE_Clock_Generator_Audio.pll_type				= "FAST",
	DE_Clock_Generator_Audio.port_activeclock			= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_areset			= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkbad0			= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkbad1			= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkloss				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkswitch				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_fbin					= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_inclk0				= "PORT_USED",
	DE_Clock_Generator_Audio.port_inclk1				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_locked				= "PORT_USED",
	DE_Clock_Generator_Audio.port_pfdena				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_pllena				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_scanaclr				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_scanclk				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_scandata				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_scandataout			= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_scandone				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_scanread				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_scanwrite				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clk0					= "PORT_USED",
	DE_Clock_Generator_Audio.port_clk1					= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clk2					= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clk3					= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clk4					= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clk5					= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkena0				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkena1				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkena2				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkena3				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkena4				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_clkena5				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_enable0				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_enable1				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_extclk0				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_extclk1				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_extclk2				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_extclk3				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_extclkena0			= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_extclkena1			= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_extclkena2			= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_extclkena3			= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_sclkout0				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.port_sclkout1				= "PORT_UNUSED",
	DE_Clock_Generator_Audio.valid_lock_multiplier		= 1;

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 *   This module counts which bits for serial audio transfers. The module     *
 * assume that the data format is I2S, as it is described in the audio        *
 * chip's datasheet.                                                          *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_Audio_Bit_Counter (
	// Inputs
	clk,
	reset,
	
	bit_clk_rising_edge,
	bit_clk_falling_edge,
	left_right_clk_rising_edge,
	left_right_clk_falling_edge,
	
	// Bidirectionals

	// Outputs
	counting
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter BIT_COUNTER_INIT	= 5'h0F;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input				clk;
input				reset;
	
input				bit_clk_rising_edge;
input				bit_clk_falling_edge;
input				left_right_clk_rising_edge;
input				left_right_clk_falling_edge;

// Bidirectionals

// Outputs
output	reg			counting;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/


/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire				reset_bit_counter;

// Internal Registers
reg			[4:0]	bit_counter;

// State Machine Registers


/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		bit_counter <= 5'h00;
	else if (reset_bit_counter == 1'b1)
		bit_counter <= BIT_COUNTER_INIT;
	else if ((bit_clk_falling_edge == 1'b1) && (bit_counter != 5'h00))
		bit_counter <= bit_counter - 5'h01;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		counting <= 1'b0;
	else if (reset_bit_counter == 1'b1)
		counting <= 1'b1;
	else if ((bit_clk_falling_edge == 1'b1) && (bit_counter == 5'h00))
		counting <= 1'b0;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign reset_bit_counter = left_right_clk_rising_edge | 
							left_right_clk_falling_edge;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module read data from the Audio ADC on the Altera DE1 board.          *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_Audio_In_Deserializer (
	// Inputs
	clk,
	reset,
	
	bit_clk_rising_edge,
	bit_clk_falling_edge,
	left_right_clk_rising_edge,
	left_right_clk_falling_edge,

	done_channel_sync,

	serial_audio_in_data,

	read_left_audio_data_en,
	read_right_audio_data_en,

	// Bidirectionals

	// Outputs
	left_audio_fifo_read_space,
	right_audio_fifo_read_space,

	left_channel_data,
	right_channel_data
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter AUDIO_DATA_WIDTH	= 16;
parameter BIT_COUNTER_INIT	= 5'h0F;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input				bit_clk_rising_edge;
input				bit_clk_falling_edge;
input				left_right_clk_rising_edge;
input				left_right_clk_falling_edge;

input				done_channel_sync;

input				serial_audio_in_data;

input				read_left_audio_data_en;
input				read_right_audio_data_en;

// Bidirectionals

// Outputs
output	reg	[7:0]	left_audio_fifo_read_space;
output	reg	[7:0]	right_audio_fifo_read_space;

output		[AUDIO_DATA_WIDTH:1]	left_channel_data;
output		[AUDIO_DATA_WIDTH:1]	right_channel_data;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				valid_audio_input;

wire				left_channel_fifo_is_empty;
wire				right_channel_fifo_is_empty;

wire				left_channel_fifo_is_full;
wire				right_channel_fifo_is_full;

wire		[6:0]	left_channel_fifo_used;
wire		[6:0]	right_channel_fifo_used;

// Internal Registers
reg			[AUDIO_DATA_WIDTH:1]	data_in_shift_reg;

// State Machine Registers


/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		left_audio_fifo_read_space			<= 8'h00;
	else
	begin
		left_audio_fifo_read_space[7]		<= left_channel_fifo_is_full;
		left_audio_fifo_read_space[6:0]		<= left_channel_fifo_used;
	end
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		right_audio_fifo_read_space			<= 8'h00;
	else
	begin
		right_audio_fifo_read_space[7]		<= right_channel_fifo_is_full;
		right_audio_fifo_read_space[6:0]	<= right_channel_fifo_used;
	end
end




always @(posedge clk)
begin
	if (reset == 1'b1)
		data_in_shift_reg	<= {AUDIO_DATA_WIDTH{1'b0}};
	else if (bit_clk_rising_edge & valid_audio_input)
		data_in_shift_reg	<= 
			{data_in_shift_reg[(AUDIO_DATA_WIDTH - 1):1], 
			 serial_audio_in_data};
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Altera_UP_Audio_Bit_Counter Audio_Out_Bit_Counter (
	// Inputs
	.clk							(clk),
	.reset							(reset),
	
	.bit_clk_rising_edge			(bit_clk_rising_edge),
	.bit_clk_falling_edge			(bit_clk_falling_edge),
	.left_right_clk_rising_edge		(left_right_clk_rising_edge),
	.left_right_clk_falling_edge	(left_right_clk_falling_edge),

	// Bidirectionals

	// Outputs
	.counting						(valid_audio_input)
);
defparam 
	Audio_Out_Bit_Counter.BIT_COUNTER_INIT	= BIT_COUNTER_INIT;

Altera_UP_SYNC_FIFO Audio_In_Left_Channel_FIFO(
	// Inputs
	.clk			(clk),
	.reset			(reset),

	.write_en		(left_right_clk_falling_edge & ~left_channel_fifo_is_full & done_channel_sync),
	.write_data		(data_in_shift_reg),

	.read_en		(read_left_audio_data_en & ~left_channel_fifo_is_empty),
	
	// Bidirectionals

	// Outputs
	.fifo_is_empty	(left_channel_fifo_is_empty),
	.fifo_is_full	(left_channel_fifo_is_full),
	.words_used		(left_channel_fifo_used),

	.read_data		(left_channel_data)
);
defparam 
	Audio_In_Left_Channel_FIFO.DATA_WIDTH	= AUDIO_DATA_WIDTH,
	Audio_In_Left_Channel_FIFO.DATA_DEPTH	= 128,
	Audio_In_Left_Channel_FIFO.ADDR_WIDTH	= 7;

Altera_UP_SYNC_FIFO Audio_In_Right_Channel_FIFO(
	// Inputs
	.clk			(clk),
	.reset			(reset),

	.write_en		(left_right_clk_rising_edge & ~right_channel_fifo_is_full & done_channel_sync),
	.write_data		(data_in_shift_reg),

	.read_en		(read_right_audio_data_en & ~right_channel_fifo_is_empty),
	
	// Bidirectionals

	// Outputs
	.fifo_is_empty	(right_channel_fifo_is_empty),
	.fifo_is_full	(right_channel_fifo_is_full),
	.words_used		(right_channel_fifo_used),

	.read_data		(right_channel_data)
);
defparam 
	Audio_In_Right_Channel_FIFO.DATA_WIDTH	= AUDIO_DATA_WIDTH,
	Audio_In_Right_Channel_FIFO.DATA_DEPTH	= 128,
	Audio_In_Right_Channel_FIFO.ADDR_WIDTH	= 7;

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module writes data to the Audio DAC on the Altera DE1 board.          *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_Audio_Out_Serializer (
	// Inputs
	clk,
	reset,
	
	bit_clk_rising_edge,
	bit_clk_falling_edge,
	left_right_clk_rising_edge,
	left_right_clk_falling_edge,
	
	left_channel_data,
	left_channel_data_en,

	right_channel_data,
	right_channel_data_en,
	
	// Bidirectionals

	// Outputs
	left_channel_fifo_write_space,
	right_channel_fifo_write_space,

	serial_audio_out_data
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter AUDIO_DATA_WIDTH	= 16;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input				bit_clk_rising_edge;
input				bit_clk_falling_edge;
input				left_right_clk_rising_edge;
input				left_right_clk_falling_edge;

input		[AUDIO_DATA_WIDTH:1]		left_channel_data;
input				left_channel_data_en;

input		[AUDIO_DATA_WIDTH:1]		right_channel_data;
input				right_channel_data_en;

// Bidirectionals

// Outputs
output	reg	[7:0]	left_channel_fifo_write_space;
output	reg	[7:0]	right_channel_fifo_write_space;

output	reg			serial_audio_out_data;


/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire				read_left_channel;
wire				read_right_channel;

wire				left_channel_fifo_is_empty;
wire				right_channel_fifo_is_empty;

wire				left_channel_fifo_is_full;
wire				right_channel_fifo_is_full;

wire		[6:0]	left_channel_fifo_used;
wire		[6:0]	right_channel_fifo_used;

wire		[AUDIO_DATA_WIDTH:1]		left_channel_from_fifo;
wire		[AUDIO_DATA_WIDTH:1]		right_channel_from_fifo;

// Internal Registers
reg					left_channel_was_read;
reg			[AUDIO_DATA_WIDTH:1]	data_out_shift_reg;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		left_channel_fifo_write_space <= 8'h00;
	else
		left_channel_fifo_write_space <= 8'h80 - {left_channel_fifo_is_full,left_channel_fifo_used};
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		right_channel_fifo_write_space <= 8'h00;
	else
		right_channel_fifo_write_space <= 8'h80 - {right_channel_fifo_is_full,right_channel_fifo_used};
end


always @(posedge clk)
begin
	if (reset == 1'b1)
		serial_audio_out_data <= 1'b0;
	else
		serial_audio_out_data <= data_out_shift_reg[AUDIO_DATA_WIDTH];
end


always @(posedge clk)
begin
	if (reset == 1'b1)
		left_channel_was_read <= 1'b0;
	else if (read_left_channel)
		left_channel_was_read <=1'b1;
	else if (read_right_channel)
		left_channel_was_read <=1'b0;
end


always @(posedge clk)
begin
	if (reset == 1'b1)
		data_out_shift_reg	<= {AUDIO_DATA_WIDTH{1'b0}};
	else if (read_left_channel)
		data_out_shift_reg	<= left_channel_from_fifo;
	else if (read_right_channel)
		data_out_shift_reg	<= right_channel_from_fifo;
	else if (left_right_clk_rising_edge | left_right_clk_falling_edge)
		data_out_shift_reg	<= {AUDIO_DATA_WIDTH{1'b0}};
	else if (bit_clk_falling_edge)
		data_out_shift_reg	<= 
			{data_out_shift_reg[(AUDIO_DATA_WIDTH - 1):1], 1'b0};
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign read_left_channel	= left_right_clk_rising_edge &
								 ~left_channel_fifo_is_empty & 
								 ~right_channel_fifo_is_empty;
assign read_right_channel	= left_right_clk_falling_edge &
								left_channel_was_read;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Altera_UP_SYNC_FIFO Audio_Out_Left_Channel_FIFO(
	// Inputs
	.clk			(clk),
	.reset			(reset),

	.write_en		(left_channel_data_en & ~left_channel_fifo_is_full),
	.write_data		(left_channel_data),

	.read_en		(read_left_channel),
	
	// Bidirectionals

	// Outputs
	.fifo_is_empty	(left_channel_fifo_is_empty),
	.fifo_is_full	(left_channel_fifo_is_full),
	.words_used		(left_channel_fifo_used),

	.read_data		(left_channel_from_fifo)
);
defparam 
	Audio_Out_Left_Channel_FIFO.DATA_WIDTH	= AUDIO_DATA_WIDTH,
	Audio_Out_Left_Channel_FIFO.DATA_DEPTH	= 128,
	Audio_Out_Left_Channel_FIFO.ADDR_WIDTH	= 7;

Altera_UP_SYNC_FIFO Audio_Out_Right_Channel_FIFO(
	// Inputs
	.clk			(clk),
	.reset			(reset),

	.write_en		(right_channel_data_en & ~right_channel_fifo_is_full),
	.write_data		(right_channel_data),

	.read_en		(read_right_channel),
	
	// Bidirectionals

	// Outputs
	.fifo_is_empty	(right_channel_fifo_is_empty),
	.fifo_is_full	(right_channel_fifo_is_full),
	.words_used		(right_channel_fifo_used),

	.read_data		(right_channel_from_fifo)
);
defparam 
	Audio_Out_Right_Channel_FIFO.DATA_WIDTH	= AUDIO_DATA_WIDTH,
	Audio_Out_Right_Channel_FIFO.DATA_DEPTH	= 128,
	Audio_Out_Right_Channel_FIFO.ADDR_WIDTH	= 7;

endmodule

/******************************************************************************
 * License Agreement                                                          *
 *                                                                            *
 * Copyright (c) 1991-2009 Altera Corporation, San Jose, California, USA.     *
 * All rights reserved.                                                       *
 *                                                                            *
 * Any megafunction design, and related net list (encrypted or decrypted),    *
 *  support information, device programming or simulation file, and any other *
 *  associated documentation or information provided by Altera or a partner   *
 *  under Altera's Megafunction Partnership Program may be used only to       *
 *  program PLD devices (but not masked PLD devices) from Altera.  Any other  *
 *  use of such megafunction design, net list, support information, device    *
 *  programming or simulation file, or any other related documentation or     *
 *  information is prohibited for any other purpose, including, but not       *
 *  limited to modification, reverse engineering, de-compiling, or use with   *
 *  any other silicon devices, unless such use is explicitly licensed under   *
 *  a separate agreement with Altera or a megafunction partner.  Title to     *
 *  the intellectual property, including patents, copyrights, trademarks,     *
 *  trade secrets, or maskworks, embodied in any such megafunction design,    *
 *  net list, support information, device programming or simulation file, or  *
 *  any other related documentation or information provided by Altera or a    *
 *  megafunction partner, remains with Altera, the megafunction partner, or   *
 *  their respective licensors.  No other licenses, including any licenses    *
 *  needed under any third party's intellectual property, are provided herein.*
 *  Copying or modifying any file, or portion thereof, to which this notice   *
 *  is attached violates this copyright.                                      *
 *                                                                            *
 * THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THIS FILE OR THE USE OR OTHER DEALINGS  *
 * IN THIS FILE.                                                              *
 *                                                                            *
 * This agreement shall be governed in all respects by the laws of the State  *
 *  of California and by the laws of the United States of America.            *
 *                                                                            *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 * This module finds clock edges of one clock at the frequency of             *
 *  another clock.                                                            *
 *                                                                            *
 ******************************************************************************/

module Altera_UP_Clock_Edge (
	// Inputs
	clk,
	reset,
	
	test_clk,
	
	// Bidirectionals

	// Outputs
	rising_edge,
	falling_edge
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input				clk;
input				reset;
	
input				test_clk;

// Bidirectionals

// Outputs
output				rising_edge;
output				falling_edge;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire				found_edge;

// Internal Registers
reg					cur_test_clk;
reg					last_test_clk;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
	cur_test_clk	<= test_clk;

always @(posedge clk)
	last_test_clk	<= cur_test_clk;

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

// Output Assignments
assign rising_edge	= found_edge & cur_test_clk;
assign falling_edge	= found_edge & last_test_clk;

// Internal Assignments
assign found_edge	= last_test_clk ^ cur_test_clk;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

endmodule



