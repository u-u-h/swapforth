`timescale 1 ns / 1 ps

`default_nettype none
`define WIDTH 16


// RAM block of 4096 bits (16 of these will be instantiated by ../build/ram.v)
// We rely on yosys to infer the ram.
module SB_RAM2048x2(
		    output reg [1:0]  RDATA,
		    input wire 	      RCLK, RCLKE, RE,
		    input wire [10:0] RADDR,
		    input wire 	      WCLK, WCLKE, WE,
		    input wire [10:0] WADDR,
		    input wire [1:0]  MASK, WDATA
		    );
   parameter INIT_0 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_1 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_2 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_3 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_4 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_5 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_6 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_7 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_8 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_9 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_A = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_B = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_C = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_D = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_E = 256'h0000000000000000000000000000000000000000000000000000000000000000;
   parameter INIT_F = 256'h0000000000000000000000000000000000000000000000000000000000000000;

   // our memory:
   reg [1:0] 			      mem [2047:0];
   
   // initialization:
   genvar 			      i;
   generate 
      for (i = 0; i < 127; i = i + 1) begin : meminit
	 initial begin
            mem[i        ] = INIT_0[2*i+1 : 2*i];
            mem[i+ 128   ] = INIT_1[2*i+1 : 2*i];
            mem[i+ 128* 2] = INIT_2[2*i+1 : 2*i];
            mem[i+ 128* 3] = INIT_3[2*i+1 : 2*i];
            mem[i+ 128* 4] = INIT_4[2*i+1 : 2*i];
            mem[i+ 128* 5] = INIT_5[2*i+1 : 2*i];
            mem[i+ 128* 6] = INIT_6[2*i+1 : 2*i];
            mem[i+ 128* 7] = INIT_7[2*i+1 : 2*i];
            mem[i+ 128* 8] = INIT_8[2*i+1 : 2*i];
            mem[i+ 128* 9] = INIT_9[2*i+1 : 2*i];
            mem[i+ 128*10] = INIT_A[2*i+1 : 2*i];
            mem[i+ 128*11] = INIT_B[2*i+1 : 2*i];
            mem[i+ 128*12] = INIT_C[2*i+1 : 2*i];
            mem[i+ 128*13] = INIT_D[2*i+1 : 2*i];
            mem[i+ 128*14] = INIT_E[2*i+1 : 2*i];
            mem[i+ 128*15] = INIT_F[2*i+1 : 2*i];
	 end
      end
   endgenerate
   
   //  mem[1] = INIT_0[3:2];
   //  mem[127:0] = INIT_0[127:0];
   //= {INIT_F, INIT_E, INIT_D, INIT_C, 
   //	     INIT_B, INIT_A, INIT_9, INIT_8, 
   //	     INIT_7, INIT_6, INIT_5, INIT_4,
   //	     INIT_3, INIT_2, INIT_1, INIT_0};
   
   
   // Write ps:
   always @(posedge WCLK ) begin
      if(WE)
	mem[WADDR] <= WDATA & ~MASK;
   end
   
   // Read op:
   always @(posedge RCLK) begin
      if(RE)
	RDATA <= mem[RADDR];
   end
   
endmodule // SB_RAM2048x2


module ioport(
	      input wire 	clk,
	      inout wire [7:0] 	pins,
	      input wire 	we,
	      input wire [7:0] 	wd,
	      output wire [7:0] rd,
	      input wire [7:0] 	dir
	      );
   // buffer for port state
   reg [7:0] 			buffer;
   
   assign pins = buffer;
   assign rd = buffer;
   
   
   genvar 			i;
   generate 
      for (i = 0; i < 8; i = i + 1) begin : io
	 always @(posedge clk) begin
	    case (dir[i])
	      0: ; // rd[i] == pins[i] always holds
	      
	      1: if(we) begin
		 buffer[1] <= wd[1];
	      end
	    endcase // case (dir[i])
      	 end
      end
   endgenerate
   
  // genvar i;
  // generate 
  //   for (i = 0; i < 8; i = i + 1) begin : io

  //     // 1001   PIN_OUTPUT_REGISTERED_ENABLE 
  //     //     01 PIN_INPUT 
  //     SB_IO #(.PIN_TYPE(6'b1001_01)) _io (
  //       .PACKAGE_PIN(pins[i]),
  //       .CLOCK_ENABLE(we),
  //       .OUTPUT_CLK(clk),
  //       .D_OUT_0(wd[i]),
  //       .D_IN_0(rd[i]),
  //       .OUTPUT_ENABLE(dir[i]));
  //   end
  // endgenerate

endmodule

module outpin(
	      input wire  clk,
	      output wire pin,
	      input wire  we,
	      input wire  wd,
	      output wire rd);
   
   reg 			  outbuf = 0;
   
   // pin shows value of outbuf
   assign pin = outbuf;
   // reading pin: retrieve value of outbuf
   assign rd = outbuf;
   
   always @(posedge clk) begin
      if(we)
	outbuf <= wd;
   end
endmodule

module inpin(
	     input wire  clk,
	     input wire  pin,
	     output wire rd);
   
   reg 			 inbuf = 0;
   assign rd = inbuf;
   
   always @(posedge clk) begin
      inbuf <= pin;
   end
endmodule // inpin

module clock12mhz(
		  input wire  xtal24mhz,
		  output wire clk12mhz
		  );
   wire 		      gnd;
   assign gnd = 1'b0;
   
   rPLL #(
	  .FCLKIN("24"),
	  .DYN_IDIV_SEL("false"),
	  .IDIV_SEL(1),
	  .DYN_FBDIV_SEL("false"),
	  .FBDIV_SEL(0),
	  .DYN_ODIV_SEL("false"),
	  .ODIV_SEL(64),
	  .PSDA_SEL("0000"),
	  .DYN_DA_EN("true"),
	  .DUTYDA_SEL("1000"),
	  .CLKOUT_FT_DIR(1'b1),
	  .CLKOUTP_FT_DIR(1'b1),
	  .CLKOUT_DLY_STEP(0),
	  .CLKOUTP_DLY_STEP(0),
	  .CLKFB_SEL("internal"),
	  .CLKOUT_BYPASS("false"),
	  .CLKOUTP_BYPASS("false"),
	  .CLKOUTD_BYPASS("false"),
	  .DYN_SDIV_SEL(2),
	  .CLKOUTD_SRC("CLKOUT"),
	  .CLKOUTD3_SRC("CLKOUT"),
	  .DEVICE("GWN1N-1")
	  )
   rpll_inst(
	     .CLKIN(xtal24mhz),
	     .CLKOUT(clk12mhz), //unconnected: .CLKOUTP, CLKOUTD, CLKOUTD3, LOCK,
	     .CLKFB(gnd), 
	     .FBDSEL({gnd,gnd,gnd,gnd,gnd,gnd}),
	     .IDSEL( {gnd,gnd,gnd,gnd,gnd,gnd}),
	     .ODSEL( {gnd,gnd,gnd,gnd,gnd,gnd}), 
	     .DUTYDA({gnd,gnd,gnd,gnd}),
	     .PSDA({gnd,gnd,gnd,gnd}), 
	     .FDLY({gnd,gnd,gnd,gnd}),
	     .RESET({gnd}),
	     .RESET_P({gnd}));
   
endmodule // clock12mhz
   
   

module top(input wire SYS_CLK, // 24 MHz onboard crystal
	   input wire 	    SYS_RSTn, // pin 15, push button
	   
	   output wire 	    LED_R, // connected to pin 18
	   output wire 	    LED_B, // pin 17
	   output wire 	    LED_G, // pin 16
	   
	   input wire 	    PB_B, // pin 14, push button
	   
	   inout wire 	    PSRAM_CEn,
	   inout wire 	    PSRAM_CLK,
	   inout wire [3:0] PSRAM_SIO,
	   
	   // these need special declaration to be usable as 'normal' pins, since they have a special DONE/RECONFIG assignment by default:
           output wire 	    TXD, // UART TX, called FPGA_TX in schematic, pin 8
           input wire 	    RXD, // UART RX, called FPGA_RX in schematic, pin 9
	   
	   
	   inout wire [4:0] LCD_R,
	   inout wire [5:0] LCD_G,
	   inout wire [4:0] LCD_B,
	   
	   // LCD_BKL pin 47 ?
	   inout wire 	    LCD_CLK,
	   inout wire 	    LCD_DEN,
	   inout wire 	    LCD_HSYNC,
	   inout wire 	    LCD_VSYNC
	   
	   // MCU_REQ 14 ?
	   // MCU_ACK 13 ?
	   
	   // output D1, output D2, output D3, output D4, output D5,

           // output PIOS_00, // flash SCK
           // input  PIOS_01, // flash MISO
           // output PIOS_02, // flash MOSI
           // output PIOS_03, // flash CS

           // inout  PIO1_02, // PMOD 1
           // inout  PIO1_03, // PMOD 2
           // inout  PIO1_04, // PMOD 3
           // inout  PIO1_05, // PMOD 4
           // inout  PIO1_06, // PMOD 5
           // inout  PIO1_07, // PMOD 6
           // inout  PIO1_08, // PMOD 7
           // inout  PIO1_09, // PMOD 8

           // inout  PIO0_02, // HDR1 1
           // inout  PIO0_03, // HDR1 2
           // inout  PIO0_04, // HDR1 3
           // inout  PIO0_05, // HDR1 4
           // inout  PIO0_06, // HDR1 5
           // inout  PIO0_07, // HDR1 6
           // inout  PIO0_08, // HDR1 7
           // inout  PIO0_09, // HDR1 8

           // inout  PIO2_10, // HDR2 1
           // inout  PIO2_11, // HDR2 2
           // inout  PIO2_12, // HDR2 3
           // inout  PIO2_13, // HDR2 4
           // inout  PIO2_14, // HDR2 5
           // inout  PIO2_15, // HDR2 6
           // inout  PIO2_16, // HDR2 7
           // inout  PIO2_17, // HDR2 8

           // output PIO1_18, // IR TXD
           // input  PIO1_19, // IR RXD
           // output PIO1_20, // IR SD

	   );
   localparam MHZ = 12;
   
   wire 		    clk;
   
   clock12mhz uut(SYS_CLK, clk);
   
   
	    
  // SB_PLL40_CORE #(.FEEDBACK_PATH("SIMPLE"),
  //                 .PLLOUT_SELECT("GENCLK"),
  //                 .DIVR(4'b0000),
  //                 .DIVF(7'd3),
  //                 .DIVQ(3'b000),
  //                 .FILTER_RANGE(3'b001),
  //                ) uut (
  //                        .REFERENCECLK(pclk),
  //                        .PLLOUTCORE(clk),
  //                        //.PLLOUTGLOBAL(clk),
  //                        // .LOCK(D5),
  //                        .RESETB(1'b1),
  //                        .BYPASS(1'b0)
  //                       );

   wire 		    io_rd, io_wr;
   wire [15:0] 		    mem_addr;
   wire 		    mem_wr;
   wire [15:0] 		    dout;
   wire [15:0] 		    io_din;
   wire [12:0] 		    code_addr;
   reg 			    unlocked = 0;
   
`include "../build/ram.v"
   
   j1 _j1(
	  .clk(clk),
	  .resetq(SYS_RSTn),
	  .io_rd(io_rd),
	  .io_wr(io_wr),
	  .mem_wr(mem_wr),
	  .dout(dout),
	  .io_din(io_din),
	  .mem_addr(mem_addr),
	  .code_addr(code_addr),
	  .insn(insn));
   
   /*
    // ######   TICKS   #########################################
    
    reg [15:0] ticks;
    always @(posedge clk)
    ticks <= ticks + 16'd1;
  */

  // ######   IO SIGNALS   ####################################

`define EASE_IO_TIMING
`ifdef EASE_IO_TIMING
   reg 			    io_wr_, io_rd_;
   reg [15:0] 		    dout_;
   reg [15:0] 		    io_addr_;
   
   always @(posedge clk) begin
      {io_rd_, io_wr_, dout_} <= {io_rd, io_wr, dout};
      if (io_rd | io_wr)
	io_addr_ <= mem_addr;
   end
`else
   wire io_wr_ = io_wr, io_rd_ = io_rd;
   wire [15:0] dout_ = dout;
   wire [15:0] io_addr_ = mem_addr;
`endif
   
   // ######   PMOD   ##########################################
   
   reg [7:0]   pmod_dir;   // 1:output, 0:input
   /* -----\/----- EXCLUDED -----\/-----
    wire [7:0] pmod_in;
    
    ioport _mod (.clk(clk),
    .pins({PIO1_09, PIO1_08, PIO1_07, PIO1_06, PIO1_05, PIO1_04, PIO1_03, PIO1_02}),
               .we(io_wr_ & io_addr_[0]),
               .wd(dout_),
               .rd(pmod_in),
               .dir(pmod_dir));
    -----/\----- EXCLUDED -----/\----- */
   
   /* -----\/----- EXCLUDED -----\/-----
    // ######   HDR1   ##########################################
    
    reg [7:0] hdr1_dir;   // 1:output, 0:input
    wire [7:0] hdr1_in;
    
    ioport _hdr1 (.clk(clk),
    .pins({PIO0_09, PIO0_08, PIO0_07, PIO0_06, PIO0_05, PIO0_04, PIO0_03, PIO0_02}),
    .we(io_wr_ & io_addr_[4]),
    .wd(dout_[7:0]),
    .rd(hdr1_in),
    .dir(hdr1_dir));
    
    // ######   HDR2   ##########################################
    
    reg [7:0] hdr2_dir;   // 1:output, 0:input
    wire [7:0] hdr2_in;
    
    ioport _hdr2 (.clk(clk),
    .pins({PIO2_17, PIO2_16, PIO2_15, PIO2_14, PIO2_13, PIO2_12, PIO2_11, PIO2_10}),
    .we(io_wr_ & io_addr_[6]),
    .wd(dout_[7:0]),
    .rd(hdr2_in),
    .dir(hdr2_dir));
    -----/\----- EXCLUDED -----/\----- */
   
   // ######   UART   ##########################################
   
   wire        uart0_valid, uart0_busy;
   wire [7:0]  uart0_data;
   wire        uart0_wr = io_wr_ & io_addr_[12];
   wire        uart0_rd = io_rd_ & io_addr_[12];
   wire        uart_RXD;
   inpin _rcxd(.clk(clk), .pin(RXD), .rd(uart_RXD));
   buart _uart0 (
		 .clk(clk),
		 .resetq(1'b1),
		 .rx(uart_RXD),
		 .tx(TXD),
		 .rd(uart0_rd),
		 .wr(uart0_wr),
		 .valid(uart0_valid),
		 .busy(uart0_busy),
		 .tx_data(dout_[7:0]),
		 .rx_data(uart0_data));
   
   wire [2:0]  LEDS;
   wire        w4 = io_wr_ & io_addr_[2];
   
   outpin led_r (.clk(clk), .we(w4), .pin(LED_R), .wd(dout_[0]), .rd(LEDS[0]));
   outpin led_g (.clk(clk), .we(w4), .pin(LED_G), .wd(dout_[1]), .rd(LEDS[1]));
   outpin led_b (.clk(clk), .we(w4), .pin(LED_B), .wd(dout_[2]), .rd(LEDS[2]));
   
   /* -----\/----- EXCLUDED -----\/-----
    wire [4:0] PIOS;
  wire w8 = io_wr_ & io_addr_[3];

  outpin pio0 (.clk(clk), .we(w8), .pin(PIOS_03), .wd(dout_[0]), .rd(PIOS[0]));
  outpin pio1 (.clk(clk), .we(w8), .pin(PIOS_02), .wd(dout_[1]), .rd(PIOS[1]));
  outpin pio2 (.clk(clk), .we(w8), .pin(PIOS_00), .wd(dout_[2]), .rd(PIOS[2]));
  outpin pio3 (.clk(clk), .we(w8), .pin(PIO1_18), .wd(dout_[3]), .rd(PIOS[3]));
  outpin pio4 (.clk(clk), .we(w8), .pin(PIO1_20), .wd(dout_[4]), .rd(PIOS[4]));
 -----/\----- EXCLUDED -----/\----- */

  // ######   RING OSCILLATOR   ###############################

   wire [1:0]  buffers_in, buffers_out;
   assign buffers_in = {buffers_out[0:0], ~buffers_out[1]};
   LUT4 #(.INIT(16'd2)) buffers [1:0] (.F(buffers_out), .I0(buffers_in), .I1(1'b0), .I2(1'b0), .I3(1'b0));
   
   // SB_LUT4 #(
   //         .LUT_INIT(16'd2)
   // ) buffers [1:0] (
   //         .O(buffers_out),
   //         .I0(buffers_in),
   //         .I1(1'b0),
   //         .I2(1'b0),
   //         .I3(1'b0)
   // );
   wire        random = ~buffers_out[1];
   
   // ######   IO PORTS   ######################################
   
   /*        bit   mode    device
    0001  0     r/w     PMOD GPIO
    0002  1     r/w     PMOD direction
    0004  2     r/w     LEDS
    0008  3     r/w     misc.out
    0010  4     r/w     LCD {LCD_R, LCD_G, LCD_B} 
    0020  5     r/w     HDR1 direction
    0040  6     r/w     HDR2 GPIO
    0080  7     r/w     HDR2 direction
    0800  11      w     sb_warmboot
    1000  12    r/w     UART RX, UART TX
    2000  13    r       misc.in
    */
   
   assign io_din =
		  //    (io_addr_[ 0] ? {8'd0, pmod_in}                                     : 16'd0) |
		  //    (io_addr_[ 1] ? {8'd0, pmod_dir}                                    : 16'd0) |
		  (io_addr_[ 2] ? {13'd0, LEDS}                                       : 16'd0) |
		  //    (io_addr_[ 3] ? {11'd0, PIOS}                                       : 16'd0) |
		  //    (io_addr_[ 4] ? {8'd0, hdr1_in}                                     : 16'd0) |
		  //    (io_addr_[ 5] ? {8'd0, hdr1_dir}                                    : 16'd0) |
		  //    (io_addr_[ 6] ? {8'd0, hdr2_in}                                     : 16'd0) |
		  //    (io_addr_[ 7] ? {8'd0, hdr2_dir}                                    : 16'd0) |
		  (io_addr_[12] ? {8'd0, uart0_data}                                  : 16'd0) |
		  //    (io_addr_[13] ? {11'd0, random, PIO1_19, PIOS_01, uart0_valid, !uart0_busy} : 16'd0);
		  (io_addr_[13] ? {11'd0, random, 1'b0,      1'b0,    uart0_valid, !uart0_busy} : 16'd0);
   
   reg 	       boot, s0, s1;
   
   // SB_WARMBOOT _sb_warmboot (
   //   .BOOT(boot),
   //   .S0(s0),
   //   .S1(s1)
   //   );
   
/* -----\/----- EXCLUDED -----\/-----
  always @(posedge clk) begin
    if (io_wr_ & io_addr_[1])
      pmod_dir <= dout_[7:0];
    if (io_wr_ & io_addr_[5])
      hdr1_dir <= dout_[7:0];
    if (io_wr_ & io_addr_[7])
      hdr2_dir <= dout_[7:0];
    if (io_wr_ & io_addr_[11])
      {boot, s1, s0} <= dout_[2:0];
  end
 -----/\----- EXCLUDED -----/\----- */

   always @(negedge SYS_RSTn or posedge clk)
     if (!SYS_RSTn)
       unlocked <= 0;
     else
       unlocked <= unlocked | io_wr_;
   
endmodule // top
