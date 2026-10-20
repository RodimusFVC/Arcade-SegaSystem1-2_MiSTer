//============================================================================
//
//  Sega System 1 / System 2 arcade hardware for MiSTer
//  Copyright (C) 2026 Rodimus
//
//  Top level carried over from the Rodimus Konami82 / MCR cores (Pac-Man core lineage, MikeJ / Sorgelig)
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//
//============================================================================

module emu
(
    `include "sys/emu_ports.vh"
);

wire        CLK_SYS;
wire        locked;
wire [127:0] status;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire [10:0] ps2_key;
wire        ioctl_download;
wire        ioctl_upload;
wire        ioctl_upload_req;
wire  [7:0] ioctl_din;
wire        ioctl_wr;
wire  [7:0] ioctl_index;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] joystick_0, joystick_1;
wire [21:0] gamma_bus;
wire        direct_video;
wire        video_rotated;
wire        pause_cpu;
wire        hblank, vblank;

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_F1 = 0;
assign VGA_SCALER = 0;
assign VGA_DISABLE = 0;
assign FB_FORCE_BLANK = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

wire signed [15:0] audio;
assign AUDIO_L = pause_cpu ? 16'd0 : audio;
assign AUDIO_R = pause_cpu ? 16'd0 : audio;
assign AUDIO_S = 1;   // signed
assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign LED_USER  = ioctl_download;
assign BUTTONS = 0;

///////////////////////////////////////////////////

// MRA index 1 (layout in tools/mra_cfg.py; first 148 bytes as the Gottlieb / Exidy / MCR cores):
//   byte 0      set id (informational)
//   byte 1      flags: [0] 4-way joystick, [1] mirror X, [2] mirror Y, [4] vertical, [7] vertical is ROT90 (cw)
//   bytes 4-11  idle level of the 8 ports (P1, P2, SYSTEM, SWA, SWB, X0, X1, X2)
//   bytes 12-19 per port: DIP switch byte that supplies it (bit 7 set = none, use the idle byte)
//   bytes 20-83 input map, one byte per port bit (bit 0 first): control id, 0 = none; a pressed control inverts its bit
//   bytes 84-147 per port bit: which bit of its DIP switch byte supplies it
//   bytes 148-157 board config: board, decryption, opcode source, banking, tiles, sprite banks, game I/O
reg [7:0] game_var    = 8'd0;
reg [7:0] game_flags  = 8'h00;
reg [7:0] cfg[10]     = '{8'd1,8'd0,8'd0,8'd0,8'd0,8'd1,8'd0,8'd0,8'd0,8'd0};
reg [5:0] in_map[64];
reg [7:0] port_idle[8] = '{8'hFF,8'hFF,8'hFF,8'hFF,8'hFF,8'hFF,8'hFF,8'hFF};
reg [7:0] port_dip[8]  = '{8'h80,8'h80,8'h80,8'h80,8'h80,8'h80,8'h80,8'h80};
wire [7:0] map_idx8  = ioctl_addr[7:0] - 8'd20;    // bytes 20-83  -> 0-63
wire [7:0] perm_idx8 = ioctl_addr[7:0] - 8'd84;    // bytes 84-147 -> 0-63
wire [7:0] cfg_idx8  = ioctl_addr[7:0] - 8'd148;   // bytes 148-157 -> 0-9
wire [5:0] map_idx   = map_idx8[5:0];
wire [5:0] perm_idx  = perm_idx8[5:0];
wire [2:0] idle_idx  = ioctl_addr[2:0] - 3'd4;     // bytes 4-11  -> 0-7
wire [2:0] dip_idx   = ioctl_addr[2:0] - 3'd4;     // bytes 12-19 -> 0-7
reg  [2:0] dip_bit[64];
initial for (int i = 0; i < 64; i++) dip_bit[i] = i % 8;
initial for (int i = 0; i < 64; i++) in_map[i] = 6'd0;

always @(posedge CLK_SYS) begin
    if (ioctl_wr && ioctl_index == 8'd1) begin
        if (ioctl_addr >= 25'd84 && ioctl_addr < 25'd148) dip_bit[perm_idx] <= ioctl_dout[2:0];
        if (ioctl_addr == 25'd0) game_var   <= ioctl_dout;
        if (ioctl_addr == 25'd1) game_flags <= ioctl_dout;
        if (ioctl_addr >= 25'd4  && ioctl_addr < 25'd12) port_idle[idle_idx] <= ioctl_dout;
        if (ioctl_addr >= 25'd12 && ioctl_addr < 25'd20) port_dip[dip_idx]   <= ioctl_dout;
        if (ioctl_addr >= 25'd20 && ioctl_addr < 25'd84) in_map[map_idx] <= ioctl_dout[5:0];
        if (ioctl_addr >= 25'd148 && ioctl_addr < 25'd158) cfg[cfg_idx8[3:0]] <= ioctl_dout;
    end
end

wire game_vert = game_flags[4];
wire vert_view = game_vert & ~status[12];

wire [1:0] ar = status[9:8];

assign VIDEO_ARX = (!ar) ? (vert_view ? 12'd3 : 12'd4) : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? (vert_view ? 12'd4 : 12'd3) : 12'd0;

// Status bit layout kept the same as the Konami82 / Pac-Man cores
`include "build_id.v"
localparam CONF_STR = {
	"SEGASYS1;;",
	"P1,Video Options;",
	"P1O89,Aspect Ratio,Original,Full screen,[ARC1],[ARC2];",
	"P1OC,Orientation,Vert,Horz;",
	"P1OB,HDMI Flip,Off,On;",
	"P1OM,CRT Flip,Off,On;",
	"P1O[40:37],H Position (CRT),0,+1,+2,+3,+4,+5,+6,+7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[44:41],V Position (CRT),0,+1,+2,+3,+4,+5,+6,+7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1OGI,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
	"P2,Game Options;",
	"P2O[36:34],SDRAM Read Latch,Auto,Normal,Early,Late,Later;",
	"P2ON,Crosshair (Light Gun),On,Off;",
	"P2O[48:45],SDRAM Clk Phase,Auto,0.0ns,0.8ns,1.6ns,2.3ns,3.1ns,3.9ns,4.7ns,5.5ns,6.3ns,7.0ns,7.8ns,8.6ns,9.4ns,10.2ns,10.9ns;",
	"-;",
	"P3,Pause Options;",
	"P3OJ,Pause when OSD is open,On,Off;",
	"P3OK,Dim video after 10s,On,Off;",
	"-;",
	"P4,High Score Options;",
	"P4OL,Autosave Hiscores,Off,On;",
	"-;",
	"DIP;",
	"-;",
	"R0,Reset;",
	"J1,Btn 1,Btn 2,Btn 3,Btn 4,Coin,Start 1P,Start 2P,Pause,Btn 5,Btn 6;",
	"jn,A,B,X,Y,Select,Start,R,L;",
	"V,v",`BUILD_DATE
};

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(CLK_SYS),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),
	.direct_video(direct_video),
	.video_rotated(video_rotated),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({direct_video}),

	.ioctl_download(ioctl_download),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_din(ioctl_din),
	.ioctl_index(ioctl_index),
	.ioctl_wait(sdr_ioctl_wait),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_l_analog_0(joy_a0),
	.joystick_l_analog_1(joy_a1),
	.ps2_key(ps2_key),
	.ps2_mouse(ps2_mouse)
);

////////////////////   CLOCKS   ///////////////////

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(CLK_SYS),
	.outclk_1(CLK_SDRAM),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll),
	.locked(locked)
);

wire [63:0] reconfig_to_pll;
wire [63:0] reconfig_from_pll;
wire        cfg_waitrequest;
wire        CLK_SDRAM;         // 80 MHz, same as CLK_SYS at phase position 0
wire        cfg_write;
wire  [5:0] cfg_address;
wire [31:0] cfg_data;

pll_cfg pll_cfg
(
	.mgmt_clk(CLK_50M),
	.mgmt_reset(0),
	.mgmt_waitrequest(cfg_waitrequest),
	.mgmt_read(0),
	.mgmt_readdata(),
	// writes come from sdram_phase_ctl
	.mgmt_write(cfg_write),
	.mgmt_address(cfg_address),
	.mgmt_writedata(cfg_data),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

// SDRAM clock phase (from the Pac-Man / Konami82 cores). One DPS step = VCO/8 = 156 ps (800 MHz VCO),
// 80 per 80 MHz period; manual OSD entries are 5 steps. Auto: sys1_sdrom sweeps all 80
// after a load and parks mid-window.
wire       ph_auto = (status[48:45] == 4'd0);
wire [6:0] cal_ph;
wire       cal_tgl, cal_hold, ph_ack_tgl;

sdram_phase_ctl #(.CNT_SEL(5'd1), .MAN_STEP(7'd5)) phase_ctl
(
	.clk(CLK_50M),
	.sel(status[48:45]),
	.cal_val(cal_ph),
	.cal_tgl(cal_tgl),
	.ack_tgl(ph_ack_tgl),
	.cfg_waitrequest(cfg_waitrequest),
	.cfg_write(cfg_write),
	.cfg_address(cfg_address),
	.cfg_data(cfg_data)
);

// Phase moved: ack the calibrator and re-measure the read latch.
reg  [2:0] ph_ack_s = 3'd0;
always @(posedge CLK_SYS) ph_ack_s <= {ph_ack_s[1:0], ph_ack_tgl};
wire       sdr_recal = ph_ack_s[2] ^ ph_ack_s[1];

// Hold the board in reset until the PLL is locked and the ROM download has finished;
// cal_hold keeps it in reset through the SDRAM phase sweep
wire reset = RESET | status[0] | buttons[1] | ioctl_download | ~locked | cal_hold;

///////////////////         Keyboard           //////////////////

reg kb_up = 0, kb_down = 0, kb_left = 0, kb_right = 0;
reg kb_b1 = 0, kb_b2 = 0;
reg kb_coin1 = 0, kb_coin2 = 0, kb_start1 = 0, kb_start2 = 0, kb_pause = 0, kb_service = 0, kb_tilt = 0, kb_service1 = 0;

wire       pressed = ~ps2_key[9];
wire [7:0] code    = ps2_key[7:0];

always @(posedge CLK_SYS) begin
	reg old_state;
	old_state <= ps2_key[10];
	if (old_state != ps2_key[10]) begin
		case (code)
			'h16: kb_start1   <= pressed; // 1
			'h1E: kb_start2   <= pressed; // 2
			'h2E: kb_coin1    <= pressed; // 5
			'h36: kb_coin2    <= pressed; // 6
			'h46: kb_service  <= pressed; // 9 (test switch)
			'h06: kb_service  <= pressed; // F2
			'h3E: kb_service1 <= pressed; // 8 (service credit)
			'h2C: kb_tilt     <= pressed; // T
			'h4D: kb_pause    <= pressed; // P

			'h75: kb_up       <= pressed; // up
			'h72: kb_down     <= pressed; // down
			'h6B: kb_left     <= pressed; // left
			'h74: kb_right    <= pressed; // right
			'h14: kb_b1       <= pressed; // ctrl
			'h11: kb_b2       <= pressed; // alt
		endcase
	end
end

// Mahjong panel (DakkoChan House), MAME's default keys: A-N letters, Kan LCtrl, Pon LAlt, Chi Space, Reach LShift,
// Ron Z, Flip Flop Y, Bet 3, Last Chance RAlt, Start 1
reg [13:0] mj_ltr = 14'd0;              // A-N
reg mj_kan = 0, mj_pon = 0, mj_chi = 0, mj_reach = 0, mj_ron = 0, mj_flip = 0, mj_bet = 0, mj_last = 0;
always @(posedge CLK_SYS) begin
	reg old_state;
	old_state <= ps2_key[10];
	if (old_state != ps2_key[10]) begin
		if (ps2_key[8]) begin
			if (code == 'h11) mj_last <= pressed;     // RAlt
		end
		else case (code)
			'h1C: mj_ltr[0]  <= pressed; 'h32: mj_ltr[1]  <= pressed; 'h21: mj_ltr[2]  <= pressed; 'h23: mj_ltr[3]  <= pressed;
			'h24: mj_ltr[4]  <= pressed; 'h2B: mj_ltr[5]  <= pressed; 'h34: mj_ltr[6]  <= pressed; 'h33: mj_ltr[7]  <= pressed;
			'h43: mj_ltr[8]  <= pressed; 'h3B: mj_ltr[9]  <= pressed; 'h42: mj_ltr[10] <= pressed; 'h4B: mj_ltr[11] <= pressed;
			'h3A: mj_ltr[12] <= pressed; 'h31: mj_ltr[13] <= pressed;
			'h14: mj_kan   <= pressed;  'h11: mj_pon  <= pressed;  'h29: mj_chi  <= pressed;  'h12: mj_reach <= pressed;
			'h1A: mj_ron   <= pressed;  'h35: mj_flip <= pressed;  'h26: mj_bet  <= pressed;
		endcase
	end
end
wire mj_start = joystick_0[9] | kb_start1;
// MAME KEY0-KEY6 rows, active low
wire [55:0] mj_rows = ~{8'h00,
                        {5'd0, mj_ron, mj_reach, mj_kan},
                        {6'd0, mj_bet, mj_start},
                        {3'd0, mj_flip, mj_pon, mj_chi, mj_ltr[13:12]},
                        {4'd0, mj_ltr[11:8]},
                        {4'd0, mj_ltr[7:4]},
                        {3'd0, mj_last, mj_ltr[3:0]}};

//////////////////  Arcade Buttons/Interfaces   ///////////////////////////

// Joystick bits: 0 R, 1 L, 2 D, 3 U, 4-7 Btn 1-4, 8 Coin, 9 Start 1P, 10 Start 2P, 11 Pause, 12-13 Btn 5-6
wire [3:0] dir1_raw = {joystick_0[3] | kb_up, joystick_0[2] | kb_down, joystick_0[1] | kb_left, joystick_0[0] | kb_right};
wire [3:0] dir2_raw = joystick_1[3:0];
wire [3:0] dir1, dir2;   // {U, D, L, R}

joy4way joy4way_1(.clk(CLK_SYS), .en(game_flags[0]), .in(dir1_raw), .out(dir1));
joy4way joy4way_2(.clk(CLK_SYS), .en(game_flags[0]), .in(dir2_raw), .out(dir2));

wire m_pause = joystick_0[11] | kb_pause;

// control ids used by the MRA input map (tools/mra_cfg.py; Gottlieb / MCR numbering)
wire [63:0] ctl =
{
	16'd0,                                          // 63-48 reserved
	kb_service1,                                    // 47 service credit
	11'd0,                                          // 46-36 reserved
	joystick_1[13:12],                              // 35-34 P2 Btn 6-5
	joystick_0[13:12],                              // 33-32 P1 Btn 6-5
	4'd0,                                           // 31-28 P2 right stick (later)
	4'd0,                                           // 27-24 P1 right stick (later)
	1'b0,                                           // 23 coin 3
	kb_service,                                     // 22 service (test switch)
	kb_tilt,                                        // 21 tilt
	joystick_0[10] | joystick_1[10] | joystick_1[9] | kb_start2,   // 20 start 2 (pad 2's Start is Start 2P)
	joystick_0[9]  | kb_start1,                     // 19 start 1
	joystick_1[8]  | kb_coin2,                      // 18 coin 2
	joystick_0[8]  | kb_coin1,                      // 17 coin 1
	joystick_1[7:4],                                // 16-13 P2 Btn 4-1
	dir2[0], dir2[1], dir2[2], dir2[3],             // 12 R, 11 L, 10 D, 9 U
	joystick_0[7:6], joystick_0[5] | kb_b2, joystick_0[4] | kb_b1,   // 8-5 P1 Btn 4-1
	dir1[0], dir1[1], dir1[2], dir1[3],             // 4 R, 3 L, 2 D, 1 U
	1'b0                                            // 0 none
};

// DIP switches arrive from the OSD via ioctl index 254
reg [7:0] dip_sw[8] = '{8'hFF,8'hFF,8'hFF,8'hFF,8'h00,8'h00,8'h00,8'h00};
always @(posedge CLK_SYS) begin
	if (ioctl_wr && (ioctl_index == 8'd254) && !ioctl_addr[24:3])
		dip_sw[ioctl_addr[2:0]] <= ioctl_dout;
end

// Dial (Block Gal; MAME IPT_DIAL, reversed): 8-bit position counters per player, from the Gottlieb core's trackball
// logic. Player 1: mouse, else the left analog stick, else the D-pad; player 2: its left analog stick, else its D-pad.
// Sticks and D-pads move a 16-bit position by the deflection (D-pad: half scale); the top byte is the count.
// One mouse count = half a position count.
wire [24:0] ps2_mouse;               // [24] toggles per packet, [23:16] dy (up +), [15:8] dx, [5] / [4] dy / dx sign
wire [15:0] joy_a0, joy_a1;          // left analog sticks: Y high byte, X low byte (signed, up / left negative)
wire signed [7:0] ax = joy_a0[7:0], bx1 = joy_a1[7:0];
wire       a_on  = (ax  > 8'sd24) || (ax  < -8'sd24);
wire       a1_on = (bx1 > 8'sd24) || (bx1 < -8'sd24);
wire signed [15:0] mdx = {{7{ps2_mouse[4]}}, ps2_mouse[15:8], 1'b0} <<< 6;
function signed [15:0] step(input on, input signed [7:0] v, input neg, input pos);   // stick, else D-pad
	step = on ? {{8{v[7]}}, v} : neg ? -16'sd64 : pos ? 16'sd64 : 16'sd0;
endfunction
reg [15:0] tb_x0 = 0, tb_x1 = 0;
reg [13:0] tb_div = 0;
reg        mouse_t = 0;
always @(posedge CLK_SYS) begin
	mouse_t <= ps2_mouse[24];
	tb_div  <= tb_div + 1'd1;
	if (mouse_t != ps2_mouse[24]) tb_x0 <= tb_x0 + mdx;
	else if (!tb_div) begin
		tb_x0 <= tb_x0 + step(a_on,  ax,  dir1[1], dir1[0]);
		tb_x1 <= tb_x1 + step(a1_on, bx1, dir2[1], dir2[0]);
	end
end
wire       io_dial = cfg[6][0];
wire [7:0] dial_p1 = 8'd0 - tb_x0[15:8];     // PORT_REVERSE: right counts down
wire [7:0] dial_p2 = 8'd0 - tb_x1[15:8];

// Light gun (Shooting Master), from the Space Invaders core: crosshair moved by the mouse, the left stick or the D-pad;
// the left mouse button, Btn 1 or Ctrl pulls the trigger. Content x / 2 (0-255) and row (0-223); CRT Flip turns the
// controls with the picture.
reg  [7:0] gun_x = 8'd128, gun_y = 8'd112;
reg        gmouse_tog = 1'b0, gun_vbl = 1'b0;
wire signed [9:0] gmouse_dx = {ps2_mouse[4], ps2_mouse[4], ps2_mouse[15:8]};
wire signed [9:0] gmouse_dy = -{ps2_mouse[5], ps2_mouse[5], ps2_mouse[23:16]};       // PS/2 Y is positive upward
wire signed [7:0] gana_x = joy_a0[7:0], gana_y = joy_a0[15:8];
wire signed [9:0] gstick_dx = (gana_x > 8'sd12 || gana_x < -8'sd12) ? (10'(gana_x) >>> 4) :
                              dir1[0] ? 10'sd3 : dir1[1] ? -10'sd3 : 10'sd0;
wire signed [9:0] gstick_dy = (gana_y > 8'sd12 || gana_y < -8'sd12) ? (10'(gana_y) >>> 4) :
                              dir1[2] ? 10'sd3 : dir1[3] ? -10'sd3 : 10'sd0;
wire signed [9:0] gun_dx = status[22] ? -(gmouse_tog != ps2_mouse[24] ? gmouse_dx : gstick_dx) :
                                         (gmouse_tog != ps2_mouse[24] ? gmouse_dx : gstick_dx);
wire signed [9:0] gun_dy = status[22] ? -(gmouse_tog != ps2_mouse[24] ? gmouse_dy : gstick_dy) :
                                         (gmouse_tog != ps2_mouse[24] ? gmouse_dy : gstick_dy);
wire signed [10:0] gun_nx = $signed({3'b000, gun_x}) + gun_dx;
wire signed [10:0] gun_ny = $signed({3'b000, gun_y}) + gun_dy;
always @(posedge CLK_SYS) begin
	gmouse_tog <= ps2_mouse[24];
	gun_vbl    <= vblank;
	if (gmouse_tog != ps2_mouse[24] || (vblank && !gun_vbl)) begin
		gun_x <= gun_nx < 0 ? 8'd0 : gun_nx > 255 ? 8'd255 : gun_nx[7:0];
		gun_y <= gun_ny < 0 ? 8'd0 : gun_ny > 223 ? 8'd223 : gun_ny[7:0];
	end
end
wire gun_trig = joystick_0[4] | kb_b1 | ps2_mouse[0];

reg [7:0] in_port[8];
always @(posedge CLK_SYS) begin
	for (int p = 0; p < 8; p++)
		for (int b = 0; b < 8; b++)
			in_port[p][b] <= (port_dip[p][7] ? port_idle[p][b] : dip_sw[port_dip[p][2:0]][dip_bit[p*8 + b]]) ^ ctl[in_map[p*8 + b]];
end

// PAUSE SYSTEM
wire [23:0] rgb_out;
pause #(8,8,8,80) pause
(
	.*,
	.clk_sys(CLK_SYS),
	.user_button(m_pause),
	.pause_request(hs_pause),
	.options(~status[20:19])
);

///////////////                 Video                  ////////////////

wire hs, vs;
wire [7:0] r, g, b;
wire ce_pix;

wire rotate_ccw = ~game_flags[7];  // ROT270 sets rotate CCW, ROT90 sets CW
wire no_rotate  = ~game_vert | status[12] | direct_video;
wire flip       = status[11];
screen_rotate screen_rotate(.*);

arcade_video #(512,24) arcade_video
(
	.*,

	.clk_video(CLK_SYS),

	.RGB_in(rgb_out),
	.HBlank(hblank),
	.VBlank(vblank),
	.HSync(hs),
	.VSync(vs),

	.fx(status[18:16])
);

///////////////                 Board                  ////////////////

segasys1_board board
(
	.clk(CLK_SYS),
	.reset(reset),
	.pause(pause_cpu),

	.cfg_board(cfg[0]),
	.cfg_crypt(cfg[1]),
	.cfg_opc(cfg[2]),
	.cfg_bank(cfg[3]),
	.cfg_tiles(cfg[4]),
	.cfg_sprbanks(cfg[5]),
	.cfg_sndclk(cfg[7]),
	.cfg_io(cfg[6]),
	.cfg_scramble(cfg[8]),
	.gun_x(gun_x),
	.gun_y(gun_y),
	.gun_trig(gun_trig),
	.xhair_en(~status[23]),
	.mj_rows(mj_rows),

	.in_p1(io_dial ? dial_p1 : in_port[0]),
	.in_p2(io_dial ? dial_p2 : in_port[1]),
	.in_sys(in_port[2]),
	.in_swa(in_port[3]),
	.in_swb(in_port[4]),

	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wr0(ioctl_wr & (ioctl_index == 8'd0)),

	.crt_flip(status[22]),
	.h_adj(status[40:37]),
	.v_adj(status[44:41]),

	.ce_pix(ce_pix),
	.video_r(r),
	.video_g(g),
	.video_b(b),
	.video_hs(hs),
	.video_vs(vs),
	.video_hblank(hblank),
	.video_vblank(vblank),

	.audio(audio),

	.sdr_req(sdr_req),
	.sdr_waddr(sdr_waddr),
	.sdr_word(sdr_word),
	.sdr_busy(sdr_busy),

	.hs_address(hs_address[11:0]),
	.hs_data_in(hs_data_in),
	.hs_data_out(hs_data_out),
	.hs_write(hs_write_enable)
);

// SDRAM: sprite ROMs (index-0 window 0x80000-0xBFFFF). Read latch codes 0 Early 1 Normal 2 Late 3 Later
wire       rd_auto = (status[36:34] == 3'd0);
wire [1:0] rd_mode = (status[36:34] == 3'd2) ? 2'd0 :
                     (status[36:34] == 3'd3) ? 2'd2 :
                     (status[36:34] == 3'd4) ? 2'd3 : 2'd1;
wire        sdr_req;
wire [16:0] sdr_waddr;
wire [15:0] sdr_word;
wire        sdr_busy;
wire        sdr_ioctl_wait;

sys1_sdrom sdrom
(
	.clk(CLK_SYS),
	.por_reset(~locked),

	.ioctl_download(ioctl_download),
	.ioctl_wr_sd(ioctl_wr & (ioctl_index == 8'd0) & (ioctl_addr[24:18] == 7'd2)),
	.ioctl_off(ioctl_addr[17:0]),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(sdr_ioctl_wait),

	.rd_auto(rd_auto),
	.rd_mode(rd_mode),
	.spr_req(sdr_req),
	.spr_waddr(sdr_waddr),
	.spr_word(sdr_word),
	.spr_busy(sdr_busy),

	.recal(sdr_recal),
	.ph_auto(ph_auto),
	.ld_hold(status[0]),
	.ph_req_val(cal_ph),
	.ph_req_tgl(cal_tgl),
	.cal_hold(cal_hold),

	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CKE(SDRAM_CKE),
	// SDRAM_CLK is driven from CLK_SDRAM below
	.SDRAM_CLK()
);

// SDRAM_CLK: same inverted DDR output as sdram.sv's, on the phase-steppable CLK_SDRAM.
altddio_out
#(
	.extend_oe_disable("OFF"),
	.intended_device_family("Cyclone V"),
	.invert_output("OFF"),
	.lpm_hint("UNUSED"),
	.lpm_type("altddio_out"),
	.oe_reg("UNREGISTERED"),
	.power_up_high("OFF"),
	.width(1)
)
sdramclk_ddr
(
	.datain_h(1'b0),
	.datain_l(1'b1),
	.outclock(CLK_SDRAM),
	.dataout(SDRAM_CLK),
	.aclr(1'b0),
	.aset(1'b0),
	.oe(1'b1),
	.outclocken(1'b1),
	.sclr(1'b0),
	.sset(1'b0)
);

// Hiscore: config = MRA index 3, dump = index 4; main RAM (C000-CFFF) via its second port while the CPU is paused
wire [15:0] hs_address;
wire  [7:0] hs_data_in;
wire  [7:0] hs_data_out;
wire        hs_write_enable;
wire        hs_access_read;
wire        hs_access_write;
wire        hs_pause;
wire        hs_configured;

hiscore #(
	.HS_ADDRESSWIDTH(16),
	.CFG_ADDRESSWIDTH(4),
	.CFG_LENGTHWIDTH(2)
) hi (
	.*,
	.clk(CLK_SYS),
	.paused(pause_cpu),
	.autosave(status[21]),
	.ram_address(hs_address),
	.data_from_ram(hs_data_out),
	.data_to_ram(hs_data_in),
	.data_from_hps(ioctl_dout),
	.data_to_hps(ioctl_din),
	.ram_write(hs_write_enable),
	.ram_intent_read(hs_access_read),
	.ram_intent_write(hs_access_write),
	.pause_cpu(hs_pause),
	.configured(hs_configured)
);

endmodule

// 4-way joystick: the most recently pressed direction wins (from the MiSTer Pac-Man port)
module joy4way
(
	input        clk,
	input        en,
	input  [3:0] in,
	output [3:0] out
);

reg  [3:0] mask = 4'hF;
reg  [3:0] in1 = 0, in2 = 0;
wire [3:0] innew = in1 & ~in2;

assign out = in1 & mask;

always @(posedge clk) begin
	in1 <= in;
	in2 <= in1;

	if (innew[0]) mask <= 4'b0001;
	if (innew[1]) mask <= 4'b0010;
	if (innew[2]) mask <= 4'b0100;
	if (innew[3]) mask <= 4'b1000;

	if (!(in & mask) || !en) mask <= 4'hF;
end

endmodule
