//============================================================================
//
//  sys1_video.sv — Sega System 1 video: timing, tilemaps, sprites, mixer
//  Copyright (C) 2026 Rodimus
//
//  From MAME sega/system1_v.cpp (the checking reference) and the Sega 834-5191 / 171-5054 board docs.
//  640 x 260 at the 10 MHz pixel clock (20 MHz / 2), visible 512 x 224 (sys1ppis / sys1pios: x 16-494).
//  Tilemaps are 8x8, 3 bpp, at half the pixel rate; sprites are 4 bpp at the pixel rate (each ROM nibble = 2 pixels).
//
//  System 2 (MAME screen_update_system2 / _rowscroll): 8 video RAM pages, foreground = page 0, background = 512 x 512
//  from 4 pages picked by vram[740/742/744/746], scroll x = vram[7C1:7C0] & 1FF - 502 (row scroll: one word per 8-line
//  row from 7C0), y = vram[7BA]; flipped from 7F6 / 7FE- / 784. Sprites 14 pixels right (left when flipped).
//
//  Every line is built one line ahead into double-buffered line buffers:
//    tile pass   foreground per tile pixel, then background (scrolled) per output pixel
//    sprite pass the 32 sprite entries in order, read from SDRAM; a non-transparent pixel landing on another sprite's
//                pixel sets the 32x32 sprite collision bit {this sprite, the other one}
//  At scanout the mixer PROM (index = transparency + priority of the three layers) picks the layer and flags the
//  mixer collisions; the 11-bit palette index addresses palette RAM (BBGGGRRR on System 1).
//
//  Clock: clk = 80 MHz; ce_pix = pix_phase 7 (10 MHz). One line = 5120 clocks for the next line's passes.
//
//============================================================================

module sys1_video
(
    input             clk,
    input             reset,
    input       [2:0] pix_phase,
    input             ce_pix,

    input             sys2,           // System 2 tilemap pages, scroll registers and sprite offset
    input             rowscroll,      // System 2 per-row x scroll
    input             narrow,         // visible x 16-494 (MAME sys1ppis / sys1pios)
    input             tile32k,        // tiles region 0x18000 (32K per plane), else 16K
    input             cprom,          // palette RAM value -> three 256 x 4 colour PROMs (System 2, Brain), else BBGGGRRR
    input       [3:0] spr_banks,      // sprite ROM banks of 32K: 1, 2, 4, 7 or 8 (shtngmst: bank 7 erased to FF)
    input             xhair_en,       // light gun crosshair (Shooting Master)
    input       [7:0] gun_x,          // content x / 2 (0-255)
    input       [7:0] gun_y,          // content row (0-223)
    input       [7:0] video_mode,     // [4] screen blank, [7] flip screen
    input             crt_flip,
    input       [3:0] h_adj,
    input       [3:0] v_adj,

    output reg [13:0] vram_a,
    input       [7:0] vram_d,
    output reg [14:0] tile_a,
    input       [7:0] tile_d0,        // plane at the region's first third (pixel bit 2)
    input       [7:0] tile_d1,
    input       [7:0] tile_d2,
    output reg  [8:0] sram_a,
    input       [7:0] sram_d,
    output reg [10:0] pal_a,
    input       [7:0] pal_d,
    output reg  [7:0] lut_a,
    input       [7:0] lut_d,
    output reg  [7:0] cp_a,
    input       [7:0] cp_r,
    input       [7:0] cp_g,
    input       [7:0] cp_b,

    output reg        sdr_req,
    output reg [16:0] sdr_waddr,
    input      [15:0] sdr_word,
    input             sdr_busy,

    output reg        spr_col_set,
    output reg  [9:0] spr_col_idx,
    output reg        mix_col_set,
    output reg  [5:0] mix_col_idx,

    output reg        irq_vbl,        // one clock at the start of line 224 (MAME screen_vblank)
    output reg        irq_snd,        // one clock at the start of lines 32, 96, 160, 224 (MAME soundirq_gen)
    output            fixst,          // VRAM access release for the main CPU, every 8 pixels
    output            frame_start,    // one clock: line 0 begins

    output reg  [7:0] video_r,
    output reg  [7:0] video_g,
    output reg  [7:0] video_b,
    output reg        video_hs,
    output reg        video_vs,
    output reg        video_hblank,
    output reg        video_vblank
);

    // ------------------------------------------------------------ timing
    reg [9:0] hcnt = 10'd0;
    reg [8:0] vcnt = 9'd0;
    wire       line_end = ce_pix && (hcnt == 10'd639);
    wire [8:0] v_next   = (vcnt == 9'd259) ? 9'd0 : vcnt + 9'd1;

    always @(posedge clk) begin
        irq_vbl <= 1'b0;
        irq_snd <= 1'b0;
        if (ce_pix) begin
            if (hcnt == 10'd639) begin
                hcnt <= 10'd0;
                vcnt <= v_next;
                if (v_next == 9'd224) irq_vbl <= 1'b1;
                if (v_next[5:0] == 6'd32) irq_snd <= 1'b1;
            end
            else hcnt <= hcnt + 10'd1;
        end
    end

    assign fixst = ce_pix && (hcnt[2:0] == 3'd3);
    assign frame_start = line_end && (v_next == 9'd0);

    // line rendered during the line that starts now: the one after it
    wire [8:0] r_line  = (v_next == 9'd259) ? 9'd0 : v_next + 9'd1;
    wire       r_go    = line_end && (r_line < 9'd224);
    wire       flip    = video_mode[7];
    wire       blank_m = video_mode[4];

    reg  [7:0] ry = 8'd0;               // content row being built (CRT flip shows the rows bottom-up)
    reg        rhalf = 1'b0;
    wire [7:0] r_line8  = r_line[7:0];
    wire [7:0] r_line_f = 8'd223 - r_line8;

    always @(posedge clk) if (r_go) begin
        ry    <= crt_flip ? r_line_f : r_line8;
        rhalf <= r_line[0];
    end

    // ------------------------------------------------------------ line buffers
    // fg: 2 x 256 tile pixels {color, pix}; bg: 2 x 512 output pixels; sprites: 2 x 512 {sprite, pix}, cleared at scanout
    reg  [8:0]  fgw_a;  reg [10:0] fgw_d;  reg fgw_we;
    reg  [9:0]  bgw_a;  reg [10:0] bgw_d;  reg bgw_we;
    reg  [9:0]  spw_a;  reg  [8:0] spw_d;  reg spw_we;
    wire [8:0]  spw_q;
    reg  spr_clr;
    wire [10:0] fgr_q, bgr_q;
    wire [8:0]  spr_q;

    dpram_dc #(.widthad_a(9), .width_a(11)) lb_fg
    (
        .clock_a(clk), .address_a(fgr_a), .data_a(11'd0), .wren_a(1'b0), .q_a(fgr_q),
        .clock_b(clk), .address_b(fgw_a), .data_b(fgw_d), .wren_b(fgw_we), .q_b()
    );

    dpram_dc #(.widthad_a(10), .width_a(11)) lb_bg
    (
        .clock_a(clk), .address_a(bgr_a), .data_a(11'd0), .wren_a(1'b0), .q_a(bgr_q),
        .clock_b(clk), .address_b(bgw_a), .data_b(bgw_d), .wren_b(bgw_we), .q_b()
    );

    dpram_dc #(.widthad_a(10), .width_a(9)) lb_spr
    (
        .clock_a(clk), .address_a(spr_ra), .data_a(9'd0), .wren_a(spr_clr), .q_a(spr_q),
        .clock_b(clk), .address_b(spw_a), .data_b(spw_d), .wren_b(spw_we), .q_b(spw_q)
    );

    // ------------------------------------------------------------ tile pass
    localparam [3:0] T_IDLE = 4'd0, T_SC = 4'd1, T_PIX = 4'd6, T_F0 = 4'd7, T_F1 = 4'd8, T_F2 = 4'd9, T_F3 = 4'd10;
    reg  [3:0] ts = T_IDLE;
    reg        tpass = 1'b0;            // 0 = foreground, 1 = background
    reg  [9:0] tx = 10'd0;              // fg: tile pixel 0-255, bg: output pixel 0-511
    reg [15:0] xsc_raw = 16'd0;
    reg  [7:0] ysc_raw = 8'd0;
    reg  [2:0] pg [4];                  // System 2 background pages (vram 740 / 742 / 744 / 746)
    reg  [3:0] sc_k = 4'd0;             // register read step: address k issued, byte k-2 captured
    reg  [7:0] t_lo = 8'd0;
    reg [13:0] tc_key = 14'h3FFF;       // {pass, page, row, col} of the cached tile
    reg        tc_ok = 1'b0;
    reg  [7:0] tc_color = 8'd0;
    reg  [7:0] tc_p0 = 8'd0, tc_p1 = 8'd0, tc_p2 = 8'd0;

    // scroll / page register addresses: System 1 FFC FFD FBD; System 2 x lo, x hi, y, 740, 742, 744, 746
    wire  [5:0] rs2    = {ry[7:3], 1'b0};                       // row scroll: 2 bytes per 8-line row
    wire [13:0] x2_lo  = flip ? (rowscroll ? 14'h07FE - {8'd0, rs2} : 14'h07F6) : (rowscroll ? 14'h07C0 + {8'd0, rs2} : 14'h07C0);
    wire [13:0] x2_hi  = flip ? (rowscroll ? 14'h07FF - {8'd0, rs2} : 14'h07F7) : (rowscroll ? 14'h07C1 + {8'd0, rs2} : 14'h07C1);
    wire  [3:0] sc_last = sys2 ? 4'd6 : 4'd2;
    reg  [13:0] sc_addr;
    always_comb begin
        case (sc_k)
            4'd0:    sc_addr = sys2 ? x2_lo : 14'h0FFC;
            4'd1:    sc_addr = sys2 ? x2_hi : 14'h0FFD;
            4'd2:    sc_addr = sys2 ? (flip ? 14'h0784 : 14'h07BA) : 14'h0FBD;
            4'd3:    sc_addr = 14'h0740;
            4'd4:    sc_addr = 14'h0742;
            4'd5:    sc_addr = 14'h0744;
            default: sc_addr = 14'h0746;
        endcase
    end

    // System 1 scroll: x = vram[FFD:FFC] + 28 (signed), y = vram[FBD]; flipped as MAME (640 - x, 764 - y, mod 512)
    wire [15:0] xsc_n  = xsc_raw + 16'd28;
    wire  [8:0] xsc_n9 = xsc_n[8:0];
    wire  [9:0] xsc_f  = 10'd640 - {1'b0, xsc_n9};
    wire [17:0] xs1    = flip ? {8'd0, xsc_f} : {{2{xsc_n[15]}}, xsc_n};
    wire  [9:0] ysc_f  = 10'd764 - {2'b00, ysc_raw};
    wire  [8:0] ys1    = flip ? ysc_f[8:0] : {1'b0, ysc_raw};
    // System 2 scroll: x = (raw & 1FF) - 512 + 10, flipped 1536 - (raw & 1FF); y = raw, flipped 1024 - raw (mod 512)
    wire [17:0] xs2    = flip ? 18'd1536 - {9'd0, xsc_raw[8:0]} : {9'd0, xsc_raw[8:0]} - 18'd502;
    wire  [8:0] ys2    = flip ? 9'd0 - {1'b0, ysc_raw} : {1'b0, ysc_raw};
    wire [17:0] xs     = sys2 ? xs2 : xs1;
    wire  [8:0] ys     = sys2 ? ys2 : ys1;

    // background coordinates of output pixel tx: bgx = trunc((x - xs) / 2) & 0x1FF (C division), bgy = (y + ys) & 0x1FF
    wire [17:0] bg_d   = {8'd0, tx} - xs;
    wire [17:0] bg_q   = {bg_d[17], bg_d[17:1]} + {17'd0, bg_d[17] & bg_d[0]};
    wire  [8:0] bgy    = {1'b0, ry} + ys;
    wire  [7:0] fx     = tx[7:0];

    // logical tile-map position (u, v) inside the 256x256 page; a flipped page is mirrored in both axes
    wire  [7:0] u_raw  = tpass ? bg_q[7:0] : fx;
    wire  [7:0] v_raw  = tpass ? bgy[7:0]  : ry;
    wire  [7:0] u      = flip ? ~u_raw : u_raw;
    wire  [7:0] v      = flip ? ~v_raw : v_raw;
    // page: System 1 fg 1, bg 0; System 2 fg 0, bg one of 4 by the unflipped 512 x 512 quadrant {bgy[8], bgx[8]}
    wire  [2:0] pg_sel = pg[{bgy[8], bg_q[8]}];
    wire  [2:0] t_page = tpass ? (sys2 ? pg_sel : 3'd0) : (sys2 ? 3'd0 : 3'd1);
    wire [13:0] t_key  = {tpass, t_page, v[7:3], u[7:3]};
    wire [13:0] t_vram = {t_page, v[7:3], u[7:3], 1'b0};
    wire  [2:0] t_bit  = ~u[2:0];                                // MSB = leftmost pixel
    wire  [2:0] t_pix  = {tc_p0[t_bit], tc_p1[t_bit], tc_p2[t_bit]};
    wire [10:0] t_val  = {tc_color, t_pix};

    always @(posedge clk) begin
        fgw_we <= 1'b0;
        bgw_we <= 1'b0;
        if (reset) begin
            ts <= T_IDLE; tc_ok <= 1'b0;
        end
        else if (r_go) begin
            ts <= T_SC; sc_k <= 4'd0; tc_ok <= 1'b0;
        end
        else case (ts)
            T_SC: begin
                // address registered, RAM registered: the byte for address k is here at step k + 2
                if (sc_k <= sc_last) vram_a <= sc_addr;
                case (sc_k)
                    4'd2: xsc_raw[7:0]  <= vram_d;
                    4'd3: xsc_raw[15:8] <= vram_d;
                    4'd4: ysc_raw       <= vram_d;
                    4'd5: pg[0]         <= vram_d[2:0];
                    4'd6: pg[1]         <= vram_d[2:0];
                    4'd7: pg[2]         <= vram_d[2:0];
                    4'd8: pg[3]         <= vram_d[2:0];
                    default: ;
                endcase
                sc_k <= sc_k + 4'd1;
                if (sc_k == sc_last + 4'd2) begin tpass <= 1'b0; tx <= 10'd0; ts <= T_PIX; end
            end
            T_PIX: begin
                if (!tc_ok || tc_key != t_key) begin
                    vram_a <= t_vram;
                    tc_key <= t_key;
                    ts     <= T_F0;
                end
                else begin
                    if (!tpass) begin fgw_a <= {rhalf, fx}; fgw_d <= t_val; fgw_we <= 1'b1; end
                    else        begin bgw_a <= {rhalf, tx[8:0]}; bgw_d <= t_val; bgw_we <= 1'b1; end
                    if (!tpass && tx == 10'd255) begin tpass <= 1'b1; tx <= 10'd0; tc_ok <= 1'b0; end
                    else if (tpass && tx == 10'd511) ts <= T_IDLE;
                    else tx <= tx + 10'd1;
                end
            end
            T_F0: begin vram_a <= vram_a + 14'd1; ts <= T_F1; end
            T_F1: begin t_lo <= vram_d; ts <= T_F2; end
            T_F2: begin
                // tiledata = {hi, lo}: code = {D15, D10-D0}, color = D12-D5 (MAME tile_get_info); 16K planes hold 2048 tiles
                tile_a   <= {tile32k & vram_d[7], vram_d[2:0], t_lo, v[2:0]};
                tc_color <= {vram_d[4:0], t_lo[7:5]};
                ts       <= T_F3;
            end
            T_F3: ts <= 4'd11;
            4'd11: begin tc_p0 <= tile_d0; tc_p1 <= tile_d1; tc_p2 <= tile_d2; tc_ok <= 1'b1; ts <= T_PIX; end
            default: ts <= T_IDLE;
        endcase
    end

    // ------------------------------------------------------------ sprite pass
    localparam [3:0] S_IDLE = 4'd0, S_HDR = 4'd1, S_CHK = 4'd2, S_ROW = 4'd3, S_FETCH = 4'd4, S_WAIT = 4'd5,
                     S_NIB = 4'd6, S_PRD = 4'd7, S_PWR = 4'd8, S_NEXT = 4'd9, S_MUL = 4'd10;
    reg  [3:0] ss = S_IDLE;
    reg  [4:0] sn = 5'd0;               // sprite number
    reg  [3:0] sk = 4'd0;               // header byte counter
    reg  [7:0] sh [8];                  // header bytes 0-7
    reg  [8:0] s_top = 9'd0, s_bot = 9'd0;
    reg  [8:0] s_row1 = 9'd0;           // rows from the top + 1
    reg [16:0] s_cur = 17'd0;           // current ROM address (MAME int curaddr: bit 15 picks the nibble order)
    reg        s_dir = 1'b0;            // 1 = address counts down (MAME: srcaddr bit 15)
    reg [10:0] s_x = 11'd0;
    reg  [2:0] s_bank = 3'd0;
    reg  [7:0] s_byte = 8'd0;
    reg        s_nib = 1'b0;            // 0 = first nibble (pixels x, x+1), 1 = second (x+2, x+3)
    reg        s_pi = 1'b0;             // pixel within the nibble
    reg [15:0] cw = 16'd0;              // last SDRAM word taken
    reg [16:0] cw_a = 17'h1FFFF;
    reg        cw_ok = 1'b0;

    wire  [8:0] hdr_top  = {1'b0, sh[0]} + 9'd1;
    wire  [8:0] hdr_bot  = {1'b0, sh[1]} + 9'd1;
    wire  [8:0] fl_top   = 9'd256 - hdr_bot;
    wire  [8:0] fl_bot   = 9'd256 - hdr_top;
    wire  [2:0] hdr_bank = {sh[3][5], sh[3][6], sh[3][7]};
    wire  [2:0] bank_m   = (spr_banks == 4'd8) ? hdr_bank :
                           (spr_banks == 4'd7) ? ((hdr_bank == 3'd7) ? 3'd0 : hdr_bank) :
                           (spr_banks == 4'd4) ? {1'b0, hdr_bank[1:0]} :
                           (spr_banks == 4'd2) ? {2'b00, hdr_bank[0]} : 3'd0;
    wire [15:0] s_src    = {sh[7], sh[6]};
    wire [15:0] s_stride = {sh[5], sh[4]};
    wire [24:0] s_prod   = s_stride * s_row1;
    wire [15:0] s_addr   = s_src + s_prod[15:0];

    wire [17:0] s_b      = {s_bank, s_cur[14:0]};                 // byte address in the 256K sprite window
    wire [16:0] s_w      = s_b[17:1];
    wire [16:0] s_cur_n  = s_dir ? s_cur - 17'd1 : s_cur + 17'd1;
    wire [17:0] s_b_n    = {s_bank, s_cur_n[14:0]};
    wire  [7:0] s_bsel   = s_b[0] ? cw[15:8] : cw[7:0];
    wire  [3:0] s_c1     = s_cur[15] ? s_byte[3:0] : s_byte[7:4];
    wire  [3:0] s_c2     = s_cur[15] ? s_byte[7:4] : s_byte[3:0];
    wire  [3:0] s_c      = s_nib ? s_c2 : s_c1;
    wire [10:0] s_px     = s_x + {8'd0, s_nib, 1'b0} + {10'd0, s_pi};
    wire [10:0] s_effx   = flip ? (11'h1FE - s_px) : s_px;
    wire        s_vis    = !s_effx[10] && !s_effx[9] &&
                           (!narrow || (s_effx[8:0] >= 9'd16 && s_effx[8:0] <= 9'd494));
    wire  [8:0] ry9      = {1'b0, ry};
    wire [10:0] s_xoff   = !sys2 ? 11'd0 : flip ? 11'h7F2 : 11'd14;   // MAME sprxoffset: System 2 +14 / -14 flipped

    always @(posedge clk) begin
        spr_col_set <= 1'b0;
        spw_we      <= 1'b0;
        if (reset) begin
            ss <= S_IDLE; sdr_req <= 1'b0; cw_ok <= 1'b0;
        end
        else if (r_go) begin
            // a pass still running when the next line starts is cut short, as the hardware runs out of line time
            ss <= blank_m ? S_IDLE : S_HDR; sn <= 5'd0; sk <= 4'd0; sram_a <= 9'd0; sdr_req <= 1'b0;
        end
        else case (ss)
            S_HDR: begin
                // header byte k at {n, k}: address registered, RAM registered, so byte k-1 is here at count k
                if (sk != 4'd0) sh[sk[2:0] - 3'd1] <= sram_d;
                sram_a <= {sn, 1'b0, sk[2:0] + 3'd1};
                sk     <= sk + 4'd1;
                if (sk == 4'd8) ss <= S_CHK;
            end
            S_CHK: begin
                if (sh[0] == 8'hFF) begin ss <= S_IDLE; sdr_req <= 1'b0; end   // MAME: 0xFF in byte 0 ends the list
                else begin
                    s_top <= flip ? fl_top : hdr_top;
                    s_bot <= flip ? fl_bot : hdr_bot;
                    ss    <= S_ROW;
                end
            end
            S_ROW: begin
                if (ry9 >= s_top && ry9 < s_bot) begin
                    s_row1 <= ry9 - s_top + 9'd1;
                    s_bank <= bank_m;
                    s_x    <= {2'b00, sh[3][0], sh[2]} + s_xoff;
                    ss     <= S_MUL;
                end
                else ss <= S_NEXT;
            end
            S_MUL: begin
                s_cur <= {1'b0, s_addr};
                s_dir <= s_addr[15];
                ss    <= S_FETCH;
            end
            S_FETCH: begin
                if (s_x[10:9] == 2'b01) ss <= S_NEXT;             // past x 511: nothing more can be visible
                else if (cw_ok && cw_a == s_w) begin
                    s_byte    <= s_bsel;
                    s_nib     <= 1'b0;
                    s_pi      <= 1'b0;
                    sdr_req   <= 1'b1;                            // prefetch the word holding the next byte
                    sdr_waddr <= s_b_n[17:1];
                    ss        <= S_NIB;
                end
                else begin
                    sdr_req   <= 1'b1;
                    sdr_waddr <= s_w;
                    ss        <= S_WAIT;
                end
            end
            S_WAIT: if (!sdr_busy) begin cw <= sdr_word; cw_a <= sdr_waddr; cw_ok <= 1'b1; ss <= S_FETCH; end
            S_NIB: begin
                if (s_c == 4'hF) ss <= S_NEXT;                    // end of this sprite row
                else if (s_c != 4'h0 && s_vis) begin
                    spw_a <= {rhalf, s_effx[8:0]};
                    ss    <= S_PRD;
                end
                else if (s_c != 4'h0 && !s_pi) s_pi <= 1'b1;
                else if (!s_nib) begin s_nib <= 1'b1; s_pi <= 1'b0; end
                else begin s_x <= s_x + 11'd4; s_cur <= s_cur_n; ss <= S_FETCH; end
            end
            S_PRD: ss <= S_PWR;
            S_PWR: begin
                if (spw_q[3:0] != 4'd0) begin
                    spr_col_set <= 1'b1;
                    spr_col_idx <= {sn, spw_q[8:4]};
                end
                spw_d  <= {sn, s_c};
                spw_we <= 1'b1;
                ss     <= S_NIB;
                if (!s_pi) s_pi <= 1'b1;
                else if (!s_nib) begin s_nib <= 1'b1; s_pi <= 1'b0; end
                else begin s_x <= s_x + 11'd4; s_cur <= s_cur_n; ss <= S_FETCH; end
            end
            S_NEXT: begin
                if (sn == 5'd31) begin ss <= S_IDLE; sdr_req <= 1'b0; end
                else begin sn <= sn + 5'd1; sk <= 4'd0; sram_a <= {sn + 5'd1, 4'd0}; ss <= S_HDR; end
            end
            default: begin ss <= S_IDLE; sdr_req <= 1'b0; end
        endcase
    end

    // ------------------------------------------------------------ scanout + mixer
    // The line buffers are addressed straight from the counters (the RAM sees the address from phase 0), so the
    // chain line buffers -> mixer PROM -> palette RAM -> colour PROMs fits in one 8-clock pixel slot.
    wire [8:0] x_raw  = hcnt[8:0];
    wire [8:0] x_disp = crt_flip ? ~x_raw : x_raw;
    wire [8:0] fgr_a  = {vcnt[0], x_disp[8:1]};
    wire [9:0] bgr_a  = {vcnt[0], x_disp};
    wire [9:0] spr_ra = {vcnt[0], x_disp};
    wire       vis_x  = !narrow || (x_disp >= 9'd16 && x_disp <= 9'd494);
    wire       active = (hcnt < 10'd512) && (vcnt < 9'd224);
    reg [10:0] m_fg = 11'd0, m_bg = 11'd0;
    reg  [8:0] m_sp = 9'd0;
    // valid in phase 7: palette RAM (since phase 5) and the colour PROMs (address from phase 5)
    wire [7:0] rgb_r = cprom ? w4(cp_r[3:0]) : w3(pal_d[2:0]);
    wire [7:0] rgb_g = cprom ? w4(cp_g[3:0]) : w3(pal_d[5:3]);
    wire [7:0] rgb_b = cprom ? w4(cp_b[3:0]) : w2(pal_d[7:6]);

    // MAME compute_resistor_weights: 3 bits (995 / 495 / 250 ohm) and 2 bits (495 / 250 ohm), auto-scaled
    function automatic [7:0] w3(input [2:0] c);
        case (c)
            3'd0: w3 = 8'd0;   3'd1: w3 = 8'd36;  3'd2: w3 = 8'd73;  3'd3: w3 = 8'd110;
            3'd4: w3 = 8'd145; 3'd5: w3 = 8'd182; 3'd6: w3 = 8'd219; default: w3 = 8'd255;
        endcase
    endfunction
    // MAME system1_palette with colour PROMs: 0x0e / 0x1f / 0x43 / 0x8f per bit
    function automatic [7:0] w4(input [3:0] c);
        w4 = (c[0] ? 8'h0E : 8'h00) + (c[1] ? 8'h1F : 8'h00) + (c[2] ? 8'h43 : 8'h00) + (c[3] ? 8'h8F : 8'h00);
    endfunction
    function automatic [7:0] w2(input [1:0] c);
        case (c)
            2'd0: w2 = 8'd0; 2'd1: w2 = 8'd86; 2'd2: w2 = 8'd169; default: w2 = 8'd255;
        endcase
    endfunction

    always @(posedge clk) begin
        spr_clr     <= 1'b0;
        mix_col_set <= 1'b0;
        case (pix_phase)
            3'd1: begin
                m_fg  <= fgr_q;
                m_bg  <= bgr_q;
                m_sp  <= spr_q;
                // lookup index: [0] sprite transparent, [1] fg transparent, [3:2] fg priority, [4] bg transparent,
                // [6:5] bg priority
                lut_a <= {1'b0, bgr_q[10:9], bgr_q[2:0] == 3'd0, fgr_q[10:9], fgr_q[2:0] == 3'd0, spr_q[3:0] == 4'd0};
                spr_clr <= active;                               // the line is shown once: clear behind the beam
            end
            3'd3: begin
                if (blank_m) pal_a <= 11'd0;                      // MAME fills the screen with pen 0
                else case (lut_d[1:0])
                    2'd0:    pal_a <= {2'b00, m_sp};
                    2'd1:    pal_a <= {2'b01, m_fg[8:0]};
                    default: pal_a <= {2'b10, m_bg[8:0]};
                endcase
                if (active && vis_x && !blank_m && !lut_d[2]) begin
                    mix_col_set <= 1'b1;
                    mix_col_idx <= {lut_d[3], m_sp[8:4]};
                end
            end
            3'd5: cp_a <= pal_d;
            default: ;
        endcase
    end

    // sync in the middle of blanking with porches on both sides; H/V position (CRT) moves it
    wire [9:0] hs_start = 10'd544 + {{5{h_adj[3]}}, h_adj, 1'b0};
    wire [8:0] vs_start = 9'd234 + {{5{v_adj[3]}}, v_adj};

    // crosshair: 9 rows x 18 pixels (9 at half resolution), centred on the gun, in content space (follows CRT flip)
    wire [7:0] xh_row = crt_flip ? 8'd223 - vcnt[7:0] : vcnt[7:0];
    wire [7:0] xh_col = x_disp[8:1];
    wire       xh     = xhair_en && ((xh_col == gun_x && xh_row + 8'd4 >= gun_y && xh_row <= gun_y + 8'd4) ||
                                     (xh_row == gun_y && xh_col + 8'd4 >= gun_x && xh_col <= gun_x + 8'd4));

    always @(posedge clk) if (ce_pix) begin
        video_r      <= (active && vis_x) ? (xh ? 8'hFF : rgb_r) : 8'd0;
        video_g      <= (active && vis_x) ? (xh ? 8'hFF : rgb_g) : 8'd0;
        video_b      <= (active && vis_x) ? (xh ? 8'hFF : rgb_b) : 8'd0;
        video_hblank <= (hcnt >= 10'd512) || !vis_x;
        video_vblank <= vcnt >= 9'd224;
        video_hs     <= (hcnt >= hs_start) && (hcnt < hs_start + 10'd48);
        video_vs     <= (vcnt >= vs_start) && (vcnt < vs_start + 9'd3);
    end

endmodule
