//============================================================================
//
//  Sega System 1 / System 2 board dispatcher
//  Copyright (C) 2026 Rodimus
//
//  One rbf for every sega/system1.cpp set; the board features are chosen at run time by MRA index-1 bytes 148-157
//  (tools/mra_cfg.py). Shared CPUs / sound chips and ROM storage live here (chip pool convention,
//  rtl/chip_pool_pkg.sv); the board modules in rtl/boards/sys1/ drive them through bus ports.
//
//  Boards so far: System 1 main board (PIO and PPI variants), video (System 1 tilemaps + sprites), sound board.
//
//  Clock: clk = 80 MHz. ce_pix = clk / 8 = 10 MHz (20 MHz / 2). Main Z80: sys1_main (20 MHz / 5, / 6 in M1).
//  Sound: 4 MHz and 2 MHz enables = 8 MHz / 2 and / 4.
//
//  ROM storage, sized for the largest single set in the driver (tools/rom_scope.py, 2026-10-19):
//    rom0  64K  main CPU region 0x00000-0x07FFF, and 0x20000-0x27FFF (wbml bootleg opcode half)
//    rom1 128K  main CPU region 0x08000-0x0FFFF or 0x10000-0x1FFFF (banks), and the same for the 0x20000 half
//    rom2  32K  sound CPU                rom3 4K  mixer PROM, bootleg decryption PROM; rom7-9 colour PROMs
//    rom4-rom6  32K each: tile planes    sprites (up to 224K): SDRAM (rtl/ram_rom/sys1_sdrom.sv)
//  ioctl index 0 (tools/make_mra.py): 00000 rom0 | 20000 rom1 | 40000 sound | 48000 PROMs | 4A000 MC-8123 key |
//                                     4C000 8751 | 50000 / 58000 / 60000 tile planes | 80000 sprites (SDRAM)
//
//============================================================================

module segasys1_board
(
    input             clk,
    input             reset,
    input             pause,

    input       [7:0] cfg_board,      // byte 148: [0] PIO, [1] System 2, [2] row scroll, [3] narrow, [4] MCU, [5] colour PROMs
    input       [7:0] cfg_crypt,      // byte 149
    input       [7:0] cfg_opc,        // byte 150
    input       [7:0] cfg_bank,       // byte 151
    input       [7:0] cfg_tiles,      // byte 152
    input       [7:0] cfg_sprbanks,   // byte 153
    input       [7:0] cfg_io,         // byte 154: [0] dial (handled at the top), [1] Block Gal I/O map
    input      [55:0] mj_rows,        // mahjong key rows 0-6 (byte n = row n), active low
    input       [7:0] cfg_sndclk,
    input       [7:0] cfg_scramble,   // byte 156: [0] myherok ROM line swaps
    input       [7:0] gun_x,          // light gun, content x / 2 and row (Shooting Master)
    input       [7:0] gun_y,
    input             gun_trig,
    input             xhair_en,     // byte 155: 0 4 / 2 / 4 MHz, 1 2.5 / 2.5 / 1.25, 2 2.5 / 2.5 / 2.5, 3 4 / 2 / 2

    input       [7:0] in_p1,
    input       [7:0] in_p2,
    input       [7:0] in_sys,
    input       [7:0] in_swa,
    input       [7:0] in_swb,

    input      [24:0] ioctl_addr,
    input       [7:0] ioctl_dout,
    input             ioctl_wr0,

    input             crt_flip,
    input       [3:0] h_adj,
    input       [3:0] v_adj,

    output            ce_pix,
    output      [7:0] video_r,
    output      [7:0] video_g,
    output      [7:0] video_b,
    output            video_hs,
    output            video_vs,
    output            video_hblank,
    output            video_vblank,

    output signed [15:0] audio,

    // sprite ROM words in SDRAM (rtl/ram_rom/sys1_sdrom.sv)
    output            sdr_req,
    output     [16:0] sdr_waddr,
    input      [15:0] sdr_word,
    input             sdr_busy,

    input      [11:0] hs_address,
    input       [7:0] hs_data_in,
    output      [7:0] hs_data_out,
    input             hs_write
);

    // ------------------------------------------------------------ CPU start
    // The CPUs leave reset at the start of a frame, as MAME powers on with the CPUs and the video counters together.
    // The video timing keeps running through a reset (no sync loss during a ROM load); the game's startup races
    // between its main loop and the vblank IRQ (the Pitfall II title logo) depend on this phase.
    reg  run = 1'b0;
    wire frame_start;
    always @(posedge clk) begin
        if (reset) run <= 1'b0;
        else if (frame_start) run <= 1'b1;
    end
    wire cpu_reset = reset | ~run;

    // ------------------------------------------------------------ clock enables
    reg [2:0] pix_phase = 3'd0;
    always @(posedge clk) pix_phase <= pix_phase + 3'd1;
    assign ce_pix = (pix_phase == 3'd7);

    reg [5:0] sphase = 6'd0;                        // 80 MHz / 40
    always @(posedge clk) sphase <= (sphase == 6'd39) ? 6'd0 : sphase + 6'd1;
    wire ce_4m = (sphase == 6'd19) || (sphase == 6'd39);
    wire ce_2m = (sphase == 6'd39);
    reg [5:0] lphase = 6'd0;                        // 80 MHz / 64 (Star Jacker, Up'n Down: 20 MHz / 8 and / 16)
    always @(posedge clk) lphase <= lphase + 6'd1;
    wire ce_2m5  = (lphase[4:0] == 5'd31);
    wire ce_1m25 = (lphase == 6'd63);

    // sound CPU, SN #1, SN #2 clock enables per MAME machine (byte 155)
    reg ce_scpu, ce_sn1, ce_sn2;
    always_comb begin
        case (cfg_sndclk[1:0])
            2'd1:    begin ce_scpu = ce_2m5; ce_sn1 = ce_2m5; ce_sn2 = ce_1m25; end
            2'd2:    begin ce_scpu = ce_2m5; ce_sn1 = ce_2m5; ce_sn2 = ce_2m5;  end
            2'd3:    begin ce_scpu = ce_4m;  ce_sn1 = ce_2m;  ce_sn2 = ce_2m;   end
            default: begin ce_scpu = ce_4m;  ce_sn1 = ce_2m;  ce_sn2 = ce_4m;   end
        endcase
    end

    /////////////////////////////  Shared CPU / sound chip pool  /////////////////////////////

    // z80_0: main CPU (every board)
    chip_pool_pkg::z80_i_t main_z80_0_i, z80_0_i;
    chip_pool_pkg::z80_o_t z80_0_o;
    assign z80_0_i = main_z80_0_i;

    wire [15:0] z80_0_a;
    wire  [7:0] z80_0_do;
    wire        z80_0_m1_n, z80_0_mreq_n, z80_0_iorq_n, z80_0_rd_n, z80_0_wr_n, z80_0_rfsh_n;
    wire        z80_0_busrq_n, z80_0_busak_n;           // 8751 bus request (main CPU only, outside the pool struct)

    T80s #(.Mode(0), .T2Write(1), .IOWait(1)) z80_0
    (
        .RESET_n(~cpu_reset), .CLK(clk), .CEN(z80_0_i.ce), .WAIT_n(1'b1), .INT_n(z80_0_i.int_n), .NMI_n(z80_0_i.nmi_n), .BUSRQ_n(z80_0_busrq_n),
        .M1_n(z80_0_m1_n), .MREQ_n(z80_0_mreq_n), .IORQ_n(z80_0_iorq_n), .RD_n(z80_0_rd_n), .WR_n(z80_0_wr_n), .RFSH_n(z80_0_rfsh_n),
        .HALT_n(), .BUSAK_n(z80_0_busak_n), .A(z80_0_a), .DI(z80_0_i.di), .DO(z80_0_do)
    );

    assign z80_0_o.a = z80_0_a;           assign z80_0_o.dout = z80_0_do;
    assign z80_0_o.m1_n = z80_0_m1_n;     assign z80_0_o.mreq_n = z80_0_mreq_n; assign z80_0_o.iorq_n = z80_0_iorq_n;
    assign z80_0_o.rd_n = z80_0_rd_n;     assign z80_0_o.wr_n = z80_0_wr_n;     assign z80_0_o.rfsh_n = z80_0_rfsh_n;

    // z80_1: sound CPU (every board)
    chip_pool_pkg::z80_i_t snd_z80_1_i, z80_1_i;
    chip_pool_pkg::z80_o_t z80_1_o;
    assign z80_1_i = snd_z80_1_i;

    wire [15:0] z80_1_a;
    wire  [7:0] z80_1_do;
    wire        z80_1_m1_n, z80_1_mreq_n, z80_1_iorq_n, z80_1_rd_n, z80_1_wr_n, z80_1_rfsh_n;

    T80s #(.Mode(0), .T2Write(1), .IOWait(1)) z80_1
    (
        .RESET_n(~cpu_reset), .CLK(clk), .CEN(z80_1_i.ce), .WAIT_n(1'b1), .INT_n(z80_1_i.int_n), .NMI_n(z80_1_i.nmi_n), .BUSRQ_n(1'b1),
        .M1_n(z80_1_m1_n), .MREQ_n(z80_1_mreq_n), .IORQ_n(z80_1_iorq_n), .RD_n(z80_1_rd_n), .WR_n(z80_1_wr_n), .RFSH_n(z80_1_rfsh_n),
        .HALT_n(), .BUSAK_n(), .A(z80_1_a), .DI(z80_1_i.di), .DO(z80_1_do)
    );

    assign z80_1_o.a = z80_1_a;           assign z80_1_o.dout = z80_1_do;
    assign z80_1_o.m1_n = z80_1_m1_n;     assign z80_1_o.mreq_n = z80_1_mreq_n; assign z80_1_o.iorq_n = z80_1_iorq_n;
    assign z80_1_o.rd_n = z80_1_rd_n;     assign z80_1_o.wr_n = z80_1_wr_n;     assign z80_1_o.rfsh_n = z80_1_rfsh_n;

    // sn76489[0]: sound board SN #1 (2 MHz), sn76489[1]: SN #2 (4 MHz)
    chip_pool_pkg::sn76489_i_t [1:0] snd_sn76489_i, sn76489_i;
    chip_pool_pkg::sn76489_o_t [1:0] sn76489_o;
    assign sn76489_i = snd_sn76489_i;

    genvar si;
    generate
        for (si = 0; si < 2; si = si + 1) begin : g_sn76489
            wire [10:0] sound;
            wire        ready;
            jt89 sn76489
            (
                .rst(sn76489_i[si].rst), .clk(clk), .clk_en(sn76489_i[si].ce), .wr_n(sn76489_i[si].wr_n),
                .cs_n(sn76489_i[si].cs_n), .din(sn76489_i[si].din), .sound(sound), .ready(ready)
            );
            assign sn76489_o[si].sound = sound; assign sn76489_o[si].ready = ready;
        end
    endgenerate

    /////////////////////////////  Shared ROM blocks  /////////////////////////////

    // main CPU region, downloaded in storage order (tools/make_mra.py main_store): ioctl 00000-0FFFF -> rom0,
    // 20000-3FFFF -> rom1. Every byte has its own download address, so the MRA's gap fills never land on a ROM.
    wire        load_r0 = ioctl_wr0 && (ioctl_addr[24:16] == 9'd0);
    wire        load_r1 = ioctl_wr0 && (ioctl_addr[24:17] == 8'd1);

    wire [15:0] rom0_a;
    wire  [7:0] rom0_d;
    dpram_dc #(.widthad_a(16)) rom0
    (
        .clock_a(clk), .address_a(ioctl_addr[15:0]), .data_a(ioctl_dout), .wren_a(load_r0), .q_a(),
        .clock_b(clk), .address_b(rom0_a), .data_b(8'h00), .wren_b(1'b0), .q_b(rom0_d)
    );

    wire [16:0] rom1_a;
    wire  [7:0] rom1_d;
    dpram_dc #(.widthad_a(17)) rom1
    (
        .clock_a(clk), .address_a(ioctl_addr[16:0]), .data_a(ioctl_dout), .wren_a(load_r1), .q_a(),
        .clock_b(clk), .address_b(rom1_a), .data_b(8'h00), .wren_b(1'b0), .q_b(rom1_d)
    );

    // rom2: sound CPU, ioctl 40000-47FFF
    wire        load_r2 = ioctl_wr0 && (ioctl_addr[24:15] == 10'd8);
    wire [14:0] rom2_a;
    wire  [7:0] rom2_d;
    dpram_dc #(.widthad_a(15)) rom2
    (
        .clock_a(clk), .address_a(ioctl_addr[14:0]), .data_a(ioctl_dout), .wren_a(load_r2), .q_a(),
        .clock_b(clk), .address_b(rom2_a), .data_b(8'h00), .wren_b(1'b0), .q_b(rom2_d)
    );

    // rom3: PROMs, ioctl 48000-48FFF (000 mixer lookup, 400 / 500 / 600 colour R G B, 800 decryption)
    wire        load_r3 = ioctl_wr0 && (ioctl_addr[24:12] == 13'h048);
    wire [11:0] rom3_a;
    wire  [7:0] rom3_d;
    dpram_dc #(.widthad_a(12)) rom3
    (
        .clock_a(clk), .address_a(ioctl_addr[11:0]), .data_a(ioctl_dout), .wren_a(load_r3), .q_a(),
        .clock_b(clk), .address_b(rom3_a), .data_b(8'h00), .wren_b(1'b0), .q_b(rom3_d)
    );

    // MC-8123 key, ioctl 4A000-4BFFF (8K), addressed by the main board's decrypt
    wire        load_key = ioctl_wr0 && (ioctl_addr[24:13] == 12'h025);
    wire [12:0] key_a;
    wire  [7:0] key_d;
    dpram_dc #(.widthad_a(13)) mckey
    (
        .clock_a(clk), .address_a(ioctl_addr[12:0]), .data_a(ioctl_dout), .wren_a(load_key), .q_a(),
        .clock_b(clk), .address_b(key_a), .data_b(8'h00), .wren_b(1'b0), .q_b(key_d)
    );

    // rom7-rom9: colour PROMs R, G, B (ioctl 48400 / 48500 / 48600), read in parallel at scanout
    wire [7:0] cp_a;
    wire [7:0] cp_d [3];
    genvar ci;
    generate
        for (ci = 0; ci < 3; ci = ci + 1) begin : g_cprom
            wire load_c = ioctl_wr0 && (ioctl_addr[24:8] == 17'h00484 + ci);
            dpram_dc #(.widthad_a(8)) rom
            (
                .clock_a(clk), .address_a(ioctl_addr[7:0]), .data_a(ioctl_dout), .wren_a(load_c), .q_a(),
                .clock_b(clk), .address_b(cp_a), .data_b(8'h00), .wren_b(1'b0), .q_b(cp_d[ci])
            );
        end
    endgenerate

    // rom4-rom6: tile planes, ioctl 50000 / 58000 / 60000 (32K slots; 16K planes fill the lower half)
    // myherok (MAME init_myherok, byte 156 [0]): the three tile ROMs have A4 / A5 swapped and data lines D0 / D6
    // (planes 0, 2) or D1 / D5 (plane 1) swapped; undone here on the read side
    wire [14:0] tile_a;
    wire  [7:0] tile_d [3];
    wire  [7:0] tile_q [3];
    wire        scr_mh = cfg_scramble[0];
    wire [14:0] tile_as = scr_mh ? {tile_a[14:6], tile_a[4], tile_a[5], tile_a[3:0]} : tile_a;
    assign tile_d[0] = scr_mh ? {tile_q[0][7], tile_q[0][0], tile_q[0][5:1], tile_q[0][6]} : tile_q[0];
    assign tile_d[1] = scr_mh ? {tile_q[1][7:6], tile_q[1][1], tile_q[1][4:2], tile_q[1][5], tile_q[1][0]} : tile_q[1];
    assign tile_d[2] = scr_mh ? {tile_q[2][7], tile_q[2][0], tile_q[2][5:1], tile_q[2][6]} : tile_q[2];
    genvar ti;
    generate
        for (ti = 0; ti < 3; ti = ti + 1) begin : g_tile
            wire load_t = ioctl_wr0 && (ioctl_addr[24:15] == 10'd10 + ti);
            dpram_dc #(.widthad_a(15)) rom
            (
                .clock_a(clk), .address_a(ioctl_addr[14:0]), .data_a(ioctl_dout), .wren_a(load_t), .q_a(),
                .clock_b(clk), .address_b(tile_as), .data_b(8'h00), .wren_b(1'b0), .q_b(tile_q[ti])
            );
        end
    endgenerate

    /////////////////////////////  Boards  /////////////////////////////

    wire [13:0] vram_va;
    wire  [7:0] vram_vd;
    wire  [8:0] sram_va;
    wire  [7:0] sram_vd;
    wire [10:0] pal_va;
    wire  [7:0] pal_vd;
    wire  [7:0] lut_a;
    wire        spr_col_set, mix_col_set, irq_vbl, irq_snd, fixst;
    wire  [9:0] spr_col_idx;
    wire  [5:0] mix_col_idx;
    wire  [7:0] video_mode, snd_latch;
    wire        snd_nmi, snd_mute, snd_astb_n;

    assign rom3_a = {4'h0, lut_a};

    // ------------------------------------------------------------ 8751 MCU (jotego jtframe_8751mcu, Oregano mc8051)
    // 8 MHz (MAME SOUND_CLOCK), divided by 12 inside. ROM 4K at ioctl 4C000. MAME system1 mcu(): INT0 = vblank,
    // INT1 = video-mode bit 6 (low asserts), T0 clocked by the Z80's interrupt acknowledge, P1 = control.
    wire        cfg_mcu = cfg_board[4];
    wire        mcu_rst = cpu_reset | ~cfg_mcu;
    wire  [7:0] mcu_p1;
    wire        mcu_acc, mcu_wr, intack;
    wire [15:0] mcu_addr;
    wire  [7:0] mcu_xo, mcu_xi;
    // 80 MHz / 9 = 8.89 MHz, not / 10: the Oregano core needs more clocks per instruction than a real 8751 (jotego's
    // note); at 8 MHz Choplifter's start-up ROM check ran 551 MOVX per frame against MAME's 619 and released the Z80
    // ~40 frames late
    reg   [3:0] c8 = 4'd0;
    always @(posedge clk) c8 <= (c8 == 4'd8) ? 4'd0 : c8 + 4'd1;
    wire        ce_8m = (c8 == 4'd8);
    // MAME GUNY 10-F0, reversed: content row 0 (top) -> F0, row 223 -> 11
    wire [15:0] gun_ym  = {8'd0, gun_y} * 16'd257;
    wire  [7:0] gun_gy  = 8'hF0 - gun_ym[15:8];

    // nob (Noboranka) uses the MCU through latches only: P0 data, P1 status, P2 control, INT0 from the main CPU
    wire        cfg_nob = cfg_io[4];
    wire  [7:0] nob_p0_i, nob_p0_o, nob_p2_o;
    wire        nob_int0n;
    // REVERT-2026-10-20: original below, uncomment to restore and delete the block after it
    // assign      z80_0_busrq_n = ~(cfg_mcu && !cfg_nob && !mcu_rst && mcu_p1[6]);
    // // a MOVX started before the Z80 has let go of its bus waits for BUSAK
    // wire        mcu_cen = ce_8m && !pause && !(mcu_acc && mcu_p1[6] && z80_0_busak_n);
    // A MOVX to the Z80 side (modes 0-2) requests the bus itself: Choplifter's game code runs with P1 = 00 / 01 (P1.6
    // clear) and still writes work RAM, the row scroll and the bg page selects E740-E746 (MAME performs every access).
    // Without the bus those writes were lost and the bg layer showed the text page (doubled title text, HW 2026-10-20).
    // The MCU waits until BUSAK has been held 8 clocks, so a read sees settled data through the RAM / ROM registers.
    reg   [3:0] bus_age = 4'd0;
    always @(posedge clk) bus_age <= z80_0_busak_n ? 4'd0 : bus_age[3] ? bus_age : bus_age + 4'd1;
    wire        mcu_xreq = cfg_mcu && !cfg_nob && mcu_acc && (mcu_p1[4:3] != 2'd3);
    assign      z80_0_busrq_n = ~(cfg_mcu && !cfg_nob && !mcu_rst && (mcu_p1[6] || mcu_xreq));
    wire        mcu_cen = ce_8m && !pause && !(mcu_xreq && !bus_age[3]);

    // T0: the acknowledge (~2 Z80 T-states) is far shorter than the core's machine cycle; stretch it to 3 us
    reg   [7:0] t0_cnt = 8'd0;
    always @(posedge clk) begin
        if (intack) t0_cnt <= 8'd240;
        else if (t0_cnt != 8'd0) t0_cnt <= t0_cnt - 8'd1;
    end
    wire        mcu_t0 = (t0_cnt == 8'd0);                  // pin low while stretched: one falling edge per acknowledge

    jtframe_8751mcu #(.DIVCEN(1), .SYNC_XDATA(1), .SYNC_INT(1)) mcu
    (
        .rst(mcu_rst), .clk(clk), .cen(mcu_cen), .t0(mcu_t0),
        .int0n(cfg_nob ? nob_int0n : ~video_vblank), .int1n(cfg_nob ? 1'b1 : video_mode[6]),
        // REVERT-2026-10-20: original below, uncomment to restore and delete the line after it
        // .p0_i(cfg_nob ? nob_p0_i : 8'hFF), .p1_i(8'hFF), .p2_i(8'hFF),
        // P1 / P2 are outputs only: their pins read back the latch. The Oregano core reads the pins for PUSH and byte
        // RMW (a real 8751 reads the latch), so with FF here Choplifter's push p1 / pop p1 left P1 = FF (BUSREQ, MOVX
        // mode 3) and its page-select writes were lost; nob's status read FF / 7F instead of MAME's 80 / 00
        .p0_i(cfg_nob ? nob_p0_i : 8'hFF), .p1_i(mcu_p1), .p2_i(nob_p2_o),
        .p3_i({3'b111, mcu_t0, video_mode[6], ~video_vblank, 2'b11}),
        .p0_o(nob_p0_o), .p1_o(mcu_p1), .p2_o(nob_p2_o), .p3_o(),
        .x_din(mcu_xi), .x_dout(mcu_xo), .x_addr(mcu_addr), .x_wr(mcu_wr), .x_acc(mcu_acc),
        .clk_rom(clk), .prog_addr(ioctl_addr[11:0]), .prom_din(ioctl_dout),
        .prom_we(ioctl_wr0 && (ioctl_addr[24:12] == 13'h04C))
    );

    sys1_main main
    (
        .clk(clk), .reset(cpu_reset), .pause(pause),
        .cfg_pio(cfg_board[0]), .cfg_sys2(cfg_board[1]), .cfg_d01swap(cfg_scramble[0]), .cfg_io_bg(cfg_io[1]), .cfg_io_mj(cfg_io[3]), .mj_rows(mj_rows),
        .cfg_gun(cfg_io[2]), .gun_gx((gun_x == 8'hFF) ? 8'hFE : gun_x), .gun_gy(gun_gy), .gun_trig(gun_trig),
        .cfg_nobo(cfg_io[6]), .cfg_nob(cfg_io[4]), .cfg_nobb(cfg_io[5]),
        .nob_p0_o(nob_p0_o), .nob_p2_o(nob_p2_o), .nob_p0_i(nob_p0_i), .nob_int0n(nob_int0n), .cfg_bank(cfg_bank[1:0]), .cfg_opc(cfg_opc[1:0]),
        .cfg_crypt((cfg_crypt[7:5] == 3'd0) ? cfg_crypt[4:0] : 5'd0), .cfg_mc8123(cfg_crypt == 8'h40),
        .key_a(key_a), .key_d(key_d),
        .cfg_mcu(cfg_mcu), .mcu_ctrl(mcu_p1), .mcu_bus(~z80_0_busak_n), .x_acc(mcu_acc), .x_wr(mcu_wr), .x_addr(mcu_addr),
        .x_dout(mcu_xo), .x_din(mcu_xi), .intack(intack),
        .z80_0_i(main_z80_0_i), .z80_0_o(z80_0_o),
        .romf_a(rom0_a), .romf_d(rom0_d), .romb_a(rom1_a), .romb_d(rom1_d),
        .in_p1(in_p1), .in_p2(in_p2), .in_sys(in_sys), .in_swa(in_swa), .in_swb(in_swb),
        .vram_va(vram_va), .vram_vd(vram_vd), .sram_va(sram_va), .sram_vd(sram_vd), .pal_va(pal_va), .pal_vd(pal_vd),
        .spr_col_set(spr_col_set), .spr_col_idx(spr_col_idx), .mix_col_set(mix_col_set), .mix_col_idx(mix_col_idx),
        .irq_vbl(irq_vbl), .fixst(fixst), .video_mode(video_mode),
        .snd_latch(snd_latch), .snd_nmi(snd_nmi), .snd_mute(snd_mute), .snd_astb_n(snd_astb_n),
        .hs_address(hs_address), .hs_data_in(hs_data_in), .hs_data_out(hs_data_out), .hs_write(hs_write)
    );

    sys1_video video
    (
        .clk(clk), .reset(reset), .pix_phase(pix_phase), .ce_pix(ce_pix),
        .xhair_en(xhair_en && cfg_io[2]), .gun_x(gun_x), .gun_y(gun_y),
        .sys2(cfg_board[1]), .rowscroll(cfg_board[2]), .narrow(cfg_board[3]), .tile32k(cfg_tiles[0]), .cprom(cfg_board[5]), .spr_banks(cfg_sprbanks[3:0]), .video_mode(video_mode),
        .crt_flip(crt_flip), .h_adj(h_adj), .v_adj(v_adj),
        .vram_a(vram_va), .vram_d(vram_vd),
        .tile_a(tile_a), .tile_d0(tile_d[0]), .tile_d1(tile_d[1]), .tile_d2(tile_d[2]),
        .sram_a(sram_va), .sram_d(sram_vd), .pal_a(pal_va), .pal_d(pal_vd), .lut_a(lut_a), .lut_d(rom3_d),
        .cp_a(cp_a), .cp_r(cp_d[0]), .cp_g(cp_d[1]), .cp_b(cp_d[2]),
        .sdr_req(sdr_req), .sdr_waddr(sdr_waddr), .sdr_word(sdr_word), .sdr_busy(sdr_busy),
        .spr_col_set(spr_col_set), .spr_col_idx(spr_col_idx), .mix_col_set(mix_col_set), .mix_col_idx(mix_col_idx),
        .irq_vbl(irq_vbl), .irq_snd(irq_snd), .fixst(fixst), .frame_start(frame_start),
        .video_r(video_r), .video_g(video_g), .video_b(video_b),
        .video_hs(video_hs), .video_vs(video_vs), .video_hblank(video_hblank), .video_vblank(video_vblank)
    );

    sys1_sound sound
    (
        .clk(clk), .reset(cpu_reset), .pause(pause), .ce_cpu(ce_scpu), .ce_sn1(ce_sn1), .ce_sn2(ce_sn2),
        .z80_1_i(snd_z80_1_i), .z80_1_o(z80_1_o), .sn76489_i(snd_sn76489_i), .sn76489_o(sn76489_o),
        .rom_a(rom2_a), .rom_d(rom2_d),
        .latch(snd_latch), .nmi(snd_nmi), .mute(snd_mute), .astb_n(snd_astb_n), .irq_line(irq_snd),
        .audio(audio)
    );

    wire unused = &{1'b0, cfg_scramble[7:1], cfg_io[7], cfg_io[0], cfg_sndclk[7:2], cfg_opc[7:2], cfg_bank[7:2], cfg_tiles[7:1], cfg_sprbanks[7:4], cfg_board[7:6],
                    1'b0};

endmodule
