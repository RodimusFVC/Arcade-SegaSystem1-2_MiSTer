//============================================================================
//
//  sys1_main.sv — Sega System 1 main CPU board: memory map, I/O, RAMs, collision latches
//  Copyright (C) 2026 Rodimus
//
//  From MAME sega/system1.cpp (system1_map, system1_pio_io_map, adjust_cycles, videoram_wait_states) and the
//  Sega 834-5191 / 171-5054 board docs. The Z80 is the pool's z80_0 (rtl/segasys1_board.sv).
//
//  CPU clock: 20 MHz divided by 5, by 6 while /M1 is low (an LS161 reloads with /M1 on its low bit), so an opcode
//  fetch costs 2 extra 20 MHz clocks. At clk = 80 MHz: 20 clocks per T-state, 24 while /M1 is low.
//  A video RAM access halts the CPU clock until FIXST, every 8 pixels = 3.2 CPU cycles. Modelled as MAME
//  videoram_wait_states (from a dump of the 315-5137 PAL): whole CPU cycles, (tenths of a cycle to the next FIXST + 5)
//  / 10, on a FIXST grid counted from CPU start. An exact halt-until-FIXST locks LDIR to 22.4 cycles per byte (MAME 24)
//  and Pitfall II reaches its title logo write before the vblank IRQ's sprite clear instead of after it, so the logo
//  is lost (HW 2026-10-19).
//
//    0000-7FFF ROM               C000-CFFF RAM              E000-EFFF video RAM (System 2: banked, 16K)
//    8000-BFFF ROM / bank        D000-D7FF sprite RAM       F000-F3FF mixer collision (F400 write: clear summary)
//                                D800-DFFF palette RAM      F800-FBFF sprite collision (FC00 write: clear summary)
//  I/O (A4-A0): 00 P1, 04 P2, 08 SYSTEM, 0C/0E SWA, 0D/0F SWB, 10 SWB,
//    PPI board: 14-17 8255 (A = sound latch, B = video mode, C = sound control: [7] sound NMI (0 = asserted),
//               [0] mute, [2:1] video RAM bank on System 2)
//    PIO board: 18-1B Z80 PIO (A = sound latch + ARDY -> sound NMI, B = video mode)
//
//============================================================================

module sys1_main
(
    input             clk,
    input             reset,
    input             pause,

    input             cfg_pio,
    input             cfg_sys2,       // System 2: 16K video RAM banked by PPI port C, mute bit inverted
    input             cfg_d01swap,    // myherok: program ROMs below C000 have D0 / D1 swapped (before the CPU decryption)
    input             cfg_io_bg,      // Block Gal I/O map (MAME blockgal_io_map): 0C/0E unmapped, 0D/0F = SWA
    input             cfg_io_mj,      // mahjong matrix (MAME dakkochn_state): P1 = key row, P2 = ~(1 << row)
    input             cfg_gun,        // Shooting Master gun ports (MAME shtngmst_io_map)
    input       [7:0] gun_gx,         // MAME GUNX (00-FE)
    input       [7:0] gun_gy,         // MAME GUNY (10-F0, reversed)
    input             gun_trig,
    input             cfg_nobo,       // Noboranka map (MAME nobo_map): collisions at C000-CFFF, RAM at F000-FFFF
    input             cfg_nob,        // Noboranka 8751 latches (MAME init_nob): I/O 18 latch, 1C MCU status
    input             cfg_nobb,       // Noboranka bootleg ports (MAME init_nobb): in 1C = 80, 02 = 0, 03 = step (out 04)
    input       [7:0] nob_p0_o,       // 8751 P0 / P2 out (nob)
    input       [7:0] nob_p2_o,
    output reg  [7:0] nob_p0_i = 8'hFF,
    output reg        nob_int0n = 1'b1,
    input      [55:0] mj_rows,
    input       [1:0] cfg_bank,       // 0 none, 1 bits 6 / 2, 2 bits 3 / 2 of the video-mode port
    input       [1:0] cfg_opc,        // 0 opcodes = data, 1 copy at region 0x10000 (A < 8000), 2 half at 0x20000
    input       [4:0] cfg_crypt,      // Sega 315-xxxx / 317-xxxx chip, 0 = none (sega_decrypt.sv)
    input             cfg_mc8123,     // NEC MC-8123 CPU (key from ioctl 4A000)
    output     [12:0] key_a,
    input       [7:0] key_d,

    // 8751 MCU (Choplifter, Shooting Master; MAME system1 mcu_control_w / mcu_data_r / mcu_data_w): P1 [6] Z80 BUSREQ,
    // [4:3] MOVX target (0 Z80 program space, 1 banked ROM region 10000+, 2 Z80 I/O space), [0] Z80 /INT
    input             cfg_mcu,
    input       [7:0] mcu_ctrl,
    input             mcu_bus,        // the Z80 has granted its bus (BUSAK)
    input             x_acc,
    input             x_wr,
    input      [15:0] x_addr,
    input       [7:0] x_dout,
    output      [7:0] x_din,
    output            intack,         // Z80 interrupt acknowledge (clocks the MCU's T0)

    output chip_pool_pkg::z80_i_t z80_0_i,
    input  chip_pool_pkg::z80_o_t z80_0_o,

    output     [15:0] romf_a,         // {half, A14-A0}: region 0x00000-0x07FFF (+0x20000)
    input       [7:0] romf_d,
    output     [16:0] romb_a,         // {half, region offset 0x10000-0x1FFFF or 0x8000-0xFFFF}
    input       [7:0] romb_d,

    input       [7:0] in_p1,
    input       [7:0] in_p2,
    input       [7:0] in_sys,
    input       [7:0] in_swa,
    input       [7:0] in_swb,

    input      [13:0] vram_va,
    output      [7:0] vram_vd,
    input       [8:0] sram_va,
    output      [7:0] sram_vd,
    input      [10:0] pal_va,
    output      [7:0] pal_vd,
    input             spr_col_set,
    input       [9:0] spr_col_idx,
    input             mix_col_set,
    input       [5:0] mix_col_idx,
    input             irq_vbl,
    input             fixst,
    output reg  [7:0] video_mode = 8'h00,

    output      [7:0] snd_latch,
    output            snd_nmi,
    output            snd_mute,
    input             snd_astb_n,

    input      [11:0] hs_address,
    input       [7:0] hs_data_in,
    output      [7:0] hs_data_out,
    input             hs_write
);

    // bus master: the Z80, or the 8751's MOVX cycles while the Z80 is held off the bus
    wire  [1:0] x_mode = mcu_ctrl[4:3];
    wire        mb     = cfg_mcu && mcu_bus;
    wire [15:0] a      = mb ? x_addr : z80_0_o.a;
    wire  [7:0] dout   = mb ? x_dout : z80_0_o.dout;
    wire        mreq   = mb ? (x_acc && x_mode == 2'd0) : ~z80_0_o.mreq_n;
    wire        iorq   = mb ? (x_acc && x_mode == 2'd2) : ~z80_0_o.iorq_n;
    wire        rd     = mb ? (x_acc && !x_wr) : ~z80_0_o.rd_n;
    wire        wr     = mb ? (x_acc && x_wr) : ~z80_0_o.wr_n;
    wire        m1     = mb ? 1'b0 : ~z80_0_o.m1_n;
    wire        rfsh   = mb ? 1'b0 : ~z80_0_o.rfsh_n;
    assign      intack = ~z80_0_o.iorq_n && ~z80_0_o.m1_n;

    // ------------------------------------------------------------ CPU clock
    // REVERT-2026-10-19: previous model (halt until the video FIXST), uncomment to restore and delete the block below
    // reg  [4:0] cdiv = 5'd0;
    // reg        vram_ok = 1'b0;
    // wire [4:0] lim      = m1 ? 5'd23 : 5'd19;
    // wire       cyc_end  = (cdiv >= lim);
    // wire       vram_acc = mreq && !rfsh && (a[15:12] == 4'hE) && (rd || wr);
    // wire       stall    = vram_acc && !vram_ok;
    // wire       ce       = cyc_end && !stall && !pause;
    //
    // always @(posedge clk) begin
    //     if (ce) cdiv <= 5'd0;
    //     else if (!cyc_end && !stall && !pause) cdiv <= cdiv + 5'd1;     // halted clock: the T-state does not advance
    //     if (!vram_acc) vram_ok <= 1'b0;
    //     else if (fixst) vram_ok <= 1'b1;
    // end
    reg  [4:0] cdiv = 5'd0;
    reg        vram_ok = 1'b0, vram_wt = 1'b0;
    reg  [5:0] fx = 6'd32;                      // 80 MHz clocks since CPU start - 32, mod 64 (MAME fixst_offset)
    reg  [5:0] vwait = 6'd0;
    wire [4:0] lim      = m1 ? 5'd23 : 5'd19;
    wire       cyc_end  = (cdiv >= lim);
    wire       vram_acc = mreq && !rfsh && (a[15:12] == 4'hE) && (rd || wr);
    wire [5:0] until10  = 6'd32 - {1'b0, fx[5:1]};                  // tenths of a CPU cycle to the next FIXST, 1-32
    wire [5:0] until5   = until10 + 6'd5;
    wire [1:0] wcyc     = (until5 >= 6'd30) ? 2'd3 : (until5 >= 6'd20) ? 2'd2 : (until5 >= 6'd10) ? 2'd1 : 2'd0;
    wire       stall    = vram_acc && !vram_ok;
    wire       ce       = cyc_end && !stall && !pause;

    always @(posedge clk) begin
        if (reset) fx <= 6'd32;
        else if (!pause) fx <= fx + 6'd1;
        if (ce) cdiv <= 5'd0;
        else if (!cyc_end && !stall && !pause) cdiv <= cdiv + 5'd1;     // halted clock: the T-state does not advance
        // the access's first clock picks the wait; the halt then lasts 20 clocks (one CPU cycle) per wait cycle
        if (!vram_acc) begin vram_ok <= 1'b0; vram_wt <= 1'b0; end
        else if (!vram_ok) begin
            if (!vram_wt) begin
                if (wcyc == 2'd0) vram_ok <= 1'b1;
                else begin vwait <= {wcyc, 4'd0} + {1'b0, wcyc, 2'd0} - 6'd2; vram_wt <= 1'b1; end   // 20 x wcyc - 2
            end
            else if (vwait == 6'd0) vram_ok <= 1'b1;
            else vwait <= vwait - 6'd1;
        end
    end
    wire unused_fixst = fixst;

    // ------------------------------------------------------------ decode
    wire mem_rd = mreq && rd && !rfsh;
    wire mem_wr = mreq && wr;
    wire io_rd  = iorq && rd && !m1;
    wire io_wr  = iorq && wr && !m1;

    reg  mem_wr_d = 1'b0, io_wr_d = 1'b0;
    always @(posedge clk) begin mem_wr_d <= mem_wr; io_wr_d <= io_wr; end
    wire mem_wp = mem_wr && !mem_wr_d;              // one clock per write cycle
    wire io_wp  = io_wr && !io_wr_d;

    wire s_ram  = a[15:12] == (cfg_nobo ? 4'hF : 4'hC);
    wire s_spr  = a[15:11] == 5'b11010;
    wire s_pal  = a[15:11] == 5'b11011;
    wire s_vram = a[15:12] == 4'hE;
    wire s_mixc = a[15:10] == (cfg_nobo ? 6'b110000 : 6'b111100);
    wire s_mixr = a[15:10] == (cfg_nobo ? 6'b110001 : 6'b111101);
    wire s_sprc = a[15:10] == (cfg_nobo ? 6'b110010 : 6'b111110);
    wire s_sprr = a[15:10] == (cfg_nobo ? 6'b110011 : 6'b111111);

    // ------------------------------------------------------------ ROM
    reg  [1:0] bank = 2'd0;
    wire       opf    = m1 && mreq;                  // opcode fetch
    wire       ohalf  = opf && (cfg_opc == 2'd2);
    wire       oboot  = opf && (cfg_opc == 2'd1) && !a[15];
    assign romf_a = {ohalf, a[14:0]};
    assign romb_a = (mb && x_mode == 2'd1) ? {1'b0, x_addr} : oboot ? {1'b0, a[15:0]} : {ohalf, bank, a[13:0]};
    wire [7:0] rom_rd  = (!a[15] && !oboot) ? romf_d : romb_d;
    wire [7:0] rom_raw = (cfg_d01swap && a[15:14] != 2'b11) ? {rom_rd[7:2], rom_rd[0], rom_rd[1]} : rom_rd;

    // MC-8123: jotego jtmc8123 (jtcores s16b, GPL3). Every ROM read below C000 (fixed and banked window, MAME
    // mc8123_device::decode), key byte key[{/M1, A15-A10, A8, A6, A4, A2-A0}]; key RAM and ROM both answer one clock
    // after the address, the decoded byte one clock later (registered; plain ROM data passes through the same register).
    // jtmc8123 takes the key inverted and has no pass-through: MAME decrypt_internal inverts the key and returns the
    // byte unchanged for a key of FF (checked exhaustively against MAME's transcription in verilator/mc8123_cmp/)
    wire [7:0] rom_pre;
    jtmc8123 mc8123
    (
        .clk(clk), .m1_n(~opf), .a(a), .enc_en(cfg_mc8123 && a[15:14] != 2'b11 && key_d != 8'hFF),
        .enc(rom_raw), .rom_ok(1'b1), .dec(rom_pre), .dec_ok(),
        .key_addr(key_a), .key_data(~key_d)
    );

    // Security CPU: decrypt on read, A15 = 0 only, opcode fetches and data reads by separate tables. Two clocks of
    // latency on every ROM read with the MC-8123 register (the T-state is 20 clocks); chip 0 passes data through.
    wire [7:0] rom_d;
    sega_decrypt decrypt
    (
        .clk(clk), .chip(cfg_crypt), .op(opf), .a(a), .din(rom_pre), .dout(rom_d)
    );

    // ------------------------------------------------------------ RAMs
    wire [7:0] ram_q, spr_q, pal_q, vram_q, scol_q;

    dpram_dc #(.widthad_a(12)) ram
    (
        .clock_a(clk), .address_a(a[11:0]), .data_a(dout), .wren_a(mem_wr && s_ram), .q_a(ram_q),
        .clock_b(clk), .address_b(hs_address), .data_b(hs_data_in), .wren_b(hs_write), .q_b(hs_data_out)
    );

    dpram_dc #(.widthad_a(11)) spriteram
    (
        .clock_a(clk), .address_a(a[10:0]), .data_a(dout), .wren_a(mem_wr && s_spr), .q_a(spr_q),
        .clock_b(clk), .address_b({2'b00, sram_va}), .data_b(8'h00), .wren_b(1'b0), .q_b(sram_vd)
    );

    dpram_dc #(.widthad_a(11)) paletteram
    (
        .clock_a(clk), .address_a(a[10:0]), .data_a(dout), .wren_a(mem_wr && s_pal), .q_a(pal_q),
        .clock_b(clk), .address_b(pal_va), .data_b(8'h00), .wren_b(1'b0), .q_b(pal_vd)
    );

    // System 1: 2 pages (4K). System 2: 8 pages, the CPU sees 4K at a time, bank = PPI port C bits 2-1
    // (MAME videoram_bank >> 1 % 4)
    wire [7:0] ppi_pc;                                             // 8255 port C pins (below)
    wire [1:0] vbank = cfg_sys2 ? ppi_pc[2:1] : 2'd0;
    dpram_dc #(.widthad_a(14)) videoram
    (
        .clock_a(clk), .address_a({vbank, a[11:0]}), .data_a(dout), .wren_a(mem_wr && s_vram), .q_a(vram_q),
        .clock_b(clk), .address_b(vram_va), .data_b(8'h00), .wren_b(1'b0), .q_b(vram_vd)
    );

    // 32 x 32 sprite collision bits (byte-wide RAM, bit 0 used): video sets, the CPU reads and clears
    dpram_dc #(.widthad_a(10)) sprcol
    (
        .clock_a(clk), .address_a(a[9:0]), .data_a(8'h00), .wren_a(mem_wp && s_sprc), .q_a(scol_q),
        .clock_b(clk), .address_b(spr_col_idx), .data_b(8'h01), .wren_b(spr_col_set), .q_b()
    );

    reg [63:0] mix_col = 64'd0;
    reg        mix_sum = 1'b0, spr_sum = 1'b0;
    always @(posedge clk) begin
        if (reset) begin
            mix_col <= 64'd0; mix_sum <= 1'b0; spr_sum <= 1'b0;
        end
        else begin
            if (mix_col_set) begin mix_col[mix_col_idx] <= 1'b1; mix_sum <= 1'b1; end
            if (spr_col_set) spr_sum <= 1'b1;
            if (mem_wp && s_mixc) mix_col[a[5:0]] <= 1'b0;
            if (mem_wp && s_mixr) mix_sum <= 1'b0;
            if (mem_wp && s_sprr) spr_sum <= 1'b0;
        end
    end

    // ------------------------------------------------------------ Z80 PIO (port A: sound latch + ARDY -> sound NMI,
    //                                                              port B: video mode / coin counters / bank)
    wire       s_pio = (a[4:2] == 3'b110);
    wire [7:0] pio_dout, pio_pa, pio_pb;
    wire       pio_pb_upd, pio_ardy;

    z80pio pio
    (
        .clk(clk), .reset(reset),
        .cs(cfg_pio && s_pio), .wr(io_wp), .ba_sel(a[0]), .cd_sel(a[1]), .din(dout), .dout(pio_dout),
        .pa_in(8'hFF), .pb_in(8'hFF), .pa_out(pio_pa), .pb_out(pio_pb), .pa_upd(), .pb_upd(pio_pb_upd),
        .astb_n(snd_astb_n), .ardy(pio_ardy)
    );

    // ------------------------------------------------------------ 8255 PPI (same roles; port C drives the sound
    //                                                              board's NMI and mute directly)
    wire       s_ppi = (a[4:2] == 3'b101);
    wire [7:0] ppi_dout, ppi_pa, ppi_pb;
    reg  [7:0] ppi_pb_d = 8'h00;

    // REVERT-2026-10-20: jotego jt8255 below, uncomment to restore (and swap files.qip / verilator Makefile back)
    // // jotego jt8255 (MIT). Port A in mode 2 (Wonder Boy Deluxe et al., control word C0): PC7 = /OBF falls on a command
    // // write and rises with the sound board's latch read (/ACK on PC6). Undriven inputs float high.
    // jt8255 ppi
    // (
    //     .rst(reset), .clk(clk),
    //     .addr(a[1:0]), .din(dout), .dout(ppi_dout),
    //     .rdn(~io_rd), .wrn(~io_wp), .csn(~(!cfg_pio && s_ppi)),
    //     .porta_din(8'hFF), .portb_din(8'hFF), .portc_din({1'b1, snd_astb_n, 6'h3F}),
    //     .porta_dout(ppi_pa), .portb_dout(ppi_pb), .portc_dout(ppi_pc)
    // );
    // Own 8255 (rtl/cpu/i8255/i8255.sv, ported from MAME i8255.cpp). Port A in mode 2 (Wonder Boy Deluxe et al.,
    // control word C0): PC7 = /OBF falls on a command write and rises with the sound board's latch read (/ACK on PC6).
    // Floating inputs as MAME sys1ppi: PA 00, PB 40, PC 80.
    i8255 #(.TRI_PA(8'h00), .TRI_PB(8'h40), .TRI_PC(8'h80)) ppi
    (
        .clk(clk), .reset(reset),
        .cs(!cfg_pio && s_ppi), .rd(io_rd), .wr(io_wp), .a(a[1:0]), .din(dout), .dout(ppi_dout),
        .pa_in(8'hFF), .pb_in(8'hFF), .pc_in({1'b1, snd_astb_n, 6'h3F}),
        .stb_a_n(1'b1), .ack_a_n(snd_astb_n), .pc2_n(1'b1),
        .pa_out(ppi_pa), .pb_out(ppi_pb), .pc_out(ppi_pc)
    );

    assign snd_latch = cfg_pio ? pio_pa : ppi_pa;
    assign snd_nmi   = cfg_pio ? pio_ardy : ~ppi_pc[7];
    assign snd_mute  = !cfg_pio && (ppi_pc[0] ^ cfg_sys2);       // MAME sys2: port C exor 0x01

    wire [7:0] vm_new = cfg_pio ? pio_pb : ppi_pb;
    wire       vm_upd = cfg_pio ? pio_pb_upd : (ppi_pb != ppi_pb_d);

    always @(posedge clk) begin
        ppi_pb_d <= ppi_pb;
        if (reset) begin
            video_mode <= 8'h00; bank <= 2'd0;
        end
        else if (vm_upd) begin
            video_mode <= vm_new;
            case (cfg_bank)
                2'd1:    bank <= {vm_new[6], vm_new[2]};
                2'd2:    bank <= vm_new[3:2];
                default: bank <= 2'd0;
            endcase
        end
    end

    // ------------------------------------------------------------ data in
    // ------------------------------------------------------------ Noboranka protection
    // nob (MAME nob_maincpu_latch_w / nob_mcu_control_p2_w): the main CPU's latch write raises the MCU's INT0; the
    // MCU's P2 falling edges move bytes between the two latches (bit 0: main -> MCU, bit 1: MCU -> main) and clear INT0
    // (bit 2). P0 writes load the MCU latch, P1 is the status the main CPU reads at 1C.
    // nobb: the bootleg's stand-in ports (MAME nobb_inport*: 1C = 80, 02 = 0, 03 = the value last written to 04).
    reg  [7:0] nob_main = 8'h00, nob_p0_d = 8'hFF, nob_p2_d = 8'hFF, nobb_step = 8'h00;
    reg  [7:0] op_pc = 8'hFF;                       // low byte of the last opcode fetch address (nob start-up fix)
    reg        op_pc_hi0 = 1'b0;
    always @(posedge clk) begin
        nob_p0_d <= nob_p0_o;
        nob_p2_d <= nob_p2_o;
        if (opf) begin op_pc <= a[7:0]; op_pc_hi0 <= (a[15:8] == 8'h00); end
        if (reset) begin
            nob_main <= 8'h00; nob_p0_i <= 8'hFF; nob_int0n <= 1'b1; nobb_step <= 8'h00;
        end
        else begin
            if (nob_p0_o != nob_p0_d) nob_p0_i <= nob_p0_o;
            if (nob_p2_d[0] && !nob_p2_o[0]) nob_p0_i <= nob_main;
            if (nob_p2_d[1] && !nob_p2_o[1]) nob_main <= nob_p0_i;
            if (nob_p2_d[2] && !nob_p2_o[2]) nob_int0n <= 1'b1;
            if (cfg_nob && io_wp && a[4:0] == 5'h18) begin nob_main <= dout; nob_int0n <= 1'b0; end
            if (cfg_nobb && io_wp && a[4:0] == 5'h04) nobb_step <= dout;
        end
    end
    // nob: the reset vector's JP reads its target low byte as 80 (MAME nob_start_r: "likely M1-dependent behaviour")
    wire nob_jp0 = cfg_nob && !m1 && a == 16'h0001 && op_pc_hi0 && op_pc <= 8'h03;

    // ------------------------------------------------------------ Shooting Master gun (MAME shtngmst_state)
    // 10/11: gun output (bit 0 readies the trigger latch, 0 clears it; bit 2 solenoid), 12/13: ~(latch << 6),
    // 18-1B: rotary offset switches (MAME defaults 68), 1C/1E: gun X, 1D/1F: gun Y
    reg [7:0] gun_out = 8'h00;
    reg       gun_latch = 1'b0, gun_trig_d = 1'b0;
    always @(posedge clk) begin
        gun_trig_d <= gun_trig;
        if (reset) begin gun_out <= 8'h00; gun_latch <= 1'b0; end
        else begin
            if (gun_trig && !gun_trig_d && gun_out[0]) gun_latch <= 1'b1;
            if (cfg_gun && io_wp && a[4:1] == 4'b1000) begin gun_out <= dout; if (!dout[0]) gun_latch <= 1'b0; end
        end
    end

    // mahjong row counter: video-mode bit 1 rising steps it, modulo 7 (MAME dakkochn_state::videomode_w)
    reg [2:0] mj_row = 3'd0;
    reg       vm1_d = 1'b0;
    always @(posedge clk) begin
        vm1_d <= video_mode[1];
        if (reset) mj_row <= 3'd0;
        else if (video_mode[1] && !vm1_d) mj_row <= (mj_row == 3'd6) ? 3'd0 : mj_row + 3'd1;
    end

    reg [7:0] io_d;
    always_comb begin
        case (a[4:2])
            3'd0:    io_d = cfg_io_mj ? mj_rows[{mj_row, 3'b000} +: 8] :
                            (cfg_nobb && a[1:0] == 2'd2) ? 8'h00 : (cfg_nobb && a[1:0] == 2'd3) ? nobb_step : in_p1;
            3'd1:    io_d = cfg_io_mj ? ~(8'd1 << mj_row) : in_p2;
            3'd2:    io_d = in_sys;
            3'd3:    io_d = cfg_io_bg ? (a[0] ? in_swa : 8'hFF) : (a[0] ? in_swb : in_swa);
            3'd4:    io_d = !cfg_gun ? in_swb : a[1] ? {1'b1, ~gun_latch, 6'h3F} : gun_out;
            3'd5:    io_d = cfg_pio ? 8'hFF : ppi_dout;
            3'd6:    io_d = cfg_gun ? 8'h68 : cfg_pio ? pio_dout : (cfg_nob && a[1:0] == 2'd0) ? nob_main : 8'hFF;
            3'd7:    io_d = cfg_gun ? (a[0] ? gun_gy : gun_gx) : (a[1:0] != 2'd0) ? 8'hFF : cfg_nob ? mcu_ctrl : cfg_nobb ? 8'h80 : 8'hFF;
            default: io_d = 8'hFF;
        endcase
    end

    reg [7:0] mem_d;
    always_comb begin
        if (nob_jp0) mem_d = 8'h80;
        else if (a[15] == 1'b0 || a[15:14] == 2'b10) mem_d = rom_d;
        else if (s_ram)  mem_d = ram_q;
        else if (s_spr)  mem_d = spr_q;
        else if (s_pal)  mem_d = pal_q;
        else if (s_vram) mem_d = vram_q;
        else if (s_mixc) mem_d = {mix_sum, 6'h3F, mix_col[a[5:0]]};
        else if (s_sprc) mem_d = {spr_sum, 6'h3F, scol_q[0]};
        else             mem_d = 8'hFF;
    end

    // vblank IRQ is held until the CPU acknowledges it (MAME HOLD_LINE)
    reg irq = 1'b0;
    always @(posedge clk) begin
        if (reset) irq <= 1'b0;
        else if (irq_vbl) irq <= 1'b1;
        else if (iorq && m1) irq <= 1'b0;
    end

    assign z80_0_i.ce    = ce;
    // vblank (HOLD) or the MCU's P1.0. Tried 2026-10-20: MCU P1.0 alone on 8751 boards -> Choplifter froze on HW (heli
    // stops rising, audio loops); reverted. The vblank IRQ is needed.
    assign z80_0_i.int_n = ~(irq | (cfg_mcu && !cfg_nob && !mcu_ctrl[0]));
    assign x_din         = (x_mode == 2'd1) ? romb_d : (x_mode == 2'd2) ? io_d : mem_d;
    assign z80_0_i.nmi_n = 1'b1;
    assign z80_0_i.di    = (iorq && m1) ? 8'hFF : io_rd ? io_d : mem_d;

    wire unused = &{1'b0, mem_rd};

endmodule
