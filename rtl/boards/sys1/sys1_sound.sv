//============================================================================
//
//  sys1_sound.sv — Sega System 1 / 2 sound board
//  Copyright (C) 2026 Rodimus
//
//  From MAME sega/system1.cpp (sound_map, soundirq_gen, sound_data_r, SN76489A routes) and the Sega board docs.
//  Z80 (pool z80_1) at 4 MHz (8 MHz / 2), two SN76489A (pool sn76489[0] at 2 MHz, [1] at 4 MHz) whose READY lines
//  hold the CPU (MAME input_merger -> WAIT; here the CPU clock enable waits for READY).
//
//    0000-7FFF ROM    8000-87FF RAM (mirrored to 9FFF)    A000 SN #1 (2 MHz)    C000 SN #2 (4 MHz)
//    E000 read: command latch, and a strobe back to the main board (PIO /ASTB or PPI PC6 acknowledge)
//  IRQ at 32V, 96V, 160V, 224V (held until acknowledged); NMI from the main board (PIO ARDY / PPI PC7).
//  Mix as MAME: SN #1 x 0.40 + SN #2 x 0.60 (SN #1's output feeds SN #2's audio input through a resistor).
//
//============================================================================

module sys1_sound
(
    input             clk,
    input             reset,
    input             pause,
    input             ce_cpu,         // 4 MHz (2.5 MHz on Star Jacker / Up'n Down)
    input             ce_sn1,         // SN #1: 2 MHz (2.5)
    input             ce_sn2,         // SN #2: 4 MHz (1.25 / 2.5 / 2)

    output chip_pool_pkg::z80_i_t z80_1_i,
    input  chip_pool_pkg::z80_o_t z80_1_o,
    output chip_pool_pkg::sn76489_i_t [1:0] sn76489_i,
    input  chip_pool_pkg::sn76489_o_t [1:0] sn76489_o,

    output     [14:0] rom_a,
    input       [7:0] rom_d,

    input       [7:0] latch,
    input             nmi,            // 1 = NMI line asserted
    input             mute,           // PPI port C bit 0 (MAME system_mute)
    output            astb_n,         // low while the CPU reads the latch
    input             irq_line,       // one clock at 32V, 96V, 160V, 224V

    output signed [15:0] audio
);

    wire [15:0] a    = z80_1_o.a;
    wire        mreq = ~z80_1_o.mreq_n;
    wire        iorq = ~z80_1_o.iorq_n;
    wire        rd   = ~z80_1_o.rd_n;
    wire        wr   = ~z80_1_o.wr_n;
    wire        m1   = ~z80_1_o.m1_n;
    wire        rfsh = ~z80_1_o.rfsh_n;

    wire s_rom   = !a[15];
    wire s_ram   = a[15:13] == 3'b100;
    wire s_sn1   = a[15:13] == 3'b101;
    wire s_sn2   = a[15:13] == 3'b110;
    wire s_latch = a[15:13] == 3'b111;

    assign rom_a  = a[14:0];
    assign astb_n = ~(mreq && rd && !rfsh && s_latch);

    wire [7:0] ram_q;
    dpram_dc #(.widthad_a(11)) ram
    (
        .clock_a(clk), .address_a(a[10:0]), .data_a(z80_1_o.dout), .wren_a(mreq && wr && s_ram), .q_a(ram_q),
        .clock_b(clk), .address_b(11'd0), .data_b(8'h00), .wren_b(1'b0), .q_b()
    );

    // SN76489A writes: chip select only on a write cycle, so READY drops for writes only
    assign sn76489_i[0].rst  = reset;
    assign sn76489_i[0].ce   = ce_sn1;
    assign sn76489_i[0].wr_n = z80_1_o.wr_n;
    assign sn76489_i[0].cs_n = ~(mreq && wr && s_sn1);
    assign sn76489_i[0].din  = z80_1_o.dout;
    assign sn76489_i[1].rst  = reset;
    assign sn76489_i[1].ce   = ce_sn2;
    assign sn76489_i[1].wr_n = z80_1_o.wr_n;
    assign sn76489_i[1].cs_n = ~(mreq && wr && s_sn2);
    assign sn76489_i[1].din  = z80_1_o.dout;

    wire sn_ready = sn76489_o[0].ready & sn76489_o[1].ready;

    reg irq = 1'b0;
    always @(posedge clk) begin
        if (reset) irq <= 1'b0;
        else if (irq_line) irq <= 1'b1;
        else if (iorq && m1) irq <= 1'b0;
    end

    assign z80_1_i.ce    = ce_cpu && sn_ready && !pause;
    assign z80_1_i.int_n = ~irq;
    assign z80_1_i.nmi_n = ~nmi;
    assign z80_1_i.di    = (iorq && m1) ? 8'hFF : s_rom ? rom_d : s_ram ? ram_q : s_latch ? latch : 8'hFF;

    // jt89: signed 11 bits per chip. 0.40 / 0.60 as 13 / 19 out of 32; every operand explicitly signed
    wire signed [10:0] sn1 = sn76489_o[0].sound;
    wire signed [10:0] sn2 = sn76489_o[1].sound;
    wire signed [16:0] mix = $signed({{6{sn1[10]}}, sn1}) * 17'sd13 + $signed({{6{sn2[10]}}, sn2}) * 17'sd19;
    assign audio = mute ? 16'sd0 : mix[15:0];

endmodule
