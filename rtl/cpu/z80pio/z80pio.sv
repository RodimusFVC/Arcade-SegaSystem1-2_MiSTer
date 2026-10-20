//============================================================================
//
//  z80pio.sv — Z80 PIO (Z8420), the parts Sega System 1 uses
//  Copyright (C) 2026 Rodimus
//
//  Written from the Z80 PIO technical manual, checked against MAME devices/machine/z80pio.cpp.
//  Register select as MAME: ba_sel = A0 (0 = port A, 1 = port B), cd_sel = A1 (0 = data, 1 = control).
//  Modes 0 (output), 1 (input), 2 (port A bidirectional: the output side and ARDY / ASTB; the input handshake on
//  port B's lines is not modelled) and 3 (bit control).
//  Interrupt control words are parsed (so a mask byte is consumed correctly) but no interrupt is driven:
//  System 1 does not wire the PIO's /INT.
//
//  RDY (port A only is brought out): an output- or bidirectional-mode data write drops RDY and raises it again; the
//  rising edge of /STB clears it; selecting output mode raises it. A write holds RDY low for RDY_LOW clocks so a
//  consumer sampling on a slower clock enable (Z80 NMI, sampled per T-state) always sees the edge.
//
//============================================================================

module z80pio #(
    parameter int RDY_LOW = 48
)(
    input             clk,
    input             reset,

    input             cs,             // one-clock write / read strobe qualifier
    input             wr,             // write strobe (one clock)
    input             ba_sel,
    input             cd_sel,
    input       [7:0] din,
    output reg  [7:0] dout,

    input       [7:0] pa_in,
    input       [7:0] pb_in,
    output      [7:0] pa_out,
    output      [7:0] pb_out,
    output reg        pa_upd,         // one clock: port A output value written (MAME out_pa_cb)
    output reg        pb_upd,         // one clock: port B output value written (MAME out_pb_cb)

    input             astb_n,         // port A strobe from the peripheral
    output            ardy
);

    localparam [1:0] M_OUT = 2'd0, M_IN = 2'd1, M_BIDIR = 2'd2, M_BIT = 2'd3;
    localparam [1:0] NX_ANY = 2'd0, NX_IOR = 2'd1, NX_MASK = 2'd2;

    reg  [1:0] mode [2];
    reg  [1:0] next [2];
    reg  [7:0] outr [2];
    reg  [7:0] ior  [2];
    reg  [7:0] icw  [2];
    reg        rdy_a = 1'b0;
    reg  [7:0] rdy_hold = 8'd0;
    reg        astb_d = 1'b1;

    wire [7:0] po_a = (mode[0] == M_BIT) ? (ior[0] | (outr[0] & ~ior[0])) : outr[0];
    wire [7:0] po_b = (mode[1] == M_BIT) ? (ior[1] | (outr[1] & ~ior[1])) : outr[1];
    assign pa_out = po_a;
    assign pb_out = po_b;
    assign ardy   = rdy_a & (rdy_hold == 8'd0);

    wire p = ba_sel;

    // reads
    always_comb begin
        if (cd_sel) dout = 8'hFF;   // control register reads are not defined on the Z8420
        else case (mode[p])
            M_OUT:   dout = outr[p];
            M_BIDIR: dout = p ? 8'hFF : pa_in;
            M_BIT:   dout = ((p ? pb_in : pa_in) & ior[p]) | (outr[p] & ~ior[p]);
            default: dout = p ? pb_in : pa_in;
        endcase
    end

    always_ff @(posedge clk) begin
        pa_upd <= 1'b0;
        pb_upd <= 1'b0;
        astb_d <= astb_n;
        if (rdy_hold != 8'd0) rdy_hold <= rdy_hold - 8'd1;

        if (reset) begin
            mode[0] <= M_IN;    mode[1] <= M_IN;
            next[0] <= NX_ANY;  next[1] <= NX_ANY;
            outr[0] <= 8'h00;   outr[1] <= 8'h00;
            ior[0]  <= 8'h00;   ior[1]  <= 8'h00;
            icw[0]  <= 8'h00;   icw[1]  <= 8'h00;
            rdy_a   <= 1'b0;    rdy_hold <= 8'd0;
        end
        else begin
            // /STB rising edge clears RDY in output mode
            if (astb_n && !astb_d && (mode[0] == M_OUT || mode[0] == M_BIDIR) && rdy_a) rdy_a <= 1'b0;

            if (cs && wr) begin
                if (!cd_sel) begin
                    outr[p] <= din;
                    if (mode[p] == M_OUT || mode[p] == M_BIT || (!p && mode[0] == M_BIDIR)) begin
                        if (p) pb_upd <= 1'b1; else pa_upd <= 1'b1;
                    end
                    if (!p && (mode[0] == M_OUT || mode[0] == M_BIDIR)) begin
                        rdy_a    <= 1'b1;
                        rdy_hold <= RDY_LOW[7:0];
                    end
                end
                else case (next[p])
                    NX_IOR: begin ior[p] <= din; next[p] <= NX_ANY; if (p) pb_upd <= 1'b1; else pa_upd <= 1'b1; end
                    NX_MASK: next[p] <= NX_ANY;
                    default:
                        if (din[0]) case (din[3:0])
                            4'hF: begin
                                mode[p] <= din[7:6];
                                if (din[7:6] == M_OUT) begin
                                    if (p) pb_upd <= 1'b1; else begin pa_upd <= 1'b1; rdy_a <= 1'b1; end
                                end
                                if (din[7:6] == M_BIT) begin
                                    next[p] <= NX_IOR;
                                    if (!p) rdy_a <= 1'b0;
                                end
                            end
                            4'h7: begin icw[p] <= din; if (din[4]) next[p] <= NX_MASK; end
                            4'h3: icw[p] <= {din[7], icw[p][6:0]};
                            default: ;
                        endcase
                endcase
            end
        end
    end

    wire unused = &{1'b0, icw[0], icw[1]};

endmodule
