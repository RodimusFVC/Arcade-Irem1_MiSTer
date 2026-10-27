//============================================================================
//
//  m6803 - MC6803 (MC6801 without internal ROM) in expanded multiplexed mode, around jt680x
//  (Jose Tejada, 6801 microcode, unmodified). Copyright (C) 2026 Rodimus
//  Timer and port behaviour based on MAME m6801 (BSD-3-Clause, Aaron Giles).
//
//  Used by the Irem sound boards (M52-SOUND-C / M62). Addresses outside the on-chip registers (0x00-0x1F) and
//  RAM (0x80-0xFF) are external (x_cs). Only ports 1 and 2 are modelled; timer = free-running counter with
//  output compare and overflow; input capture and the SCI are register stubs.
//
//  cen = crystal rate (4 x E). Port outputs are (data & ddr) | ~ddr.
//
//============================================================================

module m6803
(
    input  logic        rst,
    input  logic        clk,
    input  logic        cen,            // crystal rate (3.579545 MHz on the Irem boards)

    input  logic        irq,            // /IRQ1, active high, level
    input  logic        nmi,            // /NMI, active high, rising edge (sampled on cen)

    output logic [15:0] addr,
    output logic  [7:0] dout,
    output logic        wr,
    output logic        x_cs,           // external access (not an on-chip register or RAM)
    input  logic  [7:0] xdin,           // external read data

    input  logic  [7:0] p1_in,
    output logic  [7:0] p1_out,
    input  logic  [4:0] p2_in,
    output logic  [4:0] p2_out,
    input  logic  [2:0] mode            // PC2-PC0, read back in port 2 bits 7-5
);

localparam P1DDR = 5'h00, P2DDR = 5'h01, P1 = 5'h02, P2 = 5'h03, TCSR = 5'h08, FRCH = 5'h09, FRCL = 5'h0A,
           OCRH = 5'h0B, OCRL = 5'h0C, ICRH = 5'h0D, ICRL = 5'h0E, RMCR = 5'h10, TRCSR = 5'h11, RDR = 5'h12,
           TDR = 5'h13, RAMC = 5'h14;

logic  [7:0] din;
logic        irq_ocf, irq_tof;

/* verilator lint_off PINCONNECTEMPTY */
jt680x u_cpu
(
    .rst(rst), .clk(clk), .cen(cen), .wr(wr), .addr(addr), .din(din), .dout(dout),
    .ext_halt(1'b0), .ba(),
    .irq(irq), .nmi(nmi), .irq_icf(1'b0), .irq_ocf(irq_ocf), .irq_tof(irq_tof), .irq_sci(1'b0),
    .irq_cmf(1'b0), .irq2(1'b0)
);
/* verilator lint_on PINCONNECTEMPTY */

wire reg_cs = addr[15:5] == 11'd0;              // 0x0000-0x001F
wire ram_cs = addr[15:7] == 9'd1;               // 0x0080-0x00FF
assign x_cs = ~reg_cs & ~ram_cs;

// one strobe per bus cycle: the clock after cen (bus signals settle on cen)
logic cen_d;
always_ff @(posedge clk) cen_d <= cen;
wire  wstb = cen_d & wr;
wire  rstb = cen_d & ~wr;

// on-chip RAM, 128 bytes (kept over reset, as the real part in standby)
logic [7:0] ram[128];
logic [7:0] ram_q;
always_ff @(posedge clk) begin
    if (wstb && ram_cs) ram[addr[6:0]] <= dout;
    ram_q <= ram[addr[6:0]];
end

// ports
logic [7:0] p1, p1ddr;
logic [4:0] p2, p2ddr;
assign p1_out = (p1 & p1ddr) | ~p1ddr;
assign p2_out = (p2 & p2ddr) | ~p2ddr;

// timer: FRC counts E (cen / 4)
logic  [1:0] ediv;
logic [15:0] frc, ocr;
logic  [7:0] frc_lo_buf;
logic  [4:0] tcr;                               // TCSR [4] EICI [3] EOCI [2] ETOI [1] IEDG [0] OLVL
logic        ocf, tof, ocf_armed, tof_armed;
logic  [7:0] rmcr, trcsr, tdr;
logic  [1:0] ramc;

wire e_tick = cen && ediv == 2'd3;
assign irq_ocf = ocf & tcr[3];
assign irq_tof = tof & tcr[2];

always_ff @(posedge clk) begin
    if (rst) begin
        p1 <= 8'd0;  p1ddr <= 8'd0;  p2 <= 5'd0;  p2ddr <= 5'd0;
        ediv <= 2'd0;  frc <= 16'd0;  ocr <= 16'hFFFF;  frc_lo_buf <= 8'd0;
        tcr <= 5'd0;  ocf <= 1'b0;  tof <= 1'b0;  ocf_armed <= 1'b0;  tof_armed <= 1'b0;
        rmcr <= 8'd0;  trcsr <= 8'h20;  tdr <= 8'd0;  ramc <= 2'b01;
    end else begin
        if (cen) ediv <= ediv + 2'd1;
        if (e_tick) begin
            frc <= frc + 16'd1;
            if (frc + 16'd1 == ocr) ocf <= 1'b1;
            if (frc == 16'hFFFF) tof <= 1'b1;
        end
        if (rstb && reg_cs) begin
            case (addr[4:0])
                TCSR: begin ocf_armed <= ocf; tof_armed <= tof; end
                FRCH: begin frc_lo_buf <= frc[7:0]; if (tof_armed) begin tof <= 1'b0; tof_armed <= 1'b0; end end
                default: ;
            endcase
        end
        if (wstb && reg_cs) begin
            case (addr[4:0])
                P1DDR: p1ddr <= dout;
                P2DDR: p2ddr <= dout[4:0];
                P1:    p1 <= dout;
                P2:    p2 <= dout[4:0];
                TCSR:  tcr <= dout[4:0];
                FRCH:  frc <= 16'hFFF8;                         // any write to the counter presets it
                OCRH:  begin ocr[15:8] <= dout; if (ocf_armed) begin ocf <= 1'b0; ocf_armed <= 1'b0; end end
                OCRL:  begin ocr[7:0]  <= dout; if (ocf_armed) begin ocf <= 1'b0; ocf_armed <= 1'b0; end end
                RMCR:  rmcr <= dout;
                TRCSR: trcsr <= {trcsr[7:5], dout[4:0]};
                TDR:   tdr <= dout;
                RAMC:  ramc <= dout[7:6];
                default: ;
            endcase
        end
    end
end

logic [7:0] reg_q;
always_comb begin
    case (addr[4:0])
        P1DDR:   reg_q = p1ddr;
        P2DDR:   reg_q = {3'b111, p2ddr};
        P1:      reg_q = (p1_in & ~p1ddr) | (p1 & p1ddr);
        P2:      reg_q = {mode, (p2_in & ~p2ddr) | (p2 & p2ddr)};
        TCSR:    reg_q = {1'b0, ocf, tof, tcr};
        FRCH:    reg_q = frc[15:8];
        FRCL:    reg_q = frc_lo_buf;
        OCRH:    reg_q = ocr[15:8];
        OCRL:    reg_q = ocr[7:0];
        ICRH:    reg_q = 8'd0;
        ICRL:    reg_q = 8'd0;
        RMCR:    reg_q = {rmcr[7], 3'b111, rmcr[3:0]};
        TRCSR:   reg_q = trcsr;
        RDR:     reg_q = 8'd0;
        TDR:     reg_q = tdr;
        RAMC:    reg_q = {ramc, 6'h3F};
        default: reg_q = 8'hFF;
    endcase
end

always_comb din = reg_cs ? reg_q : ram_cs ? ram_q : xdin;

endmodule
