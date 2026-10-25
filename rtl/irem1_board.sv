//============================================================================
//
//  Irem1 — board top (dispatcher)
//  Copyright (C) 2026 Rodimus
//
//  Pools (CPUs, sound chips, ROM store, RAM) live here; boards in rtl/boards/ drive them through ports, selected by
//  variant (Projects/Arcade-Konami82_MiSTer/Claude/chip_pool_convention.md). Boards so far: M52 (variants 0-1,
//  rtl/boards/m52_board.sv) and the shared Irem sound board (rtl/boards/irem_sound.sv). Other variants: idle.
//
//  Planned variants (index 1 byte 0; numbering is tools/gen_mra.py SUPPORTED):
//    0  M52 board        (MAME irem/m52.cpp: mpatrol, mpatrolw, mranger)          M52-SOUND-C sound board
//    1  M52 board, Alpha One mod (m52.cpp alpha1v)                                M52-SOUND-C
//    2  M57 board        (irem/m57.cpp: troangel, newtangl)                       M52-SOUND-C (MAME "SOUNDC")
//    3  M58 board        (irem/m58.cpp: 10yard, 10yardj, vs10yard*, 10yard85)    M52 "large" sound map, 2x MSM5205
//    4  Traverse USA     (irem/travrusa.cpp: travrusa, travrusab*, mototour, motorace)  M52-SOUND-C
//    5  Shot Rider       (travrusa.cpp shtrider, shtridera: own sprite layout / palette)  M52-SOUND-C
//    6  Shot Rider bootleg (travrusa.cpp shtriderb: + I/O port 0x11 read)              M52-SOUND-C
//  CPU ROM decode (index 1 byte 2, decoded on read): 1 = motorace (init_motorace address + data bitswap),
//  2 = shtridera (init_shtridra data bitswap).
//  Every board: Z80 main at 18.432 MHz / 6, 6.144 MHz pixel clock, 384 x 282 raster; one shared sound board
//  (M6803 at 3.579545 MHz, 2x AY-3-8910 at 3.579545 / 4, MSM5205 at 384 kHz).
//
//  Clock: 49.152 MHz = 8 x 6.144 MHz (pixel / Z80 clock enables exact); the 3.579545 MHz sound clock is a fractional
//  enable (jtframe_frac_cen pattern, HDL/jt49 AY-3-8910 wiring in the vault).
//
//============================================================================

module irem1_board
(
    input  logic        clk,            // 49.152 MHz
    input  logic        reset,
    input  logic        pause,
    input  logic  [7:0] variant,
    input  logic  [3:0] rom_dec,
    input  logic  [7:0] game_flags2,
    input  logic [47:0][7:0] rom_rgn,   // index 1 bytes 64-111: ROM region table (Shared ROM store, below)
    output logic        ce_pix,

    input  logic  [7:0] in0,
    input  logic  [7:0] in1,
    input  logic  [7:0] in2,
    input  logic  [7:0] dsw1,
    input  logic  [7:0] dsw2,
    input  logic  [7:0] in5,

    input  logic [24:0] ioctl_addr,
    input  logic  [7:0] ioctl_dout,
    input  logic        ioctl_wr0,

    input  logic        crt_flip,
    input  logic  [3:0] h_adj,
    input  logic  [3:0] v_adj,

    output logic  [7:0] video_r,
    output logic  [7:0] video_g,
    output logic  [7:0] video_b,
    output logic        video_hs,
    output logic        video_vs,
    output logic        video_hblank,
    output logic        video_vblank,

    output logic signed [15:0] audio,

    input  logic [15:0] hs_address,
    input  logic  [7:0] hs_data_in,
    output logic  [7:0] hs_data_out,
    input  logic        hs_write
);

// boards and the sound board: end of this file (after the pools they connect to)

/////////////////////////////  Shared CPU / sound chip pool  /////////////////////////////

// Main Z80 (T80pa: the M52 CPU board takes /WAIT from the scrolling video board, schematic D-9454 / D-9456 CN2-10A):
// z80pa_0: every board's main CPU (M52, M57, M58, Traverse USA)
chip_pool_pkg::z80pa_i_t z80pa_0_i;
chip_pool_pkg::z80pa_o_t z80pa_0_o;

wire [15:0] z80pa_0_a;
wire  [7:0] z80pa_0_do;
wire        z80pa_0_m1_n, z80pa_0_mreq_n, z80pa_0_iorq_n, z80pa_0_rd_n, z80pa_0_wr_n, z80pa_0_rfsh_n;

T80pa #(.Mode(0)) z80pa_0
(
    .RESET_n (~reset & z80pa_0_i.reset_n), .CLK(clk), .CEN_p(z80pa_0_i.cen_p), .CEN_n(z80pa_0_i.cen_n),
    .WAIT_n  (z80pa_0_i.wait_n), .INT_n(z80pa_0_i.int_n), .NMI_n(z80pa_0_i.nmi_n), .BUSRQ_n(1'b1),
    .M1_n(z80pa_0_m1_n), .MREQ_n(z80pa_0_mreq_n), .IORQ_n(z80pa_0_iorq_n), .RD_n(z80pa_0_rd_n), .WR_n(z80pa_0_wr_n),
    .RFSH_n(z80pa_0_rfsh_n), .HALT_n(), .BUSAK_n(), .OUT0(1'b0), .A(z80pa_0_a), .DI(z80pa_0_i.di), .DO(z80pa_0_do),
    .R800_mode(1'b0), .REG(), .DIRSet(1'b0), .DIR(212'd0)
);

assign z80pa_0_o.a = z80pa_0_a;           assign z80pa_0_o.dout = z80pa_0_do;
assign z80pa_0_o.m1_n = z80pa_0_m1_n;     assign z80pa_0_o.mreq_n = z80pa_0_mreq_n; assign z80pa_0_o.iorq_n = z80pa_0_iorq_n;
assign z80pa_0_o.rd_n = z80pa_0_rd_n;     assign z80pa_0_o.wr_n = z80pa_0_wr_n;     assign z80pa_0_o.rfsh_n = z80pa_0_rfsh_n;

// M6803-2026-10-25: jtframe_6801mcu pool replaced by our rtl/cpu/m6803.sv (external-ROM boards). Original:
// // Sound M6803 (jtframe_6801mcu + jt680x, Jose Tejada, unmodified): m6801_0: the Irem sound board, every variant.
// // TODO(board): the 6803 runs on external ROM (small map 0x2000-0x7FFF mirrored, large map 0x4000-0xFFFF); the
// // wrapper's internal-ROM port (ROMW / rom_cs) and MODE handling must be checked against that before use.
// chip_pool_pkg::m6801_i_t m6801_0_i;
// chip_pool_pkg::m6801_o_t m6801_0_o;
// assign m6801_0_i = '{cen: 1'b0, cen_tmr: 1'b0, irq: 1'b0, nmi: 1'b0, xdin: 8'hFF, p1_din: 8'hFF, p2_din: 5'h1F,
//                      p3_din: 8'hFF, p4_din: 8'hFF, rom_data: 8'hFF};
//
// wire [15:0] m6801_0_addr;
// wire  [7:0] m6801_0_dout, m6801_0_p1, m6801_0_p3, m6801_0_p4;
// wire  [4:0] m6801_0_p2;
// wire        m6801_0_wr, m6801_0_xcs, m6801_0_romcs;
//
// jtframe_6801mcu m6801_0
// (
//     .rst(reset), .clk(clk), .cen(m6801_0_i.cen), .cen_tmr(m6801_0_i.cen_tmr), .irq(m6801_0_i.irq), .nmi(m6801_0_i.nmi),
//     .addr(m6801_0_addr), .xdin(m6801_0_i.xdin), .dout(m6801_0_dout), .wr(m6801_0_wr), .x_cs(m6801_0_xcs), .ba(),
//     .p1_din(m6801_0_i.p1_din), .p3_din(m6801_0_i.p3_din), .p4_din(m6801_0_i.p4_din),
//     .p1_dout(m6801_0_p1), .p3_dout(m6801_0_p3), .p4_dout(m6801_0_p4),
//     .p2_din(m6801_0_i.p2_din), .p2_dout(m6801_0_p2),
//     .rom_addr(), .rom_data(m6801_0_i.rom_data), .rom_cs(m6801_0_romcs)
// );
//
// assign m6801_0_o.addr = m6801_0_addr;     assign m6801_0_o.dout = m6801_0_dout;   assign m6801_0_o.wr = m6801_0_wr;
// assign m6801_0_o.x_cs = m6801_0_xcs;      assign m6801_0_o.rom_cs = m6801_0_romcs;
// assign m6801_0_o.p1_dout = m6801_0_p1;    assign m6801_0_o.p2_dout = m6801_0_p2;
// assign m6801_0_o.p3_dout = m6801_0_p3;    assign m6801_0_o.p4_dout = m6801_0_p4;

// Sound MC6803 (rtl/cpu/m6803.sv around jt680x, Jose Tejada): m6803_0: the Irem sound board, every variant
chip_pool_pkg::m6803_i_t m6803_0_i;
chip_pool_pkg::m6803_o_t m6803_0_o;

m6803 m6803_0
(
    .rst(reset), .clk(clk), .cen(m6803_0_i.cen), .irq(m6803_0_i.irq), .nmi(m6803_0_i.nmi),
    .addr(m6803_0_o.addr), .dout(m6803_0_o.dout), .wr(m6803_0_o.wr), .x_cs(m6803_0_o.x_cs), .xdin(m6803_0_i.xdin),
    .p1_in(m6803_0_i.p1_in), .p1_out(m6803_0_o.p1_out), .p2_in(m6803_0_i.p2_in), .p2_out(m6803_0_o.p2_out),
    .mode(m6803_0_i.mode)
);

// AY-3-8910 x 2 (jt49_bus, Jose Tejada): ay[0] = 45M (IOA = sound latch, IOB = MSM5205 control), ay[1] = 45L; every
// variant. MAME: M52-SOUND-C / large map = AY8910_SINGLE_OUTPUT | DISCRETE_OUTPUT, 470 ohm load (schematic R2 470):
// the boards map jt49 levels through that load curve (vault: AY channel levels need the board's output load curve).
chip_pool_pkg::ay_i_t [1:0] ay_i;
chip_pool_pkg::ay_o_t [1:0] ay_o;

genvar ai;
generate
    for (ai = 0; ai < 2; ai = ai + 1) begin : g_ay
        wire [7:0] dout, a, b, c, iob;
        wire [9:0] snd;
        jt49_bus ay
        (
            .rst_n(~reset), .clk(clk), .clk_en(ay_i[ai].ce), .bdir(ay_i[ai].bdir), .bc1(ay_i[ai].bc1), .din(ay_i[ai].din),
            .sel(1'b1), .dout(dout), .sound(snd), .A(a), .B(b), .C(c), .sample(),
            .IOA_in(ay_i[ai].ioa_in), .IOA_out(), .IOB_in(ay_i[ai].iob_in), .IOB_out(iob)
        );
        assign ay_o[ai].dout = dout; assign ay_o[ai].a = a; assign ay_o[ai].b = b; assign ay_o[ai].c = c;
        assign ay_o[ai].iob_out = iob; assign ay_o[ai].sound = snd;
    end
endgenerate

// MSM-REVERT-2026-10-25: jt5205 pool replaced by our msm5205.sv again (user: the Moon Patrol rock "thonk" was
// inaudible with jt5205). jt5205 block commented out below; MSM5205 x 2: [0] every variant, [1] M58 only.
chip_pool_pkg::msm5205_i_t [1:0] msm5205_i;
chip_pool_pkg::msm5205_o_t [1:0] msm5205_o;
genvar mi;
generate
    for (mi = 0; mi < 2; mi = mi + 1) begin : g_msm5205
        wire signed [11:0] signal;
        msm5205 msm
        (
            .clk(clk), .step(msm5205_i[mi].step), .rst(msm5205_i[mi].rst), .data(msm5205_i[mi].data), .signal(signal)
        );
        assign msm5205_o[mi].signal = signal;
    end
endgenerate

// // MSM5205 x 2 (jt5205, Jose Tejada): msm5205[0] = every variant, msm5205[1] = M58 (10-Yard Fight) only,
// // clocked by the first chip's VCK (MAME m52_large_audio_device). irq = VCK strobe -> the M6803 NMI.
// chip_pool_pkg::jt5205_i_t [1:0] msm5205_i;
// chip_pool_pkg::jt5205_o_t [1:0] msm5205_o;
//
// genvar mi;
// generate
//     for (mi = 0; mi < 2; mi = mi + 1) begin : g_msm5205
//         wire signed [11:0] sound;
//         wire irq, vclk;
//         jt5205 msm
//         (
//             .rst(msm5205_i[mi].rst), .clk(clk), .cen(msm5205_i[mi].cen), .sel(msm5205_i[mi].sel), .din(msm5205_i[mi].din),
//             .sound(sound), .sample(), .irq(irq), .vclk_o(vclk)
//         );
//         assign msm5205_o[mi].sound = sound;
//         assign msm5205_o[mi].irq   = irq;
//         assign msm5205_o[mi].vclk  = vclk;
//     end
// endgenerate

/////////////////////////////  Shared ROM blocks  /////////////////////////////

// One packed store (tools/gen_mra.py place()): rom0 = 128K, every CPU / graphics region of the set packed from 0;
// rom1 = 4K at index-0 address 0x20000, every colour / lookup PROM (decoded on read, never at load).
// Size = the largest single set (10-Yard Fight: 0x20000 + 0x520 PROM bytes), not the sum of region maxima.
// Region table (index 1 bytes 64-111), slot n at byte 64 + 6n: base, start, len (little-endian u16, 256-byte units):
// store address = base + (region offset - start); slots 0 main CPU, 1 sound CPU, 2 tiles (M52 tx), 3 sprites,
// 4-6 M52 backgrounds bg0-bg2, 7 PROMs (base relative to 0x20000). Load decode never depends on the variant.
logic [16:0] rom0_aa, rom0_ab;      // port A: second reader when not loading; port B: reader
logic  [7:0] rom0_qa, rom0_qb;
logic [11:0] rom1_aa, rom1_ab;
logic  [7:0] rom1_qa, rom1_qb;
// port A: main and sound CPU on alternate clocks (both hold their address for many clocks); data latched per CPU
logic [16:0] main_rom_a, snd_rom_a;
logic  [7:0] main_rom_q, snd_rom_q;
logic        rom_tog = 1'b0, rom_tog_d = 1'b0;
always_ff @(posedge clk) begin
    rom_tog   <= ~rom_tog;
    rom_tog_d <= rom_tog;
    if (rom_tog_d) snd_rom_q <= rom0_qa;
    else           main_rom_q <= rom0_qa;
end
assign rom0_aa = rom_tog ? snd_rom_a : main_rom_a;

wire ld_rom0 = ioctl_wr0 && ioctl_addr[24:17] == 8'd0;
wire ld_rom1 = ioctl_wr0 && ioctl_addr[24:12] == 13'h20;

dpram_dc #(.widthad_a(17)) rom0
(
    .clock_a(clk), .address_a(ld_rom0 ? ioctl_addr[16:0] : rom0_aa), .data_a(ioctl_dout), .wren_a(ld_rom0), .q_a(rom0_qa),
    .clock_b(clk), .address_b(rom0_ab), .data_b(8'd0), .wren_b(1'b0), .q_b(rom0_qb)
);

dpram_dc #(.widthad_a(12)) rom1
(
    .clock_a(clk), .address_a(ld_rom1 ? ioctl_addr[11:0] : rom1_aa), .data_a(ioctl_dout), .wren_a(ld_rom1), .q_a(rom1_qa),
    .clock_b(clk), .address_b(rom1_ab), .data_b(8'd0), .wren_b(1'b0), .q_b(rom1_qb)
);

/////////////////////////////  Shared RAM pool  /////////////////////////////

// Sized to the largest user (MAME memory maps): port A = CPU (or hiscore while paused), port B = video.
// wram   4K: main work RAM          (M52 / M57 E000-E7FF 2K; M58 / Traverse USA / alpha1v E000-EFFF 4K)
// vram   4K: tile video + colour RAM (M52 8000-87FF; M57 8000-87FF; M58 / Traverse USA 8000-8FFF)
// sram  512: sprite RAM             (M52 C800-C8FF (+alpha1v C9FF); M57 C820-C8FF; M58 C820-C87F; Traverse USA C800-C9FF)
// xram   4K: board extra RAM        (M57 scroll RAM 9000-91FF; M58 score-panel bitmap 9000-9FFF, write-only)
// lbuf  2x512: sprite line buffers  (M52 character board: two banks of 4x 2125, swapped on V1; others alike)
logic [11:0] wram_aa, wram_ab, vram_aa, vram_ab, xram_aa, xram_ab;
logic  [8:0] sram_aa, sram_ab, lb0_aa, lb0_ab, lb1_aa, lb1_ab;
logic  [7:0] wram_da, vram_da, xram_da, sram_da, lb0_da, lb1_da;
logic        wram_wa, vram_wa, xram_wa, sram_wa, lb0_wa, lb1_wa;
logic  [7:0] wram_qa, wram_qb, vram_qa, vram_qb, xram_qa, xram_qb, sram_qa, sram_qb, lb0_qa, lb0_qb, lb1_qa, lb1_qb;
logic        lb0_wb, lb1_wb;                     // line buffers: port B clears behind the display read
assign wram_ab = '0;

dpram_dc #(.widthad_a(12)) wram
(
    .clock_a(clk), .address_a(wram_aa), .data_a(wram_da), .wren_a(wram_wa), .q_a(wram_qa),
    .clock_b(clk), .address_b(wram_ab), .data_b(8'd0), .wren_b(1'b0), .q_b(wram_qb)
);

dpram_dc #(.widthad_a(12)) vram
(
    .clock_a(clk), .address_a(vram_aa), .data_a(vram_da), .wren_a(vram_wa), .q_a(vram_qa),
    .clock_b(clk), .address_b(vram_ab), .data_b(8'd0), .wren_b(1'b0), .q_b(vram_qb)
);

dpram_dc #(.widthad_a(12)) xram
(
    .clock_a(clk), .address_a(xram_aa), .data_a(xram_da), .wren_a(xram_wa), .q_a(xram_qa),
    .clock_b(clk), .address_b(xram_ab), .data_b(8'd0), .wren_b(1'b0), .q_b(xram_qb)
);

dpram_dc #(.widthad_a(9)) sram
(
    .clock_a(clk), .address_a(sram_aa), .data_a(sram_da), .wren_a(sram_wa), .q_a(sram_qa),
    .clock_b(clk), .address_b(sram_ab), .data_b(8'd0), .wren_b(1'b0), .q_b(sram_qb)
);

dpram_dc #(.widthad_a(9)) lbuf0
(
    .clock_a(clk), .address_a(lb0_aa), .data_a(lb0_da), .wren_a(lb0_wa), .q_a(lb0_qa),
    .clock_b(clk), .address_b(lb0_ab), .data_b(8'd0), .wren_b(lb0_wb), .q_b(lb0_qb)
);

dpram_dc #(.widthad_a(9)) lbuf1
(
    .clock_a(clk), .address_a(lb1_aa), .data_a(lb1_da), .wren_a(lb1_wa), .q_a(lb1_qa),
    .clock_b(clk), .address_b(lb1_ab), .data_b(8'd0), .wren_b(lb1_wb), .q_b(lb1_qb)
);

/////////////////////////////  Boards  /////////////////////////////

// Boards drive the pools through their own port sets; the active one (by variant) is muxed onto the pools, the pool
// outputs go to every board. M52 = variants 0 (Moon Patrol) and 1 (Alpha One), M58 = variant 3 (10-Yard Fight).
wire m58_sel = variant == 8'd3;

// per-board port bundles: [0] = M52, [1] = M58
chip_pool_pkg::z80pa_i_t bd_cpu_i[2];
logic [16:0] bd_main_rom_a[2], bd_vid_rom_a[2];
logic [11:0] bd_pal_a[2], bd_clut_a[2];
logic [11:0] bd_wram_a[2], bd_vram_aa[2], bd_vram_ab[2], bd_xram_aa[2], bd_xram_ab[2];
logic  [8:0] bd_sram_aa[2], bd_sram_ab[2], bd_lb_aa[2], bd_lb_ab[2];
logic  [7:0] bd_wram_d[2], bd_vram_da[2], bd_sram_da[2], bd_xram_da[2], bd_lb_da[2], bd_snd_cmd[2], bd_hs_q[2];
logic        bd_wram_we[2], bd_vram_wa[2], bd_sram_wa[2], bd_xram_wa[2], bd_snd_wr[2], bd_ce_pix[2];
logic  [1:0] bd_lb_wa[2], bd_lb_wb[2];
logic  [7:0] bd_r[2], bd_g[2], bd_b[2];
logic        bd_hs[2], bd_vs[2], bd_hbl[2], bd_vbl[2];
assign {bd_xram_aa[0], bd_xram_ab[0], bd_xram_da[0], bd_xram_wa[0]} = '0;   // M52 has no extra RAM

m52_board m52
(
    .clk(clk), .reset(reset), .pause(pause & ~m58_sel), .alpha1v(variant == 8'd1), .rom_rgn(rom_rgn),
    .in0(in0), .in1(in1), .in2(in2), .dsw1(dsw1), .dsw2(dsw2),
    .crt_flip(crt_flip), .h_adj(h_adj), .v_adj(v_adj),
    .ce_pix(bd_ce_pix[0]), .video_r(bd_r[0]), .video_g(bd_g[0]), .video_b(bd_b[0]),
    .video_hs(bd_hs[0]), .video_vs(bd_vs[0]), .video_hblank(bd_hbl[0]), .video_vblank(bd_vbl[0]),
    .cpu_i(bd_cpu_i[0]), .cpu_o(z80pa_0_o),
    .cpu_rom_a(bd_main_rom_a[0]), .cpu_rom_q(main_rom_q), .vid_rom_a(bd_vid_rom_a[0]), .vid_rom_q(rom0_qb),
    .pal_a(bd_pal_a[0]), .pal_q(rom1_qb), .clut_a(bd_clut_a[0]), .clut_q(rom1_qa),
    .wram_a(bd_wram_a[0]), .wram_d(bd_wram_d[0]), .wram_we(bd_wram_we[0]), .wram_q(wram_qa),
    .vram_aa(bd_vram_aa[0]), .vram_da(bd_vram_da[0]), .vram_wa(bd_vram_wa[0]), .vram_qa(vram_qa),
    .vram_ab(bd_vram_ab[0]), .vram_qb(vram_qb),
    .sram_aa(bd_sram_aa[0]), .sram_da(bd_sram_da[0]), .sram_wa(bd_sram_wa[0]), .sram_ab(bd_sram_ab[0]), .sram_qb(sram_qb),
    .lb_aa(bd_lb_aa[0]), .lb_da(bd_lb_da[0]), .lb_wa(bd_lb_wa[0]), .lb_ab(bd_lb_ab[0]), .lb_wb(bd_lb_wb[0]),
    .lb_qb0(lb0_qb), .lb_qb1(lb1_qb),
    .snd_wr(bd_snd_wr[0]), .snd_cmd(bd_snd_cmd[0]),
    .hs_address(hs_address), .hs_data_in(hs_data_in), .hs_data_out(bd_hs_q[0]), .hs_write(hs_write & ~m58_sel)
);

m58_board m58
(
    .clk(clk), .reset(reset), .pause(pause & m58_sel), .rom_rgn(rom_rgn),
    .in0(in0), .in1(in1), .in2(in2), .dsw1(dsw1), .dsw2(dsw2),
    .crt_flip(crt_flip), .h_adj(h_adj), .v_adj(v_adj),
    .ce_pix(bd_ce_pix[1]), .video_r(bd_r[1]), .video_g(bd_g[1]), .video_b(bd_b[1]),
    .video_hs(bd_hs[1]), .video_vs(bd_vs[1]), .video_hblank(bd_hbl[1]), .video_vblank(bd_vbl[1]),
    .cpu_i(bd_cpu_i[1]), .cpu_o(z80pa_0_o),
    .cpu_rom_a(bd_main_rom_a[1]), .cpu_rom_q(main_rom_q), .vid_rom_a(bd_vid_rom_a[1]), .vid_rom_q(rom0_qb),
    .pal_a(bd_pal_a[1]), .pal_q(rom1_qb), .clut_a(bd_clut_a[1]), .clut_q(rom1_qa),
    .wram_a(bd_wram_a[1]), .wram_d(bd_wram_d[1]), .wram_we(bd_wram_we[1]), .wram_q(wram_qa),
    .vram_aa(bd_vram_aa[1]), .vram_da(bd_vram_da[1]), .vram_wa(bd_vram_wa[1]), .vram_qa(vram_qa),
    .vram_ab(bd_vram_ab[1]), .vram_qb(vram_qb),
    .sram_aa(bd_sram_aa[1]), .sram_da(bd_sram_da[1]), .sram_wa(bd_sram_wa[1]), .sram_qa(sram_qa),
    .sram_ab(bd_sram_ab[1]), .sram_qb(sram_qb),
    .xram_aa(bd_xram_aa[1]), .xram_da(bd_xram_da[1]), .xram_wa(bd_xram_wa[1]), .xram_ab(bd_xram_ab[1]), .xram_qb(xram_qb),
    .lb_aa(bd_lb_aa[1]), .lb_da(bd_lb_da[1]), .lb_wa(bd_lb_wa[1]), .lb_ab(bd_lb_ab[1]), .lb_wb(bd_lb_wb[1]),
    .lb_qb0(lb0_qb), .lb_qb1(lb1_qb),
    .snd_wr(bd_snd_wr[1]), .snd_cmd(bd_snd_cmd[1]),
    .hs_address(hs_address), .hs_data_in(hs_data_in), .hs_data_out(bd_hs_q[1]), .hs_write(hs_write & m58_sel)
);

wire bs = m58_sel;
assign z80pa_0_i  = bd_cpu_i[bs];
assign main_rom_a = bd_main_rom_a[bs];
assign rom0_ab    = bd_vid_rom_a[bs];
assign rom1_ab    = bd_pal_a[bs];
assign rom1_aa    = bd_clut_a[bs];
assign {wram_aa, wram_da, wram_wa}          = {bd_wram_a[bs], bd_wram_d[bs], bd_wram_we[bs]};
assign {vram_aa, vram_da, vram_wa, vram_ab} = {bd_vram_aa[bs], bd_vram_da[bs], bd_vram_wa[bs], bd_vram_ab[bs]};
assign {sram_aa, sram_da, sram_wa, sram_ab} = {bd_sram_aa[bs], bd_sram_da[bs], bd_sram_wa[bs], bd_sram_ab[bs]};
assign {xram_aa, xram_da, xram_wa, xram_ab} = {bd_xram_aa[bs], bd_xram_da[bs], bd_xram_wa[bs], bd_xram_ab[bs]};
assign {lb0_aa, lb1_aa} = {bd_lb_aa[bs], bd_lb_aa[bs]};
assign {lb0_da, lb1_da} = {bd_lb_da[bs], bd_lb_da[bs]};
assign {lb1_wa, lb0_wa} = bd_lb_wa[bs];
assign {lb0_ab, lb1_ab} = {bd_lb_ab[bs], bd_lb_ab[bs]};
assign {lb1_wb, lb0_wb} = bd_lb_wb[bs];
wire       snd_wr  = bd_snd_wr[bs];
wire [7:0] snd_cmd = bd_snd_cmd[bs];
assign hs_data_out = bd_hs_q[bs];
assign ce_pix = bd_ce_pix[bs];
assign {video_r, video_g, video_b} = {bd_r[bs], bd_g[bs], bd_b[bs]};
assign {video_hs, video_vs, video_hblank, video_vblank} = {bd_hs[bs], bd_vs[bs], bd_hbl[bs], bd_vbl[bs]};

// Irem sound board (every variant; M58 = large map). MSM5205 control map = MAME's: Moon Patrol writes 3J port B = 0x13
// once; MAME's map gives a 4 kHz VCK / NMI, the drawing's IOB1 = S1 reading gives 8 kHz, and the sim's music tracks
// MAME only with 4 kHz (envelope correlation 0.86 vs 0.41, sim 2026-10-25). The drawing's S1 / S2 / 4B ground pads,
// if fitted, also give 4 kHz. Either way the MSM5205 is held in reset (bit 0) in Moon Patrol.
irem_sound sound
(
    .clk(clk), .reset(reset), .pause(pause), .large_map(variant == 8'd3), .msm_mame_map(1'b1), .rom_rgn(rom_rgn),
    .cmd_wr(snd_wr), .cmd(snd_cmd),
    .cpu_i(m6803_0_i), .cpu_o(m6803_0_o), .rom_a(snd_rom_a), .rom_q(snd_rom_q),
    .ay_i(ay_i), .ay_o(ay_o), .msm_i(msm5205_i), .msm_o(msm5205_o),
    .audio(audio)
);

endmodule
