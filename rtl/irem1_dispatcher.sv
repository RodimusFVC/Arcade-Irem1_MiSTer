//============================================================================
//
//  Irem1 - board top (dispatcher)
//  Copyright (C) 2026 Rodimus
//  Based on MAME irem/m52.cpp, m57.cpp, m58.cpp, travrusa.cpp (BSD-3-Clause; Nicola Salmoria,
//  Phil Stroffolino, Lee Taylor)
//
//  CPU, sound chip, ROM and RAM pools live here; boards in rtl/boards/ drive them through ports,
//  selected by variant (index 1 byte 0, numbering = tools/gen_mra.py SUPPORTED):
//    0  M52 board         mpatrol, mpatrolw, mranger                       M52-SOUND-C
//    1  M52, Alpha One mod (alpha1v)                                       M52-SOUND-C
//    2  M57 board         troangel, newtangl                               M52-SOUND-C
//    3  M58 board         10yard, 10yardj, vs10yard*, 10yard85             large sound map, 2x MSM5205
//    4  Traverse USA      travrusa, travrusab*, mototour, motorace         M52-SOUND-C
//    5  Shot Rider        shtrider, shtridera (own sprite layout / palette) M52-SOUND-C
//    6  Shot Rider bootleg shtriderb (+ I/O port 0x11 read)                M52-SOUND-C
//  CPU ROM decode (index 1 byte 2, on read): 1 = motorace (address + data bitswap), 2 = shtridera (data bitswap).
//  Every board: Z80 at 18.432 MHz / 6, 6.144 MHz pixel clock, 384 x 282 raster; one shared sound board
//  (M6803 at 3.579545 MHz, 2x AY-3-8910 at 3.579545 / 4, MSM5205 at 384 kHz).
//
//  Clock: 49.152 MHz = 8 x 6.144 MHz; the 3.579545 MHz sound clock is a fractional enable.
//
//============================================================================

module irem1_dispatcher
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

/////////////////////////////  Shared CPU / sound chip pool  /////////////////////////////

// Main Z80 (T80pa; /WAIT comes from the scrolling video board, schematic D-9454 / D-9456 CN2-10A)
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

// Sound MC6803 (rtl/cpu/m6803.sv around jt680x, Jose Tejada)
chip_pool_pkg::m6803_i_t m6803_0_i;
chip_pool_pkg::m6803_o_t m6803_0_o;

m6803 m6803_0
(
    .rst(reset), .clk(clk), .cen(m6803_0_i.cen), .irq(m6803_0_i.irq), .nmi(m6803_0_i.nmi),
    .addr(m6803_0_o.addr), .dout(m6803_0_o.dout), .wr(m6803_0_o.wr), .x_cs(m6803_0_o.x_cs), .xdin(m6803_0_i.xdin),
    .p1_in(m6803_0_i.p1_in), .p1_out(m6803_0_o.p1_out), .p2_in(m6803_0_i.p2_in), .p2_out(m6803_0_o.p2_out),
    .mode(m6803_0_i.mode)
);

// AY-3-8910 x 2 (jt49_bus, Jose Tejada): ay[0] = 45M (IOA = sound latch, IOB = MSM5205 control), ay[1] = 45L
// Output through the 470 ohm load (schematic R2 470); levels are mapped through that load curve in the boards.
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

// MSM5205 x 2: [0] every variant, [1] M58 only.
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

// Port A = CPU (or hiscore while paused), port B = video.
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

// Blocks: 0 wram (4K), 1 vram (4K), 2 xram (4K), 3 sram (512), 4 lbuf0 (512), 5 lbuf1 (512).

localparam RAM_BLOCKS = 6;
ram_pool_pkg::pram_i_t [RAM_BLOCKS-1:0] pool_i;
ram_pool_pkg::pram_o_t [RAM_BLOCKS-1:0] pool_o;

assign pool_i[0].a_addr = {3'd0, wram_aa}; assign pool_i[0].a_d = wram_da; assign pool_i[0].a_we = wram_wa;
assign pool_i[0].b_addr = {3'd0, wram_ab}; assign pool_i[0].b_d = 8'd0;    assign pool_i[0].b_we = 1'b0;
assign pool_i[1].a_addr = {3'd0, vram_aa}; assign pool_i[1].a_d = vram_da; assign pool_i[1].a_we = vram_wa;
assign pool_i[1].b_addr = {3'd0, vram_ab}; assign pool_i[1].b_d = 8'd0;    assign pool_i[1].b_we = 1'b0;
assign pool_i[2].a_addr = {3'd0, xram_aa}; assign pool_i[2].a_d = xram_da; assign pool_i[2].a_we = xram_wa;
assign pool_i[2].b_addr = {3'd0, xram_ab}; assign pool_i[2].b_d = 8'd0;    assign pool_i[2].b_we = 1'b0;
assign pool_i[3].a_addr = {6'd0, sram_aa}; assign pool_i[3].a_d = sram_da; assign pool_i[3].a_we = sram_wa;
assign pool_i[3].b_addr = {6'd0, sram_ab}; assign pool_i[3].b_d = 8'd0;    assign pool_i[3].b_we = 1'b0;
assign pool_i[4].a_addr = {6'd0, lb0_aa};  assign pool_i[4].a_d = lb0_da;  assign pool_i[4].a_we = lb0_wa;
assign pool_i[4].b_addr = {6'd0, lb0_ab};  assign pool_i[4].b_d = 8'd0;    assign pool_i[4].b_we = lb0_wb;
assign pool_i[5].a_addr = {6'd0, lb1_aa};  assign pool_i[5].a_d = lb1_da;  assign pool_i[5].a_we = lb1_wa;
assign pool_i[5].b_addr = {6'd0, lb1_ab};  assign pool_i[5].b_d = 8'd0;    assign pool_i[5].b_we = lb1_wb;
assign {wram_qa, wram_qb} = {pool_o[0].a_q, pool_o[0].b_q};
assign {vram_qa, vram_qb} = {pool_o[1].a_q, pool_o[1].b_q};
assign {xram_qa, xram_qb} = {pool_o[2].a_q, pool_o[2].b_q};
assign {sram_qa, sram_qb} = {pool_o[3].a_q, pool_o[3].b_q};
assign {lb0_qa, lb0_qb}   = {pool_o[4].a_q, pool_o[4].b_q};
assign {lb1_qa, lb1_qb}   = {pool_o[5].a_q, pool_o[5].b_q};

genvar rp;
generate
    for (rp = 0; rp < RAM_BLOCKS; rp++) begin : rpool
        localparam AW = (rp < 3) ? 12 : 9;       // blocks 0-2 4K, 3-5 512
        dpram_dc #(.widthad_a(AW)) ram
        (
            .clock_a(clk), .address_a(pool_i[rp].a_addr[AW-1:0]), .data_a(pool_i[rp].a_d), .wren_a(pool_i[rp].a_we),
            .q_a(pool_o[rp].a_q),
            .clock_b(clk), .address_b(pool_i[rp].b_addr[AW-1:0]), .data_b(pool_i[rp].b_d), .wren_b(pool_i[rp].b_we),
            .q_b(pool_o[rp].b_q)
        );
    end
endgenerate

/////////////////////////////  Boards  /////////////////////////////

// Boards drive the pools through their own port sets; the active one (by variant) is muxed onto the pools, the pool
// outputs go to every board. M52 = variants 0 (Moon Patrol) and 1 (Alpha One), M58 = variant 3 (10-Yard Fight),
// Traverse USA = variants 4-6 (Traverse USA family, Shot Rider, Shot Rider bootleg), M57 = variant 2 (Tropical Angel).
wire m58_sel  = variant == 8'd3;
wire trav_sel = variant >= 8'd4 && variant <= 8'd6;
wire m57_sel  = variant == 8'd2;
wire m52_sel  = ~m58_sel & ~trav_sel & ~m57_sel;

// per-board port bundles: [0] = M52, [1] = M58, [2] = Traverse USA, [3] = M57
chip_pool_pkg::z80pa_i_t bd_cpu_i[4];
logic [16:0] bd_main_rom_a[4], bd_vid_rom_a[4];
logic [11:0] bd_pal_a[4], bd_clut_a[4];
logic [11:0] bd_wram_a[4], bd_vram_aa[4], bd_vram_ab[4], bd_xram_aa[4], bd_xram_ab[4];
logic  [8:0] bd_sram_aa[4], bd_sram_ab[4], bd_lb_aa[4], bd_lb_ab[4];
logic  [7:0] bd_wram_d[4], bd_vram_da[4], bd_sram_da[4], bd_xram_da[4], bd_lb_da[4], bd_snd_cmd[4], bd_hs_q[4];
logic        bd_wram_we[4], bd_vram_wa[4], bd_sram_wa[4], bd_xram_wa[4], bd_snd_wr[4], bd_ce_pix[4];
logic  [1:0] bd_lb_wa[4], bd_lb_wb[4];
logic  [7:0] bd_r[4], bd_g[4], bd_b[4];
logic        bd_hs[4], bd_vs[4], bd_hbl[4], bd_vbl[4];
assign {bd_xram_aa[0], bd_xram_ab[0], bd_xram_da[0], bd_xram_wa[0]} = '0;   // M52 / Traverse USA: no extra RAM
assign {bd_xram_aa[2], bd_xram_ab[2], bd_xram_da[2], bd_xram_wa[2]} = '0;

m52_board m52
(
    .clk(clk), .reset(reset), .pause(pause & m52_sel), .alpha1v(variant == 8'd1), .rom_rgn(rom_rgn),
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
    .hs_address(hs_address), .hs_data_in(hs_data_in), .hs_data_out(bd_hs_q[0]), .hs_write(hs_write & m52_sel)
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

trav_board trav
(
    .clk(clk), .reset(reset), .pause(pause & trav_sel), .variant(variant), .rom_dec(rom_dec), .rom_rgn(rom_rgn),
    .in0(in0), .in1(in1), .in2(in2), .dsw1(dsw1), .dsw2(dsw2),
    .crt_flip(crt_flip), .h_adj(h_adj), .v_adj(v_adj),
    .ce_pix(bd_ce_pix[2]), .video_r(bd_r[2]), .video_g(bd_g[2]), .video_b(bd_b[2]),
    .video_hs(bd_hs[2]), .video_vs(bd_vs[2]), .video_hblank(bd_hbl[2]), .video_vblank(bd_vbl[2]),
    .cpu_i(bd_cpu_i[2]), .cpu_o(z80pa_0_o),
    .cpu_rom_a(bd_main_rom_a[2]), .cpu_rom_q(main_rom_q), .vid_rom_a(bd_vid_rom_a[2]), .vid_rom_q(rom0_qb),
    .pal_a(bd_pal_a[2]), .pal_q(rom1_qb), .clut_a(bd_clut_a[2]), .clut_q(rom1_qa),
    .wram_a(bd_wram_a[2]), .wram_d(bd_wram_d[2]), .wram_we(bd_wram_we[2]), .wram_q(wram_qa),
    .vram_aa(bd_vram_aa[2]), .vram_da(bd_vram_da[2]), .vram_wa(bd_vram_wa[2]), .vram_qa(vram_qa),
    .vram_ab(bd_vram_ab[2]), .vram_qb(vram_qb),
    .sram_aa(bd_sram_aa[2]), .sram_da(bd_sram_da[2]), .sram_wa(bd_sram_wa[2]), .sram_ab(bd_sram_ab[2]), .sram_qb(sram_qb),
    .lb_aa(bd_lb_aa[2]), .lb_da(bd_lb_da[2]), .lb_wa(bd_lb_wa[2]), .lb_ab(bd_lb_ab[2]), .lb_wb(bd_lb_wb[2]),
    .lb_qb0(lb0_qb), .lb_qb1(lb1_qb),
    .snd_wr(bd_snd_wr[2]), .snd_cmd(bd_snd_cmd[2]),
    .hs_address(hs_address), .hs_data_in(hs_data_in), .hs_data_out(bd_hs_q[2]), .hs_write(hs_write & trav_sel)
);

m57_board m57
(
    .clk(clk), .reset(reset), .pause(pause & m57_sel), .rom_rgn(rom_rgn),
    .in0(in0), .in1(in1), .in2(in2), .dsw1(dsw1), .dsw2(dsw2),
    .crt_flip(crt_flip), .h_adj(h_adj), .v_adj(v_adj),
    .ce_pix(bd_ce_pix[3]), .video_r(bd_r[3]), .video_g(bd_g[3]), .video_b(bd_b[3]),
    .video_hs(bd_hs[3]), .video_vs(bd_vs[3]), .video_hblank(bd_hbl[3]), .video_vblank(bd_vbl[3]),
    .cpu_i(bd_cpu_i[3]), .cpu_o(z80pa_0_o),
    .cpu_rom_a(bd_main_rom_a[3]), .cpu_rom_q(main_rom_q), .vid_rom_a(bd_vid_rom_a[3]), .vid_rom_q(rom0_qb),
    .pal_a(bd_pal_a[3]), .pal_q(rom1_qb), .clut_a(bd_clut_a[3]), .clut_q(rom1_qa),
    .wram_a(bd_wram_a[3]), .wram_d(bd_wram_d[3]), .wram_we(bd_wram_we[3]), .wram_q(wram_qa),
    .vram_aa(bd_vram_aa[3]), .vram_da(bd_vram_da[3]), .vram_wa(bd_vram_wa[3]), .vram_qa(vram_qa),
    .vram_ab(bd_vram_ab[3]), .vram_qb(vram_qb),
    .sram_aa(bd_sram_aa[3]), .sram_da(bd_sram_da[3]), .sram_wa(bd_sram_wa[3]), .sram_ab(bd_sram_ab[3]), .sram_qb(sram_qb),
    .xram_aa(bd_xram_aa[3]), .xram_da(bd_xram_da[3]), .xram_wa(bd_xram_wa[3]), .xram_qa(xram_qa),
    .xram_ab(bd_xram_ab[3]), .xram_qb(xram_qb),
    .lb_aa(bd_lb_aa[3]), .lb_da(bd_lb_da[3]), .lb_wa(bd_lb_wa[3]), .lb_ab(bd_lb_ab[3]), .lb_wb(bd_lb_wb[3]),
    .lb_qb0(lb0_qb), .lb_qb1(lb1_qb),
    .snd_wr(bd_snd_wr[3]), .snd_cmd(bd_snd_cmd[3]),
    .hs_address(hs_address), .hs_data_in(hs_data_in), .hs_data_out(bd_hs_q[3]), .hs_write(hs_write & m57_sel)
);

wire [1:0] bs = m57_sel ? 2'd3 : trav_sel ? 2'd2 : m58_sel ? 2'd1 : 2'd0;
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

// MSM5205 control uses the MAME map (4 kHz VCK / NMI) rather than the drawing's 8 kHz; do not change it.
irem_sound sound
(
    .clk(clk), .reset(reset), .pause(pause), .large_map(variant == 8'd3), .msm_mame_map(1'b1), .rom_rgn(rom_rgn),
    .cmd_wr(snd_wr), .cmd(snd_cmd),
    .cpu_i(m6803_0_i), .cpu_o(m6803_0_o), .rom_a(snd_rom_a), .rom_q(snd_rom_q),
    .ay_i(ay_i), .ay_o(ay_o), .msm_i(msm5205_i), .msm_o(msm5205_o),
    .audio(audio)
);

endmodule
