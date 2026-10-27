//============================================================================
//
//  m52_board - Irem M52 board set (Moon Patrol; Alpha One = the same board, variant 1)
//  Copyright (C) 2026 Rodimus
//  Based on MAME irem/m52.cpp (BSD-3-Clause; Nicola Salmoria) and the Williams drawing set 16-3007-103
//  (CPU M52-A-C, character M52-B-C, scrolling video M52-E-C).
//
//  Drives the dispatcher's pools through ports: main Z80 (z80pa), packed ROM store (CPU via the port-A arbiter,
//  video on port B, PROMs on rom1), RAM pool.
//
//  Raster: 384 x 282 at 6.144 MHz (8 clocks of 49.152 MHz per pixel). Visible x 136-375, y 18-273 (Alpha One 16-271).
//  Z80 at 3.072 MHz, IRQ0 at VBLANK start, held until acknowledged.
//  Layers: backdrop = sprite pen 0, mountains (bg0, pos 1), hills (bg1) or city (bg2) on pos 0, text layer
//  (rows 0-6 opaque), sprites on top.
//  Pixel pipeline: pixel (h, v) is fetched in the 8 clocks of h, its colour read at the start of h + 1 and output
//  during h + 2; blank and sync are delayed by the same two pixels.
//  Flip (game flip XOR CRT Flip): every layer is computed at the mirrored game position, so the picture turns 180 degrees.
//  Not modelled: the scrolling board's Z80 /WAIT (drawing D-9456).
//
//============================================================================

module m52_board
(
    input  logic        clk,                // 49.152 MHz
    input  logic        reset,
    input  logic        pause,
    input  logic        alpha1v,            // variant 1: Alpha One (ROM to 6FFF, RAM to EFFF, 128 sprites, no bg fill)
    input  logic [47:0][7:0] rom_rgn,

    input  logic  [7:0] in0, in1, in2, dsw1, dsw2,
    input  logic        crt_flip,
    input  logic  [3:0] h_adj,
    input  logic  [3:0] v_adj,

    output logic        ce_pix,
    output logic  [7:0] video_r, video_g, video_b,
    output logic        video_hs, video_vs, video_hblank, video_vblank,

    // main CPU (pool z80pa_0)
    output chip_pool_pkg::z80pa_i_t cpu_i,
    input  chip_pool_pkg::z80pa_o_t cpu_o,

    // ROM store
    output logic [16:0] cpu_rom_a,          // rom0 via the dispatcher's port-A arbiter
    input  logic  [7:0] cpu_rom_q,
    output logic [16:0] vid_rom_a,          // rom0 port B
    input  logic  [7:0] vid_rom_q,
    output logic [11:0] pal_a,              // rom1 port B (colour PROMs)
    input  logic  [7:0] pal_q,
    output logic [11:0] clut_a,             // rom1 port A (sprite lookup PROM)
    input  logic  [7:0] clut_q,

    // RAM pool
    output logic [11:0] wram_a,   output logic [7:0] wram_d,  output logic wram_we,  input logic [7:0] wram_q,
    output logic [11:0] vram_aa,  output logic [7:0] vram_da, output logic vram_wa,  input logic [7:0] vram_qa,
    output logic [11:0] vram_ab,                                                     input logic [7:0] vram_qb,
    output logic  [8:0] sram_aa,  output logic [7:0] sram_da, output logic sram_wa,
    output logic  [8:0] sram_ab,                                                     input logic [7:0] sram_qb,
    output logic  [8:0] lb_aa,    output logic [7:0] lb_da,   output logic [1:0] lb_wa,
    output logic  [8:0] lb_ab,                                output logic [1:0] lb_wb,
    input  logic  [7:0] lb_qb0,   input  logic [7:0] lb_qb1,

    // sound board
    output logic        snd_wr,
    output logic  [7:0] snd_cmd,

    // hiscore (RAM through the CPU port while paused)
    input  logic [15:0] hs_address,
    input  logic  [7:0] hs_data_in,
    output logic  [7:0] hs_data_out,
    input  logic        hs_write
);

/////////////////////////////  Region table  /////////////////////////////

// slot n at bytes 6n: base, start, len (u16 LE, 256-byte units)
wire [16:0] rom_base  = {rom_rgn[1][0],  rom_rgn[0],  8'h00};
wire [16:0] rom_start = {rom_rgn[3][0],  rom_rgn[2],  8'h00};
wire [16:0] rom_len   = {rom_rgn[5][0],  rom_rgn[4],  8'h00};
wire [16:0] tx_base   = {rom_rgn[13][0], rom_rgn[12], 8'h00};
wire [16:0] sp_base   = {rom_rgn[19][0], rom_rgn[18], 8'h00};
wire [16:0] sp_len    = {rom_rgn[23][0], rom_rgn[22], 8'h00};
wire [16:0] bg0_base  = {rom_rgn[25][0], rom_rgn[24], 8'h00};
wire [16:0] bg0_len   = {rom_rgn[29][0], rom_rgn[28], 8'h00};
wire [16:0] bg1_base  = {rom_rgn[31][0], rom_rgn[30], 8'h00};
wire [16:0] bg1_len   = {rom_rgn[35][0], rom_rgn[34], 8'h00};
wire [16:0] bg2_base  = {rom_rgn[37][0], rom_rgn[36], 8'h00};
wire [16:0] bg2_len   = {rom_rgn[41][0], rom_rgn[40], 8'h00};
wire [11:0] prom      = {rom_rgn[42][3:0], 8'h00};
wire [11:0] TX_PAL = prom, BG_PAL = prom + 12'h200, SP_PAL = prom + 12'h220, SP_CLUT = prom + 12'h240;

/////////////////////////////  Raster  /////////////////////////////

logic [2:0] ph = 3'd0;
logic [8:0] hcnt = 9'd0, vcnt = 9'd0;
assign ce_pix = ph == 3'd7;
always_ff @(posedge clk) begin
    ph <= ph + 3'd1;
    if (ce_pix) begin
        if (hcnt == 9'd383) begin
            hcnt <= 9'd0;
            vcnt <= (vcnt == 9'd281) ? 9'd0 : vcnt + 9'd1;
        end else hcnt <= hcnt + 9'd1;
    end
end

wire [8:0] vtop = alpha1v ? 9'd16  : 9'd18;
wire [8:0] vbot = alpha1v ? 9'd271 : 9'd273;
wire hblank_raw = hcnt < 9'd136 || hcnt >= 9'd376;
wire vblank_raw = vcnt < vtop   || vcnt > vbot;
// HSYNC 32 px from x 24 (+2 x H adjust: 8-38, blanking 376-135 wraps, both porches > 0); VSYNC 3 lines from y 277
// (V adjust clamped to +-3: 274-280, both porches > 0)
wire [8:0] hs_start = 9'd24 + {{4{h_adj[3]}}, h_adj, 1'b0};
wire [3:0] vadj_c   = ($signed(v_adj) > 4'sd3) ? 4'd3 : ($signed(v_adj) < -4'sd3) ? 4'b1101 : v_adj;
wire [8:0] vs_start = 9'd277 + {{5{vadj_c[3]}}, vadj_c};
wire [8:0] vs_rel   = (vcnt >= vs_start) ? vcnt - vs_start : vcnt + 9'd282 - vs_start;
wire hsync_raw = hcnt >= hs_start && hcnt < hs_start + 9'd32;
wire vsync_raw = vs_rel < 9'd3;

/////////////////////////////  Main CPU  /////////////////////////////

wire [15:0] a = cpu_o.a;
wire mem_rd = ~cpu_o.mreq_n & ~cpu_o.rd_n & cpu_o.rfsh_n;
wire mem_wr = ~cpu_o.mreq_n & ~cpu_o.wr_n & cpu_o.rfsh_n;
wire io_wr  = ~cpu_o.iorq_n & ~cpu_o.wr_n & cpu_o.m1_n;
// interrupt acknowledge = M1 + IORQ with MREQ high: T80pa's IntCycleD_n has no reset value, so the first M1 after
// power-up drives IORQ_n low together with MREQ_n (no real Z80 does); without the MREQ term that fetch read FF = RST 38
wire int_ack = ~cpu_o.iorq_n & ~cpu_o.m1_n & cpu_o.mreq_n;
logic mem_wr_d = 1'b0, io_wr_d = 1'b0;
always_ff @(posedge clk) begin mem_wr_d <= mem_wr; io_wr_d <= io_wr; end
wire mem_wstb = mem_wr & ~mem_wr_d;
wire io_wstb  = io_wr & ~io_wr_d;

// 3.072 MHz: CEN_p / CEN_n half a cycle apart (8 clocks), locked to the pixel clock as on the board
wire [3:0] c16 = {hcnt[0], ph};
logic int_n = 1'b1;
always_ff @(posedge clk) begin
    if (reset)                                           int_n <= 1'b1;
    else if (int_ack)                                    int_n <= 1'b1;
    else if (ce_pix && hcnt == 9'd383 && vcnt == vbot)   int_n <= 1'b0;       // start of the first VBLANK line
end

wire sel_rom  = alpha1v ? a < 16'h7000 : a[15:14] == 2'b00;
wire sel_vram = a[15:11] == 5'b10000;                                         // 8000-87FF
wire sel_prot = a[15:11] == 5'b10001;                                         // 8800-8FFF
wire sel_sprt = a[15:11] == 5'b11001;                                         // C800-CFFF
wire sel_io   = a[15:11] == 5'b11010;                                         // D000-D7FF
wire sel_wram = alpha1v ? a[15:12] == 4'hE : a[15:11] == 5'b11100;            // E000-E7FF / EFFF

// ROM: region slot 0
wire [16:0] rom_off = {1'b0, a} - rom_start;
assign cpu_rom_a = rom_base + rom_off;
wire rom_in = !rom_off[16] && rom_off < rom_len;

// scroll / background registers (I/O ports, A7-A5), flip latch, protection
logic [7:0] scroll, bgxpos0, bgypos0, bgxpos1, bgypos1, bgctrl;
logic       flip_reg;
always_ff @(posedge clk) begin
    if (reset) begin
        scroll <= 8'd0;  bgxpos0 <= 8'd0;  bgypos0 <= 8'd0;  bgxpos1 <= 8'd0;  bgypos1 <= 8'd0;  bgctrl <= 8'd0;
        flip_reg <= 1'b0;
    end else begin
        if (io_wstb) case (a[7:5])
            3'd0: scroll  <= cpu_o.dout;
            3'd2: bgxpos0 <= cpu_o.dout;
            3'd3: bgypos0 <= cpu_o.dout;
            3'd4: bgxpos1 <= cpu_o.dout;
            3'd5: bgypos1 <= cpu_o.dout;
            3'd6: bgctrl  <= cpu_o.dout;
            default: ;
        endcase
        if (mem_wstb && sel_io && a[1:0] == 2'd1) flip_reg <= cpu_o.dout[0];  // D001: flip (+ coin counters)
    end
end
assign snd_wr  = mem_wstb && sel_io && a[1:0] == 2'd0;                         // D000: sound command
assign snd_cmd = cpu_o.dout;
wire [2:0] pop7 = 3'(bgxpos0[0]) + 3'(bgxpos0[1]) + 3'(bgxpos0[2]) + 3'(bgxpos0[3]) + 3'(bgxpos0[4]) +
                  3'(bgxpos0[5]) + 3'(bgxpos0[6]);
wire [7:0] prot = {5'd0, pop7 ^ {2'b00, bgxpos0[7]}};         // protection read: popcount(x & 7F) ^ (x >> 7)

// RAM ports A: CPU, or the hiscore engine while paused (same address map)
wire [15:0] ra = pause ? hs_address : a;
wire        ra_wram = alpha1v ? ra[15:12] == 4'hE : ra[15:11] == 5'b11100;
wire        ra_vram = ra[15:11] == 5'b10000;
assign wram_a  = ra[11:0];
assign wram_d  = pause ? hs_data_in : cpu_o.dout;
assign wram_we = pause ? hs_write & ra_wram : mem_wr & sel_wram;
assign vram_aa = {1'b0, ra[10:0]};
assign vram_da = wram_d;
assign vram_wa = pause ? hs_write & ra_vram : mem_wr & sel_vram;
assign hs_data_out = ra_vram ? vram_qa : wram_q;
assign sram_aa = a[8:0];
assign sram_da = cpu_o.dout;
assign sram_wa = mem_wr & sel_sprt & ~a[9];          // C800-C9FF kept (video reads 000-0FF, Alpha One 000-1FF)

logic [7:0] io_q;
always_comb case (a[2:0])
    3'd0: io_q = in0;   3'd1: io_q = in1;   3'd2: io_q = in2;   3'd3: io_q = dsw1;   3'd4: io_q = dsw2;
    default: io_q = 8'hFF;
endcase

logic [7:0] cpu_di;
always_comb begin
    if (int_ack)       cpu_di = 8'hFF;
    else if (sel_rom)  cpu_di = rom_in ? cpu_rom_q : 8'hFF;
    else if (sel_vram) cpu_di = vram_qa;
    else if (sel_prot) cpu_di = prot;
    else if (sel_io)   cpu_di = io_q;
    else if (sel_wram) cpu_di = wram_q;
    else               cpu_di = alpha1v ? 8'h00 : 8'hFF;    // Alpha One reads 7107 at boot and needs 00 here
end

// per-field assigns: Quartus 17 rejects assignment patterns with non-constant members (error 10734)
assign cpu_i.reset_n = 1'b1;
assign cpu_i.cen_p   = c16 == 4'd0 && !pause;
assign cpu_i.cen_n   = c16 == 4'd8 && !pause;
assign cpu_i.wait_n  = 1'b1;
assign cpu_i.int_n   = int_n;
assign cpu_i.nmi_n   = 1'b1;
assign cpu_i.di      = cpu_di;

/////////////////////////////  Game position  /////////////////////////////

wire       game_flip = alpha1v ? flip_reg : flip_reg ^ ~dsw2[0];
wire       flip = game_flip ^ crt_flip;
wire [8:0] gx = flip ? 9'd511 - hcnt : hcnt;
wire [8:0] gy = flip ? vtop + vbot - vcnt : vcnt;     // mirror about the visible window's centre
wire [8:0] vn = (vcnt == 9'd281) ? 9'd0 : vcnt + 9'd1;
wire [8:0] gyn = flip ? vtop + vbot - vn : vn;              // the line the sprite engine renders

/////////////////////////////  Per-pixel fetch (8 clocks)  /////////////////////////////

// text layer: tilemap y = gy - 16; x = gx + 128, minus the scroll register on the scrolling band(s)
wire [7:0] ty   = 8'(gy - 9'd16);
wire       band = alpha1v ? ty[7:6] != 2'd0 : ty[7:6] == 2'd3;
wire [7:0] tx   = 8'(gx + 9'd128) - (band ? scroll : 8'd0);
logic [7:0] t_vid, t_col, t_lsb, t_msb, b0_byte, b12_byte, spr_px;
wire  [8:0] t_code_now = {vram_qb[7], t_vid};        // ph2: colour byte arriving
wire  [8:0] t_code     = {t_col[7], t_vid};

// backgrounds: image x = gx - (xpos + 124), y = gy - (ypos + 16)
wire [7:0] b0x = 8'(gx - 9'd124 - {1'b0, bgxpos1});
wire [9:0] b0y = 10'({1'b0, gy} - 10'd16 - {2'b00, bgypos1});
wire [7:0] b1x = 8'(gx - 9'd124 - {1'b0, bgxpos0});
wire [9:0] b1y = 10'({1'b0, gy} - 10'd16 - {2'b00, bgypos0});
wire       city = bgctrl[1];                           // hills hidden -> city (if enabled)
wire [16:0] b12_base = city ? bg2_base : bg1_base;

always_comb begin
    vram_ab   = {2'b00, ty[7:3], tx[7:3]};
    vid_rom_a = spr_rom_a;
    case (ph)
        3'd0: vram_ab   = {2'b00, ty[7:3], tx[7:3]};                         // video byte
        3'd1: vram_ab   = {2'b01, ty[7:3], tx[7:3]};                         // colour byte
        3'd2: vid_rom_a = tx_base + {5'd0, t_code_now, ty[2:0]};             // LSB plane
        3'd3: vid_rom_a = tx_base + 17'h1000 + {5'd0, t_code, ty[2:0]};      // MSB plane
        3'd4: vid_rom_a = bg0_base + {4'd0, b0y[6:0], b0x[7:2]};
        3'd5: vid_rom_a = b12_base + {4'd0, b1y[6:0], b1x[7:2]};
        default: ;                                                            // 6, 7, 0, 1: sprite engine
    endcase
end
wire spr_slot = ph == 3'd6 || ph == 3'd7 || ph == 3'd0 || ph == 3'd1;

assign lb_ab = gx;
assign lb_wb = (ph == 3'd6) ? (gy[0] ? 2'b10 : 2'b01) : 2'b00;               // clear behind the read

always_ff @(posedge clk) begin
    case (ph)
        3'd1: t_vid <= vram_qb;
        3'd2: t_col <= vram_qb;
        3'd3: t_lsb <= vid_rom_q;
        3'd4: t_msb <= vid_rom_q;
        3'd5: b0_byte <= vid_rom_q;
        3'd6: begin b12_byte <= vid_rom_q; spr_px <= gy[0] ? lb_qb1 : lb_qb0; end
        default: ;
    endcase
end

// pens (valid in ph7)
wire [2:0] txb  = 3'd7 - tx[2:0];
wire [1:0] t_pen = {t_msb[txb], t_lsb[txb]};
wire       t_opq = t_pen != 2'd0 || ty < 8'd56;                              // rows 0-6 opaque
// image 256 x 128, 64 bytes per row; rows past the loaded ROM read FF = pen 3
// (Moon Patrol: one 4K ROM = rows 0-63; Alpha One: two = rows 0-127; Alpha One's bg2 region is empty)
function automatic [1:0] bg_pen(input [7:0] byte_, input [1:0] x, input [9:0] y, input [16:0] len, input fill);
    if (y[9])                 bg_pen = 2'd0;                                  // above the image
    else if (y < 10'd128)     bg_pen = ({4'd0, y[6:0], 6'd0} < len) ? {byte_[3'd3 - {1'b0, x}], byte_[3'd7 - {1'b0, x}]}
                                                                    : 2'd3;
    else if (y < 10'd256)     bg_pen = fill ? 2'd3 : 2'd0;                    // solid fill below the image
    else                      bg_pen = 2'd0;
endfunction
wire [1:0] b0_pen  = bgctrl[5] || bgctrl[4] ? 2'd0 : bg_pen(b0_byte, b0x[1:0], b0y, bg0_len, ~alpha1v);
wire       b12_on  = !bgctrl[5] && (!bgctrl[1] || !bgctrl[2]);
wire [1:0] b12_pen = b12_on ? bg_pen(b12_byte, b1x[1:0], b1y, city ? bg2_len : bg1_len, ~alpha1v) : 2'd0;

// colour select (ph7): sprite > text > hills / city > mountains > backdrop (sprite pen 0)
logic [3:0] clut0;
logic       pal_spr, pal_spr_d;
always_comb begin
    pal_spr = 1'b0;
    if (spr_px[3:0] != 4'd0)  begin pal_a = SP_PAL + {8'd0, spr_px[3:0]};                    pal_spr = 1'b1; end
    else if (t_opq)           pal_a = TX_PAL + {3'd0, t_col[6:0], t_pen};
    else if (b12_pen != 2'd0) pal_a = BG_PAL + (city ? {7'd0, 3'b100, b12_pen} : {10'd0, b12_pen});
    else if (b0_pen != 2'd0)  pal_a = BG_PAL + {8'd0, b0_pen, 2'b00};
    else                      begin pal_a = SP_PAL + {8'd0, clut0};                          pal_spr = 1'b1; end
end

// resistor DACs (values match the drawing): text / backgrounds 1K-470-220,
// blue 470-220; sprites 2-bit red 470-220, 3-bit green / blue 1K-470-220, all with the 470 ohm trimmers
function automatic [7:0] w3(input [2:0] v);
    case (v) 3'd0: w3 = 8'd0; 3'd1: w3 = 8'd33; 3'd2: w3 = 8'd71; 3'd3: w3 = 8'd104;
             3'd4: w3 = 8'd151; 3'd5: w3 = 8'd184; 3'd6: w3 = 8'd222; default: w3 = 8'd255; endcase
endfunction
function automatic [7:0] w2(input [1:0] v);
    case (v) 2'd0: w2 = 8'd0; 2'd1: w2 = 8'd81; 2'd2: w2 = 8'd174; default: w2 = 8'd255; endcase
endfunction
function automatic [7:0] s3(input [2:0] v);
    case (v) 3'd0: s3 = 8'd0; 3'd1: s3 = 8'd26; 3'd2: s3 = 8'd55; 3'd3: s3 = 8'd81;
             3'd4: s3 = 8'd118; 3'd5: s3 = 8'd144; 3'd6: s3 = 8'd174; default: s3 = 8'd200; endcase
endfunction
function automatic [7:0] s2(input [1:0] v);
    case (v) 2'd0: s2 = 8'd0; 2'd1: s2 = 8'd62; 2'd2: s2 = 8'd132; default: s2 = 8'd193; endcase
endfunction

logic [7:0] rgb_r, rgb_g, rgb_b;
logic       hbl_d, vbl_d, hs_d, vs_d;
always_ff @(posedge clk) begin
    if (ph == 3'd7) pal_spr_d <= pal_spr;
    if (ph == 3'd0) begin                                                     // colour of the previous pixel
        if (pal_spr_d) begin rgb_r <= s2(pal_q[7:6]); rgb_g <= s3(pal_q[5:3]); rgb_b <= s3(pal_q[2:0]); end
        else           begin rgb_r <= w3(pal_q[2:0]); rgb_g <= w3(pal_q[5:3]); rgb_b <= w2(pal_q[7:6]); end
    end
    if (ce_pix) begin
        hbl_d <= hblank_raw;  vbl_d <= vblank_raw;  hs_d <= hsync_raw;  vs_d <= vsync_raw;
        video_hblank <= hbl_d;  video_vblank <= vbl_d;  video_hs <= hs_d;  video_vs <= vs_d;
        video_r <= rgb_r;  video_g <= rgb_g;  video_b <= rgb_b;
    end
end

/////////////////////////////  Sprite engine (one line ahead)  /////////////////////////////

// Draw order (later = on top): groups 0x00-0x3F, 0x40-0x7F, ..., each from offset 0x3C down to 0x00.
// Sprite: [0] 257 - y, [1] flipy / flipx / colour (6 bits, low 4 used), [2] code, [3] x (+129).
// 16 x 16, 3 planes (0x2000 = pen bit 2, 0x0000 = bit 1, 0x1000 = bit 0; Moon Patrol has no 0x2000 ROM);
// lookup PROM {colour[3:0], pen}; lookup value 0 = transparent. Line buffer holds the lookup value (0 = empty).
typedef enum logic [2:0] {S_IDLE, S_CLUT0, S_ATTR, S_CHECK, S_FETCH, S_DRAW, S_NEXT} sstate_t;
sstate_t     ss = S_IDLE;
logic  [6:0] sk;                                     // sprite index in draw order
logic  [2:0] sa;                                     // attribute byte counter
logic  [7:0] s_y, s_at, s_code, s_x;
logic  [3:0] s_row;
logic  [2:0] sf;                                     // fetch counter: {half, plane}
logic        sf_cap;
logic  [2:0] sf_idx;
logic  [7:0] s_pl[6];                                // [half*3 + plane]
logic  [4:0] sd;                                     // draw counter
logic  [16:0] spr_rom_a;
logic        spr_rom_zero;

wire [6:0] s_last = alpha1v ? 7'd127 : 7'd63;
wire [8:0] s_offs = {sk[6:4], 4'd15 - sk[3:0], 2'b00};
assign sram_ab = s_offs + {6'd0, sa};

wire [9:0] s_sy  = 10'd257 - {2'b00, s_y};
wire [9:0] s_rr  = {1'b0, gyn} - s_sy;
wire [1:0] sf_half  = sf >= 3'd3 ? 2'd1 : 2'd0;
wire [1:0] sf_plane = sf >= 3'd3 ? 2'(sf - 3'd3) : 2'(sf);
wire [16:0] sf_off  = (sf_plane == 2'd0 ? 17'h2000 : sf_plane == 2'd1 ? 17'h0000 : 17'h1000) +
                      {5'd0, s_code[6:0], sf_half[0], s_row};
always_comb spr_rom_a = sp_base + sf_off;

// draw: pixel sd (0-15) looked up in clock sd, written in clock sd + 1
wire [3:0] sd_x  = s_at[6] ? 4'd15 - sd[3:0] : sd[3:0];
wire [2:0] sd_b  = 3'd7 - sd_x[2:0];
wire [2:0] sd_pen = sd_x[3] ? {s_pl[3][sd_b], s_pl[4][sd_b], s_pl[5][sd_b]}
                            : {s_pl[0][sd_b], s_pl[1][sd_b], s_pl[2][sd_b]};

always_comb begin
    clut_a = SP_CLUT + {5'd0, s_at[3:0], sd_pen};
    if (ss == S_CLUT0) clut_a = SP_CLUT;
    if (ss == S_FETCH) clut_a = SP_CLUT + {5'd0, s_at[3:0], 3'd0};          // pen 0 of this colour: blank skip
end
// a sprite row whose six plane bytes are all 0 draws only pen 0; if that pen is transparent nothing is drawn, so the
// 17-clock draw is skipped (Alpha One parks its unused sprites at y 0 = rows 257-272: 128 sprites overran the line)
logic s_pen0_clear;
wire  s_blank = s_pen0_clear && s_pl[0] == 8'd0 && s_pl[1] == 8'd0 && s_pl[2] == 8'd0 &&
                s_pl[3] == 8'd0 && s_pl[4] == 8'd0 && s_pl[5] == 8'd0;

// pixel sd - 1 is written while pixel sd is looked up
wire   lb_we = ss == S_DRAW && !sf_cap && sd != 5'd0 && clut_q[3:0] != 4'd0;
assign lb_aa = 9'({1'b0, s_x} + 9'd129 + {5'd0, 4'(sd - 5'd1)});
assign lb_da = {4'd0, clut_q[3:0]};
assign lb_wa = lb_we ? (gyn[0] ? 2'b10 : 2'b01) : 2'b00;
logic  clut0_cap = 1'b0;

always_ff @(posedge clk) begin
    clut0_cap <= ss == S_CLUT0;
    if (clut0_cap) clut0 <= clut_q[3:0];
    if (sf_cap) begin
        s_pl[sf_idx] <= (spr_rom_zero) ? 8'd0 : vid_rom_q;
        sf_cap <= 1'b0;
    end
    if (ce_pix && hcnt == 9'd383) begin                                      // new line: start over
        ss <= S_CLUT0;
        sk <= 7'd0;
        sa <= 3'd0;
    end else case (ss)
        S_IDLE: ;
        S_CLUT0: ss <= S_ATTR;                                               // lookup entry 0 (backdrop)
        S_ATTR: begin
            case (sa)
                3'd1: s_y    <= sram_qb;
                3'd2: s_at   <= sram_qb;
                3'd3: s_code <= sram_qb;
                3'd4: s_x    <= sram_qb;
                default: ;
            endcase
            if (sa == 3'd4) begin sa <= 3'd0; ss <= S_CHECK; end
            else sa <= sa + 3'd1;
        end
        S_CHECK: begin
            if (s_rr[9:4] == 6'd0) begin
                s_row <= s_at[7] ? 4'd15 - s_rr[3:0] : s_rr[3:0];
                sf <= 3'd0;
                ss <= S_FETCH;
            end else ss <= S_NEXT;
        end
        S_FETCH: begin
            s_pen0_clear <= clut_q[3:0] == 4'd0;                             // valid from the 2nd FETCH clock on
            if (spr_slot) begin                                              // one read per slot clock, captured next clock
                sf_cap <= 1'b1;
                sf_idx <= sf;
                spr_rom_zero <= sf_off >= sp_len;
                if (sf == 3'd5) begin sd <= 5'd0; ss <= S_DRAW; end
                else sf <= sf + 3'd1;
            end
        end
        S_DRAW: begin
            if (!sf_cap) begin                                               // last plane byte in
                if (sd == 5'd16 || (sd == 5'd0 && s_blank)) ss <= S_NEXT;
                sd <= sd + 5'd1;
            end
        end
        S_NEXT: begin
            if (sk == s_last) ss <= S_IDLE;
            else begin sk <= sk + 7'd1; ss <= S_ATTR; end
        end
        default: ss <= S_IDLE;
    endcase
end

endmodule
