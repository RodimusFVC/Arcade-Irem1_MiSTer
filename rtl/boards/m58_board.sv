//============================================================================
//
//  m58_board - Irem M58 board set (10-Yard Fight family, variant 3)
//  Copyright (C) 2026 Rodimus
//
//  M58-A-A (CPU, tilemap) + M58-B-A (timing, sprites, radar panel), 10-Yard Fight manual schematics p31-41 (facts in
//  Claude/irem1_schematic_facts.md) + MAME irem/m58.cpp. Same pool ports as m52_board.
//
//  Raster: 384 x 282 at 6.144 MHz (as M52). Visible = MAME m58: x 0-255, y 42-265 (224 lines = the drawing's VBLANK
//  272-495); IRQ0 at y 266, held until acknowledged.
//  Layers (MAME screen_update): opaque 64 x 32 tilemap (8x8, 3 bpp, scroll x 9 bits / y 8 bits, dy 26), sprites
//  (24 x 16x32 = two stacked 16x16), radar / score panel bitmap on top (x 208-255, written at 9000-9FFF, A800 = off).
//  Colours: tilemap and panel = two 4-bit PROMs each (low / high nibble), sprites = lookup PROM + 32 x 8 PROM.
//  Pixel pipeline: fetched in the 8 clocks of h, two palette reads at ph6 / ph7, colour formed at ph0 of h + 1,
//  output during h + 2; blank and sync delayed to match.
//  Flip (game flip XOR CRT Flip): raster mirror about the visible window (x -> 255 - x, y -> 307 - y).
//  Not modelled: Z80 /WAIT from both video boards (drawing p33 / p41; MAME does not model it either).
//
//============================================================================

module m58_board
(
    input  logic        clk,                // 49.152 MHz
    input  logic        reset,
    input  logic        pause,
    input  logic [47:0][7:0] rom_rgn,

    input  logic  [7:0] in0, in1, in2, dsw1, dsw2,
    input  logic        crt_flip,
    input  logic  [3:0] h_adj,
    input  logic  [3:0] v_adj,

    output logic        ce_pix,
    output logic  [7:0] video_r, video_g, video_b,
    output logic        video_hs, video_vs, video_hblank, video_vblank,

    output chip_pool_pkg::z80pa_i_t cpu_i,
    input  chip_pool_pkg::z80pa_o_t cpu_o,

    output logic [16:0] cpu_rom_a,
    input  logic  [7:0] cpu_rom_q,
    output logic [16:0] vid_rom_a,
    input  logic  [7:0] vid_rom_q,
    output logic [11:0] pal_a,
    input  logic  [7:0] pal_q,
    output logic [11:0] clut_a,
    input  logic  [7:0] clut_q,

    output logic [11:0] wram_a,   output logic [7:0] wram_d,  output logic wram_we,  input logic [7:0] wram_q,
    output logic [11:0] vram_aa,  output logic [7:0] vram_da, output logic vram_wa,  input logic [7:0] vram_qa,
    output logic [11:0] vram_ab,                                                     input logic [7:0] vram_qb,
    output logic  [8:0] sram_aa,  output logic [7:0] sram_da, output logic sram_wa,  input logic [7:0] sram_qa,
    output logic  [8:0] sram_ab,                                                     input logic [7:0] sram_qb,
    output logic [11:0] xram_aa,  output logic [7:0] xram_da, output logic xram_wa,
    output logic [11:0] xram_ab,                                                     input logic [7:0] xram_qb,
    output logic  [8:0] lb_aa,    output logic [7:0] lb_da,   output logic [1:0] lb_wa,
    output logic  [8:0] lb_ab,                                output logic [1:0] lb_wb,
    input  logic  [7:0] lb_qb0,   input  logic [7:0] lb_qb1,

    output logic        snd_wr,
    output logic  [7:0] snd_cmd,

    input  logic [15:0] hs_address,
    input  logic  [7:0] hs_data_in,
    output logic  [7:0] hs_data_out,
    input  logic        hs_write
);

/////////////////////////////  Region table  /////////////////////////////

wire [16:0] rom_base  = {rom_rgn[1][0],  rom_rgn[0],  8'h00};
wire [16:0] rom_start = {rom_rgn[3][0],  rom_rgn[2],  8'h00};
wire [16:0] rom_len   = {rom_rgn[5][0],  rom_rgn[4],  8'h00};
wire [16:0] tx_base   = {rom_rgn[13][0], rom_rgn[12], 8'h00};
wire [16:0] sp_base   = {rom_rgn[19][0], rom_rgn[18], 8'h00};
wire [11:0] prom      = {rom_rgn[42][3:0], 8'h00};
// PROM block (MAME ROM_START order): char lo +000, char hi +100, sprite pal +200, sprite lookup +220,
// radar lo +320, radar hi +420
wire [11:0] CH_LO = prom, CH_HI = prom + 12'h100, SP_PAL = prom + 12'h200, SP_CLUT = prom + 12'h220,
            RD_LO = prom + 12'h320, RD_HI = prom + 12'h420;

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

wire hblank_raw = hcnt >= 9'd256;
wire vblank_raw = vcnt < 9'd42 || vcnt > 9'd265;
// HSYNC 32 px from x 288 (+2 x H adjust: 272-302; blanking 256-383, both porches > 0); VSYNC 3 lines from y 270
// (V adjust clamped to +-3: 267-273; blanking 266-281 / 0-41)
wire [8:0] hs_start = 9'd288 + {{4{h_adj[3]}}, h_adj, 1'b0};
wire [3:0] vadj_c   = ($signed(v_adj) > 4'sd3) ? 4'd3 : ($signed(v_adj) < -4'sd3) ? 4'b1101 : v_adj;
wire [8:0] vs_start = 9'd270 + {{5{vadj_c[3]}}, vadj_c};
wire hsync_raw = hcnt >= hs_start && hcnt < hs_start + 9'd32;
wire vsync_raw = vcnt >= vs_start && vcnt < vs_start + 9'd3;

/////////////////////////////  Main CPU  /////////////////////////////

wire [15:0] a = cpu_o.a;
wire mem_wr = ~cpu_o.mreq_n & ~cpu_o.wr_n & cpu_o.rfsh_n;
// interrupt acknowledge = M1 + IORQ with MREQ high (T80pa IntCycleD_n power-up glitch, see m52_board)
wire int_ack = ~cpu_o.iorq_n & ~cpu_o.m1_n & cpu_o.mreq_n;
logic mem_wr_d = 1'b0;
always_ff @(posedge clk) mem_wr_d <= mem_wr;
wire mem_wstb = mem_wr & ~mem_wr_d;

wire [3:0] c16 = {hcnt[0], ph};
logic int_n = 1'b1;
always_ff @(posedge clk) begin
    if (reset)                                           int_n <= 1'b1;
    else if (int_ack)                                    int_n <= 1'b1;
    else if (ce_pix && hcnt == 9'd383 && vcnt == 9'd265) int_n <= 1'b0;       // start of line 266
end

// MAME yard_map (no mirrors): 0000-5FFF ROM, 8000-8FFF video RAM, 9000-9FFF panel (write only), A000 / A200 / A400 /
// A800 scroll x low / high, scroll y, panel off, C820-C87F sprites, D000-D004 I/O, E000-EFFF RAM
wire sel_rom  = a < 16'h6000;
wire sel_vram = a[15:12] == 4'h8;
wire sel_pnl  = a[15:12] == 4'h9;
wire sel_sxl  = a == 16'hA000, sel_sxh = a == 16'hA200, sel_syl = a == 16'hA400, sel_pdis = a == 16'hA800;
wire sel_sprt = a >= 16'hC820 && a <= 16'hC87F;
wire sel_io   = a[15:3] == 13'h1A00 && a[2:0] <= 3'd4;                       // D000-D004
wire sel_wram = a[15:12] == 4'hE;

wire [16:0] rom_off = {1'b0, a} - rom_start;
assign cpu_rom_a = rom_base + rom_off;
wire rom_in = !rom_off[16] && rom_off < rom_len;

logic [7:0] scrollx_lo, scrollx_hi, scrolly, pnl_dis;
logic       flip_reg;
always_ff @(posedge clk) begin
    if (reset) begin
        scrollx_lo <= 8'd0;  scrollx_hi <= 8'd0;  scrolly <= 8'd0;  pnl_dis <= 8'd0;  flip_reg <= 1'b0;
    end else if (mem_wstb) begin
        if (sel_sxl)  scrollx_lo <= cpu_o.dout;
        if (sel_sxh)  scrollx_hi <= cpu_o.dout;
        if (sel_syl)  scrolly    <= cpu_o.dout;
        if (sel_pdis) pnl_dis    <= cpu_o.dout;
        if (sel_io && a[2:0] == 3'd1) flip_reg <= cpu_o.dout[0];             // D001: flip (+ coin counters)
    end
end
assign snd_wr  = mem_wstb && sel_io && a[2:0] == 3'd0;                       // D000: sound command
assign snd_cmd = cpu_o.dout;

// RAM ports A: CPU, or the hiscore engine while paused (same address map)
wire [15:0] ra = pause ? hs_address : a;
wire        ra_wram = ra[15:12] == 4'hE;
wire        ra_vram = ra[15:12] == 4'h8;
assign wram_a  = ra[11:0];
assign wram_d  = pause ? hs_data_in : cpu_o.dout;
assign wram_we = pause ? hs_write & ra_wram : mem_wr & sel_wram;
assign vram_aa = ra[11:0];
assign vram_da = wram_d;
assign vram_wa = pause ? hs_write & ra_vram : mem_wr & sel_vram;
assign hs_data_out = ra_vram ? vram_qa : wram_q;
assign sram_aa = a[8:0];
assign sram_da = cpu_o.dout;
assign sram_wa = mem_wr & sel_sprt;
assign xram_aa = a[11:0];
assign xram_da = cpu_o.dout;
assign xram_wa = mem_wr & sel_pnl;

logic [7:0] io_q;
always_comb case (a[2:0])
    3'd0: io_q = in0;   3'd1: io_q = in1;   3'd2: io_q = in2;   3'd3: io_q = dsw1;   default: io_q = dsw2;
endcase

logic [7:0] cpu_di;
always_comb begin
    if (int_ack)       cpu_di = 8'hFF;
    else if (sel_rom)  cpu_di = rom_in ? cpu_rom_q : 8'h00;
    else if (sel_vram) cpu_di = vram_qa;
    else if (sel_sxl)  cpu_di = scrollx_lo;
    else if (sel_sxh)  cpu_di = scrollx_hi;
    else if (sel_syl)  cpu_di = scrolly;
    else if (sel_pdis) cpu_di = pnl_dis;
    else if (sel_sprt) cpu_di = sram_qa;
    else if (sel_io)   cpu_di = io_q;
    else if (sel_wram) cpu_di = wram_q;
    else               cpu_di = 8'h00;                                       // MAME unmapped read
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

wire       flip = (flip_reg ^ ~dsw2[0]) ^ crt_flip;
wire [8:0] gx = flip ? 9'd255 - hcnt : hcnt;
wire [8:0] gy = flip ? 9'd307 - vcnt : vcnt;
wire [8:0] vn = (vcnt == 9'd281) ? 9'd0 : vcnt + 9'd1;
wire [8:0] gyn = flip ? 9'd307 - vn : vn;

/////////////////////////////  Per-pixel fetch (8 clocks)  /////////////////////////////

// tilemap: x = gx + scroll x (512 wide), y = gy - 26 + scroll y; tile (col, row) at video RAM byte
// {col[5], row, col[4:0], b}: b 0 = code, b 1 = attr (code bits 9-8 = attr 7-6, flip x = attr 5, colour = attr 4-0)
wire [8:0] mx = gx + {scrollx_hi[0], scrollx_lo};
wire [7:0] my = 8'(gy - 9'd26) + scrolly;
wire [10:0] t_idx = {mx[8], my[7:3], mx[7:3]};
logic [7:0] t_code, t_attr, t_p0, t_p1, t_p2;
wire  [9:0] t_tile_now = {vram_qb[7:6], t_code};      // ph2: attr byte arriving
wire  [9:0] t_tile     = {t_attr[7:6], t_code};
wire  [2:0] t_y        = my[2:0];

// panel: bitmap row = gy - 26, column = gx - 200 (shown at gx 208-255); byte = row * 16 + column / 4 + 1
wire [7:0] p_row = 8'(gy - 9'd26);
wire [7:0] p_col = 8'(gx - 9'd200);
wire       p_on  = gx >= 9'd208 && gx <= 9'd255 && pnl_dis == 8'd0;
assign xram_ab = {p_row, 4'(p_col[5:2] + 4'd1)};

always_comb begin
    vram_ab   = {t_idx, 1'b0};
    vid_rom_a = spr_rom_a;
    case (ph)
        3'd0: vram_ab   = {t_idx, 1'b0};
        3'd1: vram_ab   = {t_idx, 1'b1};
        3'd2: vid_rom_a = tx_base +           {4'd0, t_tile_now, t_y};      // plane 2 (pen bit 0)
        3'd3: vid_rom_a = tx_base + 17'h2000 + {4'd0, t_tile, t_y};          // plane 1
        3'd4: vid_rom_a = tx_base + 17'h4000 + {4'd0, t_tile, t_y};          // plane 0 (pen bit 2)
        default: ;                                                            // 5, 6, 7, 0, 1: sprite engine
    endcase
end
wire spr_slot = ph != 3'd2 && ph != 3'd3 && ph != 3'd4;

assign lb_ab = gx;
assign lb_wb = (ph == 3'd6) ? (gy[0] ? 2'b10 : 2'b01) : 2'b00;

always_ff @(posedge clk) begin
    case (ph)
        3'd1: t_code <= vram_qb;
        3'd2: t_attr <= vram_qb;
        3'd3: t_p2   <= vid_rom_q;
        3'd4: t_p1   <= vid_rom_q;
        3'd5: t_p0   <= vid_rom_q;
        default: ;
    endcase
end

// colour select (ph6, live sprite / panel data): panel > sprite > tilemap (opaque)
wire [2:0] txb    = t_attr[5] ? mx[2:0] : 3'd7 - mx[2:0];
wire [2:0] t_pen  = {t_p0[txb], t_p1[txb], t_p2[txb]};
wire [7:0] spr_now = gy[0] ? lb_qb1 : lb_qb0;
wire [1:0] p_pix  = {xram_qb[{1'b1, p_col[1:0]}], xram_qb[{1'b0, p_col[1:0]}]};
typedef enum logic [1:0] {K_TILE, K_SPR, K_PNL} kind_t;
kind_t      kind, kind_d;
logic [7:0] pidx, pidx_d;
always_comb begin
    if (p_on)                         begin kind = K_PNL;  pidx = {p_row[7:2], p_pix};    end
    else if (spr_now[3:0] != 4'd0)    begin kind = K_SPR;  pidx = {4'd0, spr_now[3:0]};   end
    else                              begin kind = K_TILE; pidx = {t_attr[4:0], t_pen};   end
end
always_comb begin
    pal_a = CH_LO + {4'd0, pidx_d};
    case (ph)
        3'd6: case (kind)
                  K_PNL:   pal_a = RD_LO  + {4'd0, pidx};
                  K_SPR:   pal_a = SP_PAL + {4'd0, pidx};
                  default: pal_a = CH_LO  + {4'd0, pidx};
              endcase
        3'd7: pal_a = (kind_d == K_PNL) ? RD_HI + {4'd0, pidx_d} : CH_HI + {4'd0, pidx_d};
        default: ;
    endcase
end

// resistor DACs (MAME m58 palette, values match the drawing): tilemap / panel red 470-220, green / blue
// 1K-470-220; sprites the same with the 470 ohm pull-down per gun
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

logic [7:0] pal_lo, rgb_r, rgb_g, rgb_b;
logic       hbl_d, vbl_d, hs_d, vs_d;
wire  [7:0] pv = (kind_d == K_SPR) ? pal_lo : {pal_q[3:0], pal_lo[3:0]};   // ph0: high nibble PROM arriving
always_ff @(posedge clk) begin
    if (ph == 3'd6) begin kind_d <= kind; pidx_d <= pidx; end
    if (ph == 3'd7) pal_lo <= pal_q;
    if (ph == 3'd0) begin
        if (kind_d == K_SPR) begin rgb_r <= s2(pv[7:6]); rgb_g <= s3(pv[5:3]); rgb_b <= s3(pv[2:0]); end
        else                 begin rgb_r <= w2(pv[7:6]); rgb_g <= w3(pv[5:3]); rgb_b <= w3(pv[2:0]); end
    end
    if (ce_pix) begin
        hbl_d <= hblank_raw;  vbl_d <= vblank_raw;  hs_d <= hsync_raw;  vs_d <= vsync_raw;
        video_hblank <= hbl_d;  video_vblank <= vbl_d;  video_hs <= hs_d;  video_vs <= vs_d;
        video_r <= rgb_r;  video_g <= rgb_g;  video_b <= rgb_b;
    end
end

/////////////////////////////  Sprite engine (one line ahead)  /////////////////////////////

// MAME draw_sprites: offsets 0x5C down to 0x00 (0x00 on top). [0] y (top = 42 + 210 - y), [1] flip y / flip x / bank /
// colour (5 bits), [2] code (bit 6 cleared; the lower 16x16 half = code + 0x40, swapped when flipped in y), [3] x.
// 3 planes at 0x8000 (pen bit 2), 0x4000, 0x0000; lookup {colour, pen}; lookup value 0 = transparent.
typedef enum logic [2:0] {S_IDLE, S_ATTR, S_CHECK, S_FETCH, S_DRAW, S_NEXT} sstate_t;
sstate_t     ss = S_IDLE;
logic  [4:0] sk;
logic  [2:0] sa;
logic  [7:0] s_y, s_at, s_code, s_x;
logic  [8:0] s_tile;
logic  [3:0] s_row;
logic  [2:0] sf, sf_idx;
logic        sf_cap = 1'b0, spr_rom_zero;
logic  [7:0] s_pl[6];
logic  [4:0] sd;
logic [16:0] spr_rom_a;

wire [6:0] s_offs = 7'h20 + {(5'd23 - sk), 2'b00};
assign sram_ab = {2'b00, s_offs} + {6'd0, sa};

wire [9:0] s_sy = 10'd252 - {2'b00, s_y};
wire [9:0] s_rr = {1'b0, gyn} - s_sy;                                        // 0-31 = on this line
wire [1:0] sf_half  = sf >= 3'd3 ? 2'd1 : 2'd0;
wire [1:0] sf_plane = sf >= 3'd3 ? 2'(sf - 3'd3) : 2'(sf);
wire [16:0] sf_off  = (sf_plane == 2'd0 ? 17'h8000 : sf_plane == 2'd1 ? 17'h4000 : 17'h0000) +
                      {3'd0, s_tile, sf_half[0], s_row};
always_comb spr_rom_a = sp_base + sf_off;

wire [3:0] sd_x  = s_at[6] ? 4'd15 - sd[3:0] : sd[3:0];
wire [2:0] sd_b  = 3'd7 - sd_x[2:0];
wire [2:0] sd_pen = sd_x[3] ? {s_pl[3][sd_b], s_pl[4][sd_b], s_pl[5][sd_b]}
                            : {s_pl[0][sd_b], s_pl[1][sd_b], s_pl[2][sd_b]};
assign clut_a = SP_CLUT + {4'd0, s_at[4:0], sd_pen};

wire   lb_we = ss == S_DRAW && !sf_cap && sd != 5'd0 && clut_q[3:0] != 4'd0;
assign lb_aa = 9'({1'b0, s_x} + {5'd0, 4'(sd - 5'd1)});
assign lb_da = {4'd0, clut_q[3:0]};
assign lb_wa = lb_we ? (gyn[0] ? 2'b10 : 2'b01) : 2'b00;

always_ff @(posedge clk) begin
    if (sf_cap) begin
        s_pl[sf_idx] <= spr_rom_zero ? 8'd0 : vid_rom_q;
        sf_cap <= 1'b0;
    end
    if (ce_pix && hcnt == 9'd383) begin
        ss <= S_ATTR;  sk <= 5'd0;  sa <= 3'd0;
    end else case (ss)
        S_IDLE: ;
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
            if (s_rr[9:5] == 5'd0) begin
                s_tile <= {s_at[5], s_code[7], s_rr[4] ^ s_at[7], s_code[5:0]};
                s_row  <= s_at[7] ? 4'd15 - s_rr[3:0] : s_rr[3:0];
                sf <= 3'd0;
                ss <= S_FETCH;
            end else ss <= S_NEXT;
        end
        S_FETCH: begin
            if (spr_slot) begin
                sf_cap <= 1'b1;
                sf_idx <= sf;
                spr_rom_zero <= 1'b0;
                if (sf == 3'd5) begin sd <= 5'd0; ss <= S_DRAW; end
                else sf <= sf + 3'd1;
            end
        end
        S_DRAW: begin
            if (!sf_cap) begin
                if (sd == 5'd16) ss <= S_NEXT;
                sd <= sd + 5'd1;
            end
        end
        S_NEXT: begin
            if (sk == 5'd23) ss <= S_IDLE;
            else begin sk <= sk + 5'd1; ss <= S_ATTR; end
        end
        default: ss <= S_IDLE;
    endcase
end

endmodule
