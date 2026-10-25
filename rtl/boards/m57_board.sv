//============================================================================
//
//  m57_board - Irem M57 board (Tropical Angel, New Tropical Angel; variant 2)
//  Copyright (C) 2026 Rodimus
//
//  No schematics found: MAME irem/m57.cpp is the model. Same pool ports as the other boards (+ xram = scroll RAM).
//
//  Raster: 384 x 282 at 6.144 MHz; visible x 8-247, y 8-247 (MAME set_raw); IRQ0 at y 248, held until acknowledged.
//  Tilemap 32 x 32 (8x8, 3 bpp, flip x / y per tile), opaque. Lines 64-127: x scroll = scroll RAM 0x40. Lines 128 and
//  below ("sea"): each line shifted by the signed 16-bit scroll RAM [y] | [y + 0x100] << 8; a shifted-in pixel
//  repeats the edge (MAME draw_background copies within the visible bitmap, clamped to x 8-247).
//  Sprites: 56 x 16x32 3 bpp (offset 0xDC first, 0x00 on top), 4 banks (code bit 7, attr bit 5); planes MSB first
//  (0x0000 = pen bit 2); colour = ~lookup & 15 into the 16-entry sprite palette, lookup 0 = transparent.
//  Pixel pipeline as m58_board (two palette reads, ph6 / ph7; output at h + 2). Flip (game XOR CRT): raster mirror.
//
//============================================================================

module m57_board
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
    output logic  [8:0] sram_aa,  output logic [7:0] sram_da, output logic sram_wa,
    output logic  [8:0] sram_ab,                                                     input logic [7:0] sram_qb,
    output logic [11:0] xram_aa,  output logic [7:0] xram_da, output logic xram_wa,  input logic [7:0] xram_qa,
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
// PROM block (MAME region offsets): char palette low +000, high +100, sprite palette +200, sprite lookup +220
wire [11:0] CH_LO = prom, CH_HI = prom + 12'h100, SP_PAL = prom + 12'h200, SP_CLUT = prom + 12'h220;

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

wire hblank_raw = hcnt < 9'd8 || hcnt >= 9'd248;
wire vblank_raw = vcnt < 9'd8 || vcnt >= 9'd248;
// HSYNC 32 px from x 280 (+2 x H adjust: 264-294; blanking 248-383 / 0-7); VSYNC 3 lines from y 254 (+-3: 251-257)
wire [8:0] hs_start = 9'd280 + {{4{h_adj[3]}}, h_adj, 1'b0};
wire [3:0] vadj_c   = ($signed(v_adj) > 4'sd3) ? 4'd3 : ($signed(v_adj) < -4'sd3) ? 4'b1101 : v_adj;
wire [8:0] vs_start = 9'd254 + {{5{vadj_c[3]}}, vadj_c};
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
    else if (ce_pix && hcnt == 9'd383 && vcnt == 9'd247) int_n <= 1'b0;       // start of line 248
end

// MAME main_map (no mirrors): 0000-7FFF ROM, 8000-87FF video RAM, 9000-91FF scroll RAM, C820-C8FF sprites (write
// only), D000-D004 I/O, E000-E7FF RAM
wire sel_rom  = ~a[15];
wire sel_vram = a[15:11] == 5'b10000;
wire sel_scrl = a[15:9] == 7'b1001000;                                       // 9000-91FF
wire sel_sprt = a >= 16'hC820 && a <= 16'hC8FF;
wire sel_io   = a[15:3] == 13'h1A00 && a[2:0] <= 3'd4;
wire sel_wram = a[15:11] == 5'b11100;

wire [16:0] rom_off = {1'b0, a} - rom_start;
assign cpu_rom_a = rom_base + rom_off;
wire rom_in = !rom_off[16] && rom_off < rom_len;

logic flip_reg;
always_ff @(posedge clk) begin
    if (reset) flip_reg <= 1'b0;
    else if (mem_wstb && sel_io && a[2:0] == 3'd1) flip_reg <= cpu_o.dout[0];  // D001: flip (+ coin counters)
end
assign snd_wr  = mem_wstb && sel_io && a[2:0] == 3'd0;                       // D000: sound command
assign snd_cmd = cpu_o.dout;

wire [15:0] ra = pause ? hs_address : a;
wire        ra_wram = ra[15:11] == 5'b11100;
wire        ra_vram = ra[15:11] == 5'b10000;
assign wram_a  = {1'b0, ra[10:0]};
assign wram_d  = pause ? hs_data_in : cpu_o.dout;
assign wram_we = pause ? hs_write & ra_wram : mem_wr & sel_wram;
assign vram_aa = {1'b0, ra[10:0]};
assign vram_da = wram_d;
assign vram_wa = pause ? hs_write & ra_vram : mem_wr & sel_vram;
assign hs_data_out = ra_vram ? vram_qa : wram_q;
assign sram_aa = a[8:0];
assign sram_da = cpu_o.dout;
assign sram_wa = mem_wr & sel_sprt;
assign xram_aa = {3'd0, a[8:0]};
assign xram_da = cpu_o.dout;
assign xram_wa = mem_wr & sel_scrl;

logic [7:0] io_q;
always_comb case (a[2:0])
    3'd0: io_q = in0;   3'd1: io_q = in1;   3'd2: io_q = in2;   3'd3: io_q = dsw1;   default: io_q = dsw2;
endcase

logic [7:0] cpu_di;
always_comb begin
    if (int_ack)       cpu_di = 8'hFF;
    else if (sel_rom)  cpu_di = rom_in ? cpu_rom_q : 8'h00;
    else if (sel_vram) cpu_di = vram_qa;
    else if (sel_scrl) cpu_di = xram_qa;
    else if (sel_io)   cpu_di = io_q;
    else if (sel_wram) cpu_di = wram_q;
    else               cpu_di = 8'h00;                                       // MAME unmapped read
end

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
wire [8:0] gy = flip ? 9'd255 - vcnt : vcnt;
wire [8:0] vn = (vcnt == 9'd281) ? 9'd0 : vcnt + 9'd1;
wire [8:0] gyn = flip ? 9'd255 - vn : vn;

/////////////////////////////  Line scroll values (scroll RAM, start of each line)  /////////////////////////////

// hcnt 0 (blanking): [0x40] = x scroll of lines 64-127; [y], [y + 0x100] = this line's sea shift (signed 16 bits)
logic [7:0] sc40, sea_lo, sea_hi;
logic [11:0] xram_ab_l;
always_comb begin
    xram_ab_l = 12'h040;
    case (ph)
        3'd0: xram_ab_l = {4'd0, gy[7:0]};
        3'd1: xram_ab_l = {4'd1, gy[7:0]};
        default: xram_ab_l = 12'h040;
    endcase
end
assign xram_ab = xram_ab_l;
always_ff @(posedge clk) if (hcnt == 9'd0) begin
    case (ph)
        3'd1: sea_lo <= xram_qb;
        3'd2: sea_hi <= xram_qb;
        3'd3: sc40   <= xram_qb;
        default: ;
    endcase
end

/////////////////////////////  Per-pixel fetch (8 clocks)  /////////////////////////////

// screen x of the tilemap pixel: sea lines = gx + shift clamped to 8-247; tilemap x = that (+ scroll on 64-127)
wire signed [17:0] sea_x  = $signed({9'd0, gx}) + $signed({{2{sea_hi[7]}}, sea_hi, sea_lo});
wire        [8:0] sx_sea = (sea_x < 18'sd8) ? 9'd8 : (sea_x > 18'sd247) ? 9'd247 : sea_x[8:0];
wire        [7:0] my     = gy[7:0];
wire        [7:0] mx     = (my >= 8'd128) ? sx_sea[7:0] : (my >= 8'd64) ? 8'(gx[7:0] + sc40) : gx[7:0];
wire        [9:0] t_idx  = {my[7:3], mx[7:3]};
logic [7:0] t_attr, t_code, t_p0, t_p1, t_p2;
// video RAM byte 0 = attr (code bits 9-8 = attr 7-6, flip x / y = attr 5 / 4 (MAME TILE_FLIPXY), colour = 3-0),
// byte 1 = code
wire  [9:0] t_tile_now = {t_attr[7:6], vram_qb};      // ph2: code byte arriving
wire  [9:0] t_tile     = {t_attr[7:6], t_code};
wire  [2:0] t_y        = t_attr[4] ? 3'd7 - my[2:0] : my[2:0];

always_comb begin
    vram_ab   = {1'b0, t_idx, 1'b0};
    vid_rom_a = spr_rom_a;
    case (ph)
        3'd0: vram_ab   = {1'b0, t_idx, 1'b0};
        3'd1: vram_ab   = {1'b0, t_idx, 1'b1};
        3'd2: vid_rom_a = tx_base +           {4'd0, t_tile_now, t_y};      // plane 2 (pen bit 0)
        3'd3: vid_rom_a = tx_base + 17'h2000 + {4'd0, t_tile, t_y};          // plane 1
        3'd4: vid_rom_a = tx_base + 17'h4000 + {4'd0, t_tile, t_y};          // plane 0 (pen bit 2)
        default: ;
    endcase
end
wire spr_slot = ph != 3'd2 && ph != 3'd3 && ph != 3'd4;

assign lb_ab = gx;
assign lb_wb = (ph == 3'd6) ? (gy[0] ? 2'b10 : 2'b01) : 2'b00;

always_ff @(posedge clk) begin
    case (ph)
        3'd1: t_attr <= vram_qb;
        3'd2: t_code <= vram_qb;
        3'd3: t_p2   <= vid_rom_q;
        3'd4: t_p1   <= vid_rom_q;
        3'd5: t_p0   <= vid_rom_q;
        default: ;
    endcase
end

// colour select (ph6): sprite > tilemap
wire [2:0] txb    = t_attr[5] ? mx[2:0] : 3'd7 - mx[2:0];
wire [2:0] t_pen  = {t_p0[txb], t_p1[txb], t_p2[txb]};
wire [7:0] spr_now = gy[0] ? lb_qb1 : lb_qb0;
logic       k_spr, k_spr_d;
logic [7:0] pidx, pidx_d;
always_comb begin
    if (spr_now[3:0] != 4'd0) begin k_spr = 1'b1; pidx = {4'd0, ~spr_now[3:0]};  end   // MAME: ~lookup & 15
    else                      begin k_spr = 1'b0; pidx = {1'b0, t_attr[3:0], t_pen}; end
end
always_comb begin
    pal_a = CH_LO + {4'd0, pidx_d};
    case (ph)
        3'd6: pal_a = k_spr ? SP_PAL + {4'd0, pidx} : CH_LO + {4'd0, pidx};
        3'd7: pal_a = CH_HI + {4'd0, pidx_d};
        default: ;
    endcase
end

// MAME m57 palette: chars r = 0x47 / 0x97 (high PROM bits 2, 3), g = 0x21 (low bit 3) / 0x47 / 0x97 (high bits 0, 1),
// b = 0x21 / 0x47 / 0x97 (low bits 0-2); sprites r = 0x47 / 0x97 (bits 6, 7), g bits 3-5, b bits 0-2
function automatic [7:0] w3(input [2:0] v);
    case (v) 3'd0: w3 = 8'd0; 3'd1: w3 = 8'd33; 3'd2: w3 = 8'd71; 3'd3: w3 = 8'd104;
             3'd4: w3 = 8'd151; 3'd5: w3 = 8'd184; 3'd6: w3 = 8'd222; default: w3 = 8'd255; endcase
endfunction
function automatic [7:0] r2s(input [1:0] v);
    case (v) 2'd0: r2s = 8'd0; 2'd1: r2s = 8'd71; 2'd2: r2s = 8'd151; default: r2s = 8'd222; endcase
endfunction

logic [7:0] pal_lo, rgb_r, rgb_g, rgb_b;
logic       hbl_d, vbl_d, hs_d, vs_d;
always_ff @(posedge clk) begin
    if (ph == 3'd6) begin k_spr_d <= k_spr; pidx_d <= pidx; end
    if (ph == 3'd7) pal_lo <= pal_q;
    if (ph == 3'd0) begin                                                     // pal_q = high char PROM now
        if (k_spr_d) begin rgb_r <= r2s(pal_lo[7:6]); rgb_g <= w3(pal_lo[5:3]);          rgb_b <= w3(pal_lo[2:0]); end
        else         begin rgb_r <= r2s(pal_q[3:2]);  rgb_g <= w3({pal_q[1:0], pal_lo[3]}); rgb_b <= w3(pal_lo[2:0]); end
    end
    if (ce_pix) begin
        hbl_d <= hblank_raw;  vbl_d <= vblank_raw;  hs_d <= hsync_raw;  vs_d <= vsync_raw;
        video_hblank <= hbl_d;  video_vblank <= vbl_d;  video_hs <= hs_d;  video_vs <= vs_d;
        video_r <= rgb_r;  video_g <= rgb_g;  video_b <= rgb_b;
    end
end

/////////////////////////////  Sprite engine (one line ahead)  /////////////////////////////

// MAME draw_sprites: offsets 0xDC down to 0x00 (0x00 on top) at C820. [0] y (top = ((224 - y - 32) & 0xFF) + 32),
// [1] flip y / flip x / bank bit 1 / colour (5 bits), [2] bank bit 0 (bit 7) / tile (bits 5-0), [3] x.
// 16x32: rows 0-15 at +0, rows 16-31 at +0x800; right half +16; tile * 32; bank * 0x1000;
// planes 0x0000 (pen bit 2), 0x4000, 0x8000 (pen bit 0).
typedef enum logic [2:0] {S_IDLE, S_ATTR, S_CHECK, S_FETCH, S_DRAW, S_NEXT} sstate_t;
sstate_t     ss = S_IDLE;
logic  [5:0] sk;
logic  [2:0] sa;
logic  [7:0] s_y, s_at, s_code, s_x;
logic  [4:0] s_row;
logic  [2:0] sf, sf_idx;
logic        sf_cap = 1'b0;
logic  [7:0] s_pl[6];
logic  [4:0] sd;
logic [16:0] spr_rom_a;

wire [8:0] s_offs = 9'h020 + {1'b0, (6'd55 - sk), 2'b00};
assign sram_ab = s_offs + {6'd0, sa};

wire [9:0] s_sy = {2'b00, 8'(8'd192 - s_y)} + 10'd32;                         // ((224 - y - 32) & 0xFF) + 32
wire [9:0] s_rr = {1'b0, gyn} - s_sy;                                        // 0-31 = on this line
wire [1:0] sf_half  = sf >= 3'd3 ? 2'd1 : 2'd0;
wire [1:0] sf_plane = sf >= 3'd3 ? 2'(sf - 3'd3) : 2'(sf);
wire [1:0] s_bank   = {s_at[5], s_code[7]};
wire [13:0] sf_byte = {s_bank, 12'd0} + {2'd0, s_row[4], 11'd0} + {3'd0, s_code[5:0], 5'd0} +
                      {9'd0, sf_half[0], s_row[3:0]};                        // bank * 0x1000 + row 16-31 * 0x800 + ...
wire [16:0] sf_off  = (sf_plane == 2'd0 ? 17'h0000 : sf_plane == 2'd1 ? 17'h4000 : 17'h8000) + {3'd0, sf_byte};
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
        s_pl[sf_idx] <= vid_rom_q;
        sf_cap <= 1'b0;
    end
    if (ce_pix && hcnt == 9'd383) begin
        ss <= S_ATTR;  sk <= 6'd0;  sa <= 3'd0;
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
                s_row <= s_at[7] ? 5'd31 - s_rr[4:0] : s_rr[4:0];
                sf <= 3'd0;
                ss <= S_FETCH;
            end else ss <= S_NEXT;
        end
        S_FETCH: begin
            if (spr_slot) begin
                sf_cap <= 1'b1;
                sf_idx <= sf;
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
            if (sk == 6'd55) ss <= S_IDLE;
            else begin sk <= sk + 6'd1; ss <= S_ATTR; end
        end
        default: ss <= S_IDLE;
    endcase
end

endmodule
