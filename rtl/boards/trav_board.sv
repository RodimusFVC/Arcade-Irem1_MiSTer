//============================================================================
//
//  trav_board - Irem Traverse USA board (variant 4: travrusa family; 5: Shot Rider; 6: Shot Rider bootleg)
//  Copyright (C) 2026 Rodimus
//
//  No schematics found (Traverse USA manuals: pinout / DIPs only): MAME irem/travrusa.cpp is the model.
//  Same pool ports as m52_board / m58_board.
//
//  Raster: 384 x 282 at 6.144 MHz; visible x 8-247, y 0-255 (MAME set_raw); IRQ0 = VBLANK level (MAME
//  screen_vblank -> IRQ0, asserted for lines 256-281). Screen ROT270 (Shot Rider also ORIENTATION_FLIP_X = native
//  x mirrored here).
//  Tilemap 64 x 32, 8x8 3 bpp, flip x / y per tile; rows 0-191 scroll x (9 bits), rows 192-255 fixed (status band).
//  Colour 15 tiles = "tunnels": their pens 6 and 7 are drawn above the sprites (MAME split group 1).
//  Sprites: 128 x 16x16 3 bpp (offset 0x1FC first, 0x000 on top), pen 0 transparent, clipped to y 0-191 / x 8-247;
//  colour lookup PROM -> 16-entry sprite palette.
//  CPU ROM decode on read (index 1 byte 2): 1 = MotoRace (0000-1FFF address + data bitswap), 2 = Shot Rider Sigma
//  (0000-1FFF data bitswap). Shot Rider (variant 5): Z80 at 6 MHz (MAME), own palette wiring and sprite layout;
//  the bootleg (6) reads 0x88 from I/O port 0x11.
//  Pixel pipeline as m58_board (two palette reads, ph6 / ph7; output at h + 2). Flip (game XOR CRT): raster mirror.
//
//============================================================================

module trav_board
(
    input  logic        clk,                // 49.152 MHz
    input  logic        reset,
    input  logic        pause,
    input  logic  [7:0] variant,
    input  logic  [3:0] rom_dec,
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

wire shtrider  = variant == 8'd5;                    // 6 MHz CPU, own palette
wire sh_gfx    = variant == 8'd5 || variant == 8'd6; // Shot Rider sprite layout + FLIP_X
wire shtriderb = variant == 8'd6;

/////////////////////////////  Region table  /////////////////////////////

wire [16:0] rom_base  = {rom_rgn[1][0],  rom_rgn[0],  8'h00};
wire [16:0] rom_start = {rom_rgn[3][0],  rom_rgn[2],  8'h00};
wire [16:0] rom_len   = {rom_rgn[5][0],  rom_rgn[4],  8'h00};
wire [16:0] tx_base   = {rom_rgn[13][0], rom_rgn[12], 8'h00};
wire [16:0] sp_base   = {rom_rgn[19][0], rom_rgn[18], 8'h00};
wire [11:0] prom      = {rom_rgn[42][3:0], 8'h00};
// PROM block (MAME region offsets): tile palette +000 (Shot Rider: second tile PROM +100), sprite palette +200,
// sprite lookup +220
wire [11:0] TP0 = prom, TP1 = prom + 12'h100, SP_PAL = prom + 12'h200, SP_CLUT = prom + 12'h220;

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
wire vblank_raw = vcnt >= 9'd256;
// HSYNC 32 px from x 280 (+2 x H adjust: 264-294; blanking 248-383 / 0-7); VSYNC 3 lines from y 262 (+-3: 259-265)
wire [8:0] hs_start = 9'd280 + {{4{h_adj[3]}}, h_adj, 1'b0};
wire [3:0] vadj_c   = ($signed(v_adj) > 4'sd3) ? 4'd3 : ($signed(v_adj) < -4'sd3) ? 4'b1101 : v_adj;
wire [8:0] vs_start = 9'd262 + {{5{vadj_c[3]}}, vadj_c};
wire hsync_raw = hcnt >= hs_start && hcnt < hs_start + 9'd32;
wire vsync_raw = vcnt >= vs_start && vcnt < vs_start + 9'd3;

/////////////////////////////  Main CPU  /////////////////////////////

wire [15:0] a = cpu_o.a;
wire mem_wr = ~cpu_o.mreq_n & ~cpu_o.wr_n & cpu_o.rfsh_n;
wire io_rd  = ~cpu_o.iorq_n & ~cpu_o.rd_n & cpu_o.m1_n;
// interrupt acknowledge = M1 + IORQ with MREQ high (T80pa IntCycleD_n power-up glitch, see m52_board)
wire int_ack = ~cpu_o.iorq_n & ~cpu_o.m1_n & cpu_o.mreq_n;
logic mem_wr_d = 1'b0;
always_ff @(posedge clk) mem_wr_d <= mem_wr;
wire mem_wstb = mem_wr & ~mem_wr_d;

// 3.072 MHz (Shot Rider 6.144 MHz: MAME 6 MHz), CEN_p / CEN_n half a cycle apart
wire [3:0] c16 = {hcnt[0], ph};
wire cen_p = shtrider ? ph == 3'd0 : c16 == 4'd0;
wire cen_n = shtrider ? ph == 3'd4 : c16 == 4'd8;

// MAME program_map (no mirrors): 0000-7FFF ROM, 8000-8FFF video RAM, 9000 / A000 scroll x low / high (write),
// C800-C9FF sprites (write only), D000-D004 I/O, E000-EFFF RAM
wire sel_rom  = ~a[15];
wire sel_vram = a[15:12] == 4'h8;
wire sel_sxl  = a == 16'h9000, sel_sxh = a == 16'hA000;
wire sel_sprt = a[15:9] == 7'b1100100;                                       // C800-C9FF
wire sel_io   = a[15:3] == 13'h1A00 && a[2:0] <= 3'd4;
wire sel_wram = a[15:12] == 4'hE;

// ROM decode on read (MAME init_motorace / init_shtridra rewrote 0000-1FFF at load; here the read is undone)
wire        dec_lo = a[15:13] == 3'b000;
wire [15:0] a_mr   = {a[15:13], a[7], a[0], a[6], a[12], a[5], a[11], a[4], a[10], a[3], a[9], a[2], a[8], a[1]};
wire [15:0] a_rd   = (rom_dec == 4'd1 && dec_lo) ? a_mr : a;
wire [16:0] rom_off = {1'b0, a_rd} - rom_start;
assign cpu_rom_a = rom_base + rom_off;
wire rom_in = !rom_off[16] && rom_off < rom_len;
wire [7:0] d  = cpu_rom_q;
wire [7:0] rom_d = !dec_lo           ? d :
                   rom_dec == 4'd1   ? {d[2], d[7], d[4], d[1], d[6], d[3], d[0], d[5]} :
                   rom_dec == 4'd2   ? {d[7], d[5], d[6], d[3], d[4], d[2], d[1], d[0]} : d;

logic [7:0] scrollx_lo, scrollx_hi;
logic       flip_reg;
always_ff @(posedge clk) begin
    if (reset) begin
        scrollx_lo <= 8'd0;  scrollx_hi <= 8'd0;  flip_reg <= 1'b0;
    end else if (mem_wstb) begin
        if (sel_sxl) scrollx_lo <= cpu_o.dout;
        if (sel_sxh) scrollx_hi <= cpu_o.dout;
        if (sel_io && a[2:0] == 3'd1) flip_reg <= cpu_o.dout[0];             // D001: flip (+ coin counters)
    end
end
assign snd_wr  = mem_wstb && sel_io && a[2:0] == 3'd0;                       // D000: sound command
assign snd_cmd = cpu_o.dout;

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

logic [7:0] io_q;
always_comb case (a[2:0])
    3'd0: io_q = in0;   3'd1: io_q = in1;   3'd2: io_q = in2;   3'd3: io_q = dsw1;   default: io_q = dsw2;
endcase

logic [7:0] cpu_di;
always_comb begin
    if (int_ack)       cpu_di = 8'hFF;
    else if (io_rd)    cpu_di = (shtriderb && a[7:0] == 8'h11) ? 8'h88 : 8'h00;   // MAME shtriderb_port11_r
    else if (sel_rom)  cpu_di = rom_in ? rom_d : 8'h00;
    else if (sel_vram) cpu_di = vram_qa;
    else if (sel_io)   cpu_di = io_q;
    else if (sel_wram) cpu_di = wram_q;
    else               cpu_di = 8'h00;                                       // MAME unmapped read
end

assign cpu_i.reset_n = 1'b1;
assign cpu_i.cen_p   = cen_p && !pause;
assign cpu_i.cen_n   = cen_n && !pause;
assign cpu_i.wait_n  = 1'b1;
assign cpu_i.int_n   = ~vblank_raw;                                          // MAME: IRQ0 = VBLANK level
assign cpu_i.nmi_n   = 1'b1;
assign cpu_i.di      = cpu_di;

/////////////////////////////  Game position  /////////////////////////////

wire       flip = (flip_reg ^ ~dsw2[0]) ^ crt_flip;
wire [8:0] fx = flip ? 9'd255 - hcnt : hcnt;
wire [8:0] gx = sh_gfx ? 9'd255 - fx : fx;                                   // Shot Rider: ORIENTATION_FLIP_X
wire [8:0] gy = flip ? 9'd255 - vcnt : vcnt;
wire [8:0] vn = (vcnt == 9'd281) ? 9'd0 : vcnt + 9'd1;
wire [8:0] gyn = flip ? 9'd255 - vn : vn;

/////////////////////////////  Per-pixel fetch (8 clocks)  /////////////////////////////

// tilemap: rows 0-191 x = gx + scroll x, rows 192-255 x = gx; y = gy; tile at video RAM byte {row, col, b}:
// b 0 = code, b 1 = attr (code bits 9-8 = attr 7-6, flip x / y = attr 5 / 4 (MAME TILE_FLIPXY), colour = attr 3-0)
wire [7:0] my = gy[7:0];
wire [8:0] mx = gx + ((my[7:6] == 2'd3) ? 9'd0 : {scrollx_hi[0], scrollx_lo});
wire [10:0] t_idx = {my[7:3], mx[8:3]};
logic [7:0] t_code, t_attr, t_p0, t_p1, t_p2;
wire  [9:0] t_tile_now = {vram_qb[7:6], t_code};
wire  [9:0] t_tile     = {t_attr[7:6], t_code};
wire  [2:0] t_y_now    = vram_qb[4] ? 3'd7 - my[2:0] : my[2:0];
wire  [2:0] t_y        = t_attr[4]  ? 3'd7 - my[2:0] : my[2:0];

always_comb begin
    vram_ab   = {t_idx, 1'b0};
    vid_rom_a = spr_rom_a;
    case (ph)
        3'd0: vram_ab   = {t_idx, 1'b0};
        3'd1: vram_ab   = {t_idx, 1'b1};
        3'd2: vid_rom_a = tx_base +           {4'd0, t_tile_now, t_y_now};  // plane 2 (pen bit 0)
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
        3'd1: t_code <= vram_qb;
        3'd2: t_attr <= vram_qb;
        3'd3: t_p2   <= vid_rom_q;
        3'd4: t_p1   <= vid_rom_q;
        3'd5: t_p0   <= vid_rom_q;
        default: ;
    endcase
end

// colour select (ph6): tunnel pens (colour 15, pens 6-7) > sprite > tile
wire [2:0] txb    = t_attr[5] ? mx[2:0] : 3'd7 - mx[2:0];
wire [2:0] t_pen  = {t_p0[txb], t_p1[txb], t_p2[txb]};
wire       tunnel = t_attr[3:0] == 4'hF && t_pen[2:1] == 2'b11;
wire [7:0] spr_now = gy[0] ? lb_qb1 : lb_qb0;
logic       k_spr, k_spr_d;
logic [6:0] pidx, pidx_d;
always_comb begin
    if (spr_now[4] && !tunnel) begin k_spr = 1'b1; pidx = {3'd0, spr_now[3:0]};   end
    else                       begin k_spr = 1'b0; pidx = {t_attr[3:0], t_pen};   end
end
always_comb begin
    pal_a = TP0 + {5'd0, pidx_d};
    case (ph)
        3'd6: pal_a = k_spr ? SP_PAL + {5'd0, pidx} : TP0 + {5'd0, pidx};
        3'd7: pal_a = TP1 + {5'd0, pidx_d};                                  // Shot Rider's second tile PROM
        default: ;
    endcase
end

// MAME travrusa_palette: r = 0x52 / 0xAD (bits 6, 7), g / b = 0x21 / 0x47 / 0x97 (bits 3-5 / 0-2).
// shtrider_palette: tiles r = 0x47 / 0x97 (p0 bits 2, 3), g = 0x21 (p1 bit 3) / 0x47 / 0x97 (p0 bits 0, 1),
// b = p1 bits 0-2; sprites r = 0x47 / 0x97 (bits 6, 7), g / b as Traverse USA.
function automatic [7:0] w3(input [2:0] v);
    case (v) 3'd0: w3 = 8'd0; 3'd1: w3 = 8'd33; 3'd2: w3 = 8'd71; 3'd3: w3 = 8'd104;
             3'd4: w3 = 8'd151; 3'd5: w3 = 8'd184; 3'd6: w3 = 8'd222; default: w3 = 8'd255; endcase
endfunction
function automatic [7:0] r2(input [1:0] v);
    case (v) 2'd0: r2 = 8'd0; 2'd1: r2 = 8'd82; 2'd2: r2 = 8'd173; default: r2 = 8'd255; endcase
endfunction
function automatic [7:0] r2s(input [1:0] v);
    case (v) 2'd0: r2s = 8'd0; 2'd1: r2s = 8'd71; 2'd2: r2s = 8'd151; default: r2s = 8'd222; endcase
endfunction

logic [7:0] pal_lo, rgb_r, rgb_g, rgb_b;
logic       hbl_d, vbl_d, hs_d, vs_d;
always_ff @(posedge clk) begin
    if (ph == 3'd6) begin k_spr_d <= k_spr; pidx_d <= pidx; end
    if (ph == 3'd7) pal_lo <= pal_q;
    if (ph == 3'd0) begin                                                     // pal_q = second tile PROM now
        if (!shtrider || k_spr_d) begin
            rgb_r <= shtrider ? r2s(pal_lo[7:6]) : r2(pal_lo[7:6]);
            rgb_g <= w3(pal_lo[5:3]);
            rgb_b <= w3(pal_lo[2:0]);
        end else begin
            rgb_r <= r2s(pal_lo[3:2]);
            rgb_g <= w3({pal_lo[1:0], pal_q[3]});
            rgb_b <= w3(pal_q[2:0]);
        end
    end
    if (ce_pix) begin
        hbl_d <= hblank_raw;  vbl_d <= vblank_raw;  hs_d <= hsync_raw;  vs_d <= vsync_raw;
        video_hblank <= hbl_d;  video_vblank <= vbl_d;  video_hs <= hs_d;  video_vs <= vs_d;
        video_r <= rgb_r;  video_g <= rgb_g;  video_b <= rgb_b;
    end
end

/////////////////////////////  Sprite engine (one line ahead)  /////////////////////////////

// MAME draw_sprites: offsets 0x1FC down to 0x000 (0x000 on top). [0] y (top = 240 - y), [1] flip y / flip x /
// colour (4 bits), [2] code, [3] x (((x + 8) & 0xFF) - 8). 3 planes at 0x4000 (pen bit 2), 0x2000, 0x0000.
// Traverse USA layout: right half +16 bytes, row = byte; Shot Rider: right half +8, rows 8-15 at +16.
// Pen 0 transparent; line buffer = {opaque, lookup value}. Clip: y 0-191, x 8-247.
typedef enum logic [2:0] {S_IDLE, S_ATTR, S_CHECK, S_FETCH, S_DRAW, S_NEXT} sstate_t;
sstate_t     ss = S_IDLE;
logic  [6:0] sk;
logic  [2:0] sa;
logic  [7:0] s_y, s_at, s_code, s_x;
logic  [3:0] s_row;
logic  [2:0] sf, sf_idx;
logic        sf_cap = 1'b0;
logic  [7:0] s_pl[6];
logic  [4:0] sd;
logic [16:0] spr_rom_a;

wire [8:0] s_offs = {7'd127 - sk, 2'b00};
assign sram_ab = s_offs + {6'd0, sa};

wire [9:0] s_sy = 10'd240 - {2'b00, s_y};
wire [9:0] s_rr = {1'b0, gyn} - s_sy;
wire [1:0] sf_half  = sf >= 3'd3 ? 2'd1 : 2'd0;
wire [1:0] sf_plane = sf >= 3'd3 ? 2'(sf - 3'd3) : 2'(sf);
wire [4:0] sf_byte  = sh_gfx ? {s_row[3], sf_half[0], s_row[2:0]} : {sf_half[0], s_row};
wire [16:0] sf_off  = (sf_plane == 2'd0 ? 17'h4000 : sf_plane == 2'd1 ? 17'h2000 : 17'h0000) +
                      {4'd0, s_code, sf_byte};
always_comb spr_rom_a = sp_base + sf_off;

wire [3:0] sd_x  = s_at[6] ? 4'd15 - sd[3:0] : sd[3:0];
wire [2:0] sd_b  = 3'd7 - sd_x[2:0];
wire [2:0] sd_pen = sd_x[3] ? {s_pl[3][sd_b], s_pl[4][sd_b], s_pl[5][sd_b]}
                            : {s_pl[0][sd_b], s_pl[1][sd_b], s_pl[2][sd_b]};
logic [2:0] sd_pen_d;
assign clut_a = SP_CLUT + {5'd0, s_at[3:0], sd_pen};

// screen x = ((x + 8) & 0xFF) - 8 + pixel; clip 8-247 (10-bit signed)
wire [9:0] s_sx   = {2'b00, 8'(s_x + 8'd8)} - 10'd8;
wire [9:0] s_px   = s_sx + {6'd0, 4'(sd - 5'd1)};
wire       s_clip = !s_px[9] && s_px >= 10'd8 && s_px <= 10'd247 && gyn < 9'd192;
wire   lb_we = ss == S_DRAW && !sf_cap && sd != 5'd0 && sd_pen_d != 3'd0 && s_clip;
assign lb_aa = s_px[8:0];
assign lb_da = {3'd0, 1'b1, clut_q[3:0]};
assign lb_wa = lb_we ? (gyn[0] ? 2'b10 : 2'b01) : 2'b00;

always_ff @(posedge clk) begin
    sd_pen_d <= sd_pen;
    if (sf_cap) begin
        s_pl[sf_idx] <= vid_rom_q;
        sf_cap <= 1'b0;
    end
    if (ce_pix && hcnt == 9'd383) begin
        ss <= S_ATTR;  sk <= 7'd0;  sa <= 3'd0;
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
            if (s_rr[9:4] == 6'd0 && gyn < 9'd192) begin
                s_row <= s_at[7] ? 4'd15 - s_rr[3:0] : s_rr[3:0];
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
            if (sk == 7'd127) ss <= S_IDLE;
            else begin sk <= sk + 7'd1; ss <= S_ATTR; end
        end
        default: ss <= S_IDLE;
    endcase
end

endmodule
