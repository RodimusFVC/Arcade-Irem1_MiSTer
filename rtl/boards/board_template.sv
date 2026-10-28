//============================================================================
//  board_template.sv - starting point for a new Irem1 board (NOT in files.qip, never compiled).
//  Copyright (C) 2026 Rodimus
//
//  Boards instantiate no CPU, sound chip, ROM store or board RAM; those live once in rtl/irem1_dispatcher.sv
//  (main Z80, sound m6803 / AY x2 / MSM5205 x2 via irem_sound, ROM rom0 / rom1, RAM pool blocks 0-5 =
//  wram / vram / xram / sram / lbuf0 / lbuf1). A board only drives the ports below.
//  To add one: copy to rtl/boards/<board>_board.sv, add it to files.qip + verilator/Makefile, give it the next
//  slot in the dispatcher's bd_*[n] bundles plus a variant select, and tie unused RAM ports to 0.
//  A RAM larger than its block: grow the block (AW) in the dispatcher's pool. A private RAM / chip that cannot
//  pool: mark the instance "// POOL-EXCEPTION: <reason>" (checked by verilator/common/pool_check.sh).
//============================================================================

module board_template
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

endmodule
