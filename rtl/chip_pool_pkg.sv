//============================================================================
//
//  chip_pool_pkg — bus types for a multi-board core's shared CPU / sound chip pool (canonical; same file in every core)
//  Copyright (C) 2026 Rodimus
//
//  Only one board runs at a time, so the CPUs and sound chips are instanced once in rtl/konami82_board.sv
//  and the active board drives them through these buses (selected by variant). *_i = board to chip,
//  *_o = chip to every board. Reset is the core reset.
//
//============================================================================

package chip_pool_pkg;

// Z80 (T80s, Mode 0, T2Write 1, IOWait 1; WAIT_n / BUSRQ_n inactive)
typedef struct packed {
    logic        ce;
    logic        int_n;
    logic        nmi_n;
    logic  [7:0] di;
} z80_i_t;

typedef struct packed {
    logic [15:0] a;
    logic  [7:0] dout;
    logic        m1_n;
    logic        mreq_n;
    logic        iorq_n;
    logic        rd_n;
    logic        wr_n;
    logic        rfsh_n;
} z80_o_t;

// MC6809E (mc6809is; HALT / DMABREQ inactive)
typedef struct packed {
    logic        fall_e;
    logic        fall_q;
    logic        irq_n;
    logic        firq_n;
    logic        nmi_n;
    logic  [7:0] d;
} m6809_i_t;

typedef struct packed {
    logic [15:0] a;
    logic  [7:0] dout;
    logic        rw;
    logic        lic;
} m6809_o_t;

// Z80 (T80s, as z80_i_t) with RESET_n on the bus: Konami82's board watchdogs reset the main and sound CPUs
typedef struct packed {
    logic        reset_n;
    logic        ce;
    logic        int_n;
    logic        nmi_n;
    logic  [7:0] di;
} z80r_i_t;

typedef struct packed {
    logic [15:0] a;
    logic  [7:0] dout;
    logic        m1_n;
    logic        mreq_n;
    logic        iorq_n;
    logic        rd_n;
    logic        wr_n;
    logic        rfsh_n;
} z80r_o_t;

// MC6809E (mc6809is, as m6809_i_t) with RESET on the bus: Konami82's board watchdogs
typedef struct packed {
    logic        reset_n;
    logic        fall_e;
    logic        fall_q;
    logic        irq_n;
    logic        firq_n;
    logic        nmi_n;
    logic  [7:0] d;
} m6809r_i_t;

typedef struct packed {
    logic [15:0] a;
    logic  [7:0] dout;
    logic        rw;
    logic        lic;
} m6809r_o_t;

// AY-3-8910 (jt49_bus, sel = 1)
typedef struct packed {
    logic        ce;
    logic        bdir;
    logic        bc1;
    logic  [7:0] din;
    logic  [7:0] ioa_in;
    logic  [7:0] iob_in;
} ay_i_t;

typedef struct packed {
    logic  [7:0] dout;
    logic  [7:0] a;
    logic  [7:0] b;
    logic  [7:0] c;
    logic  [7:0] iob_out;
    logic  [9:0] sound;                 // jt49 internal A+B+C mix (registered per accumulator cycle)
} ay_o_t;

// 8039 (T48 t8039_notri_extrom, EA low, program on pmem; T0 / T1 low, P1 / P2 inputs high)
typedef struct packed {
    logic        xtal_en;
    logic        int_n;
    logic  [7:0] db;
    logic  [7:0] pmem_d;
} mcs48_i_t;

typedef struct packed {
    logic  [7:0] p1;
    logic  [7:0] p2;
    logic [11:0] pmem_a;
    logic        rd_n;
} mcs48_o_t;

// Z80 (T80pa, Mode 0; BUSRQ_n inactive). RESET_n is on the bus: Lady Bug / 8203 hold their CPUs in reset ~48 ms past the core reset
typedef struct packed {
    logic        reset_n;
    logic        cen_p;
    logic        cen_n;
    logic        wait_n;
    logic        int_n;
    logic        nmi_n;
    logic  [7:0] di;
} z80pa_i_t;

typedef struct packed {
    logic [15:0] a;
    logic  [7:0] dout;
    logic        m1_n;
    logic        mreq_n;
    logic        iorq_n;
    logic        rd_n;
    logic        wr_n;
    logic        rfsh_n;
} z80pa_o_t;

// SN76489 (jt89, INTERPOL16 0, MODE 0). rst is on the bus: Lady Bug resets its pair on power-on only
typedef struct packed {
    logic        rst;
    logic        ce;
    logic        wr_n;
    logic        cs_n;
    logic  [7:0] din;
} sn76489_i_t;

typedef struct packed {
    logic [10:0] sound;                 // signed
    logic        ready;
} sn76489_o_t;

// MB88 (mb88 / mb88_core; STBY / SC inputs inactive, RAM clear = core reset)
typedef struct packed {
    logic        ena;
    logic        reset_n;
    logic  [3:0] k;
    logic [15:0] r_in;          // {R3, R2, R1, R0}
    logic        irq_n;
    logic        tc_n;
    logic        si_n;
    logic  [7:0] rom_d;
} mb88_i_t;

typedef struct packed {
    logic [15:0] r_out;         // {R3, R2, R1, R0}
    logic  [7:0] o;             // {OH, OL}
    logic        o_wr;
    logic  [3:0] p;
    logic [10:0] rom_a;
} mb88_o_t;

// NEW-Irem1-2026-10-24: types added by the Irem1 core, to be merged into the master (Konami82) copy

// M6803-2026-10-25: jtframe_6801mcu (internal-ROM MODE 6 only) replaced by our rtl/cpu/m6803.sv. Old types:
// // MC6801 / MC6803 (jtframe_6801mcu + jt680x 6801 microcode; SCI not implemented). The Irem sound boards run a
// // 6803 (no internal ROM) on external ROM; rom_data / rom_cs are the wrapper's internal-ROM port
// typedef struct packed {
//     logic        cen;
//     logic        cen_tmr;
//     logic        irq;
//     logic        nmi;
//     logic  [7:0] xdin;
//     logic  [7:0] p1_din;
//     logic  [4:0] p2_din;
//     logic  [7:0] p3_din;
//     logic  [7:0] p4_din;
//     logic  [7:0] rom_data;
// } m6801_i_t;
//
// typedef struct packed {
//     logic [15:0] addr;
//     logic  [7:0] dout;
//     logic        wr;
//     logic        x_cs;
//     logic  [7:0] p1_dout;
//     logic  [4:0] p2_dout;
//     logic  [7:0] p3_dout;
//     logic  [7:0] p4_dout;
//     logic        rom_cs;
// } m6801_o_t;

// MC6803 (rtl/cpu/m6803.sv around jt680x, 6801 microcode): expanded multiplexed mode, external ROM; on-chip
// registers 0x00-0x1F and RAM 0x80-0xFF, everything else is an external access (x_cs). cen = crystal rate (4 x E)
typedef struct packed {
    logic        cen;
    logic        irq;
    logic        nmi;
    logic  [7:0] xdin;
    logic  [7:0] p1_in;
    logic  [4:0] p2_in;
    logic  [2:0] mode;
} m6803_i_t;

typedef struct packed {
    logic [15:0] addr;
    logic  [7:0] dout;
    logic        wr;
    logic        x_cs;
    logic  [7:0] p1_out;
    logic  [4:0] p2_out;
} m6803_o_t;

// JT5205-2026-10-24: MSM5205 types for our rtl/sound/msm5205.sv, replaced by jotego's jt5205 (user: fleet-wide).
// MSM-REVERT-2026-10-25: our msm5205.sv back in the pool (user: jt5205 Moon Patrol rock "thonk" inaudible on HW)
typedef struct packed {
    logic        step;
    logic        rst;
    logic  [3:0] data;
} msm5205_i_t;

typedef struct packed {
    logic [11:0] signal;                // signed
} msm5205_o_t;

// MSM5205 ADPCM (rtl/sound/jt5205, Jose Tejada): cen = the chip's 384 kHz clock, sel = S1/S2 prescaler pins,
// rst = RESET pin. irq = the VCK strobe (sample rate) that feeds the sound CPU's NMI; vclk = the VCK pin level.
typedef struct packed {
    logic        rst;
    logic        cen;
    logic  [1:0] sel;
    logic  [3:0] din;
} jt5205_i_t;

typedef struct packed {
    logic [11:0] sound;                 // signed
    logic        irq;
    logic        vclk;
} jt5205_o_t;

endpackage
