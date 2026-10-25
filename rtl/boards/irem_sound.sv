//============================================================================
//
//  irem_sound - Irem M52-SOUND-C sound board (every Irem1 variant; M58 uses its "large" map)
//  Copyright (C) 2026 Rodimus
//
//  Drawing C-9457 (Moon Patrol drawing set, Useful Information/schematics, page 11) + MAME irem/irem.cpp.
//  6803 at 3.579545 MHz (pool m6803), E clocks both AY-3-8910s (pool ay[0] = 3J / MAME 45M, ay[1] = 3F / 45L),
//  MSM5205 at 384 kHz (pool msm5205[0], our msm5205.sv; [1] = M58's second chip, not wired yet).
//    Port 1 = AY data bus. Port 2: bit 0 falling edge = AY strobe, bit 2 = address (1) / data (0), bit 3 = 3J,
//    bit 4 = 3F; port 1 reads the selected AY. 3J port A = sound command latch, 3J port B = MSM5205 control.
//    Small map (A15 ignored): 0x0000-0x0FFF MSM5205 data (A0 = chip 1, A1 = chip 2), 0x1000-0x1FFF IRQ ack,
//    0x2000-0x7FFF ROM. Large map: 0x0000-0x1FFF MSM data, 0x2000-0x3FFF ack, 0x4000-0xFFFF ROM.
//    IRQ (MAME cmd_w): set by a command with bit 7 clear (and at reset), cleared by an ack while bit 7 is set.
//    NMI = MSM5205 VCK through the TR1 inverter (6803 NMI is edge triggered).
//  MSM5205 control from 3J port B. Drawing: IOB0 = RES, IOB1 = S1, IOB2 = S2, IOB3 = 4B/3B, each with a 4.7K
//  pull-up (so the pins read high while the AY port is an input). MAME's map (bit 0 / 1 = reset chip 1 / 2,
//  bits 2-4 = playmode) is one bit higher; msm_mame_map selects it (open: which one the Moon Patrol code expects).
//
//  Mixer, from the drawing except R15 (MAME m52_sound_c_discrete differs: C32 0.022 uF, Sallen-Key bypassed):
//    all six AY outputs share one 470 ohm load (R2) -> R25 10K -> summing node;
//    MSM DA OUT -> R19 10K -> C37 0.1u -> R22 47K (+5) / R23 2.2K -> 3M follower -> R12 10K / C30 0.022u ->
//    Sallen-Key low-pass (R13 = R14 = 10K, C32 0.027u feedback, C36 0.0068u) -> R15 2.2K (drawing: 22K) -> summing node;
//    summing node -> VR1 50K -> C28 1u -> R9 560 / R10 330 -> M51516 amp (high-pass ~18 Hz at a mid volume setting).
//    AY node voltage = MAME's AY8910 output model (ay8910_param r_up / r_down / res[]) with all six channels on
//    the one 470 ohm load, solved with a divider at 768 kHz and averaged to 192 kHz. The filter chain runs at
//    192 kHz (forward Euler, one time-shared 27 x 18 multiplier).
//
//============================================================================

module irem_sound
(
    input  logic        clk,                // 49.152 MHz
    input  logic        reset,
    input  logic        pause,
    input  logic        large_map,          // M58 (10-Yard Fight) sound map
    input  logic        msm_mame_map,       // MSM5205 control bits: 0 = M52-SOUND-C drawing, 1 = MAME
    input  logic [47:0][7:0] rom_rgn,       // ROM region table (slot 1 = sound CPU)

    input  logic        cmd_wr,             // main CPU sound command write (one clock)
    input  logic  [7:0] cmd,

    output chip_pool_pkg::m6803_i_t cpu_i,
    input  chip_pool_pkg::m6803_o_t cpu_o,

    output logic [16:0] rom_a,
    input  logic  [7:0] rom_q,

    output chip_pool_pkg::ay_i_t     [1:0] ay_i,
    input  chip_pool_pkg::ay_o_t     [1:0] ay_o,
    // MSM-REVERT-2026-10-25: pool back on our msm5205.sv; jt5205 ports were:
    // output chip_pool_pkg::jt5205_i_t [1:0] msm_i,
    // input  chip_pool_pkg::jt5205_o_t [1:0] msm_o,
    output chip_pool_pkg::msm5205_i_t [1:0] msm_i,
    input  chip_pool_pkg::msm5205_o_t [1:0] msm_o,

    output logic signed [15:0] audio
);

/////////////////////////////  Clocks  /////////////////////////////

// 3.579545 MHz crystal enable (fractional, from 49.152 MHz); E = crystal / 4 clocks the AYs
logic [25:0] facc = 26'd0;
logic        cen_x = 1'b0, cen_x_d = 1'b0;
logic  [1:0] ediv = 2'd0;
wire  [25:0] facc_nx = facc + 26'd3579545;
always_ff @(posedge clk) begin
    if (facc_nx >= 26'd49152000) begin facc <= facc_nx - 26'd49152000; cen_x <= ~pause; end
    else                         begin facc <= facc_nx;                 cen_x <= 1'b0;   end
    cen_x_d <= cen_x;
    if (cen_x) ediv <= ediv + 2'd1;
end
wire cen_e = cen_x && ediv == 2'd3;

// MSM5205 384 kHz = 49.152 MHz / 128
logic [6:0] mdiv = 7'd0;
always_ff @(posedge clk) mdiv <= mdiv + 7'd1;
wire cen_msm = mdiv == 7'd0;

/////////////////////////////  6803 bus  /////////////////////////////

wire [15:0] a  = cpu_o.addr;
wire [15:0] am = large_map ? a : {1'b0, a[14:0]};
wire        wstb = cen_x_d & cpu_o.wr & cpu_o.x_cs;         // one strobe per external write cycle

// ROM (region slot 1): store address = base + (offset - start)
wire [15:0] rg_base  = {rom_rgn[7],  rom_rgn[6]};
wire [15:0] rg_start = {rom_rgn[9],  rom_rgn[8]};
wire [15:0] rg_len   = {rom_rgn[11], rom_rgn[10]};
wire        rom_area = large_map ? a[15:14] != 2'b00 : am[14:13] != 2'b00;
wire [16:0] rom_off  = {1'b0, am} - {1'b0, rg_start[7:0], 8'h00};
wire        rom_in   = !rom_off[16] && rom_off < {1'b0, rg_len[7:0], 8'h00};
assign rom_a = {rg_base[8:0], 8'h00} + rom_off;

wire msm_area = large_map ? a[15:13] == 3'b000 : am[14:12] == 3'b000;
wire ack_area = large_map ? a[15:13] == 3'b001 : am[14:12] == 3'b001;

logic [7:0] latch = 8'd0;
logic       irq   = 1'b1;
logic [3:0] msm_d0 = 4'd0, msm_d1 = 4'd0;
always_ff @(posedge clk) begin
    if (reset) begin
        latch <= 8'd0;
        irq   <= 1'b1;                                      // MAME device_reset asserts the IRQ
        msm_d0 <= 4'd0;  msm_d1 <= 4'd0;
    end else begin
        if (cmd_wr) begin
            latch <= cmd;
            if (!cmd[7]) irq <= 1'b1;
        end
        if (wstb && ack_area && latch[7]) irq <= 1'b0;
        if (wstb && msm_area) begin
            if (a[0]) msm_d0 <= cpu_o.dout[3:0];
            if (a[1]) msm_d1 <= cpu_o.dout[3:0];
        end
    end
end

// NMI = VCK rising edge, stretched so the 6803 (sampling on cen) sees it
logic [5:0] nmi_cnt = 6'd0;
logic       msm_step = 1'b0;                          // MSM5205 VCK strobe (made in the MSM5205 control section)
always_ff @(posedge clk)
    // MSM-REVERT-2026-10-25: was  if (msm_o[0].irq) nmi_cnt <= 6'd63;   (jt5205 VCK strobe)
    if (msm_step) nmi_cnt <= 6'd63;
    else if (nmi_cnt != 6'd0) nmi_cnt <= nmi_cnt - 6'd1;

/////////////////////////////  AY bus (6803 ports)  /////////////////////////////

wire [7:0] p1 = cpu_o.p1_out;
wire [4:0] p2 = cpu_o.p2_out;
logic [4:0] p2_d = 5'h1F;
logic       ay_addr_stb = 1'b0, ay_data_stb = 1'b0;
logic [1:0] ay_sel_stb = 2'b00;
logic [3:0] ay_reg[2];                                      // shadow address latch, for register 7 (port direction)
logic [1:0] iob_out_en = 2'b00;                             // register 7 bit 7 per chip
always_ff @(posedge clk) begin
    p2_d <= p2;
    ay_addr_stb <= 1'b0;
    ay_data_stb <= 1'b0;
    if (p2_d[0] && !p2[0]) begin                            // MAME m6803_port2_w: uses the previous port 2 value
        ay_sel_stb  <= p2_d[4:3];
        ay_addr_stb <= p2_d[2];
        ay_data_stb <= ~p2_d[2];
        for (int n = 0; n < 2; n++) if (p2_d[3 + n]) begin
            if (p2_d[2])               ay_reg[n] <= p1[3:0];
            else if (ay_reg[n] == 4'd7) iob_out_en[n] <= p1[7];
        end
    end
    if (reset) begin iob_out_en <= 2'b00; ay_reg[0] <= 4'd0; ay_reg[1] <= 4'd0; end
end

always_comb begin
    for (int n = 0; n < 2; n++) begin
        ay_i[n].ce     = cen_e;
        ay_i[n].din    = p1;
        ay_i[n].ioa_in = n == 0 ? latch : 8'hFF;
        ay_i[n].iob_in = 8'hFF;
        if (ay_sel_stb[n] && ay_addr_stb)      {ay_i[n].bdir, ay_i[n].bc1} = 2'b11;
        else if (ay_sel_stb[n] && ay_data_stb) {ay_i[n].bdir, ay_i[n].bc1} = 2'b10;
        else if (p2[3 + n])                    {ay_i[n].bdir, ay_i[n].bc1} = 2'b01;   // read the selected chip
        else                                   {ay_i[n].bdir, ay_i[n].bc1} = 2'b00;
    end
end

wire [7:0] p1_in = p2[3] ? ay_o[0].dout : p2[4] ? ay_o[1].dout : 8'hFF;

// per-field assigns: Quartus 17 rejects assignment patterns with non-constant members (error 10734)
assign cpu_i.cen   = cen_x;
assign cpu_i.irq   = irq;
assign cpu_i.nmi   = nmi_cnt != 6'd0;
assign cpu_i.xdin  = (rom_area && rom_in) ? rom_q : 8'hFF;
assign cpu_i.p1_in = p1_in;
assign cpu_i.p2_in = 5'd0;
assign cpu_i.mode  = 3'd0;

/////////////////////////////  MSM5205 control (3J port B)  /////////////////////////////

wire [7:0] iob = iob_out_en[0] ? ay_o[0].iob_out : 8'hFF;   // 4.7K pull-ups while the port is an input
// MAME playmode index: 0 = /96, 1 = /48, 2 = /64, 3 = slave; jt5205 sel: 0 = /96, 1 = /64, 2 = /48, 3 = slave
function automatic [1:0] jt_sel(input [1:0] idx);
    jt_sel = {idx[0], idx[1]};
endfunction
wire [1:0] idx0 = msm_mame_map ? iob[3:2] : iob[2:1];
wire       b4_0 = msm_mame_map ? iob[4]   : iob[3];
wire       rst0 = iob[0];
wire       rst1 = msm_mame_map ? iob[1] : 1'b1;

// MSM-REVERT-2026-10-25: VCK made here for our msm5205.sv (no prescaler of its own): 384 kHz / 96, 48, 64 by the
// MAME playmode index (3 = slave: no VCK); one nibble decoded and one 6803 NMI per VCK, as MAME update_adpcm.
// jt5205 wiring was:
// assign msm_i[0].rst = rst0 | reset;
// assign msm_i[0].cen = cen_msm;
// assign msm_i[0].sel = jt_sel(idx0);
// assign msm_i[0].din = b4_0 ? msm_d0 : {msm_d0[2:0], 1'b0};
logic [6:0] vck_cnt = 7'd0;
wire  [6:0] vck_div = (idx0 == 2'd0) ? 7'd95 : (idx0 == 2'd1) ? 7'd47 : 7'd63;
always_ff @(posedge clk) begin
    msm_step <= 1'b0;
    if (cen_msm) begin
        if (idx0 == 2'd3)            vck_cnt <= 7'd0;
        else if (vck_cnt >= vck_div) begin vck_cnt <= 7'd0; msm_step <= 1'b1; end
        else                         vck_cnt <= vck_cnt + 7'd1;
    end
end
assign msm_i[0].step = msm_step;
assign msm_i[0].rst  = rst0 | reset;
assign msm_i[0].data = b4_0 ? msm_d0 : {msm_d0[2:0], 1'b0};
// TODO(M58): second MSM5205 in slave mode, clocked by the first one's VCK
// MSM-REVERT-2026-10-25: was (jt5205) rst 1, cen 0, sel 3, din msm_d1
assign msm_i[1].step = 1'b0;
assign msm_i[1].rst  = 1'b1;
assign msm_i[1].data = msm_d1;

/////////////////////////////  Mixer  /////////////////////////////

// AY output step from jt49's linear level (jt49_exp, COMP 0): step = exp-table index >> 1
function automatic [3:0] ay_step(input [7:0] lin);
    ay_step = 4'(lin >= 8'd1)  + 4'(lin >= 8'd2)  + 4'(lin >= 8'd3)  + 4'(lin >= 8'd4)   + 4'(lin >= 8'd6)   +
              4'(lin >= 8'd9)  + 4'(lin >= 8'd13) + 4'(lin >= 8'd18) + 4'(lin >= 8'd26)  + 4'(lin >= 8'd37)  +
              4'(lin >= 8'd53) + 4'(lin >= 8'd75) + 4'(lin >= 8'd107) + 4'(lin >= 8'd151) + 4'(lin >= 8'd214);
endfunction
// conductance of one output at each step, units of 1e-7 S: 1 / res[step] + (step ? 1 / r_up : 0) (MAME ay8910_param)
function automatic [12:0] ay_g(input [3:0] s);
    case (s)
        4'd0:  ay_g = 13'd627;   4'd1:  ay_g = 13'd664;   4'd2:  ay_g = 13'd676;   4'd3:  ay_g = 13'd691;
        4'd4:  ay_g = 13'd714;   4'd5:  ay_g = 13'd747;   4'd6:  ay_g = 13'd789;   4'd7:  ay_g = 13'd893;
        4'd8:  ay_g = 13'd956;   4'd9:  ay_g = 13'd1177;  4'd10: ay_g = 13'd1404;  4'd11: ay_g = 13'd1684;
        4'd12: ay_g = 13'd2088;  4'd13: ay_g = 13'd2548;  4'd14: ay_g = 13'd3328;  default: ay_g = 13'd4277;
    endcase
endfunction
localparam [16:0] AY_GL = 17'd21284;                        // 1 / 470 + 6 / r_down

// 768 kHz: node voltage num / (num + GL) by restoring division, 16-bit fraction; four averaged per 192 kHz step
logic  [5:0] sub = 6'd0;
logic  [1:0] sub4 = 2'd0;
logic [16:0] num, den, rem;
logic [15:0] quo;
logic [17:0] vacc;
logic [15:0] vay;                                           // AY node / Vcc, 16-bit fraction
logic        tick = 1'b0;
always_ff @(posedge clk) begin
    sub  <= sub + 6'd1;
    tick <= 1'b0;
    if (sub == 6'd0) begin
        num <= 17'(ay_g(ay_step(ay_o[0].a))) + 17'(ay_g(ay_step(ay_o[0].b))) + 17'(ay_g(ay_step(ay_o[0].c))) +
               17'(ay_g(ay_step(ay_o[1].a))) + 17'(ay_g(ay_step(ay_o[1].b))) + 17'(ay_g(ay_step(ay_o[1].c)));
    end else if (sub == 6'd1) begin
        den <= num + AY_GL;
        rem <= num;
        quo <= 16'd0;
    end else if (sub >= 6'd2 && sub < 6'd18) begin
        if ({rem, 1'b0} >= {1'b0, den}) begin rem <= 17'({rem, 1'b0} - {1'b0, den}); quo <= {quo[14:0], 1'b1}; end
        else                            begin rem <= 17'({rem, 1'b0});                quo <= {quo[14:0], 1'b0}; end
    end else if (sub == 6'd18) begin
        sub4 <= sub4 + 2'd1;
        if (sub4 == 2'd3) begin vay <= 16'((vacc + 18'(quo)) >> 2); vacc <= 18'd0; tick <= 1'b1; end
        else vacc <= vacc + 18'(quo);
    end
end

// 192 kHz filter chain, states Q.24 (1.0 = Vcc), coefficients Q0.18 (python, from the drawing values).
// GAIN: loudest Moon Patrol passage (rock shooting, sim) peaks ~27700 = -1.5 dBFS. 1600 (= MAME's loudness) was too
// quiet on HW: the rock "thump" got lost (user 2026-10-25); offline mixer models showed the thump content = MAME's.
localparam [17:0] K1 = 18'd1128, KDIV = 18'd45525, K2 = 18'd6206, K3 = 18'd5057, K4 = 18'd20078,
                  WAY = 18'd45626, WMSM = 18'd207392, K5 = 18'd153, GAIN = 18'd10070;
// R15-2026-10-25: R15 = 2.2K (MAME M52_R15, the 10-Yard SOUND-G drawing, MAME's Tropical Angel PCB photos), not the
// Williams Moon Patrol drawing's 22K: with 22K the rock "thonk" (MSM5205 released, ADPCM noise burst at a rock hit;
// MAME NMI-rate log 2026-10-25) was inaudible on HW. GAIN raised to keep the AY level of the 22K build. 22K values:
//                WAY = 18'd158439, WMSM = 18'd72018, K5 = 18'd153, GAIN = 18'd2900;
logic signed [31:0] x, lp1, hp, y2, u, vb, mix, lp5;
logic signed [26:0] ma;
logic        [17:0] mb;
logic signed [45:0] mp;
logic signed [31:0] mq;
logic         [5:0] st = 6'd63;
logic               primed = 1'b0;
always_ff @(posedge clk) mp <= ma * $signed({1'b0, mb});
assign mq = 32'(mp >>> 18);

function automatic signed [26:0] sat27(input signed [33:0] v);
    sat27 = (v > 34'sd67108863) ? 27'sd67108863 : (v < -34'sd67108864) ? -27'sd67108864 : 27'(v);
endfunction

always_ff @(posedge clk) begin
    if (reset) begin
        lp1 <= 32'sd0;  hp <= 32'sd0;  y2 <= 32'sd0;  u <= 32'sd0;  vb <= 32'sd0;  mix <= 32'sd0;  lp5 <= 32'sd0;
        audio <= 16'sd0;  st <= 6'd63;  primed <= 1'b0;
    end else begin
        if (tick) begin
            st <= 6'd0;
            // MSM-REVERT-2026-10-25: was  x <= 32'($signed(msm_o[0].sound)) <<< 12;   (jt5205)
            x  <= 32'($signed(msm_o[0].signal)) <<< 12;                   // +-2048 -> +-0.5 Vcc
        end else if (st != 6'd63) st <= st + 6'd1;
        case (st)
            6'd0:  begin ma <= sat27(34'(x) - 34'(lp1));                              mb <= K1;   end
            6'd2:  lp1 <= lp1 + mq;
            6'd3:  begin ma <= sat27(34'(x) - 34'(lp1));                              mb <= KDIV; end
            6'd5:  hp <= mq;
            6'd6:  begin ma <= sat27(34'(hp) - 34'(y2) - 34'(y2) + 34'(vb) + 34'(u));  mb <= K2;   end
            6'd8:  y2 <= y2 + mq;
            6'd9:  begin ma <= sat27(34'(y2) - 34'(vb) - 34'(u) - 34'(u));            mb <= K3;   end
            6'd11: u <= u + mq;
            6'd12: begin ma <= sat27(34'(u));                                         mb <= K4;   end
            6'd14: vb <= vb + mq;
            6'd15: begin ma <= sat27({2'b00, vay, 16'd0} >>> 8);                      mb <= WAY;  end
            6'd17: mix <= mq;
            6'd18: begin ma <= sat27(34'(vb));                                        mb <= WMSM; end
            6'd20: begin mix <= mix + mq; if (!primed) begin lp5 <= mix + mq; primed <= 1'b1; end end   // no start-up thump
            6'd21: begin ma <= sat27(34'(mix) - 34'(lp5));                            mb <= K5;   end
            6'd23: lp5 <= lp5 + mq;
            6'd24: begin ma <= sat27(34'(mix) - 34'(lp5));                            mb <= GAIN; end
            6'd26: audio <= (mq > 32'sd32767) ? 16'sd32767 : (mq < -32'sd32768) ? -16'sd32768 : 16'(mq);
            default: ;
        endcase
    end
end

endmodule
