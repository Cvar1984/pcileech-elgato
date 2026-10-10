//
// PCILeech FPGA - MZ0380 no-signal splash DMA engine.
//
// On mailbox START (mz_arm rising) streams the card's "NO SIGNAL" splash
// burst (0x30a5c0 bytes of payload, geometry per tools/gen_logo_mem.py,
// M50/M127b) into video slot 0 as MemWr TLPs, then pulses mz_frame_done,
// sets the frame token, requests INTx, and parks until the next START
// (M38/M39: exactly one frame per stream-on).
//
// TLP TX conventions (reference: pcileech_tlps128_bar_rdengine/wrengine
// in pcileech_tlps128_bar_controller.sv):
//   - tdata dwords are packed LSB-dword-first; WITHIN a dword the byte
//     order on the wire is tdata[7:0] first (little-endian beats).
//   - Header dword values are written into tdata naturally:
//     tdata[31:0] = DW0, [63:32] = DW1, [95:64] = DW2, [127:96] = DW3
//     (4DW header: DW3 = address hi). Data dwords follow from beat 1.
//     The rdengine packs pcie_id as {pcie_id[7:0], pcie_id[15:8]} - the
//     16-bit ID occupies [15:0] of the dword in natural bit order.
//   - has_data: registered packet count, the pkt_count pattern of
//     pcileech_tlps128_src_fifo (1-cycle source latency assumed by
//     pcileech_tlps128_sink_mux1).
//
// Beat layout per MemWr TLP (Max_Payload_Size = 512B, 16B per 128b beat):
//   beat 0            : DW0, DW1, DW2, DW3                (tkeepdw 1111)
//   beats 1..N-2      : 16 data bytes each                (tkeepdw 1111)
//   beat N-1 (partial): last 12 data bytes                (tkeepdw 0111)
//   full TLP: N=33 beats, 32*16 - 4 + ... = 512 data bytes exactly
//             (31 full data beats + one 12-byte tail), Length=128 dwords
//   tail TLP: N=29 beats, 27 full data beats + 12-byte tail = 448 bytes,
//             Length=112 dwords
//   6226 full + 1 tail = 6227 TLPs = 0x30a5c0 bytes exactly.
//
// One frame per START; STOP mid-send aborts at the next TLP boundary.
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"
import mz0380_pkg::*;

module pcileech_dma_engine (
    input               rst,
    input               clk,            // clk_pcie domain
    input  [15:0]       pcie_id,
    IfAXIS128.source    tlps_out,       // 5th input of pcileech_tlps128_sink_mux1
    // buffer table from the mailbox (via bar_controller)
    input  [255:0]      mz_vid_base,    // 4 x {hi,lo} phys, video window (op 0x02)
    input  [2:0]        mz_vid_slots,
    input               mz_arm,         // START level
    input               mz_int_ack,     // BAR0 doorbell 0x400 pulse
    // completion to mailbox
    output bit          mz_frame_done,  // 1-cycle pulse
    output bit [31:0]   mz_frame_token,
    // interrupt request to cfg block (via pcie_a7)
    output bit          mz_int_req      // level; cleared by mz_int_ack
);

    // ------------------------------------------------------------------
    // burst geometry
    // ------------------------------------------------------------------
    localparam int unsigned TLP_FULL   = 6226;                  // full 512B-payload TLPs
    localparam bit [9:0]    DWLEN_FULL = 10'd128;               // 512B data
    localparam bit [9:0]    DWLEN_TAIL = 10'd112;               // 448B data
    localparam bit [5:0]    BEATS_FULL = 6'd33;                 // 1 hdr + 31 + partial
    localparam bit [5:0]    BEATS_TAIL = 6'd29;                 // 1 hdr + 27 + partial

    // ------------------------------------------------------------------
    // splash byte generator (matches tools/gen_logo_mem.py exactly)
    // ------------------------------------------------------------------
    (* rom_style = "block" *)
    reg [31:0] tile [0:19199];
    initial $readmemh("mz0380_logo.mem", tile);

    function automatic bit [7:0] splash_byte( input bit [21:0] off );
        bit [21:0] r, c;
        bit [16:0] toff;
        begin
            if ( off >= MZ_FRAME_SIZE )
                splash_byte = 8'hff;                             // junk fill
            else if ( off >= MZ_UV_OFFSET )
                splash_byte = 8'h80;                             // neutral chroma
            else begin
                r = off / MZ_STRIDE;
                c = off % MZ_STRIDE;
                if ( (r >= MZ_TILE_Y) && (r < (MZ_TILE_Y + MZ_TILE_H)) &&
                     (c >= MZ_TILE_X) && (c < (MZ_TILE_X + MZ_TILE_W)) ) begin
                    toff = (17'((r - MZ_TILE_Y) * 22'(MZ_TILE_W) + (c - MZ_TILE_X)));
                    case ( toff[1:0] )                           // .mem words: LE bytes
                        2'd0:    splash_byte = tile[toff[16:2]][7:0];
                        2'd1:    splash_byte = tile[toff[16:2]][15:8];
                        2'd2:    splash_byte = tile[toff[16:2]][23:16];
                        default: splash_byte = tile[toff[16:2]][31:24];
                    endcase
                end
                else
                    splash_byte = 8'h11;                         // background
            end
        end
    endfunction

    // ------------------------------------------------------------------
    // FSM + datapath state
    // ------------------------------------------------------------------
    localparam bit [1:0] S_IDLE = 2'd0, S_SEND = 2'd1, S_DONE = 2'd2;

    bit [1:0]   state;
    bit [12:0]  tlp_cnt;                                     // 0..6226
    bit [5:0]   beat_cnt;                                    // beats emitted in TLP
    bit [31:0]  addr_lo;                                     // TLP byte addr, low 32
    bit [21:0]  byte_idx;                                    // burst offset, next 16B group
    bit [31:0]  frame_cnt;

    // output registers (engine -> mux)
    bit         first_q;
    bit         tlast_q;
    bit [3:0]   tkeepdw_q;
    bit [127:0] tdata_q;
    bit         beat_presented;                              // a beat sits in the regs
    bit         beat_consumed;                               // mux took it this clk

    wire        arm_rise  = (state == S_IDLE) && mz_arm && (mz_vid_slots != 3'd0);
    wire        abort     = (state == S_SEND) && !mz_arm;    // STOP mid-send
    wire        last_tlp  = (tlp_cnt == TLP_FULL);
    wire [9:0]  cur_dwlen = last_tlp ? DWLEN_TAIL : DWLEN_FULL;
    wire [5:0]  cur_beats = last_tlp ? BEATS_TAIL : BEATS_FULL;

    // header dwords (natural bit order; see conventions above)
    // DW0: Fmt=011 (4DW hdr+data), Type=00000, TC=0, LD/TH=0, Attr=00, EP/TD=0,
    //      rsvd[13:12]=00, Length[9:0]=cur_dwlen
    wire [31:0] dw0 = { 3'b011, 5'b00000, 3'b000, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0, 1'b0, 2'b00, 2'b00, cur_dwlen };
    wire [31:0] dw1 = { pcie_id, 8'h00, 4'hf, 4'hf };        // ReqID, Tag, BE all-f
    wire [31:0] dw2 = { addr_lo[31:2], 2'b00 };
    wire [31:0] dw3 = mz_vid_base[95:64];                    // slot0 hi

    // ------------------------------------------------------------------
    // output + packet count (pcileech_tlps128_src_fifo shape)
    // ------------------------------------------------------------------
    assign tlps_out.tdata      = tdata_q;
    assign tlps_out.tkeepdw    = tkeepdw_q;
    assign tlps_out.tlast      = tlast_q;
    assign tlps_out.tuser[0]   = first_q;
    assign tlps_out.tuser[8:1] = 8'h0;

    bit [10:0]  pkt_count      = 0;
    wire        beat_ready     = tlps_out.tready;
    wire        pkt_count_dec  = tlps_out.tvalid && tlps_out.tlast;
    wire        pkt_count_inc  = beat_consumed && tlast_q;
    wire [10:0] pkt_count_next = pkt_count + {10'b0, pkt_count_inc} - {10'b0, pkt_count_dec};
    assign tlps_out.has_data   = (pkt_count_next > 0);
    assign tlps_out.tvalid     = beat_presented;

    // ------------------------------------------------------------------
    // main sequential logic
    // ------------------------------------------------------------------
    always @ ( posedge clk ) begin
        if ( rst ) begin
            state          <= S_IDLE;
            tlp_cnt        <= 13'd0;
            beat_cnt       <= 6'd0;
            addr_lo        <= 32'h0;
            byte_idx       <= 22'd0;
            frame_cnt      <= 32'h0;
            first_q        <= 1'b0;
            tlast_q        <= 1'b0;
            tkeepdw_q      <= 4'b0000;
            tdata_q        <= 128'h0;
            beat_presented <= 1'b0;
            beat_consumed  <= 1'b0;
            mz_frame_done  <= 1'b0;
            mz_frame_token <= 32'h0;
            mz_int_req     <= 1'b0;
        end
        else begin
            mz_frame_done <= 1'b0;                               // default pulse low

            // interrupt handshake: level until host acks via 0x400 doorbell
            if ( mz_int_ack )
                mz_int_req <= 1'b0;

            // ---- beat consume: mux took the presented beat this clk ----
            beat_consumed <= beat_presented && beat_ready;

            // ---- FSM ----
            case ( state )
                S_IDLE: begin
                    tlp_cnt  <= 13'd0;
                    beat_cnt <= 6'd0;
                    byte_idx <= 22'd0;
                    addr_lo  <= mz_vid_base[31:0];               // slot0 lo
                    if ( arm_rise )
                        state <= S_SEND;
                end

                S_SEND: begin
                    if ( abort ) begin
                        state <= S_IDLE;                         // no completion pulse
                    end
                    else if ( beat_consumed && tlast_q ) begin
                        if ( last_tlp ) begin
                            state          <= S_DONE;
                            mz_frame_done  <= 1'b1;
                            frame_cnt      <= frame_cnt + 32'd1;
                            mz_frame_token <= { frame_cnt[28:0], 3'b000 };  // buf idx 0 (M17)
                            mz_int_req     <= 1'b1;
                        end
                        else begin
                            tlp_cnt  <= tlp_cnt + 13'd1;
                            beat_cnt <= 6'd0;
                            // address advance: 512B per TLP, low-half wrap (M25/M26)
                            addr_lo  <= addr_lo + 32'h200;
                        end
                    end
                end

                S_DONE: begin
                    if ( !mz_arm )                               // STOP rearms
                        state <= S_IDLE;
                end
                default: begin
                    state <= S_IDLE;
                end
            endcase

            // ---- beat present ----
            // One beat outstanding at a time: when the slot is free and the
            // mux can take the next beat immediately, load it.
            //   beat 0          : header (DW0..DW3), tkeepdw 1111
            //   beats 1..N-2    : 16 data bytes,         tkeepdw 1111
            //   beat N-1        : 12 data bytes,         tkeepdw 0111
            if ( (state == S_SEND) && !abort && !beat_presented && beat_ready ) begin
                if ( beat_cnt == 6'd0 ) begin
                    // header beat
                    tdata_q        <= { dw3, dw2, dw1, dw0 };
                    tkeepdw_q      <= 4'b1111;
                    first_q        <= 1'b1;
                    tlast_q        <= 1'b0;
                    beat_presented <= 1'b1;
                    beat_cnt       <= 6'd1;
                end
                else if ( beat_cnt < (cur_beats - 6'd1) ) begin
                    // full data beat: 16 splash bytes
                    tdata_q        <= { splash_byte(byte_idx + 22'd15),
                                        splash_byte(byte_idx + 22'd14),
                                        splash_byte(byte_idx + 22'd13),
                                        splash_byte(byte_idx + 22'd12),
                                        splash_byte(byte_idx + 22'd11),
                                        splash_byte(byte_idx + 22'd10),
                                        splash_byte(byte_idx + 22'd9),
                                        splash_byte(byte_idx + 22'd8),
                                        splash_byte(byte_idx + 22'd7),
                                        splash_byte(byte_idx + 22'd6),
                                        splash_byte(byte_idx + 22'd5),
                                        splash_byte(byte_idx + 22'd4),
                                        splash_byte(byte_idx + 22'd3),
                                        splash_byte(byte_idx + 22'd2),
                                        splash_byte(byte_idx + 22'd1),
                                        splash_byte(byte_idx + 22'd0) };
                    tkeepdw_q      <= 4'b1111;
                    first_q        <= 1'b0;
                    tlast_q        <= 1'b0;
                    beat_presented <= 1'b1;
                    beat_cnt       <= beat_cnt + 6'd1;
                    byte_idx       <= byte_idx + 22'd16;
                end
                else begin
                    // partial data beat: last 12 bytes (tkeepdw 0111)
                    tdata_q        <= { 32'h0,
                                        splash_byte(byte_idx + 22'd11),
                                        splash_byte(byte_idx + 22'd10),
                                        splash_byte(byte_idx + 22'd9),
                                        splash_byte(byte_idx + 22'd8),
                                        splash_byte(byte_idx + 22'd7),
                                        splash_byte(byte_idx + 22'd6),
                                        splash_byte(byte_idx + 22'd5),
                                        splash_byte(byte_idx + 22'd4),
                                        splash_byte(byte_idx + 22'd3),
                                        splash_byte(byte_idx + 22'd2),
                                        splash_byte(byte_idx + 22'd1),
                                        splash_byte(byte_idx + 22'd0) };
                    tkeepdw_q      <= 4'b0111;
                    first_q        <= 1'b0;
                    tlast_q        <= 1'b1;
                    beat_presented <= 1'b1;
                    byte_idx       <= byte_idx + 22'd12;
                end
            end

            // free the slot the cycle after the mux consumed the beat
            if ( beat_consumed )
                beat_presented <= 1'b0;
        end
    end

endmodule
