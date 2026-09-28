/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RetroAchievements RAM mirror for the Williams (Robotron) MiSTer core.
 *
 * Copied from jtframe_ra_mirror.v (RetroAchievements fork of jotego's jtcores,
 * modules/jtframe/target/mister/hdl/jtframe_ra_mirror.v,
 * https://github.com/pepasjc/jtcores branch ra-mirror; JTFRAME by Jose Tejada
 * Gomez, https://github.com/jotego/jtcores, GPL-3.0-or-later), by way of the
 * Irem M72, Pac-Man and Galaxian ports. Changes for this core: renamed to
 * williams_ra_mirror; the shadow lanes are inferred here instead of using
 * jtframe_dual_ram and are dual-clock (written from the core clock wr_clk,
 * read by the DDR client clock clk); the copy waits for a start window
 * (start_ok) so it never collides with another DDR client.
 * The shadow is written with NIBBLE enables (sixteen 4-bit lanes), because
 * the Williams blitter writes single pixels (nibbles) into the video RAM that
 * FinalBurn Neo exposes.
 *
 * Keeps a shadow copy of up to 64 kB of RAM, fed by a generic write port
 * (williams_ra_tap.sv), and copies it to DDR3 every VBlank so the ARM side (the
 * RetroAchievements fork of Main_MiSTer) can evaluate achievements.
 *
 * DDR layout at byte address 0x3D000000 (the "RACH" Full Mirror header used
 * by the RA fork, see Main_MiSTer ra_ramread.h):
 *   0x00  magic "RACH" (0x52414348 LE), region count 0, flags (bit0 busy),
 *         core version
 *   0x08  frame counter (u32)
 *   0x10  zero (keeps stale mailbox/protocol bytes of other cores clear)
 *   0x100 the shadow RAM. Each 16-bit word is stored as-is in a little-endian
 *         DDR lane: shadow byte 2w = wr_din[7:0], byte 2w+1 = wr_din[15:8].
 *         The 6809 is 8-bit, so the tap writes single bytes and shadow byte k
 *         is the FinalBurn Neo "All Ram" byte k (no swapping).
 *
 * Copy order: header with busy=1, data, frame counter, header with busy=0.
 * The ARM only accepts a snapshot when busy is clear and the frame counter
 * did not move during its copy.
 */

module williams_ra_mirror #(parameter
    AW        = 16,            // shadow size as a byte address width: 16 = 64 kB
    DDR_BASE  = 29'h07A0_0000, // 0x3D000000 / 8
    VERSION   = 16'h0100
)(
    input               rst,
    input               clk,        // DDR client clock (CLK_VIDEO here)
    input               lvbl,       // active-low vertical blank, clk domain
    input               hold,       // ROM download in progress: skip the copy
    input               start_ok,   // no other DDR client is writing (clk domain)
    // Shadow write port, wr_clk domain: 16-bit word in the 64 kB window and
    // nibble enables (wr_ne[n] writes bits 4n+3:4n; bits 7:0 = even byte)
    input               wr_clk,
    input        [14:0] wr_word,
    input        [15:0] wr_din,
    input        [ 3:0] wr_ne,
    // DDR client
    output reg          active,     // owns the DDR client port
    input               ddr_busy,
    output reg   [ 7:0] ddr_burstcnt,
    output reg   [28:0] ddr_addr,
    output reg          ddr_we,
    output       [ 7:0] ddr_be,
    output reg   [63:0] ddr_din
);

localparam [31:0] MAGIC  = 32'h5241_4348;
localparam  [7:0] BURST  = 8'd32;     // 256 B: aligned, never crosses 4 kB
localparam        QW     = AW-3;     // qword address width
localparam        QWORDS = 1<<QW;

localparam [2:0] IDLE=0, HDR_BUSY=1, PRE=2, DATA=3, FRAME=4, HDR_DONE=5, ZERO=6;

// ---------------------------------------------------------------------------
// Shadow RAM: sixteen nibble lanes, so a whole DDR qword reads at once.
// Written on wr_clk, read on clk (dual-clock simple dual-port M10K)
wire [QW-1:0] wr_q = wr_word[QW+1:2];
wire [ 1:0] lane = wr_word[1:0];
// writes past a reduced shadow are dropped, not wrapped around
wire        in_win = AW == 16 || (wr_word >> (AW-1)) == 0;

reg  [ 2:0] st;
reg  [QW-1:0] ptr;
wire        accept = ddr_we && !ddr_busy && st == DATA;
wire [QW-1:0] rd_q = accept ? ptr + 1'd1 : ptr;
wire [63:0] q;

genvar b;
generate
    for( b=0; b<16; b=b+1 ) begin : lanes
        // qword nibble b = nibble b%4 of 16-bit word lane b/4
        wire       we = in_win && lane == b/4 && wr_ne[b%4];
        reg  [3:0] mem[0:QWORDS-1];
        reg  [3:0] rd;
        always @(posedge wr_clk) begin
            if( we ) mem[wr_q] <= wr_din[(b%4)*4+:4];
        end
        always @(posedge clk) rd <= mem[rd_q];
        assign q[b*4+:4] = rd;
    end
endgenerate

// ---------------------------------------------------------------------------
// Copier
reg         lvbl_l;
reg         pending;    // a VBlank started and the copy has not begun yet
reg  [31:0] frame;
reg  [ 4:0] beat;
reg         zeroed;

assign ddr_be = 8'hff;

always @(*) begin
    ddr_din = q;
    case( st )
        HDR_BUSY: ddr_din = { VERSION, 8'h01, 8'd0, MAGIC };
        HDR_DONE: ddr_din = { VERSION, 8'h00, 8'd0, MAGIC };
        FRAME:    ddr_din = { 32'd0, frame };
        ZERO:     ddr_din = 64'd0;
        default:;
    endcase
end

always @(posedge clk, posedge rst) begin
    if( rst ) begin
        st           <= IDLE;
        active       <= 0;
        ddr_we       <= 0;
        ddr_burstcnt <= 8'd1;
        ddr_addr     <= DDR_BASE;
        ptr          <= 0;
        beat         <= 0;
        frame        <= 0;
        lvbl_l       <= 1;
        pending      <= 0;
        zeroed       <= 0;
    end else begin
        lvbl_l <= lvbl;
        if( lvbl_l && !lvbl ) pending <= !hold;
        if( hold || lvbl ) pending <= 0;   // only copy inside VBlank
        case( st )
            IDLE: begin
                ddr_we <= 0;
                active <= 0;
                if( pending && start_ok && !hold ) begin
                    pending      <= 0;
                    active       <= 1;
                    ddr_addr     <= DDR_BASE;
                    ddr_burstcnt <= 8'd1;
                    ddr_we       <= 1;
                    st           <= HDR_BUSY;
                end
            end
            HDR_BUSY: if( !ddr_busy ) begin
                ddr_we <= 0;
                ptr    <= 0;
                st     <= PRE;
            end
            PRE: begin // ptr settles into the read port
                ddr_addr     <= DDR_BASE + 29'h20;
                ddr_burstcnt <= BURST;
                beat         <= 0;
                ddr_we       <= 1;
                st           <= DATA;
            end
            DATA: if( accept ) begin
                ptr  <= ptr + 1'd1;
                beat <= beat + 5'd1;
                if( &beat ) begin
                    ddr_addr <= ddr_addr + { 21'd0, BURST };
                    if( &ptr ) begin
                        ddr_addr     <= DDR_BASE + 29'h1;
                        ddr_burstcnt <= 8'd1;
                        frame        <= frame + 32'd1;
                        st           <= FRAME;
                    end
                end
            end
            FRAME: if( !ddr_busy ) begin
                ddr_addr <= zeroed ? DDR_BASE : DDR_BASE + 29'h2;
                st       <= zeroed ? HDR_DONE : ZERO;
            end
            ZERO: if( !ddr_busy ) begin
                zeroed   <= 1;
                ddr_addr <= DDR_BASE;
                st       <= HDR_DONE;
            end
            HDR_DONE: if( !ddr_busy ) begin
                ddr_we <= 0;
                active <= 0;
                st     <= IDLE;
            end
            default: st <= IDLE;
        endcase
    end
end

endmodule
