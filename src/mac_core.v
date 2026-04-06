`timescale 1ns/1ps

// Outer-product W3A8 matrix multiply core: 4x4 uint8 activations (A) x 3-bit sign-magnitude weights (B).
// B encoding: bit[2]=sign, bits[1:0]=magnitude {0,1,2,3} → value {0,+1,+2,+3} or negated.
//
// LANES-wide outer-product engine using per-lane generate blocks (assumes LANES <= N).
// For N=4 and LANES=4:
//   - One shared a_spm[grp][ck] read, broadcast to all lanes (was 4 identical mux chains).
//   - Each lane reads b_spm[ck][l] with compile-time column l (no col_i runtime mux).
//   - Accumulators split into per-lane columns: acc_col[l][row], each lane independent.
//   - Each acc register has its own always block (16 independent write enables on grp).
//   - Computation takes N * (N*N/LANES) = 16 cycles.

module mac_core #(
    parameter N  = 4,
    parameter DW = 8,
    parameter CW = 13,
    parameter LANES = 8
) (
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   start,
    output reg                    busy,
    output reg                    done,

    // Scratchpad load interface
    input  wire                   load_en,
    input  wire                   load_sel,  // 0: A, 1: B
    input  wire [$clog2(N)-1:0]   load_row,
    input  wire [$clog2(N)-1:0]   load_col,
    input  wire [DW-1:0]          load_data,

    // Result read interface (valid after done)
    input  wire                   c_rd_en,
    input  wire [$clog2(N)-1:0]   c_rd_row,
    input  wire [$clog2(N)-1:0]   c_rd_col,
    output reg  [CW-1:0]          c_rd_data
);

    localparam ROW_W = $clog2(N);
    localparam OUTS  = N * N;
    localparam GRPS  = (OUTS + LANES - 1) / LANES;
    localparam GRP_W = (GRPS <= 1) ? 1 : $clog2(GRPS);

    localparam ST_IDLE    = 1'd0;
    localparam ST_COMPUTE = 1'd1;

    reg state;

    // Input scratchpads
    reg [DW-1:0] a_spm [0:N-1][0:N-1];
    reg [2:0]    b_spm [0:N-1][0:N-1];

    // Per-lane accumulator columns: acc_col[lane][row] = C[row][lane].
    // Splitting by column keeps each lane's state physically independent.
    reg signed [CW-1:0] acc_col [0:LANES-1][0:N-1];

    // Compute counters
    reg [ROW_W-1:0] ck;
    reg [GRP_W-1:0] grp;

    // Shared A read: all lanes use the same A element each cycle.
    // One 16:1 mux instance shared across all LANES (was one copy per lane).
    wire [DW-1:0] a_val = a_spm[grp][ck];

    // Result read: column = lane index, row selected by c_rd_row.
    always @(*) begin
        c_rd_data = {CW{1'b0}};
        if (c_rd_en)
            c_rd_data = acc_col[c_rd_col][c_rd_row];
    end

    // Per-lane generate: each lane handles one fixed column of C.
    // Per-row inner generate: each acc register gets its own always block
    // with a static write-enable (grp == r), avoiding a shared write mux.
    genvar l, r;
    generate
        for (l = 0; l < LANES; l = l + 1) begin : lane_g

            // B weight for this lane's column — static index l, no col_i mux.
            wire [2:0] bw = b_spm[ck][l];

            // Shift-add: A scaled by B magnitude. No multiplier needed.
            reg [DW+1:0] a_sc;
            always @(*) begin
                case (bw[1:0])
                    2'b01: a_sc = {2'b0,  a_val};
                    2'b10: a_sc = {1'b0,  a_val, 1'b0};
                    2'b11: a_sc = {1'b0,  a_val, 1'b0} + {2'b0, a_val};
                    default: a_sc = {(DW+2){1'b0}};
                endcase
            end

            // Current accumulator row for this lane (4:1 mux on grp).
            wire signed [CW-1:0] acc_cur = acc_col[l][grp];

            // MAC: accumulate signed scaled A into running sum.
            wire signed [CW-1:0] pe_next = acc_cur + (bw[2]
                ? -$signed({{(CW-DW-2){1'b0}}, a_sc})
                :  $signed({{(CW-DW-2){1'b0}}, a_sc}));

            // Per-row sequential update: one always block per accumulator register.
            // Write enable is (state==ST_COMPUTE && grp==r) — a static equality per r.
            for (r = 0; r < N; r = r + 1) begin : row_g
                always @(posedge clk or posedge rst) begin
                    if (rst) begin
                        acc_col[l][r] <= {CW{1'b0}};
                    end else if (start && (state == ST_IDLE)) begin
                        acc_col[l][r] <= {CW{1'b0}};
                    end else if ((state == ST_COMPUTE) && (grp == r)) begin
                        acc_col[l][r] <= pe_next;
                    end
                end
            end

        end
    endgenerate

    // Main FSM: state, counters, scratchpad loads.
    // Accumulator writes handled entirely by the generate blocks above.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= ST_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            ck    <= {ROW_W{1'b0}};
            grp   <= {GRP_W{1'b0}};
        end else begin
            done <= 1'b0;

            if (load_en && !busy) begin
                if (load_sel)
                    b_spm[load_row][load_col] <= load_data[2:0];
                else
                    a_spm[load_row][load_col] <= load_data;
            end

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        busy  <= 1'b1;
                        ck    <= {ROW_W{1'b0}};
                        grp   <= {GRP_W{1'b0}};
                        state <= ST_COMPUTE;
                    end
                end

                ST_COMPUTE: begin
                    if (grp == GRPS[GRP_W-1:0] - 1'b1) begin
                        grp <= {GRP_W{1'b0}};
                        if (ck == N[ROW_W-1:0] - 1'b1) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            ck <= ck + 1'b1;
                        end
                    end else begin
                        grp <= grp + 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
