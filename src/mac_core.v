`timescale 1ns/1ps

// Outer-product W3A8 matrix multiply core: 4x4 uint8 activations (A) x 3-bit sign-magnitude weights (B).
// B encoding: bit[2]=sign, bits[1:0]=magnitude {0,1,2,3} → value {0,+1,+2,+3} or negated.
//
// LANES-wide outer-product engine:
// - Each k-step is processed in groups of LANES outputs.
// - For N=4 and LANES=4, computation takes N * (16/4) + 1 = 17 cycles (pipeline flush).
// acc[] doubles as c_spm after done.

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

    // Result read interface (valid after done; reads acc[] directly)
    input  wire                   c_rd_en,
    input  wire [$clog2(N)-1:0]   c_rd_row,
    input  wire [$clog2(N)-1:0]   c_rd_col,
    output reg  [CW-1:0]          c_rd_data
);

    localparam ROW_W = $clog2(N);
    localparam OUTS  = N * N;
    localparam OUT_W = (OUTS <= 1) ? 1 : $clog2(OUTS);
    localparam GRPS  = (OUTS + LANES - 1) / LANES;
    localparam GRP_W = (GRPS <= 1) ? 1 : $clog2(GRPS);
    localparam FULL_GROUPS = ((OUTS % LANES) == 0);

    localparam ST_IDLE    = 2'd0;
    localparam ST_COMPUTE = 2'd1;
    localparam ST_FLUSH   = 2'd2;

    reg [1:0] state;

    // Pipeline stage registers: decouple pe_next compute from acc write.
    // Stage 1 (ST_COMPUTE): latch pe_next into pipe_pe.
    // Stage 2 (next cycle):  write pipe_pe to acc.
    // This breaks the large a_spm/b_spm/acc combinational cone into two
    // smaller cones, reducing local routing density in the MAC array.
    reg signed [CW-1:0] pipe_pe    [0:LANES-1];
    reg [OUT_W-1:0]     pipe_idx   [0:LANES-1];
    reg                 pipe_valid [0:LANES-1];
    reg                 pipe_en;

    // Input scratchpads
    reg [DW-1:0] a_spm [0:N-1][0:N-1];
    reg [2:0]    b_spm [0:N-1][0:N-1];

    // Output accumulators — also serve as c_spm after done.
    // acc[row*N + col] holds C[row][col].
    reg signed [CW-1:0] acc [0:OUTS-1];

    // Compute counters
    reg [ROW_W-1:0] ck;
    reg [GRP_W-1:0] grp;

    // Combinational result read
    always @(*) begin
        c_rd_data = {CW{1'b0}};
        if (c_rd_en) begin
            if (N == 4)
                c_rd_data = acc[{c_rd_row, c_rd_col}];
            else
                c_rd_data = acc[c_rd_row * N + c_rd_col];
        end
    end

    // LANES-wide outer-product datapath (combinational stage 1 only; acc write in stage 2):
    // for lane l, global output index p = grp*LANES + l.
    integer l_comb;
    reg [OUT_W-1:0] idx_i;
    reg [ROW_W-1:0] row_i;
    reg [ROW_W-1:0] col_i;
    reg                 lane_valid [0:LANES-1];
    reg [OUT_W-1:0]     lane_idx   [0:LANES-1];
    reg [2:0]           b_val      [0:LANES-1];
    reg [DW+1:0]        a_scaled   [0:LANES-1];
    reg signed [CW-1:0] pe_next    [0:LANES-1];

    always @(*) begin
        for (l_comb = 0; l_comb < LANES; l_comb = l_comb + 1) begin
            // Fast paths for known configurations (N=4): avoids divide/multiply in synthesis.
            if (N == 4 && LANES == 8) begin
                idx_i = {grp, l_comb[2:0]};
                row_i = idx_i[3:2];
                col_i = idx_i[1:0];
            end else if (N == 4 && LANES == 4) begin
                idx_i = {grp[1:0], l_comb[1:0]};
                row_i = idx_i[3:2];
                col_i = idx_i[1:0];
            end else begin
                idx_i = grp * LANES + l_comb;
                row_i = idx_i / N;
                col_i = idx_i - (row_i * N);
            end

            if (FULL_GROUPS)
                lane_valid[l_comb] = 1'b1;
            else
                lane_valid[l_comb] = (idx_i < OUTS);
            lane_idx[l_comb]   = idx_i;
            b_val[l_comb]      = 3'b000;
            a_scaled[l_comb]   = {(DW+2){1'b0}};
            pe_next[l_comb]    = {CW{1'b0}};

            if (lane_valid[l_comb]) begin
                b_val[l_comb] = b_spm[ck][col_i];

                // Shift-add: magnitude 0→0, 1→A, 2→A<<1, 3→(A<<1)+A
                case (b_val[l_comb][1:0])
                    2'b01: a_scaled[l_comb] = {2'b0,  a_spm[row_i][ck]};
                    2'b10: a_scaled[l_comb] = {1'b0,  a_spm[row_i][ck], 1'b0};
                    2'b11: a_scaled[l_comb] = {1'b0,  a_spm[row_i][ck], 1'b0}
                                            + {2'b0,  a_spm[row_i][ck]};
                    default: a_scaled[l_comb] = {(DW+2){1'b0}};
                endcase

                pe_next[l_comb] = acc[lane_idx[l_comb]] + (b_val[l_comb][2]
                    ? -$signed({{(CW-DW-2){1'b0}}, a_scaled[l_comb]})
                    :  $signed({{(CW-DW-2){1'b0}}, a_scaled[l_comb]}));
            end
        end
    end

    integer p_seq;
    integer l_seq;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state   <= ST_IDLE;
            busy    <= 1'b0;
            done    <= 1'b0;
            pipe_en <= 1'b0;
            ck      <= {ROW_W{1'b0}};
            grp     <= {GRP_W{1'b0}};
        end else begin
            done <= 1'b0;

            if (load_en && !busy) begin
                if (load_sel)
                    b_spm[load_row][load_col] <= load_data[2:0];
                else
                    a_spm[load_row][load_col] <= load_data;
            end

            // Pipeline stage 2: write latched results to acc.
            // pipe_en is 0 during ST_IDLE, so this never conflicts with acc clear.
            if (pipe_en) begin
                for (l_seq = 0; l_seq < LANES; l_seq = l_seq + 1) begin
                    if (pipe_valid[l_seq])
                        acc[pipe_idx[l_seq]] <= pipe_pe[l_seq];
                end
            end

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        busy    <= 1'b1;
                        pipe_en <= 1'b0;
                        ck      <= {ROW_W{1'b0}};
                        grp     <= {GRP_W{1'b0}};
                        for (p_seq = 0; p_seq < OUTS; p_seq = p_seq + 1)
                            acc[p_seq] <= {CW{1'b0}};
                        state <= ST_COMPUTE;
                    end
                end

                ST_COMPUTE: begin
                    // Pipeline stage 1: latch pe_next (combinational) into pipe regs.
                    // acc write happens the following cycle via the stage-2 block above.
                    pipe_en <= 1'b1;
                    for (l_seq = 0; l_seq < LANES; l_seq = l_seq + 1) begin
                        pipe_pe[l_seq]    <= pe_next[l_seq];
                        pipe_idx[l_seq]   <= lane_idx[l_seq];
                        pipe_valid[l_seq] <= lane_valid[l_seq];
                    end

                    if (grp == GRPS[GRP_W-1:0] - 1'b1) begin
                        grp <= {GRP_W{1'b0}};
                        if (ck == N[ROW_W-1:0] - 1'b1) begin
                            state <= ST_FLUSH;  // one extra cycle to drain pipeline
                        end else begin
                            ck <= ck + 1'b1;
                        end
                    end else begin
                        grp <= grp + 1'b1;
                    end
                end

                ST_FLUSH: begin
                    // Stage-2 block above writes the last batch to acc this cycle.
                    pipe_en <= 1'b0;
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    state   <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
