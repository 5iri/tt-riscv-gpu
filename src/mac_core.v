`timescale 1ns/1ps

// Outer-product W3A8 matrix multiply core: 4x4 uint8 activations (A) x 3-bit sign-magnitude weights (B).
// B encoding: bit[2]=sign, bits[1:0]=magnitude {0,1,2,3} → value {0,+1,+2,+3} or negated.
//
// All N² output accumulators compute in parallel (outer product per k-step).
// Computation takes exactly N clock cycles. acc[] doubles as c_spm after done.

module mac_core #(
    parameter N  = 4,
    parameter DW = 8,
    parameter CW = 13
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

    localparam ST_IDLE    = 1'd0;
    localparam ST_COMPUTE = 1'd1;

    reg state;

    // Input scratchpads
    reg [DW-1:0] a_spm [0:N-1][0:N-1];
    reg [2:0]    b_spm [0:N-1][0:N-1];

    // Output accumulators — also serve as c_spm after done.
    // acc[row*N + col] holds C[row][col].
    reg signed [CW-1:0] acc [0:OUTS-1];

    // k-step counter (only counter needed)
    reg [ROW_W-1:0] ck;

    // Combinational result read
    always @(*) begin
        c_rd_data = {CW{1'b0}};
        if (c_rd_en)
            c_rd_data = acc[{c_rd_row, c_rd_col}];
    end

    // Outer-product PE array: for each output p, row=p/N, col=p%N.
    // At each k-step: pe_next[p] = acc[p] + A[row][k] * B[k][col]
    integer p_comb;
    reg [DW+1:0]        a_scaled [0:OUTS-1];
    reg [2:0]           b_val    [0:OUTS-1];
    reg signed [CW-1:0] pe_next  [0:OUTS-1];

    always @(*) begin
        for (p_comb = 0; p_comb < OUTS; p_comb = p_comb + 1) begin
            b_val[p_comb] = b_spm[ck][p_comb % N];

            // Shift-add: magnitude 0→0, 1→A, 2→A<<1, 3→(A<<1)+A
            case (b_val[p_comb][1:0])
                2'b01: a_scaled[p_comb] = {2'b0,  a_spm[p_comb / N][ck]};
                2'b10: a_scaled[p_comb] = {1'b0,  a_spm[p_comb / N][ck], 1'b0};
                2'b11: a_scaled[p_comb] = {1'b0,  a_spm[p_comb / N][ck], 1'b0}
                                        + {2'b0,  a_spm[p_comb / N][ck]};
                default: a_scaled[p_comb] = {(DW+2){1'b0}};
            endcase

            pe_next[p_comb] = acc[p_comb] + (b_val[p_comb][2]
                ? -$signed({{(CW-DW-2){1'b0}}, a_scaled[p_comb]})
                :  $signed({{(CW-DW-2){1'b0}}, a_scaled[p_comb]}));
        end
    end

    integer p_seq;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= ST_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
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
                        for (p_seq = 0; p_seq < OUTS; p_seq = p_seq + 1)
                            acc[p_seq] <= {CW{1'b0}};
                        state <= ST_COMPUTE;
                    end
                end

                ST_COMPUTE: begin
                    for (p_seq = 0; p_seq < OUTS; p_seq = p_seq + 1)
                        acc[p_seq] <= pe_next[p_seq];

                    if (ck == N[ROW_W-1:0] - 1'b1) begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        ck <= ck + 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
