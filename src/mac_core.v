`timescale 1ns/1ps

// 8-bit unsigned matrix multiply core for 4x4 uint8 A and uint8 B.
// C[i][j] = sum_k A[i][k] * B[k][j]  (unsigned, 18-bit accumulator)
// Multiple output lanes (PE) run in parallel to improve throughput.

module mac_core #(
    parameter N  = 4,
    parameter DW = 8,
    parameter CW = 18,
    parameter PE = 2
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

    // Result read interface
    input  wire                   c_rd_en,
    input  wire [$clog2(N)-1:0]   c_rd_row,
    input  wire [$clog2(N)-1:0]   c_rd_col,
    output reg  [CW-1:0]          c_rd_data
);

    localparam ROW_W = $clog2(N);
    localparam OUTS  = N * N;
    localparam OUT_W = $clog2(OUTS);

    localparam [1:0] ST_IDLE    = 2'd0;
    localparam [1:0] ST_COMPUTE = 2'd1;
    localparam [1:0] ST_DONE    = 2'd2;

    reg [1:0] state;

    // Scratchpads
    reg [DW-1:0] a_spm [0:N-1][0:N-1];
    reg [DW-1:0] b_spm [0:N-1][0:N-1];
    reg [CW-1:0] c_spm [0:N-1][0:N-1];

    // Loop counters
    reg [ROW_W-1:0] ck;
    reg [OUT_W-1:0] co;  // Flat output index base for current PE batch

    // Per-lane accumulators and datapath signals
    reg [CW-1:0]    acc      [0:PE-1];
    reg [CW-1:0]    pe_term  [0:PE-1];
    reg [CW-1:0]    pe_next  [0:PE-1];
    reg             pe_valid [0:PE-1];
    reg [ROW_W-1:0] pe_row   [0:PE-1];
    reg [ROW_W-1:0] pe_col   [0:PE-1];

    integer i, j;
    integer p_comb, p_seq;
    reg [OUT_W:0] flat_idx;

    // Combinational read
    always @(*) begin
        c_rd_data = {CW{1'b0}};
        if (c_rd_en)
            c_rd_data = c_spm[c_rd_row][c_rd_col];
    end

    // Per-lane multiply-accumulate datapath
    always @(*) begin
        for (p_comb = 0; p_comb < PE; p_comb = p_comb + 1) begin
            flat_idx = {1'b0, co} + p_comb;
            pe_valid[p_comb] = (flat_idx < OUTS);
            pe_row[p_comb]   = {ROW_W{1'b0}};
            pe_col[p_comb]   = {ROW_W{1'b0}};
            pe_term[p_comb]  = {CW{1'b0}};
            pe_next[p_comb]  = acc[p_comb];

            if (pe_valid[p_comb]) begin
                pe_row[p_comb] = flat_idx[OUT_W-1:ROW_W];
                pe_col[p_comb] = flat_idx[ROW_W-1:0];

                pe_term[p_comb] = {{(CW-2*DW){1'b0}},
                                   a_spm[pe_row[p_comb]][ck] *
                                   b_spm[ck][pe_col[p_comb]]};

                pe_next[p_comb] = acc[p_comb] + pe_term[p_comb];
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= ST_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            ck    <= {ROW_W{1'b0}};
            co    <= {OUT_W{1'b0}};

            for (i = 0; i < N; i = i + 1)
                for (j = 0; j < N; j = j + 1) begin
                    a_spm[i][j] <= {DW{1'b0}};
                    b_spm[i][j] <= {DW{1'b0}};
                    c_spm[i][j] <= {CW{1'b0}};
                end

            for (p_seq = 0; p_seq < PE; p_seq = p_seq + 1)
                acc[p_seq] <= {CW{1'b0}};
        end else begin
            done <= 1'b0;

            // Load scratchpads when idle
            if (load_en && !busy) begin
                if (load_sel)
                    b_spm[load_row][load_col] <= load_data;
                else
                    a_spm[load_row][load_col] <= load_data;
            end

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        ck   <= {ROW_W{1'b0}};
                        co   <= {OUT_W{1'b0}};
                        for (p_seq = 0; p_seq < PE; p_seq = p_seq + 1)
                            acc[p_seq] <= {CW{1'b0}};
                        state <= ST_COMPUTE;
                    end
                end

                ST_COMPUTE: begin
                    if (ck == N[ROW_W-1:0] - 1'b1) begin
                        // Dot-product end for all active lanes: store and reset lane accumulators.
                        for (p_seq = 0; p_seq < PE; p_seq = p_seq + 1) begin
                            if (pe_valid[p_seq])
                                c_spm[pe_row[p_seq]][pe_col[p_seq]] <= pe_next[p_seq];
                            acc[p_seq] <= {CW{1'b0}};
                        end

                        ck <= {ROW_W{1'b0}};

                        if ((co + PE) >= OUTS) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= ST_DONE;
                        end else begin
                            co <= co + PE;
                        end
                    end else begin
                        ck <= ck + 1'b1;
                        for (p_seq = 0; p_seq < PE; p_seq = p_seq + 1) begin
                            if (pe_valid[p_seq])
                                acc[p_seq] <= pe_next[p_seq];
                            else
                                acc[p_seq] <= {CW{1'b0}};
                        end
                    end
                end

                ST_DONE: begin
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
