`timescale 1ns/1ps

// Outer-product W3A8 matrix multiply core: 4x4 uint8 activations (A) x 3-bit sign-magnitude weights (B).
// B encoding: bit[2]=sign, bits[1:0]=magnitude {0,1,2,3} → value {0,+1,+2,+3} or negated.
//
// 8-lane outer-product engine:
// - Each k-step is processed in groups of LANES outputs.
// - For N=4 and LANES=8, computation takes N * (16/8) = 8 cycles.
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
    localparam GRPS  = (OUTS + LANES - 1) / LANES;
    localparam GRP_W = (GRPS <= 1) ? 1 : $clog2(GRPS);

    localparam ST_IDLE    = 1'd0;
    localparam ST_COMPUTE = 1'd1;

    reg state;

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
        if (c_rd_en)
            c_rd_data = acc[{c_rd_row, c_rd_col}];
    end

    function signed [CW-1:0] mul_w3a8;
        input [DW-1:0] a_val;
        input [2:0]    b_val;
        reg [DW+1:0] scaled;
        begin
            case (b_val[1:0])
                2'b01: scaled = {2'b0, a_val};
                2'b10: scaled = {1'b0, a_val, 1'b0};
                2'b11: scaled = {1'b0, a_val, 1'b0} + {2'b0, a_val};
                default: scaled = {(DW+2){1'b0}};
            endcase

            mul_w3a8 = b_val[2]
                ? -$signed({{(CW-DW-2){1'b0}}, scaled})
                :  $signed({{(CW-DW-2){1'b0}}, scaled});
        end
    endfunction

    reg [DW-1:0] a_row_lo;
    reg [DW-1:0] a_row_hi;
    reg [2:0] b_col0;
    reg [2:0] b_col1;
    reg [2:0] b_col2;
    reg [2:0] b_col3;
    reg signed [CW-1:0] pe_next0;
    reg signed [CW-1:0] pe_next1;
    reg signed [CW-1:0] pe_next2;
    reg signed [CW-1:0] pe_next3;
    reg signed [CW-1:0] pe_next4;
    reg signed [CW-1:0] pe_next5;
    reg signed [CW-1:0] pe_next6;
    reg signed [CW-1:0] pe_next7;

    always @(*) begin
        // For N=4 / LANES=8, grp=0 handles rows 0/1, grp=1 handles rows 2/3.
        if (grp[0]) begin
            a_row_lo = a_spm[2][ck];
            a_row_hi = a_spm[3][ck];
        end else begin
            a_row_lo = a_spm[0][ck];
            a_row_hi = a_spm[1][ck];
        end

        b_col0 = b_spm[ck][0];
        b_col1 = b_spm[ck][1];
        b_col2 = b_spm[ck][2];
        b_col3 = b_spm[ck][3];

        pe_next0 = acc[{grp[0], 3'b000}] + mul_w3a8(a_row_lo, b_col0);
        pe_next1 = acc[{grp[0], 3'b001}] + mul_w3a8(a_row_lo, b_col1);
        pe_next2 = acc[{grp[0], 3'b010}] + mul_w3a8(a_row_lo, b_col2);
        pe_next3 = acc[{grp[0], 3'b011}] + mul_w3a8(a_row_lo, b_col3);
        pe_next4 = acc[{grp[0], 3'b100}] + mul_w3a8(a_row_hi, b_col0);
        pe_next5 = acc[{grp[0], 3'b101}] + mul_w3a8(a_row_hi, b_col1);
        pe_next6 = acc[{grp[0], 3'b110}] + mul_w3a8(a_row_hi, b_col2);
        pe_next7 = acc[{grp[0], 3'b111}] + mul_w3a8(a_row_hi, b_col3);
    end

    integer p_seq;
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
                        for (p_seq = 0; p_seq < OUTS; p_seq = p_seq + 1)
                            acc[p_seq] <= {CW{1'b0}};
                        state <= ST_COMPUTE;
                    end
                end

                ST_COMPUTE: begin
                    if (grp[0]) begin
                        acc[8]  <= pe_next0;
                        acc[9]  <= pe_next1;
                        acc[10] <= pe_next2;
                        acc[11] <= pe_next3;
                        acc[12] <= pe_next4;
                        acc[13] <= pe_next5;
                        acc[14] <= pe_next6;
                        acc[15] <= pe_next7;

                        grp <= {GRP_W{1'b0}};
                        if (ck == N[ROW_W-1:0] - 1'b1) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            ck <= ck + 1'b1;
                        end
                    end else begin
                        acc[0] <= pe_next0;
                        acc[1] <= pe_next1;
                        acc[2] <= pe_next2;
                        acc[3] <= pe_next3;
                        acc[4] <= pe_next4;
                        acc[5] <= pe_next5;
                        acc[6] <= pe_next6;
                        acc[7] <= pe_next7;

                        grp <= grp + 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
