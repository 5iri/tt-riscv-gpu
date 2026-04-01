`timescale 1ns/1ps

// 3-bit sign-magnitude weight matrix multiply core: 4x4 uint8 activations (A) x W3A8 weights (B).
// B encoding (sign-magnitude, bit[2]=sign, bits[1:0]=magnitude):
//   3'b000 =>  0
//   3'b001 => +1
//   3'b010 => +2
//   3'b011 => +3
//   3'b100 =>  0 (reserved)
//   3'b101 => -1
//   3'b110 => -2
//   3'b111 => -3
//
// Datapath uses shift-add (x2=shift, x3=shift+add) and negate — no multiplier.
// Multiple output lanes (PE) run in parallel to improve throughput.

module mac_core #(
    parameter N  = 4,
    parameter DW = 8,
    parameter CW = 13,
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
    input  wire [$clog2(N)-1:0]   c_rd_row,
    input  wire [$clog2(N)-1:0]   c_rd_col,
    output wire [CW-1:0]          c_rd_data
);

    localparam ROW_W = $clog2(N);
    localparam OUTS  = N * N;
    localparam OUT_W = $clog2(OUTS);

    localparam ST_IDLE    = 1'd0;
    localparam ST_COMPUTE = 1'd1;

    reg state;

    // Scratchpads
    reg [DW-1:0] a_spm [0:N-1][0:N-1];
    reg [2:0]    b_spm [0:N-1][0:N-1];
    reg [CW-1:0] c_spm [0:N-1][0:N-1];

    // Loop counters
    reg [ROW_W-1:0] ck;
    reg [OUT_W-1:0] co;  // Flat output index base for current PE batch

    // Per-lane accumulators and datapath signals
    reg signed [CW-1:0] acc      [0:PE-1];
    wire signed [CW-1:0] pe_term [0:PE-1];
    wire signed [CW-1:0] pe_next [0:PE-1];
    wire                 pe_valid [0:PE-1];
    wire [ROW_W-1:0]     pe_row  [0:PE-1];
    wire [ROW_W-1:0]     pe_col  [0:PE-1];

    // Combinational read
    assign c_rd_data = c_spm[c_rd_row][c_rd_col];

    // Per-lane 3-bit weight datapath: scale at narrow width, then extend and negate
    // a_scaled is DW+2 bits wide: max A*3 = 255*3 = 765 < 2^10
    wire [OUT_W:0]  flat_idx [0:PE-1];
    wire [2:0]      b_val    [0:PE-1];
    wire [DW-1:0]   a_val    [0:PE-1];
    wire [DW+1:0]   a_scaled [0:PE-1];

    genvar p_comb;
    generate
        for (p_comb = 0; p_comb < PE; p_comb = p_comb + 1) begin : pe_comb_blk
            localparam [OUT_W:0] LANE_OFF = p_comb;

            assign flat_idx[p_comb] = {1'b0, co} + LANE_OFF;
            assign pe_valid[p_comb] = (flat_idx[p_comb] < OUTS);
            assign pe_row[p_comb]   = pe_valid[p_comb] ? flat_idx[p_comb][OUT_W-1:ROW_W] : {ROW_W{1'b0}};
            assign pe_col[p_comb]   = pe_valid[p_comb] ? flat_idx[p_comb][ROW_W-1:0]      : {ROW_W{1'b0}};
            assign b_val[p_comb]    = pe_valid[p_comb] ? b_spm[ck][pe_col[p_comb]]         : 3'b000;
            assign a_val[p_comb]    = a_spm[pe_row[p_comb]][ck];

            assign a_scaled[p_comb] = (b_val[p_comb][1:0] == 2'b01) ? {2'b0, a_val[p_comb]} :
                                      (b_val[p_comb][1:0] == 2'b10) ? {1'b0, a_val[p_comb], 1'b0} :
                                      (b_val[p_comb][1:0] == 2'b11) ? ({1'b0, a_val[p_comb], 1'b0} + {2'b0, a_val[p_comb]}) :
                                                                       {(DW+2){1'b0}};

            assign pe_term[p_comb] = b_val[p_comb][2]
                ? -$signed({{(CW-DW-2){1'b0}}, a_scaled[p_comb]})
                :  $signed({{(CW-DW-2){1'b0}}, a_scaled[p_comb]});

            assign pe_next[p_comb] = acc[p_comb] + pe_term[p_comb];
        end
    endgenerate

    // Control-only synchronous reset: avoid async deassert X-propagation in GL.
    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
        end else begin
            // Deterministic defaults each cycle to avoid X-retention in GL.
            done <= 1'b0;
            busy <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        busy  <= 1'b1;
                        state <= ST_COMPUTE;
                    end
                end

                ST_COMPUTE: begin
                    busy <= 1'b1;
                    if ((ck == N[ROW_W-1:0] - 1'b1) && ((co + PE) >= OUTS)) begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    busy  <= 1'b0;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

    // Datapath/scratchpad FFs: no reset, values are initialized before use.
    always @(posedge clk) begin : datapath_seq_blk
        integer p_seq;
        // Load scratchpads when idle
        if (load_en && !busy) begin
            if (load_sel)
                b_spm[load_row][load_col] <= load_data[2:0];
            else
                a_spm[load_row][load_col] <= load_data;
        end

        case (state)
            ST_IDLE: begin
                if (start) begin
                    ck <= {ROW_W{1'b0}};
                    co <= {OUT_W{1'b0}};
                    for (p_seq = 0; p_seq < PE; p_seq = p_seq + 1)
                        acc[p_seq] <= {CW{1'b0}};
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
                    if ((co + PE) < OUTS)
                        co <= co + PE;
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

            default: ;
        endcase
    end

endmodule
