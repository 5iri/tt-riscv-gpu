`timescale 1ns/1ps

// 4x4 W3A8 matrix multiply core with an 8-lane outer-product engine.
// - A: 4x4 uint8 activations
// - B: 4x4 sign-magnitude weights (bit2=sign, bits[1:0]=magnitude 0..3)
// - C: 4x4 signed 13-bit outputs
//
// Scheduling:
// - Each k-step uses two cycles (grp=0 then grp=1), 8 outputs per cycle.
// - Total compute latency is 8 cycles after start.

module mac_core #(
    parameter DW = 8,
    parameter CW = 13
) (
    input  wire             clk,
    input  wire             rst,
    input  wire             start,
    output reg              busy,
    output reg              done,

    // Scratchpad load interface
    input  wire             load_en,
    input  wire             load_sel,  // 0: A, 1: B
    input  wire [1:0]       load_row,
    input  wire [1:0]       load_col,
    input  wire [DW-1:0]    load_data,

    // Result read interface
    input  wire             c_rd_en,
    input  wire [1:0]       c_rd_row,
    input  wire [1:0]       c_rd_col,
    output reg  [CW-1:0]    c_rd_data
);

    localparam ST_IDLE    = 1'b0;
    localparam ST_COMPUTE = 1'b1;

    reg state;

    reg [DW-1:0] a_spm [0:3][0:3];
    reg [2:0]    b_spm [0:3][0:3];
    reg signed [CW-1:0] acc [0:15];

    reg [1:0] ck;
    reg       grp;

    function automatic signed [CW-1:0] w3_mul;
        input [DW-1:0] a;
        input [2:0]    b;
        reg [DW+1:0] scaled;
        begin
            case (b[1:0])
                2'b01:   scaled = {2'b00, a};
                2'b10:   scaled = {1'b0, a, 1'b0};
                2'b11:   scaled = {1'b0, a, 1'b0} + {2'b00, a};
                default: scaled = {(DW+2){1'b0}};
            endcase

            if (b[2])
                w3_mul = -$signed({{(CW-DW-2){1'b0}}, scaled});
            else
                w3_mul =  $signed({{(CW-DW-2){1'b0}}, scaled});
        end
    endfunction

    always @(*) begin
        c_rd_data = {CW{1'b0}};
        if (c_rd_en) begin
            case ({c_rd_row, c_rd_col})
                4'h0: c_rd_data = acc[0];
                4'h1: c_rd_data = acc[1];
                4'h2: c_rd_data = acc[2];
                4'h3: c_rd_data = acc[3];
                4'h4: c_rd_data = acc[4];
                4'h5: c_rd_data = acc[5];
                4'h6: c_rd_data = acc[6];
                4'h7: c_rd_data = acc[7];
                4'h8: c_rd_data = acc[8];
                4'h9: c_rd_data = acc[9];
                4'hA: c_rd_data = acc[10];
                4'hB: c_rd_data = acc[11];
                4'hC: c_rd_data = acc[12];
                4'hD: c_rd_data = acc[13];
                4'hE: c_rd_data = acc[14];
                4'hF: c_rd_data = acc[15];
            endcase
        end
    end

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= ST_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            ck    <= 2'b00;
            grp   <= 1'b0;
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
                        busy <= 1'b1;
                        ck   <= 2'b00;
                        grp  <= 1'b0;
                        for (i = 0; i < 16; i = i + 1)
                            acc[i] <= {CW{1'b0}};
                        state <= ST_COMPUTE;
                    end
                end

                ST_COMPUTE: begin
                    if (!grp) begin
                        // grp=0: rows 0 and 1 (8 outputs)
                        acc[0] <= acc[0] + w3_mul(a_spm[0][ck], b_spm[ck][0]);
                        acc[1] <= acc[1] + w3_mul(a_spm[0][ck], b_spm[ck][1]);
                        acc[2] <= acc[2] + w3_mul(a_spm[0][ck], b_spm[ck][2]);
                        acc[3] <= acc[3] + w3_mul(a_spm[0][ck], b_spm[ck][3]);
                        acc[4] <= acc[4] + w3_mul(a_spm[1][ck], b_spm[ck][0]);
                        acc[5] <= acc[5] + w3_mul(a_spm[1][ck], b_spm[ck][1]);
                        acc[6] <= acc[6] + w3_mul(a_spm[1][ck], b_spm[ck][2]);
                        acc[7] <= acc[7] + w3_mul(a_spm[1][ck], b_spm[ck][3]);
                        grp    <= 1'b1;
                    end else begin
                        // grp=1: rows 2 and 3 (8 outputs)
                        acc[8]  <= acc[8]  + w3_mul(a_spm[2][ck], b_spm[ck][0]);
                        acc[9]  <= acc[9]  + w3_mul(a_spm[2][ck], b_spm[ck][1]);
                        acc[10] <= acc[10] + w3_mul(a_spm[2][ck], b_spm[ck][2]);
                        acc[11] <= acc[11] + w3_mul(a_spm[2][ck], b_spm[ck][3]);
                        acc[12] <= acc[12] + w3_mul(a_spm[3][ck], b_spm[ck][0]);
                        acc[13] <= acc[13] + w3_mul(a_spm[3][ck], b_spm[ck][1]);
                        acc[14] <= acc[14] + w3_mul(a_spm[3][ck], b_spm[ck][2]);
                        acc[15] <= acc[15] + w3_mul(a_spm[3][ck], b_spm[ck][3]);

                        grp <= 1'b0;
                        if (ck == 2'd3) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            ck <= ck + 2'd1;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
