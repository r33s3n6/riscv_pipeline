`timescale 1ns / 1ns
module clint_device #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    // clk and reset
    input wire clk_i,
    input wire rst_i,

    // wishbone slave interface
    input  wire                    wb_cyc_i,
    input  wire                    wb_stb_i,
    output wire                    wb_ack_o,
    input  wire [  ADDR_WIDTH-1:0] wb_adr_i,
    input  wire [  DATA_WIDTH-1:0] wb_dat_i,
    output logic[  DATA_WIDTH-1:0] wb_dat_o,
    input  wire [DATA_WIDTH/8-1:0] wb_sel_i,
    input  wire                    wb_we_i,

    output wire                    irq_o,
    output wire [            63:0] time_o
);

    localparam MTIME_ADDR     = 16'hBFF8;
    localparam MTIMEH_ADDR    = 16'hBFFC;
    localparam MTIMECMP_ADDR  = 16'h4000;
    localparam MTIMECMPH_ADDR = 16'h4004;

    logic [63:0] mtime;
    logic [63:0] mtimecmp;

    assign time_o = mtime;

    assign wb_ack_o = wb_stb_i & wb_cyc_i; // always ready

    assign irq_o = mtime >= mtimecmp;

    always_comb begin 
        wb_dat_o = {32{1'bx}};
        if(wb_stb_i && ~wb_we_i) begin
            case (wb_adr_i[15:0])
                MTIME_ADDR: begin
                    // ignore wb_sel_i
                    wb_dat_o = mtime[31:0];
                end

                MTIMEH_ADDR: begin
                    // ignore wb_sel_i
                    wb_dat_o = mtime[63:32];
                end

                MTIMECMP_ADDR: begin
                    // ignore wb_sel_i
                    wb_dat_o = mtimecmp[31:0];
                end

                MTIMECMPH_ADDR: begin
                    // ignore wb_sel_i
                    wb_dat_o = mtimecmp[63:32];
                end
    
                default: ;  // do nothing
            endcase
        end
    end

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            mtime <= 64'h0;
        end else begin
            mtime <= mtime + 1;
        end
    end

    // mtimecmp
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            mtimecmp <= 64'b0;
        end else if(wb_stb_i && wb_we_i) begin
            case (wb_adr_i[15:0])
                MTIMECMP_ADDR: begin
                    // ignore wb_sel_i
                    mtimecmp[31:0] <= wb_dat_i;
                end

                MTIMECMPH_ADDR: begin
                    // ignore wb_sel_i
                    mtimecmp[63:32] <= wb_dat_i;
                end
  
                default: ;  // do nothing
            endcase
        end
    end


endmodule
