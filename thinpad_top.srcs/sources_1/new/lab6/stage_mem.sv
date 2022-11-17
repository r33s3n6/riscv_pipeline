`include "alu_define.sv"

module mem_dm_controller(
    input  wire          operation_i,
    input  wire          write_enable_i,
    input  wire [ 3:0]   byte_sel_i,
    input  wire [31:0]   data_write_i,
    output wire [31:0]   data_read_o,
    input  wire [31:0]   addr_i,
    output wire          done_o,

    // wb signals
    output wire           wb_cyc_o,
    output wire           wb_stb_o,
    input  wire           wb_ack_i,
    output wire  [31:0]   wb_adr_o,
    output wire  [31:0]   wb_dat_o,
    input  wire  [31:0]   wb_dat_i,
    output wire  [3:0]    wb_sel_o,
    output wire           wb_we_o
);

assign wb_cyc_o = operation_i;
assign wb_stb_o = operation_i;

assign wb_adr_o = addr_i;
assign wb_dat_o = data_write_i;
assign wb_sel_o = byte_sel_i << addr_i[1:0]; // not aligned
assign wb_we_o  = write_enable_i;

assign data_read_o = wb_dat_i >> (addr_i[1:0] * 8); // not aligned
assign done_o      = wb_ack_i;

endmodule


module mem_imm_gen(
    input  wire [31:0]   imm_i,
    input  wire [ 3:0]   byte_sel_i,
    input  wire          unsigned_ext_i,
    output logic [31:0]  imm_o
);


always_comb begin
  if (unsigned_ext_i) begin
    if (byte_sel_i[3]) begin
      imm_o = imm_i;
    end else if (byte_sel_i[1]) begin
      imm_o = {16'b0, imm_i[15:0]};
    end else if (byte_sel_i[0]) begin
      imm_o = {24'b0, imm_i[7:0]};
    end else begin
      imm_o = {32'b0};
    end
  end else begin
    if (byte_sel_i[3]) begin
      imm_o = imm_i;
    end else if (byte_sel_i[1]) begin
      imm_o = {{16{imm_i[15]}}, imm_i[15:0]};
    end else if (byte_sel_i[0]) begin
      imm_o = {{24{imm_i[7]}}, imm_i[7:0]};
    end else begin
      imm_o = {32'b0};
    end
  end
end



endmodule

module mem_data_rd_mux (
    input  wire  [31:0]   pc_i,
    input  wire  [31:0]   alu_i,
    input  wire  [31:0]   mem_i,
    input  wire  [31:0]   csr_i,
    input  wire  [ 1:0]   data_rd_mux_i,
    output logic [31:0]   data_rd_o
);

always_comb begin
    case (data_rd_mux_i)
        `DATA_RD_PC_NEXT: data_rd_o = pc_i;
        `DATA_RD_ALU    : data_rd_o = alu_i;
        `DATA_RD_MEM    : data_rd_o = mem_i;
        `DATA_RD_CSR    : data_rd_o = csr_i;
    endcase
end

endmodule

module mem_stall_controller(
    output wire mem_stall_o
);

    assign mem_stall_o = 1'b0;

endmodule

module mem_bubble_controller (
    input wire mem_operation_i,
    input wire mem_done_i,

    input wire wb_prev_exception_i,

    output wire mem_bubble_o
);

    assign mem_bubble_o = (mem_operation_i & ~mem_done_i) || wb_prev_exception_i;
endmodule

module mem_pipeline_regs (
    input wire clk_i,
    input wire rst_i,

    input wire bubble_i,
    input wire stall_i,

    input wire [ 1:0] mode_i,
    output reg [ 1:0] mode_o,

    input wire [31:0] inst_pc_i,
    output reg [31:0] inst_pc_o,

    input wire [31:0] data_rd_i,
    output reg [31:0] data_rd_o,

    input wire [ 4:0] reg_rd_i,
    output reg [ 4:0] reg_rd_o,

    input wire        rf_write_enable_i,
    output reg        rf_write_enable_o,

    input wire [31:0] data_csr_i,
    output reg [31:0] data_csr_o,

    input wire [ 4:0] id_csr_i,
    output reg [ 4:0] id_csr_o,

    input wire        csr_write_enable_i,
    output reg        csr_write_enable_o,

    input wire        exception_i,
    output reg        exception_o,

    input wire [ 1:0] trap_mode_i,
    output reg [ 1:0] trap_mode_o,

    input wire [31:0] mcause_i,
    output reg [31:0] mcause_o,

    input wire [31:0] mtval_i,
    output reg [31:0] mtval_o
);

    always @(posedge clk_i) begin
        if (rst_i) begin
            mode_o  <= 2'b0;
            inst_pc_o <= 32'b0;
            data_rd_o <= 32'b0;
            reg_rd_o <= 32'b0;
            rf_write_enable_o <= 1'b0;
            data_csr_o <= 32'b0;
            id_csr_o <= 32'b0;
            csr_write_enable_o <= 1'b0;

            exception_o <= 1'b0;
            trap_mode_o <= 2'b0;
            mcause_o    <= 32'b0;
            mtval_o     <= 32'b0;
        end else begin
            if (stall_i) begin

            end else if (bubble_i) begin
                inst_pc_o <= 32'b0;
                data_rd_o <= 32'b0;
                reg_rd_o <= 32'b0;
                rf_write_enable_o <= 1'b0;
                data_csr_o <= 32'b0;
                id_csr_o <= 32'b0;
                csr_write_enable_o <= 1'b0;

                exception_o <= 1'b0;
                trap_mode_o <= 2'b0;
                mcause_o    <= 32'b0;
                mtval_o     <= 32'b0;
            end else begin
                mode_o <= mode_i;
                inst_pc_o <= inst_pc_i;
                data_rd_o <= data_rd_i;
                reg_rd_o <= reg_rd_i;
                rf_write_enable_o <= rf_write_enable_i;
                data_csr_o <= data_csr_i;
                id_csr_o <= id_csr_i;
                csr_write_enable_o <= csr_write_enable_i;

                if (!exception_i) begin
                    csr_write_enable_o <= csr_write_enable_i;
                    rf_write_enable_o <= rf_write_enable_i;
                end else begin // disable side effects on exception
                    csr_write_enable_o <= 1'b0;
                    rf_write_enable_o <= 1'b0;
                end

                exception_o <= exception_i;
                trap_mode_o <= trap_mode_i;
                mcause_o    <= mcause_i;
                mtval_o     <= mtval_i;
            end
        end
    end
endmodule

