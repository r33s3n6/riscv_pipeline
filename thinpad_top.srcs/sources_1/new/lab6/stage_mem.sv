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
    input  wire [31:0]   pc_next_i,
    input  wire [31:0]   alu_i,
    input  wire [31:0]   mem_i,
    input  wire [ 1:0]   data_rd_mux_i,
    output logic [31:0]   data_rd_o
);

always_comb begin
  case (data_rd_mux_i)
    `DATA_RD_PC_NEXT: data_rd_o = pc_next_i;
    `DATA_RD_ALU: data_rd_o = alu_i;
    `DATA_RD_MEM: data_rd_o = mem_i;
    default: data_rd_o = 32'b0;
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
    output wire mem_bubble_o
);

    assign mem_bubble_o = mem_operation_i & ~mem_done_i;
endmodule

module mem_pipeline_regs (
    input wire clk_i,
    input wire rst_i,

    input wire bubble_i,
    input wire stall_i,

    input wire [31:0] data_rd_i,
    output reg [31:0] data_rd_o,

    input wire [ 4:0] reg_rd_i,
    output reg [ 4:0] reg_rd_o,

    input wire        rf_write_enable_i,
    output reg        rf_write_enable_o
);

    always @(posedge clk_i) begin
        if (rst_i) begin
            data_rd_o <= 32'b0;
            reg_rd_o <= 32'b0;
            rf_write_enable_o <= 1'b0;
        end else begin
            if (stall_i) begin

            end else if (bubble_i) begin
                data_rd_o <= 32'b0;
                reg_rd_o <= 32'b0;
                rf_write_enable_o <= 1'b0;
            end else begin
                data_rd_o <= data_rd_i;
                reg_rd_o <= reg_rd_i;
                rf_write_enable_o <= rf_write_enable_i;
            end
        end
    end
endmodule

module stage_mem (
    input  wire         clk_i,
    input  wire         rst_i,

    // input  wire         exe_stall_i,
    output wire         stall_o,
    output wire         done_o,
    output wire [31:0]  data_rd_o,

    input  wire [31:0]  exe_reg_inst_pc_i,
    input  wire [31:0]  exe_reg_alu_y_i,
    input  wire         exe_reg_mem_operation_i,
    input  wire         exe_reg_mem_write_enable_i,
    input  wire         exe_reg_mem_unsigned_ext_i,
    input  wire         exe_reg_rf_write_enable_i,
    input  wire [ 1:0]  exe_reg_data_rd_mux_i,
    input  wire [ 3:0]  exe_reg_byte_sel_i, 
    input  wire [31:0]  exe_reg_data_rs2_i,
    input  wire [ 4:0]  exe_reg_reg_rd_i,

    output wire         wb_cyc_o,
    output wire         wb_stb_o,
    input  wire         wb_ack_i,
    output wire [31:0]  wb_adr_o,
    output wire [31:0]  wb_dat_o,
    input  wire [31:0]  wb_dat_i,
    output wire [ 3:0]  wb_sel_o,
    output wire         wb_we_o,

    output wire [31:0]  reg_data_rd_o,
    output wire [ 4:0]  reg_reg_rd_o,
    output wire         reg_rf_write_enable_o
);

// internal wires
wire [31:0] _mem_inst_pc_plus4;
wire [31:0] _mem_data_read_final;
wire [31:0] _mem_data_read;

wire        _mem_bubble;



adder4 pc_adder(
    .data_i(exe_reg_inst_pc_i),
    .data_plus4_o(_mem_inst_pc_plus4)
);

mem_imm_gen mem_imm_gen_inst(
    .imm_i(_mem_data_read),
    .byte_sel_i(exe_reg_byte_sel_i),
    .unsigned_ext_i(exe_reg_mem_unsigned_ext_i),
    .imm_o(_mem_data_read_final)
);

mem_data_rd_mux mem_data_rd_mux_inst(
    .pc_next_i(_mem_inst_pc_plus4),
    .alu_i(exe_reg_alu_y_i),
    .mem_i(_mem_data_read_final),
    .data_rd_mux_i(exe_reg_data_rd_mux_i),
    .data_rd_o(data_rd_o)
);

mem_dm_controller mem_dm_controller_inst(
    .operation_i(exe_reg_mem_operation_i),
    .write_enable_i(exe_reg_mem_write_enable_i),
    .byte_sel_i(exe_reg_byte_sel_i),
    .data_write_i(exe_reg_data_rs2_i),
    .data_read_o(_mem_data_read),
    .addr_i(exe_reg_alu_y_i),
    .done_o(done_o),

    .wb_cyc_o(wb_cyc_o),
    .wb_stb_o(wb_stb_o),
    .wb_ack_i(wb_ack_i),
    .wb_adr_o(wb_adr_o),
    .wb_dat_o(wb_dat_o),
    .wb_dat_i(wb_dat_i),
    .wb_sel_o(wb_sel_o),
    .wb_we_o(wb_we_o)
);



mem_stall_controller mem_stall_controller_inst(
    .mem_stall_o(stall_o)
);

mem_bubble_controller mem_bubble_controller_inst(
    .mem_operation_i(exe_reg_mem_operation_i),
    .mem_done_i(done_o),
    .mem_bubble_o(_mem_bubble)
);

mem_pipeline_regs mem_pipeline_regs_inst(
    .clk_i(clk_i),
    .rst_i(rst_i),

    .bubble_i(_mem_bubble),
    .stall_i(stall_o),

    .data_rd_i(data_rd_o),
    .reg_rd_i(exe_reg_reg_rd_i),
    .rf_write_enable_i(exe_reg_rf_write_enable_i),

    .data_rd_o(reg_data_rd_o),
    .reg_rd_o(reg_reg_rd_o),
    .rf_write_enable_o(reg_rf_write_enable_o)
);

endmodule