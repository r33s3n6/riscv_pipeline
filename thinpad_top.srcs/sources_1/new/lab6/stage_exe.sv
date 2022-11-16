`include "alu_define.sv"

module exe_clz(
    input  wire  [31:0]  val32,
    output logic [5:0]   result
);

    logic [15:0] val16;
    logic [7:0]  val8;
    logic [3:0]  val4;

    always_comb begin
        if(val32[31:0] == 32'b0) begin
            result[5:0] = 6'd32;
        end else begin
            result[5] = 1'b0;
            result[4] = (val32[31:16] == 16'b0);
            val16     = result[4] ? val32[15:0] : val32[31:16];
            result[3] = (val16[15:8] == 8'b0);
            val8      = result[3] ? val16[7:0] : val16[15:8];
            result[2] = (val8[7:4] == 4'b0);
            val4      = result[2] ? val8[3:0] : val8[7:4];
            result[1] = (val4[3:2] == 2'b0);
            result[0] = result[1] ? ~val4[1] : ~val4[3];
        end
    end

endmodule


module exe_alu(
  input  wire signed [31:0] a,
  input  wire        [31:0] b,
  input  wire        [ 3:0] op,
  output logic       [31:0] y
);

  wire [5:0] clz_a;
  exe_clz exe_clz_inst(
    .val32(a),
    .result(clz_a)
  );

  always_comb begin
    case (op)
      `ALU_ADD : y = a  +  b;
      `ALU_SUB : y = a  -  b;
      `ALU_SLL : y = a <<  b[4:0];
      `ALU_SLT : y =   signed'(a) <   signed'(b);
      `ALU_SLTU: y = unsigned'(a) < unsigned'(b);
      `ALU_XOR : y = a  ^  b;
      `ALU_SRL : y = a >>  b[4:0];
      `ALU_SRA : y = a >>> b[4:0];
      `ALU_OR  : y = a  |  b;
      `ALU_AND : y = a  &  b;
      `ALU_XNOR: y = a  ^ ~b;
      `ALU_CLZ : y = {26'b0, clz_a[5:0]};
      `ALU_MIN : y =   signed'(a) <   signed'(b) ? a : b;
        default  y = 32'b0;
    endcase
  end
    
endmodule

module exe_comparator (
  input  wire signed [31:0] a,
  input  wire signed [31:0] b,
  input  wire [2:0] op,
  output logic y
);

  always_comb begin
    case (op)
        `CMP_EQ : y = (a==b);
        `CMP_NE : y = (a!=b);
        `CMP_LT : y = (a< b);
        `CMP_GE : y = (a>=b);
        `CMP_LTU: y = (unsigned'(a)< unsigned'(b));
        `CMP_GEU: y = (unsigned'(a)>=unsigned'(b));
        `CMP_NONE: y = 1'b1;
         default: y = 1'b0;
    endcase
  end

endmodule

module exe_pipeline_regs (
    input  wire         clk_i,
    input  wire         rst_i,

    input wire bubble_i,
    input wire stall_i,


    input wire [31:0] inst_pc_i,
    output reg [31:0] inst_pc_o,

    input wire [31:0] alu_y_i,
    output reg [31:0] alu_y_o,

    input wire       mem_operation_i,
    output reg       mem_operation_o,

    input wire       mem_write_enable_i,
    output reg       mem_write_enable_o,

    input wire       mem_unsigned_ext_i,
    output reg       mem_unsigned_ext_o,

    input wire       rf_write_enable_i,
    output reg       rf_write_enable_o,

    input wire [ 1:0] data_rd_mux_i,
    output reg [ 1:0] data_rd_mux_o,

    input wire [3:0] byte_sel_i, 
    output reg [3:0] byte_sel_o, 

    input wire [31:0] data_rs2_i,
    output reg [31:0] data_rs2_o,

    input wire [4:0] reg_rd_i,
    output reg [4:0] reg_rd_o
);

always_ff @(posedge clk_i) begin
    if (rst_i) begin
        inst_pc_o <= 32'b0;
        alu_y_o <= 32'b0;
        mem_operation_o <= 32'b0;
        mem_write_enable_o <= 32'b0;
        mem_unsigned_ext_o <= 32'b0;
        rf_write_enable_o <= 32'b0;
        data_rd_mux_o <= 2'b0;
        byte_sel_o <= 4'b0;
        data_rs2_o <= 32'b0;
        reg_rd_o <= 5'b0;
    end else begin
        if (stall_i) begin

        end else if (bubble_i) begin
            inst_pc_o <= 32'b0;
            alu_y_o <= 32'b0;
            mem_operation_o <= 32'b0;
            mem_write_enable_o <= 32'b0;
            mem_unsigned_ext_o <= 32'b0;
            rf_write_enable_o <= 32'b0;
            data_rd_mux_o <= 2'b0;
            byte_sel_o <= 4'b0;
            data_rs2_o <= 32'b0;
            reg_rd_o <= 5'b0;
        end else begin
            inst_pc_o <= inst_pc_i;
            alu_y_o <= alu_y_i;
            mem_operation_o <= mem_operation_i;
            mem_write_enable_o <= mem_write_enable_i;
            mem_unsigned_ext_o <= mem_unsigned_ext_i;
            rf_write_enable_o <= rf_write_enable_i;
            data_rd_mux_o <= data_rd_mux_i;
            byte_sel_o <= byte_sel_i;
            data_rs2_o <= data_rs2_i;
            reg_rd_o <= reg_rd_i;
        end
    end
end

endmodule

module exe_stall_controller (
    input wire mem_stall_i,
    input wire mem_operation_i,
    input wire mem_done_i,

    output wire exe_stall_o
);

    assign exe_stall_o = mem_stall_i || (mem_operation_i && !mem_done_i);

endmodule

module exe_bubble_controller (
    output wire exe_bubble_o
);

    assign exe_bubble_o = 1'b0;

endmodule

module stage_exe (
    input  wire        clk_i,
    input  wire        rst_i,

    input  wire        mem_stall_i,
    input  wire        mem_done_i,
    output wire        stall_o,

    output wire        use_alu_pc_o,
    output wire [31:0] alu_y_o,

    input wire  [ 3:0] id_reg_alu_op_i,
    input wire  [ 2:0] id_reg_cmp_op_i,
    input wire  [31:0] id_reg_imm_i,
    input wire  [31:0] id_reg_data_rs1_i,
    input wire  [31:0] id_reg_data_rs2_i,
    input wire  [ 4:0] id_reg_reg_rd_i,
    // input wire         id_reg_is_branch_i,
    input wire  [31:0] id_reg_inst_pc_i,
    input wire         id_reg_mem_operation_i,
    input wire         id_reg_mem_write_enable_i,
    input wire         id_reg_mem_unsigned_ext_i,
    input wire         id_reg_rf_write_enable_i,
    input wire  [ 1:0] id_reg_data_rd_mux_i,
    input wire  [ 3:0] id_reg_byte_sel_i,
    input wire         id_reg_alu_a_use_pc_i,
    input wire         id_reg_alu_b_use_imm_i,


    output wire [31:0] reg_inst_pc_o,
    output wire [31:0] reg_alu_y_o,
    output wire        reg_mem_operation_o,
    output wire        reg_mem_write_enable_o,
    output wire        reg_mem_unsigned_ext_o,
    output wire        reg_rf_write_enable_o,
    output wire [ 1:0] reg_data_rd_mux_o,
    output wire [ 3:0] reg_byte_sel_o, 
    output wire [31:0] reg_data_rs2_o,
    output wire [ 4:0] reg_reg_rd_o


);

  // internal wires
  wire [31:0] _exe_alu_a;
  wire [31:0] _exe_alu_b;
  wire        _exe_bubble;
  
  assign _exe_alu_a = id_reg_alu_a_use_pc_i  ? id_reg_inst_pc_i : id_reg_data_rs1_i;
  assign _exe_alu_b = id_reg_alu_b_use_imm_i ? id_reg_imm_i     : id_reg_data_rs2_i;
  
  // instantiate modules
  
  exe_alu exe_alu_inst (
      .op(id_reg_alu_op_i),
      .a(_exe_alu_a),
      .b(_exe_alu_b),
      .y(alu_y_o)
  );
  
  exe_comparator exe_comparator_inst (
      .op(id_reg_cmp_op_i),
      .a(id_reg_data_rs1_i),
      .b(id_reg_data_rs2_i),
      .y(use_alu_pc_o)
  );
  
  exe_stall_controller exe_stall_controller_inst (
      .mem_stall_i(mem_stall_i),
      .mem_operation_i(reg_mem_operation_o),
      .mem_done_i(mem_done_i),
      .exe_stall_o(stall_o)
  );
  
  exe_bubble_controller exe_bubble_controller_inst (
      .exe_bubble_o(_exe_bubble)
  );
  
  // register outputs
  exe_pipeline_regs exe_pipeline_regs_inst (
      .clk_i(clk_i),
      .rst_i(rst_i),
  
      .stall_i(stall_o),
      .bubble_i(_exe_bubble),
  
      .inst_pc_i(id_reg_inst_pc_i),
      .alu_y_i(alu_y_o),
      .mem_operation_i(id_reg_mem_operation_i),
      .mem_write_enable_i(id_reg_mem_write_enable_i),
      .mem_unsigned_ext_i(id_reg_mem_unsigned_ext_i),
      .rf_write_enable_i(id_reg_rf_write_enable_i),
      .data_rd_mux_i(id_reg_data_rd_mux_i),
      .byte_sel_i(id_reg_byte_sel_i),
      .data_rs2_i(id_reg_data_rs2_i),
      .reg_rd_i(id_reg_reg_rd_i),
  
      .inst_pc_o(reg_inst_pc_o),
      .alu_y_o(reg_alu_y_o),
      .mem_operation_o(reg_mem_operation_o),
      .mem_write_enable_o(reg_mem_write_enable_o),
      .mem_unsigned_ext_o(reg_mem_unsigned_ext_o),
      .rf_write_enable_o(reg_rf_write_enable_o),
      .data_rd_mux_o(reg_data_rd_mux_o),
      .byte_sel_o(reg_byte_sel_o),
      .data_rs2_o(reg_data_rs2_o),
      .reg_rd_o(reg_reg_rd_o)
  );


endmodule
