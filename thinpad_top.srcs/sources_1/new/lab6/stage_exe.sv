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
      `ALU_ADD   : y = a  +  b;
      `ALU_SUB   : y = a  -  b;
      `ALU_SLL   : y = a <<  b[4:0];
      `ALU_SLT   : y =   signed'(a) <   signed'(b);
      `ALU_SLTU  : y = unsigned'(a) < unsigned'(b);
      `ALU_XOR   : y = a  ^  b;
      `ALU_SRL   : y = a >>  b[4:0];
      `ALU_SRA   : y = a >>> b[4:0];
      `ALU_OR    : y = a  |  b;
      `ALU_AND   : y = a  &  b;
      `ALU_XNOR  : y = a  ^ ~b;
      `ALU_CLZ   : y = {26'b0, clz_a[5:0]};
      `ALU_MIN   : y =   signed'(a) <   signed'(b) ? a : b;
      `ALU_CLR   : y = b & ~a;
      `ALU_USE_A : y = a;
      default    : y = 32'bx;
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
         default: y = 1'bx;
    endcase
  end

endmodule

module exe_pipeline_regs (
    input wire        clk_i,
    input wire        rst_i,

    input wire        bubble_i,
    input wire        stall_i,

    input wire [ 1:0] mode_i,
    output reg [ 1:0] mode_o,

    input wire [31:0] inst_pc_i,
    output reg [31:0] inst_pc_o,

    input wire [31:0] pc_plus4_i,
    output reg [31:0] pc_plus4_o,

    input wire [31:0] alu_y_i,
    output reg [31:0] alu_y_o,

    input wire        mem_operation_i,
    output reg        mem_operation_o,

    input wire        mem_write_enable_i,
    output reg        mem_write_enable_o,

    input wire        mem_unsigned_ext_i,
    output reg        mem_unsigned_ext_o,

    input wire        rf_write_enable_i,
    output reg        rf_write_enable_o,

    input wire [ 1:0] data_rd_mux_i,
    output reg [ 1:0] data_rd_mux_o,

    input wire [ 3:0] byte_sel_i, 
    output reg [ 3:0] byte_sel_o, 

    input wire [31:0] data_rs2_i,
    output reg [31:0] data_rs2_o,

    input wire [ 4:0] reg_rd_i,
    output reg [ 4:0] reg_rd_o,

    input wire [ 4:0] id_csr_i,
    output reg [ 4:0] id_csr_o,

    input wire [31:0] data_csr_i,
    output reg [31:0] data_csr_o,

    input wire        csr_write_enable_i,
    output reg        csr_write_enable_o,

    input wire        exception_i,
    output reg        exception_o,

    input wire [ 1:0] trap_mode_i,
    output reg [ 1:0] trap_mode_o,

    input wire [31:0] mcause_i,
    output reg [31:0] mcause_o,

    input wire [31:0] mtval_i,
    output reg [31:0] mtval_o,

    input wire [31:0] medeleg_i,
    output reg [31:0] medeleg_o
);

always_ff @(posedge clk_i) begin
    if (rst_i) begin
        mode_o <= 2'b00;
        inst_pc_o <= 32'b0;
        pc_plus4_o <= 32'b0;
        alu_y_o <= 32'b0;
        mem_operation_o <= 32'b0;
        mem_write_enable_o <= 32'b0;
        mem_unsigned_ext_o <= 32'b0;
        rf_write_enable_o <= 32'b0;
        data_rd_mux_o <= 2'b0;
        byte_sel_o <= 4'b0;
        data_rs2_o <= 32'b0;
        reg_rd_o <= 5'b0;
        id_csr_o <= 5'b0;
        data_csr_o <= 32'b0;
        csr_write_enable_o <= 32'b0;

        exception_o <= 1'b0;
        trap_mode_o <= 2'b0;
        mcause_o    <= 32'b0;
        mtval_o     <= 32'b0;
        medeleg_o   <= 32'b0;

    end else begin
        if (stall_i) begin

        end else if (bubble_i) begin
            // mode_o <= 2'b00;
            inst_pc_o <= 32'b0;
            pc_plus4_o <= 32'b0;
            alu_y_o <= 32'b0;
            mem_operation_o <= 32'b0;
            mem_write_enable_o <= 32'b0;
            mem_unsigned_ext_o <= 32'b0;
            rf_write_enable_o <= 32'b0;
            data_rd_mux_o <= 2'b0;
            byte_sel_o <= 4'b0;
            data_rs2_o <= 32'b0;
            reg_rd_o <= 5'b0;
            id_csr_o <= 5'b0;
            data_csr_o <= 32'b0;
            csr_write_enable_o <= 32'b0;

            exception_o <= 1'b0;
            trap_mode_o <= 2'b0;
            mcause_o    <= 32'b0;
            mtval_o     <= 32'b0;
            medeleg_o   <= 32'b0;

        end else begin
            mode_o <= mode_i;
            inst_pc_o <= inst_pc_i;
            pc_plus4_o <= pc_plus4_i;
            alu_y_o <= alu_y_i;
            mem_operation_o <= mem_operation_i;

            mem_unsigned_ext_o <= mem_unsigned_ext_i;

            data_rd_mux_o <= data_rd_mux_i;
            byte_sel_o <= byte_sel_i;
            data_rs2_o <= data_rs2_i;
            reg_rd_o <= reg_rd_i;
            id_csr_o <= id_csr_i;
            data_csr_o <= data_csr_i;

            if (!exception_i) begin
                mem_write_enable_o <= mem_write_enable_i;
                csr_write_enable_o <= csr_write_enable_i;
                rf_write_enable_o <= rf_write_enable_i;
            end else begin // disable side effects on exception
                mem_write_enable_o <= 1'b0;
                csr_write_enable_o <= 1'b0;
                rf_write_enable_o <= 1'b0;
            end


            exception_o <= exception_i;
            trap_mode_o <= trap_mode_i;
            mcause_o    <= mcause_i;
            mtval_o     <= mtval_i;
            medeleg_o   <= medeleg_i;
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
    input wire        mem_next_exception_i,
    input wire        wb_prev_exception_i,

    output wire       exe_bubble_o
);

    assign exe_bubble_o = mem_next_exception_i
                        || wb_prev_exception_i;

endmodule


