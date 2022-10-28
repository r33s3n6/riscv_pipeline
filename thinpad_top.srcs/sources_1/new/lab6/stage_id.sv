`include "alu_define.sv"

// no csr support
module id_instruction_decoder (
    input wire   [31:0] inst_i,

    output wire  [ 4:0] reg_rd_o,
    output wire  [ 4:0] reg_rs1_o,
    output wire  [ 4:0] reg_rs2_o,

    output logic [31:0] imm_o,

    output logic [ 3:0] alu_op_o, // add/slt ...
    output wire  [ 2:0] cmp_op_o, // beq/bge ...
 
    output logic [ 3:0] byte_sel_o, // lb/sw ...

    output wire         is_branch_o,
    output wire         invalid_inst_o,
    output wire         alu_a_use_pc_o,
    output wire         alu_b_use_imm_o,

    output wire         mem_operation_o,
    output wire         mem_write_enable_o,
    output wire         mem_unsigned_ext_o,

    output wire         rf_write_enable_o,
    output logic [ 1:0] data_rd_mux_o // 0: alu, 1: mem, 2: pc+4
);

    typedef enum logic [2:0] {
        R_TYPE,
        I_TYPE,
        S_TYPE,
        B_TYPE,
        U_TYPE,
        J_TYPE,
        UNKNOWN
    } inst_type_t;

    inst_type_t inst_type;
    
    logic [ 2:0] funct3;
    logic [ 6:0] funct7;

    assign funct3 = inst_i[14:12];
    assign funct7 = inst_i[31:25];
    
    logic [ 6:0] opcode;

    assign opcode = inst_i[6:0];

    assign reg_rd_o = inst_i[11:7];
    assign reg_rs1_o = (opcode[6:0] == 7'b0110111) ? 5'b0 : inst_i[19:15]; // lui use rs1 as 0
    assign reg_rs2_o = inst_i[24:20];

    always_comb begin
        if (opcode[6:0] == 7'b0110111 // lui
        ||  opcode[6:0] == 7'b0010111 // auipc
        ) begin 
            inst_type = U_TYPE;
        end else if (opcode[6:0] == 7'b1101111) begin // jal
            inst_type = J_TYPE;
        end else if (opcode[6:0] == 7'b1100011) begin // branch
            inst_type = B_TYPE;
        end else if (opcode[6:0] == 7'b0110011) begin // arithmetic
            inst_type = R_TYPE;
        end else if (opcode[6:0] == 7'b0100011) begin // store
            inst_type = S_TYPE;
        end else if (   opcode[6:0] == 7'b1100111 // jalr
                    ||  opcode[6:0] == 7'b0000011 // load
                    ||  opcode[6:0] == 7'b0010011 // imm
                    ||  opcode[6:0] == 7'b0001111 // fence
                    ||  opcode[6:0] == 7'b1110011 // env & csr
                     ) begin
            inst_type = I_TYPE;
        end else begin
            inst_type = UNKNOWN;
        end
    end

    // generate immediate
    always_comb begin
        case (inst_type)
            I_TYPE: begin
                imm_o = { {20{inst_i[31]}}, inst_i[31:20] };
            end
            S_TYPE: begin
                imm_o = { {20{inst_i[31]}}, inst_i[31:25], inst_i[11:7] }; //?
            end
            B_TYPE: begin
                imm_o = { {19{inst_i[31]}}, inst_i[31], inst_i[7], inst_i[30:25], inst_i[11:8], 1'b0 };
            end
            U_TYPE: begin
                imm_o = { inst_i[31:12], 12'b0 };
            end
            J_TYPE: begin
                imm_o = { {11{inst_i[31]}}, inst_i[31], inst_i[19:12], inst_i[20], inst_i[30:21], 1'b0 };
            end
            default: begin
                imm_o = 32'b0;
            end
        endcase
    end

    assign is_branch_o = (inst_type == B_TYPE || inst_type == J_TYPE || opcode[6:0] == 7'b1100111); // branch, jal, jalr

    always_comb begin
        if (   inst_type == B_TYPE 
            || inst_type == U_TYPE 
            || inst_type == J_TYPE 
            || inst_type == S_TYPE
            || opcode == 7'b0000011 //load
            ) begin
            alu_op_o[3:0] = `ALU_ADD;
        end else begin
            alu_op_o[2:0] = funct3[2:0];
            alu_op_o[3] = (inst_type == R_TYPE 
                        || funct3[2:0] == 3'b101 // I-type srli/srai
                        || funct3[2:0] == 3'b001 // I-type slli
                        ) ? funct7[5] : 1'b0;
        end
    end

    assign cmp_op_o = (opcode[6:0]==7'b1100111 || opcode[6:0]==7'b1101111)? `CMP_NONE : funct3[2:0]; // jal/jalr no cmp

    always_comb begin
        case (funct3[2:0])
            3'b000: begin // lb
                byte_sel_o = 4'b0001;
            end
            3'b001: begin // lh
                byte_sel_o = 4'b0011;
            end
            3'b010: begin // lw
                byte_sel_o = 4'b1111;
            end
            3'b100: begin // lbu
                byte_sel_o = 4'b0001;
            end
            3'b101: begin // lhu
                byte_sel_o = 4'b0011;
            end
            default: begin
                byte_sel_o = 4'b0000;
            end
        endcase
    end

    assign alu_a_use_pc_o = (  opcode == 7'b0010111 // auipc
                            || opcode == 7'b1101111 // jal
                            || inst_type == B_TYPE // branch
                            );
    assign alu_b_use_imm_o = !( inst_type == R_TYPE
                            ||  opcode == 7'b1110011 && funct3[2] // csrr{w,s,c}i
                             );

    assign mem_operation_o = (  opcode == 7'b0100011 // store
                             || opcode == 7'b0000011 // load
                             );

    assign mem_write_enable_o = ( opcode == 7'b0100011 // store
                                );

    assign rf_write_enable_o = ( inst_type == R_TYPE
                              || (inst_type == I_TYPE 
                                    && opcode != 7'b0001111 
                                    && opcode != 7'b1110011 ) // fence & env & csr
                              || inst_type == U_TYPE
                              || inst_type == J_TYPE
                                )  && reg_rd_o != 5'b00000; // x0 is not writable
    
    assign mem_unsigned_ext_o = funct3[2]; // lbu, lhu

    always_comb begin
        if (opcode == 7'b1101111 || opcode == 7'b1100111) begin // jal, jalr
            data_rd_mux_o = `DATA_RD_PC_NEXT;
        end else if (opcode == 7'b0000011) begin // load
            data_rd_mux_o = `DATA_RD_MEM;
        end else begin
            data_rd_mux_o = `DATA_RD_ALU;
        end
    end
    
    assign invalid_inst_o = (inst_type == UNKNOWN);

endmodule

module id_register_file(
  input  wire         clk_i,
  input  wire         rst_i,

  input  wire [ 4: 0] waddr_i,
  input  wire [31: 0] wdata_i,
  input  wire         we_i,
  input  wire [ 4: 0] raddr_a_i,
  input  wire [ 4: 0] raddr_b_i,
  output wire [31: 0] rdata_a_o,
  output wire [31: 0] rdata_b_o
);
  // real registers
  reg [31:0] reg_file [0:31];

  assign rdata_a_o = (waddr_i == raddr_a_i && we_i) ? wdata_i : reg_file[raddr_a_i];
  assign rdata_b_o = (waddr_i == raddr_b_i && we_i) ? wdata_i : reg_file[raddr_b_i];

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      reg_file <= '{default: '0};
    end else begin
      if (we_i && waddr_i) begin
        reg_file[waddr_i] <= wdata_i;
      end
    end
  end

endmodule

module id_pipeline_regs (
    input  wire         clk_i,
    input  wire         rst_i,

    input wire bubble_i,
    input wire stall_i,

    input wire [3:0] alu_op_i,
    output reg [3:0] alu_op_o,

    input wire [2:0] cmp_op_i,
    output reg [2:0] cmp_op_o,

    input wire [31:0] imm_i,
    output reg [31:0] imm_o,

    input wire [31:0] data_rs1_i,
    output reg [31:0] data_rs1_o,
    input wire [31:0] data_rs2_i,
    output reg [31:0] data_rs2_o,

    input wire [4:0] reg_rd_i,
    output reg [4:0] reg_rd_o,

    input wire        is_branch_i,
    output reg        is_branch_o,

    input wire [31:0] inst_pc_i,
    output reg [31:0] inst_pc_o,

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

    input wire [3:0] byte_sel_i, 
    output reg [3:0] byte_sel_o, 

    input wire alu_a_use_pc_i,
    output reg alu_a_use_pc_o,

    input wire alu_b_use_imm_i,
    output reg alu_b_use_imm_o


);

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            alu_op_o <= 4'b0;
            cmp_op_o <= 3'b0;
            imm_o <= 32'b0;
            data_rs1_o <= 32'b0;
            data_rs2_o <= 32'b0;
            reg_rd_o <= 5'b0;
            is_branch_o <= 1'b0;
            inst_pc_o <= 32'b0;
            mem_operation_o <= 1'b0;
            mem_write_enable_o <= 1'b0;
            mem_unsigned_ext_o <= 1'b0;
            rf_write_enable_o <= 1'b0;
            data_rd_mux_o <= 2'b0;
            byte_sel_o <= 4'b0;
            alu_a_use_pc_o <= 1'b0;
            alu_b_use_imm_o <= 1'b0;

        end else begin
            if (stall_i) begin

            end else if(bubble_i) begin
                alu_op_o <= 4'b0;
                cmp_op_o <= 3'b0;
                imm_o <= 32'b0;
                data_rs1_o <= 32'b0;
                data_rs2_o <= 32'b0;
                reg_rd_o <= 5'b0;
                is_branch_o <= 1'b0;
                inst_pc_o <= 32'b0;
                mem_operation_o <= 1'b0;
                mem_write_enable_o <= 1'b0;
                mem_unsigned_ext_o <= 1'b0;
                rf_write_enable_o <= 1'b0;
                data_rd_mux_o <= 2'b0;
                byte_sel_o <= 4'b0;
                alu_a_use_pc_o <= 1'b0;
                alu_b_use_imm_o <= 1'b0;
            end else begin
                alu_op_o <= alu_op_i;
                cmp_op_o <= cmp_op_i;
                imm_o <= imm_i;
                data_rs1_o <= data_rs1_i;
                data_rs2_o <= data_rs2_i;
                reg_rd_o <= reg_rd_i;
                is_branch_o <= is_branch_i;
                inst_pc_o <= inst_pc_i;
                mem_operation_o <= mem_operation_i;
                mem_write_enable_o <= mem_write_enable_i;
                mem_unsigned_ext_o <= mem_unsigned_ext_i;
                rf_write_enable_o <= rf_write_enable_i;
                data_rd_mux_o <= data_rd_mux_i;
                byte_sel_o <= byte_sel_i;
                alu_a_use_pc_o <= alu_a_use_pc_i;
                alu_b_use_imm_o <= alu_b_use_imm_i;
            end
        end
    end

endmodule

module id_stall_controller (
    input wire exe_stall_i,

    output wire id_stall_o
);

    assign id_stall_o = exe_stall_i;

endmodule

module id_bubble_controller (
    input wire wait_reg_i,
    output wire id_bubble_o
);

    assign id_bubble_o = wait_reg_i;

endmodule

module id_wait_reg_controller (
    input wire [4:0] reg_rs1_i,
    input wire [4:0] reg_rs2_i,

    input wire [4:0] id_reg_reg_rd_i,
    input wire       id_reg_rf_write_enable_i,

    input wire [4:0] exe_reg_reg_rd_i,
    input wire       exe_reg_rf_write_enable_i,

    output wire wait_reg_o
);

    assign wait_reg_o = (id_reg_rf_write_enable_i 
                            && ((reg_rs1_i==id_reg_reg_rd_i) 
                                    || (reg_rs2_i==id_reg_reg_rd_i)))
                    || (exe_reg_rf_write_enable_i 
                            && ((reg_rs1_i==exe_reg_reg_rd_i) 
                                    || (reg_rs2_i==exe_reg_reg_rd_i)));   

endmodule

module stage_id (
    input  wire         clk_i,
    input  wire         rst_i,

    input  wire         exe_stall_i,
    output wire         stall_o,
    output wire         wait_reg_o,

    input wire [4:0]    id_reg_reg_rd_i,
    input wire          id_reg_rf_write_enable_i,
   
    input wire [4:0]    exe_reg_reg_rd_i,
    input wire          exe_reg_rf_write_enable_i,

    input wire [31:0]   mem_reg_data_rd_i,
    input wire [ 4:0]   mem_reg_reg_rd_i,
    input wire          mem_reg_rf_write_enable_i,

    input wire [31:0]   if_reg_inst_i,
    input wire [31:0]   if_reg_inst_pc_i,

    output wire         is_branch_o,
    // registers
    output wire [ 3:0]  reg_alu_op_o,
    output wire [ 2:0]  reg_cmp_op_o,
    output wire [31:0]  reg_imm_o,
    output wire [31:0]  reg_data_rs1_o,
    output wire [31:0]  reg_data_rs2_o,
    output wire [ 4:0]  reg_reg_rd_o,
    output wire         reg_is_branch_o,
    output wire [31:0]  reg_inst_pc_o,
    output wire         reg_mem_operation_o,
    output wire         reg_mem_write_enable_o,
    output wire         reg_mem_unsigned_ext_o,
    output wire         reg_rf_write_enable_o,
    output wire [ 1:0]  reg_data_rd_mux_o,
    output wire [ 3:0]  reg_byte_sel_o,
    output wire         reg_alu_a_use_pc_o,
    output wire         reg_alu_b_use_imm_o

);


  // internal wires

  wire [4:0] _id_reg_rs1;
  wire [4:0] _id_reg_rs2;
  
  wire [31:0] _id_data_rs1;
  wire [31:0] _id_data_rs2;
  
  wire [31:0] _id_imm;
  wire [3:0]  _id_alu_op;
  wire [2:0]  _id_cmp_op;
  wire [4:0]  _id_reg_rd;

  wire        _id_mem_operation;
  wire        _id_mem_write_enable;
  wire        _id_mem_unsigned_ext;
  wire        _id_rf_write_enable;
  wire [1:0]  _id_data_rd_mux;
  wire [3:0]  _id_byte_sel;
  wire        _id_alu_a_use_pc;
  wire        _id_alu_b_use_imm;

  wire        _id_invalid_inst;
  wire        _id_bubble;

  // instruction decoder
    id_instruction_decoder id_instruction_decoder_inst (
        .inst_i(if_reg_inst_i),

        .reg_rs1_o(_id_reg_rs1),
        .reg_rs2_o(_id_reg_rs2),
        .reg_rd_o(_id_reg_rd),

        .imm_o(_id_imm),

        .alu_op_o(_id_alu_op),
        .cmp_op_o(_id_cmp_op),
        
        .is_branch_o(is_branch_o),
        .invalid_inst_o(_id_invalid_inst),

        .byte_sel_o(_id_byte_sel),
        .alu_a_use_pc_o(_id_alu_a_use_pc),
        .alu_b_use_imm_o(_id_alu_b_use_imm),
        

        .mem_operation_o(_id_mem_operation),
        .mem_write_enable_o(_id_mem_write_enable),
        .mem_unsigned_ext_o(_id_mem_unsigned_ext),
        .rf_write_enable_o(_id_rf_write_enable),
        .data_rd_mux_o(_id_data_rd_mux)
    );

    // register file
    id_register_file id_register_file_inst (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .waddr_i(mem_reg_reg_rd_i),
        .wdata_i(mem_reg_data_rd_i),
        .we_i(mem_reg_rf_write_enable_i),
        .raddr_a_i(_id_reg_rs1),
        .raddr_b_i(_id_reg_rs2),
        .rdata_a_o(_id_data_rs1),
        .rdata_b_o(_id_data_rs2)
    );

    // stall controller
    id_stall_controller id_stall_controller_inst (
        .exe_stall_i(exe_stall_i),
        .id_stall_o(stall_o)
    );

    // bubble controller
    id_bubble_controller id_bubble_controller_inst (
        .wait_reg_i(wait_reg_o),
        .id_bubble_o(_id_bubble)
    );

    // wait register controller

    id_wait_reg_controller id_wait_reg_controller_inst (
        .reg_rs1_i(_id_reg_rs1),
        .reg_rs2_i(_id_reg_rs2),
        .id_reg_reg_rd_i(id_reg_reg_rd_i),
        .id_reg_rf_write_enable_i(id_reg_rf_write_enable_i),
        .exe_reg_reg_rd_i(exe_reg_reg_rd_i),
        .exe_reg_rf_write_enable_i(exe_reg_rf_write_enable_i),
        .wait_reg_o(wait_reg_o)
    );

    // pipeline registers
    id_pipeline_regs id_pipeline_regs_inst (
        .clk_i(clk_i),
        .rst_i(rst_i),

        .bubble_i(_id_bubble),
        .stall_i(stall_o),

        .alu_op_i(_id_alu_op),
        .cmp_op_i(_id_cmp_op),
        .imm_i(_id_imm),
        .data_rs1_i(_id_data_rs1),
        .data_rs2_i(_id_data_rs2),

        .reg_rd_i(_id_reg_rd),
        .is_branch_i(is_branch_o),
        .inst_pc_i(if_reg_inst_pc_i),

        .mem_operation_i(_id_mem_operation),
        .mem_write_enable_i(_id_mem_write_enable),
        .mem_unsigned_ext_i(_id_mem_unsigned_ext),

        .rf_write_enable_i(_id_rf_write_enable),
        .data_rd_mux_i(_id_data_rd_mux),

        .byte_sel_i(_id_byte_sel),
        .alu_a_use_pc_i(_id_alu_a_use_pc),
        .alu_b_use_imm_i(_id_alu_b_use_imm),
        

        .alu_op_o(reg_alu_op_o),
        .cmp_op_o(reg_cmp_op_o),
        .imm_o(reg_imm_o),
        .data_rs1_o(reg_data_rs1_o),
        .data_rs2_o(reg_data_rs2_o),

        .reg_rd_o(reg_reg_rd_o),
        .is_branch_o(reg_is_branch_o),
        .inst_pc_o(reg_inst_pc_o),

        .mem_operation_o(reg_mem_operation_o),
        .mem_write_enable_o(reg_mem_write_enable_o),
        .mem_unsigned_ext_o(reg_mem_unsigned_ext_o),

        .rf_write_enable_o(reg_rf_write_enable_o),
        .data_rd_mux_o(reg_data_rd_mux_o),

        .byte_sel_o(reg_byte_sel_o),
        .alu_a_use_pc_o(reg_alu_a_use_pc_o),
        .alu_b_use_imm_o(reg_alu_b_use_imm_o)

    );

endmodule

