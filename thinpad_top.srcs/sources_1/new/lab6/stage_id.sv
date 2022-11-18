`include "alu_define.sv"

// no csr support
module id_instruction_decoder (
    input wire   [31:0] inst_i,
    input wire   [ 1:0] mode_i,

    output logic        invalid_inst_o,
    output logic        ecall_o,            
    output logic        ebreak_o,           
    output logic        mret_o,
    output logic        sret_o,

    output wire         is_branch_o,
    
    output wire  [ 4:0] reg_rs1_o,
    output wire  [ 4:0] reg_rs2_o,
    output wire  [ 4:0] reg_rd_o,
    output wire         rf_write_enable_o,
    output logic [ 1:0] data_rd_mux_o, // 0: alu, 1: mem, 2: pc+4, 3: csr

    output logic [ 4:0] id_csr_o,
    output wire         csr_write_enable_o,

    output logic [31:0] imm_o,
    output logic [31:0] uimm_o,

    output logic [ 3:0] alu_op_o, // add/slt ...
    output logic [ 1:0] alu_a_mux_o,
    output logic [ 1:0] alu_b_mux_o,

    output wire  [ 2:0] cmp_op_o, // beq/bge ...

    output logic [ 3:0] byte_sel_o, // lb/sw ...
    output wire         mem_operation_o,
    output wire         mem_write_enable_o,
    output wire         mem_unsigned_ext_o
    
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

    mode_t mode;
    assign mode = mode_t'(mode_i);
    
    logic [ 2:0] funct3;
    logic [ 6:0] funct7;
    logic [ 6:0] opcode;
    logic [ 4:0] rd;
    logic [ 4:0] rs1;
    logic [ 4:0] rs2;
    logic [11:0] csr;
    logic [ 4:0] uimm;

    assign funct3 = inst_i[14:12];
    assign funct7 = inst_i[31:25];

    assign opcode = inst_i[6:0];
    assign rd     = inst_i[11:7];
    assign rs1    = inst_i[19:15];
    assign rs2    = inst_i[24:20];

    // used by csr instructions
    assign csr    = inst_i[31:20];
    assign uimm   = inst_i[19:15];

    assign reg_rd_o  = rd;
    assign reg_rs1_o = (opcode[6:0] == 7'b0110111) ? 5'b0 : rs1; // lui use rs1 as 0
    assign reg_rs2_o = rs2;

    // csr id
    always_comb begin
        case(csr)
            `CSR_CYCLE    : id_csr_o = `CSR_ID_CYCLE    ;
            `CSR_TIME     : id_csr_o = `CSR_ID_TIME     ;
            `CSR_CYCLEH   : id_csr_o = `CSR_ID_CYCLEH   ;
            `CSR_TIMEH    : id_csr_o = `CSR_ID_TIMEH    ;

            `CSR_SSTATUS  : id_csr_o = `CSR_ID_SSTATUS  ;
            `CSR_SIE      : id_csr_o = `CSR_ID_SIE      ;
            `CSR_STVEC    : id_csr_o = `CSR_ID_STVEC    ;

            `CSR_SSCRATCH : id_csr_o = `CSR_ID_SSCRATCH ;
            `CSR_SEPC     : id_csr_o = `CSR_ID_SEPC     ;
            `CSR_SCAUSE   : id_csr_o = `CSR_ID_SCAUSE   ;
            `CSR_STVAL    : id_csr_o = `CSR_ID_STVAL    ;
            `CSR_SIP      : id_csr_o = `CSR_ID_SIP      ;

            `CSR_SATP     : id_csr_o = `CSR_ID_SATP     ;

            `CSR_MHARTID  : id_csr_o = `CSR_ID_MHARTID  ;

            `CSR_MSTATUS  : id_csr_o = `CSR_ID_MSTATUS  ;
            `CSR_MEDELEG  : id_csr_o = `CSR_ID_MEDELEG  ;
            `CSR_MIDELEG  : id_csr_o = `CSR_ID_MIDELEG  ;
            `CSR_MIE      : id_csr_o = `CSR_ID_MIE      ;
            `CSR_MTVEC    : id_csr_o = `CSR_ID_MTVEC    ;

            `CSR_MSCRATCH : id_csr_o = `CSR_ID_MSCRATCH ;
            `CSR_MEPC     : id_csr_o = `CSR_ID_MEPC     ;
            `CSR_MCAUSE   : id_csr_o = `CSR_ID_MCAUSE   ;
            `CSR_MTVAL    : id_csr_o = `CSR_ID_MTVAL    ;
            `CSR_MIP      : id_csr_o = `CSR_ID_MIP      ;

            `CSR_MCYCLE   : id_csr_o = `CSR_ID_MCYCLE   ;
            `CSR_MCYCLEH  : id_csr_o = `CSR_ID_MCYCLEH  ;

            `CSR_PMPCFG0  : id_csr_o = `CSR_ID_PMPCFG0  ;
            `CSR_PMPADDR0 : id_csr_o = `CSR_ID_PMPADDR0 ;
            default       : id_csr_o = `CSR_ID_UNKNOWN  ;
        endcase
    end
    logic csr_inst;
    logic csr_read_only;
    logic csr_read;
    logic csr_write;
    logic csr_priv_ok;

    assign csr_inst = (opcode == 7'b1110011 && funct3!=3'b000);

    assign csr_read_only = (csr[11:10] == 2'b11);
    assign csr_read      = !(!funct3[1] && rd == 5'b0); // csrrw{,i}
    assign csr_write     = !(funct3[1] && rs1 == 5'b0); // csrr{s,c}{,i}
    assign csr_priv_ok   =  (csr[9:8] <= mode) && (!(csr_read_only && csr_write)) && (id_csr_o != `CSR_ID_UNKNOWN);

    assign csr_write_enable_o = csr_inst && csr_write && csr_priv_ok;


    // inst_type
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

    assign uimm_o = {  27'b0, uimm[4:0] };

    assign is_branch_o = (inst_type == B_TYPE || inst_type == J_TYPE || opcode[6:0] == 7'b1100111); // branch, jal, jalr

    // alu_operation
    always_comb begin
        if (   inst_type == B_TYPE 
            || inst_type == U_TYPE 
            || inst_type == J_TYPE 
            || inst_type == S_TYPE
            || opcode == 7'b0000011 //load
            ) begin
            alu_op_o[3:0] = `ALU_ADD;
        end else begin
            if (inst_type == R_TYPE && funct7[6:0] == 7'b0000101) begin // min
                alu_op_o[3:0] = `ALU_MIN;
            end else if (opcode == 7'b1110011 && funct3[1:0] == 2'b01) begin // csrrw{,i}
                alu_op_o[3:0] = `ALU_USE_A;
            end else if (opcode == 7'b1110011 && funct3[1:0] == 2'b10) begin // csrrs{,i}
                alu_op_o[3:0] = `ALU_OR;
            end else if (opcode == 7'b1110011 && funct3[1:0] == 2'b11) begin // csrrc{,i}
                alu_op_o[3:0] = `ALU_CLR;
            end else begin
                alu_op_o[2:0] = funct3[2:0];
                alu_op_o[3] = (inst_type == R_TYPE 
                            || funct3[2:0] == 3'b101 // I-type srli/srai
                            || funct3[2:0] == 3'b001 // I-type slli/clz
                            || funct3[2:0] == 3'b100 // I-type xnor
                            ) ? funct7[5] : 1'b0;
            end
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


    assign mem_operation_o = (  opcode == 7'b0100011 // store
                             || opcode == 7'b0000011 // load
                             );

    assign mem_write_enable_o = ( opcode == 7'b0100011 // store
                                );
    
    assign mem_unsigned_ext_o = funct3[2]; // lbu, lhu

    assign rf_write_enable_o =  (   opcode == 7'b0110011 // arithmetic (reg)
                                ||  opcode == 7'b0010011 // arithmetic (imm)
                                ||  opcode == 7'b0000011 // load
                                ||  opcode == 7'b1101111 // jal
                                ||  opcode == 7'b1100111 // jalr
                                ||  opcode == 7'b0110111 // lui
                                ||  opcode == 7'b0010111 // auipc
                                ||  csr_inst // csr
                                ) && reg_rd_o != 5'b00000;


    // alu_a_mux
    always_comb begin
        if  (   opcode == 7'b0010111 // auipc
            ||  opcode == 7'b1101111 // jal
            ||  inst_type == B_TYPE // branch
        ) begin
            alu_a_mux_o = `ALU_A_PC;
        end else if (opcode == 7'b1110011 && funct3[2]) begin // csr{w,s,c}i
            alu_a_mux_o = `ALU_A_UIMM;
        end else begin
            alu_a_mux_o = `ALU_A_RS1;
        end
    end

    // alu_b_mux
    always_comb begin
        if (inst_type == R_TYPE) begin
            alu_b_mux_o = `ALU_B_RS2;
        end else if (opcode == 7'b1110011 && funct3 != 3'b0) // csr{w,s,c}{,i}
            alu_b_mux_o = `ALU_B_CSR;
        else begin
            alu_b_mux_o = `ALU_B_IMM;
        end
    end

    // data_rd_mux
    always_comb begin
        if (opcode == 7'b1101111 || opcode == 7'b1100111) begin // jal, jalr
            data_rd_mux_o = `DATA_RD_PC_NEXT;
        end else if (opcode == 7'b0000011) begin // load
            data_rd_mux_o = `DATA_RD_MEM;
        end else if (opcode == 7'b1110011 && funct3 != 3'b0) begin // csr{w,s,c}{,i}
            data_rd_mux_o = `DATA_RD_CSR;
        end else begin
            data_rd_mux_o = `DATA_RD_ALU;
        end
    end
    
    // invalid instruction
    always_comb begin
        invalid_inst_o = 1'b1; // default invalid

        if  (   opcode == 7'b0110111  // lui
            ||  opcode == 7'b0010111  // auipc
            ||  opcode == 7'b1101111  // jal
        ) begin
            invalid_inst_o = 1'b0;

        end else if (opcode == 7'b1100111 && funct3 == 3'b0) begin // jalr
            invalid_inst_o = 1'b0;

        end else if (opcode == 7'b1100011) begin // branch
            if  (   funct3 == 3'b000 // beq
                ||  funct3 == 3'b001 // bne
                ||  funct3 == 3'b100 // blt
                ||  funct3 == 3'b101 // bge
                ||  funct3 == 3'b110 // bltu
                ||  funct3 == 3'b111 // bgeu
            ) begin
                invalid_inst_o = 1'b0;
            end

        end else if (opcode == 7'b0000011) begin // load
            if  (   funct3 == 3'b000 // lb
                ||  funct3 == 3'b001 // lh
                ||  funct3 == 3'b010 // lw
                ||  funct3 == 3'b100 // lbu
                ||  funct3 == 3'b101 // lhu
            ) begin
                invalid_inst_o = 1'b0;
            end

        end else if (opcode == 7'b0100011) begin // store
            if  (   funct3 == 3'b000 // sb
                ||  funct3 == 3'b001 // sh
                ||  funct3 == 3'b010 // sw
            ) begin
                invalid_inst_o = 1'b0;
            end

        end else if (opcode == 7'b0010011) begin // arithmetic(imm)
            if  (   funct3 == 3'b000 // addi
                ||  funct3 == 3'b010 // slti
                ||  funct3 == 3'b011 // sltiu
                ||  funct3 == 3'b100 // xori
                ||  funct3 == 3'b110 // ori
                ||  funct3 == 3'b111 // andi
                || (funct3 == 3'b001 && funct7 == 7'b0000000) // slli
                || (funct3 == 3'b101 && funct7 == 7'b0000000) // srli
                || (funct3 == 3'b101 && funct7 == 7'b0100000) // srai
                || (funct3 == 3'b001 && funct7 == 7'b0110000 && rs2 == 5'b0) // clz
            ) begin
                invalid_inst_o = 1'b0;
            end

        end else if (opcode == 7'b0110011) begin // arithmetic(reg)
            if  (   (funct3 == 3'b000 && funct7 == 7'b0000000) // add
                ||  (funct3 == 3'b000 && funct7 == 7'b0100000) // sub
                ||  (funct3 == 3'b001 && funct7 == 7'b0000000) // sll
                ||  (funct3 == 3'b010 && funct7 == 7'b0000000) // slt
                ||  (funct3 == 3'b011 && funct7 == 7'b0000000) // sltu
                ||  (funct3 == 3'b100 && funct7 == 7'b0000000) // xor
                ||  (funct3 == 3'b100 && funct7 == 7'b0100000) // xnor
                ||  (funct3 == 3'b100 && funct7 == 7'b0000101) // min
                ||  (funct3 == 3'b101 && funct7 == 7'b0000000) // srl
                ||  (funct3 == 3'b101 && funct7 == 7'b0100000) // sra
                ||  (funct3 == 3'b110 && funct7 == 7'b0000000) // or
                ||  (funct3 == 3'b111 && funct7 == 7'b0000000) // and
            ) begin
                invalid_inst_o = 1'b0;
            end

        end else if (opcode == 7'b0001111 && rd == 5'b0 && rs1 == 5'b0) begin // fence

            if  (   (funct3 == 3'b000 && inst_i == 4'b0)  // fence
                ||  (funct3 == 3'b001 && inst_i == 12'b0) // fence.i
            ) begin
                invalid_inst_o = 1'b1; // TODO: implement fence and fence.i
            end

        end else if (opcode == 7'b1110011) begin // env & csr
            if  (funct3 == 3'b000) begin// ecall, ebreak, {m,s}ret, wfi
                if (funct7 == 7'b0000000) begin // ecall, ebreak
                    if (rd == 5'b0 && rs1 == 5'b0 && rs2 == 5'b0) begin // ecall
                        invalid_inst_o = 1'b0; 
                    end else if (rs1 == 5'b0 && rs1 == 5'b0 && rs2 == 5'b1) begin // ebreak
                        invalid_inst_o = 1'b0; 
                    end
                end else if (funct7 == 7'b0001000) begin // sret
                    if (rd == 5'b0 && rs1 == 5'b0 && rs2 == 5'b00010) begin
                        if (mode >= S_MODE) begin
                            invalid_inst_o = 1'b1; // TODO: implement sret
                        end
                    end
                end else if (funct7 == 7'b0011000) begin // mret
                    if (rd == 5'b0 && rs1 == 5'b0 && rs2 == 5'b00010) begin
                        if (mode == M_MODE) begin
                            invalid_inst_o = 1'b0;
                        end
                    end
                end else if (funct7 == 7'b0001000) begin // wfi
                    if (rd == 5'b0 && rs1 == 5'b0 && rs2 == 5'b00101) begin
                        invalid_inst_o = 1'b1; // TODO: implement wfi
                    end
                end

            end else if (   (funct3 == 3'b001 ) // csrrw
                        ||  (funct3 == 3'b010 ) // csrrs
                        ||  (funct3 == 3'b011 ) // csrrc
                        ||  (funct3 == 3'b101 ) // csrrwi
                        ||  (funct3 == 3'b110 ) // csrrsi
                        ||  (funct3 == 3'b111 ) // csrrci
            ) begin
                if (csr_priv_ok) begin
                    case (csr)
                        `CSR_CYCLE    : invalid_inst_o = 1'b0;
                        `CSR_TIME     : invalid_inst_o = 1'b0;
                        `CSR_CYCLEH   : invalid_inst_o = 1'b0;
                        `CSR_TIMEH    : invalid_inst_o = 1'b0;
                        `CSR_SSTATUS  : invalid_inst_o = 1'b1;
                        `CSR_SIE      : invalid_inst_o = 1'b1;
                        `CSR_STVEC    : invalid_inst_o = 1'b0;
                        `CSR_SSCRATCH : invalid_inst_o = 1'b0;
                        `CSR_SEPC     : invalid_inst_o = 1'b0;
                        `CSR_SCAUSE   : invalid_inst_o = 1'b0;
                        `CSR_STVAL    : invalid_inst_o = 1'b0;
                        `CSR_SIP      : invalid_inst_o = 1'b1;
                        `CSR_SATP     : invalid_inst_o = 1'b1;
                        `CSR_MHARTID  : invalid_inst_o = 1'b0;
                        `CSR_MSTATUS  : invalid_inst_o = 1'b0;
                        `CSR_MEDELEG  : invalid_inst_o = 1'b0;
                        `CSR_MIDELEG  : invalid_inst_o = 1'b0;
                        `CSR_MIE      : invalid_inst_o = 1'b0;
                        `CSR_MTVEC    : invalid_inst_o = 1'b0;
                        `CSR_MSCRATCH : invalid_inst_o = 1'b0;
                        `CSR_MEPC     : invalid_inst_o = 1'b0;
                        `CSR_MCAUSE   : invalid_inst_o = 1'b0;
                        `CSR_MTVAL    : invalid_inst_o = 1'b0;
                        `CSR_MIP      : invalid_inst_o = 1'b0;
                        `CSR_MCYCLE   : invalid_inst_o = 1'b0;
                        `CSR_MCYCLEH  : invalid_inst_o = 1'b0;
                        `CSR_PMPCFG0  : invalid_inst_o = 1'b0;
                        `CSR_PMPADDR0 : invalid_inst_o = 1'b0;
                        default       : invalid_inst_o = 1'b1;
                    endcase
                end
            end 

        end
    end

    assign ecall_o   = (opcode == 7'b1110011 && funct3 == 3'b000 && funct7 == 7'b0000000 && rd == 5'b0 && rs1 == 5'b0 && rs2 == 5'b0);
    assign ebreak_o  = (opcode == 7'b1110011 && funct3 == 3'b000 && funct7 == 7'b0000000 && rd == 5'b0 && rs1 == 5'b0 && rs2 == 5'b1);
    assign sret_o    = (opcode == 7'b1110011 && funct3 == 3'b000 && funct7 == 7'b0001000 && rd == 5'b0 && rs1 == 5'b0 && rs2 == 5'b00010);
    assign mret_o    = (opcode == 7'b1110011 && funct3 == 3'b000 && funct7 == 7'b0011000 && rd == 5'b0 && rs1 == 5'b0 && rs2 == 5'b00010);


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
            if (we_i) begin
                reg_file[waddr_i] <= wdata_i;
            end
        end
    end

endmodule

// TODO: mstatus/sstatus, mip/sip, mie/sie should be implemented as one register
module id_csr_file(
    input  wire         clk_i,
    input  wire         rst_i,

    input  wire [ 4: 0] raddr_a_i,
    output wire [31: 0] rdata_a_o,
    
    input  wire [ 4: 0] waddr_i,
    input  wire [31: 0] wdata_i,
    input  wire         we_i,

    // forward
    input  wire [ 4: 0] mem_waddr_i,
    input  wire [31: 0] mem_wdata_i,
    input  wire         mem_we_i,

    input  wire [ 4: 0] exe_waddr_i,
    input  wire [31: 0] exe_wdata_i,
    input  wire         exe_we_i,

    // irq
    input  wire         core_time_irq_i,

    // exception
    input  wire         wb_exception_i,
    input  wire  [31:0] mstatus_i,

    input  wire  [31:0] mepc_i,
    input  wire  [31:0] mcause_i,
    input  wire  [31:0] mtval_i,

    input  wire  [31:0] sepc_i,
    input  wire  [31:0] scause_i,
    input  wire  [31:0] stval_i,

    // important status registers
    output wire [31: 0] sstatus_o,
    output wire [31: 0] sie_o,
    output wire [31: 0] stvec_o,
    output wire [31: 0] sepc_o,
    output wire [31: 0] sip_o,
    output wire [31: 0] satp_o,
    output wire [31: 0] scause_o,
    output wire [31: 0] stval_o,

    output wire [31: 0] mstatus_o,
    output wire [31: 0] medeleg_o,
    output wire [31: 0] mideleg_o,
    output wire [31: 0] mie_o,
    output wire [31: 0] mtvec_o,
    output wire [31: 0] mepc_o,
    output wire [31: 0] mip_o,
    output wire [31: 0] mcause_o,
    output wire [31: 0] mtval_o,

    output wire [31: 0] old_mstatus_o // no forwarding //TODO: remove this(no use)
);
    // real registers
    reg   [31:0] reg_file [0:31];
    // forward
    logic [31:0] forwarded_reg [0:31];

    assign rdata_a_o = forwarded_reg[raddr_a_i];

    assign sstatus_o = forwarded_reg[`CSR_ID_SSTATUS];
    assign sie_o     = forwarded_reg[`CSR_ID_SIE];
    assign stvec_o   = forwarded_reg[`CSR_ID_STVEC];
    assign sepc_o    = forwarded_reg[`CSR_ID_SEPC];
    assign sip_o     = forwarded_reg[`CSR_ID_SIP];
    assign satp_o    = forwarded_reg[`CSR_ID_SATP];
    assign scause_o  = forwarded_reg[`CSR_ID_SCAUSE];
    assign stval_o   = forwarded_reg[`CSR_ID_STVAL];

    assign mstatus_o = forwarded_reg[`CSR_ID_MSTATUS];
    assign medeleg_o = forwarded_reg[`CSR_ID_MEDELEG];
    assign mideleg_o = forwarded_reg[`CSR_ID_MIDELEG];
    assign mie_o     = forwarded_reg[`CSR_ID_MIE];
    assign mtvec_o   = forwarded_reg[`CSR_ID_MTVEC];
    assign mepc_o    = forwarded_reg[`CSR_ID_MEPC];
    assign mip_o     = forwarded_reg[`CSR_ID_MIP];
    assign mcause_o  = forwarded_reg[`CSR_ID_MCAUSE];
    assign mtval_o   = forwarded_reg[`CSR_ID_MTVAL];

    assign old_mstatus_o = reg_file[`CSR_ID_MSTATUS];

    // forward registers
    always_comb begin
        forwarded_reg = reg_file;
        if (we_i) begin
            forwarded_reg[waddr_i] = wdata_i;
        end
        if (mem_we_i) begin
            forwarded_reg[mem_waddr_i] = mem_wdata_i;
        end
        if (exe_we_i) begin
            forwarded_reg[exe_waddr_i] = exe_wdata_i;
        end
    end
  
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            reg_file <= '{default: '0};
        end else begin
            if (wb_exception_i) begin
                reg_file[`CSR_ID_MSTATUS] <= mstatus_i;

                reg_file[`CSR_ID_MEPC]    <= mepc_i;
                reg_file[`CSR_ID_MCAUSE]  <= mcause_i;
                reg_file[`CSR_ID_MTVAL]   <= mtval_i;

                reg_file[`CSR_ID_SEPC]    <= sepc_i;
                reg_file[`CSR_ID_SCAUSE]  <= scause_i;
                reg_file[`CSR_ID_STVAL]   <= stval_i;

            end
            else if (we_i) begin
                reg_file[waddr_i] <= wdata_i;
            end 


            reg_file[`CSR_ID_MIP][7] <= core_time_irq_i;


            
            // not writing mcycle, then we increment it
            if (!(we_i && waddr_i == `CSR_ID_MCYCLE)) begin
                reg_file[`CSR_ID_MCYCLE] <= reg_file[`CSR_ID_MCYCLE] + 1;
                if (reg_file[`CSR_ID_MCYCLE] == 32'hffffffff) begin
                    reg_file[`CSR_ID_MCYCLEH] <= reg_file[`CSR_ID_MCYCLEH] + 1;
                end
            end
            // cycle
            reg_file[`CSR_ID_CYCLE] <= reg_file[`CSR_ID_CYCLE] + 1;
            if (reg_file[`CSR_ID_CYCLE] == 32'hffffffff) begin
                reg_file[`CSR_ID_CYCLEH] <= reg_file[`CSR_ID_CYCLEH] + 1;
            end
            // time
            reg_file[`CSR_ID_TIME] <= reg_file[`CSR_ID_TIME] + 1;
            if (reg_file[`CSR_ID_TIME] == 32'hffffffff) begin
                reg_file[`CSR_ID_TIMEH] <= reg_file[`CSR_ID_TIMEH] + 1;
            end
        end
    end

endmodule



module id_pipeline_regs (
    input wire        clk_i,
    input wire        rst_i,

    input wire        bubble_i,
    input wire        stall_i,

    output reg        nop_o,

    input wire [ 1:0] mode_i,
    output reg [ 1:0] mode_o,

    input wire [ 3:0] alu_op_i,
    output reg [ 3:0] alu_op_o,

    input wire [ 2:0] cmp_op_i,
    output reg [ 2:0] cmp_op_o,

    input wire [31:0] imm_i,
    output reg [31:0] imm_o,

    input wire [31:0] uimm_i,
    output reg [31:0] uimm_o,

    input wire [31:0] data_rs1_i,
    output reg [31:0] data_rs1_o,
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

    input wire        is_branch_i,
    output reg        is_branch_o,

    input wire [31:0] inst_pc_i,
    output reg [31:0] inst_pc_o,

    input wire [31:0] pc_plus4_i,
    output reg [31:0] pc_plus4_o,

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

    input wire [ 1:0] alu_a_mux_i,
    output reg [ 1:0] alu_a_mux_o,
    
    input wire [ 1:0] alu_b_mux_i,
    output reg [ 1:0] alu_b_mux_o,

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

            alu_op_o <= 4'b0;
            cmp_op_o <= 3'b0;
            imm_o <= 32'b0;
            uimm_o <= 32'b0;

            data_rs1_o <= 32'b0;
            data_rs2_o <= 32'b0;
            reg_rd_o <= 5'b0;

            id_csr_o <= 5'b0;
            data_csr_o <= 32'b0;
            csr_write_enable_o <= 1'b0;

            is_branch_o <= 1'b0;
            inst_pc_o <= 32'b0;
            pc_plus4_o <= 32'b0;

            mem_operation_o <= 1'b0;
            mem_write_enable_o <= 1'b0;
            mem_unsigned_ext_o <= 1'b0;
            rf_write_enable_o <= 1'b0;

            data_rd_mux_o <= 2'b0;
            byte_sel_o <= 4'b0;
            alu_a_mux_o <= 2'b0;
            alu_b_mux_o <= 2'b0;

            exception_o <= 1'b0;
            trap_mode_o <= 2'b0;
            mcause_o    <= 32'b0;
            mtval_o     <= 32'b0;

            medeleg_o   <= 32'b0;
            nop_o       <= 1'b0;

        end else begin
            if (stall_i) begin

            end else if(bubble_i) begin
                // mode_o <= 2'b00;

                alu_op_o <= 4'b0;
                cmp_op_o <= 3'b0;
                imm_o <= 32'b0;
                uimm_o <= 32'b0;

                data_rs1_o <= 32'b0;
                data_rs2_o <= 32'b0;
                reg_rd_o <= 5'b0;

                id_csr_o <= 5'b0;
                data_csr_o <= 32'b0;
                csr_write_enable_o <= 1'b0;

                is_branch_o <= 1'b0;
                inst_pc_o <= 32'b0;
                pc_plus4_o <= 32'b0;

                mem_operation_o <= 1'b0;
                mem_write_enable_o <= 1'b0;
                mem_unsigned_ext_o <= 1'b0;
                rf_write_enable_o <= 1'b0;

                data_rd_mux_o <= 2'b0;
                byte_sel_o <= 4'b0;
                alu_a_mux_o <= 2'b0;
                alu_b_mux_o <= 2'b0;

                exception_o <= 1'b0;
                trap_mode_o <= 2'b0;
                mcause_o    <= 32'b0;
                mtval_o     <= 32'b0;

                medeleg_o   <= 32'b0;
                nop_o       <= 1'b1;
            end else begin
                mode_o <= mode_i;

                alu_op_o <= alu_op_i;
                cmp_op_o <= cmp_op_i;
                imm_o <= imm_i;
                uimm_o <= uimm_i;

                data_rs1_o <= data_rs1_i;
                data_rs2_o <= data_rs2_i;
                reg_rd_o <= reg_rd_i;

                id_csr_o <= id_csr_i;
                data_csr_o <= data_csr_i;
                

                is_branch_o <= is_branch_i;
                inst_pc_o <= inst_pc_i;
                pc_plus4_o <= pc_plus4_i;

                mem_operation_o <= mem_operation_i;
                mem_unsigned_ext_o <= mem_unsigned_ext_i;

                if (!exception_i) begin
                    mem_write_enable_o <= mem_write_enable_i;
                    csr_write_enable_o <= csr_write_enable_i;
                    rf_write_enable_o <= rf_write_enable_i;
                end else begin // disable side effects on exception
                    mem_write_enable_o <= 1'b0;
                    csr_write_enable_o <= 1'b0;
                    rf_write_enable_o <= 1'b0;
                end

                data_rd_mux_o <= data_rd_mux_i;
                byte_sel_o <= byte_sel_i;
                alu_a_mux_o <= alu_a_mux_i;
                alu_b_mux_o <= alu_b_mux_i;

                exception_o <= exception_i;
                trap_mode_o <= trap_mode_i;
                mcause_o    <= mcause_i;
                mtval_o     <= mtval_i;

                medeleg_o   <= medeleg_i;
                nop_o       <= 1'b0;
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

    input wire        exe_next_exception_i,
    input wire        mem_next_exception_i,
    input wire        wb_prev_exception_i,

    output wire id_bubble_o
);

    assign id_bubble_o = wait_reg_i
                        || exe_next_exception_i
                        || mem_next_exception_i
                        || wb_prev_exception_i;

endmodule

module id_wait_reg_controller (
    input  wire  [ 4:0] reg_rs1_i,
    input  wire  [ 4:0] reg_rs2_i,
  
    input  wire  [31:0] data_rs1_i,
    input  wire  [31:0] data_rs2_i,
  
    // forward from exe
    input  wire  [ 4:0] exe_reg_rd_i,
    input  wire         exe_rf_write_enable_i,
    input  wire  [31:0] exe_alu_y_i,       // (arithmetic instruction)
    input  wire  [31:0] exe_data_csr_i,    // (csr instruction)
    input  wire  [31:0] exe_pc_plus4_i,    // (jal/jalr instruction)
    input  wire  [ 1:0] exe_data_rd_mux_i,
  
    // forward from mem 
    input  wire  [ 4:0] mem_reg_rd_i,
    input  wire         mem_rf_write_enable_i,
    input  wire  [31:0] mem_data_rd_i,    // after mux
    input  wire         mem_load_data_i,
    input  wire         mem_done_i,       // is memory operation done

    output logic [31:0] data_rs1_o,
    output logic [31:0] data_rs2_o,

    output logic        wait_reg_o
);
    logic wait_reg_rs1, wait_reg_rs2;
    assign wait_reg_o = wait_reg_rs1 | wait_reg_rs2;

    logic [31:0] exe_data_rd;

    mem_data_rd_mux exe_data_rd_mux_inst (
        .pc_i           (exe_pc_plus4_i),
        .alu_i          (exe_alu_y_i),
        .mem_i          (32'b0),
        .csr_i          (exe_data_csr_i),
        .data_rd_mux_i  (exe_data_rd_mux_i),
        .data_rd_o      (exe_data_rd)
    );

    always_comb begin
        // default
        data_rs1_o = data_rs1_i;
        data_rs2_o = data_rs2_i;
        wait_reg_rs1 = 1'b0;
        wait_reg_rs2 = 1'b0;

        // forward from mem
        if (mem_rf_write_enable_i) begin
            if (mem_load_data_i && !mem_done_i) begin // wait reg (load instruction)
                if (mem_reg_rd_i == reg_rs1_i) begin
                    wait_reg_rs1 = 1'b1;
                end
                if (mem_reg_rd_i == reg_rs2_i) begin
                    wait_reg_rs2 = 1'b1;
                end    
            end else begin
                // forward rs1 from mem (after mux)
                if (mem_reg_rd_i == reg_rs1_i) begin
                    data_rs1_o = mem_data_rd_i;
                    wait_reg_rs1 = 1'b0;
                end
                // forward rs2 from mem (after mux)
                if (mem_reg_rd_i == reg_rs2_i) begin
                    data_rs2_o = mem_data_rd_i;
                    wait_reg_rs2 = 1'b0;
                end
            end
        end

        // nearer instruction has higher priority
        if (exe_rf_write_enable_i) begin
            if(exe_data_rd_mux_i == `DATA_RD_MEM) begin // wait reg (load instruction)
                if (exe_reg_rd_i == reg_rs1_i) begin
                    wait_reg_rs1 = 1'b1;
                end
                if (exe_reg_rd_i == reg_rs2_i) begin
                    wait_reg_rs2 = 1'b1;
                end
            end else begin
                // forward rs1 from exe (after mux)
                if (exe_reg_rd_i == reg_rs1_i) begin
                    data_rs1_o = exe_data_rd;
                    wait_reg_rs1 = 1'b0;
                end
                // forward rs2 from exe (after mux)
                if (exe_reg_rd_i == reg_rs2_i) begin
                    data_rs2_o = exe_data_rd;
                    wait_reg_rs2 = 1'b0;
                end
            end
        end

    end

endmodule


