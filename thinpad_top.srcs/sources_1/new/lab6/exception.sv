`default_nettype none
`include "alu_define.sv"

module exp_mstatus_decoder (
    input  wire  [31:0] mstatus,
    input  wire  [31:0] mstatush,
    output wire         sie,    // Supervisor Interrupt Enable
    output wire         mie,    // Machine Interrupt Enable
    output wire         spie,   // Supervisor Previous Interrupt Enable
    output wire         ube,
    output wire         mpie,   // Machine Previous Interrupt Enable
    output wire         spp,    // Supervisor Previous Privilege (WARL)
    output wire  [ 1:0] vs,
    output wire  [ 1:0] mpp,    // Machine Previous Privilege (WARL)
    output wire  [ 1:0] fs,
    output wire  [ 1:0] xs,
    output wire         mprv, //?
    output wire         sum,
    output wire         mxr,
    output wire         tvm,
    output wire         tw,
    output wire         tsr,
    output wire         sd,
    output wire         sbe,
    output wire         mbe
);

    assign sie  = mstatus[1];
    assign mie  = mstatus[3];
    assign spie = mstatus[5];
    assign ube  = mstatus[6];
    assign mpie = mstatus[7];
    assign spp  = mstatus[8];
    assign vs   = mstatus[10:9];
    assign mpp  = mstatus[12:11];
    assign fs   = mstatus[14:13];
    assign xs   = mstatus[16:15];
    assign mprv = mstatus[17];
    assign sum  = mstatus[18];
    assign mxr  = mstatus[19];
    assign tvm  = mstatus[20];
    assign tw   = mstatus[21];
    assign tsr  = mstatus[22];
    assign sd   = mstatus[31];
    assign sbe  = mstatush[4];
    assign mbe  = mstatush[5];
endmodule

module exp_mstatus_encoder (
    input  wire         sie,
    input  wire         mie,
    input  wire         spie,
    input  wire         ube,
    input  wire         mpie,
    input  wire         spp,
    input  wire  [ 1:0] vs,
    input  wire  [ 1:0] mpp,
    input  wire  [ 1:0] fs,
    input  wire  [ 1:0] xs,
    input  wire         mprv,
    input  wire         sum,
    input  wire         mxr,
    input  wire         tvm,
    input  wire         tw,
    input  wire         tsr,
    input  wire         sd,
    input  wire         sbe,
    input  wire         mbe,
    output logic [31:0] mstatus,
    output logic [31:0] mstatush
);
    always_comb begin
        // all other bits are zero
        mstatus  = 32'b0;
        mstatush = 32'b0;

        mstatus[ 1]     = sie;
        mstatus[ 3]     = mie;
        mstatus[ 5]     = spie;
        mstatus[ 6]     = ube;
        mstatus[ 7]     = mpie;
        mstatus[ 8]     = spp;
        mstatus[10:9]   = vs;
        mstatus[12:11]  = mpp;
        mstatus[14:13]  = fs;
        mstatus[16:15]  = xs;
        mstatus[17]     = mprv;
        mstatus[18]     = sum;
        mstatus[19]     = mxr;
        mstatus[20]     = tvm;
        mstatus[21]     = tw;
        mstatus[22]     = tsr;
        mstatus[31]     = sd;
        mstatush[4]     = sbe;
        mstatush[5]     = mbe;
    end
    

endmodule

// for mie/mip
module exp_mix_decoder (
    input  wire  [ 31:0] mix,
    output logic         meix,
    output logic         msix,
    output logic         mtix,
    output logic         seix,
    output logic         ssix,
    output logic         stix
);
    always_comb begin
        meix = mix[11];
        msix = mix[3];
        mtix = mix[7];
        seix = mix[9];
        ssix = mix[1];
        stix = mix[5];
    end
endmodule

module exp_mcause_encoder (
    input  wire  [31:0] int_i,

    input  wire  [30:0] exception_code_i,
    output logic [31:0] mcause_o
);
    logic [30:0] int_exp_code;
    logic        interrupt;

    logic mei, msi, mti, sei, ssi, sti;

    exp_mix_decoder int_decoder(
        .mix(int_i),
        .meix(mei),
        .msix(msi),
        .mtix(mti),
        .seix(sei),
        .ssix(ssi),
        .stix(sti)
    );

    always_comb begin
        interrupt = int_i != 0;
        if(mei) int_exp_code = 11'd11;
        else if(msi) int_exp_code = 11'd3;
        else if(mti) int_exp_code = 11'd7;
        else if(sei) int_exp_code = 11'd9;
        else if(ssi) int_exp_code = 11'd1;
        else if(sti) int_exp_code = 11'd5;
        else int_exp_code = { 31{1'bx} };
    end
    always_comb begin
        mcause_o[31] = interrupt;
        mcause_o[30:0] = interrupt ? int_exp_code: exception_code_i;
    end

endmodule

module exp_exception_encoder (
    input  wire  [ 1:0] mode_i,
    input  wire  [31:0] medeleg_i,

    input  wire         inst_addr_bp,
    input  wire         inst_access_fault,
    input  wire         inst_page_fault,
    input  wire         inst_illegal,
    input  wire         inst_misaligned,
    input  wire         ecall,
    input  wire         ebreak,
    input  wire         lsa_addr_bp,       // load/store/AMO address breakpoint
    input  wire         load_misaligned,
    input  wire         sa_misaligned,     // store/AMO misaligned
    input  wire         load_page_fault,
    input  wire         sa_page_fault,     // store/AMO page fault
    input  wire         load_access_fault,
    input  wire         sa_access_fault,   // store/AMO access 

    input  wire  [31:0] addr_i,
    input  wire  [31:0] inst_i,

    output wire         exception_o,
    output logic [30:0] exception_code_o,
    output logic [ 1:0] exp_mode_o,
    output logic [31:0] mtval_o

);

    assign exception_o = inst_addr_bp 
                | inst_access_fault
                | inst_page_fault
                | inst_illegal
                | inst_misaligned
                | ecall
                | ebreak
                | lsa_addr_bp
                | load_misaligned
                | sa_misaligned
                | load_page_fault
                | sa_page_fault
                | load_access_fault
                | sa_access_fault;

    
    always_comb begin
        mtval_o = 32'b0;
             if (inst_addr_bp)      begin exception_code_o = 31'd3;  end
        else if (inst_access_fault) begin exception_code_o = 31'd1;  mtval_o = addr_i; end
        else if (inst_page_fault)   begin exception_code_o = 31'd12; mtval_o = addr_i; end 
        else if (inst_illegal)      begin exception_code_o = 31'd2;  mtval_o = inst_i; end
        else if (inst_misaligned)   begin exception_code_o = 31'd0;  mtval_o = addr_i; end
        else if (ecall)             begin exception_code_o = (mode_i == M_MODE) ? 11 : ((mode_i == S_MODE) ? 9 : 8); end
        else if (ebreak)            begin exception_code_o = 31'd3;  end
        else if (lsa_addr_bp)       begin exception_code_o = 31'd3;  end
        else if (load_misaligned)   begin exception_code_o = 31'd4;  mtval_o = addr_i; end
        else if (sa_misaligned)     begin exception_code_o = 31'd6;  mtval_o = addr_i; end
        else if (load_page_fault)   begin exception_code_o = 31'd13; mtval_o = addr_i; end 
        else if (sa_page_fault)     begin exception_code_o = 31'd15; mtval_o = addr_i; end 
        else if (load_access_fault) begin exception_code_o = 31'd5;  mtval_o = addr_i; end
        else if (sa_access_fault)   begin exception_code_o = 31'd7;  mtval_o = addr_i; end
        else                        begin exception_code_o = 31'd24; end 
    end

        // handle exception
    wire   delegated_to_supervisor;
    assign delegated_to_supervisor = (mode_i <= S_MODE) && (medeleg_i[exception_code_o] == 1'b1);

    assign exp_mode_o = delegated_to_supervisor ? S_MODE : M_MODE;
    



endmodule




// we are NOT delegate any interrupts to supervisor mode now
module exp_interrupt_encoder (
    input  wire  [ 1:0] mode_i,
    
    input  wire  [31:0] mideleg_i,
    input  wire  [31:0] mstatus_i, // mie, sie
    input  wire  [31:0] mip_i,
    input  wire  [31:0] mie_i,

    output wire  [31:0] int_o,
    output wire  [ 1:0] int_mode_o

);
    // handle interrupt
    wire  mstatus_mie;
    wire  mstatus_sie;

    exp_mstatus_decoder emd(
        .mstatus(mstatus_i),
        .mie(mstatus_mie),
        .sie(mstatus_sie)
    );

    wire real_mie;
    wire real_sie;

    assign real_mie = ((mode_i == M_MODE) && mstatus_mie) || mode_i < M_MODE;
    assign real_sie = ((mode_i == S_MODE) && mstatus_sie) || mode_i < S_MODE;

    wire m_mti, m_msi, m_mei;
    wire m_sti, m_ssi, m_sei;

    wire s_sti, s_ssi, s_sei;

    wire [31:0] mi; // machine all interrupts
    assign mi = mie_i & mip_i;

    wire [31:0] m_int;
    wire [31:0] s_int;
    assign m_int = mi & ~mideleg_i;
    assign s_int = mi & mideleg_i;
    

    exp_mix_decoder m_int_decoder(
        .mix(m_int), // not delegated
        .meix(m_mei),
        .msix(m_msi),
        .mtix(m_mti),
        .seix(m_sei),
        .ssix(m_ssi),
        .stix(m_sti)
    );

    exp_mix_decoder s_int_decoder(
        .mix(s_int), // delegated
        .seix(s_sei),
        .ssix(s_ssi),
        .stix(s_sti)
    );


    wire machine_interrupt;
    wire supervisor_interrupt;
    assign machine_interrupt = real_mie && (m_mei | m_msi | m_mti | m_sei | m_ssi | m_sti);
    assign supervisor_interrupt = real_sie &&  (s_sei | s_ssi | s_sti);

    wire [31:0] real_int;
    wire [ 1:0] int_mode;
    assign real_int    = machine_interrupt ? m_int : supervisor_interrupt ? s_int : 32'b0;
    assign int_mode    = machine_interrupt ? M_MODE : supervisor_interrupt ? S_MODE : 2'bxx; // should not be used if there's not interrupt


    assign int_o       = real_int;
    assign int_mode_o  = int_mode;

endmodule

module exp_next_mode_mcause_encoder (
    input  wire  [31:0] int_i,
    input  wire  [ 1:0] int_mode_i,
    input  wire  [31:0] exception_code_i,
    input  wire  [ 1:0] exp_mode_i,

    output wire  [31:0] mcause_o,
    output wire  [ 1:0] next_mode_o
);

    exp_mcause_encoder mcause_encoder_inst (
        .int_i(int_i),
        .exception_code_i(exception_code_i),
        .mcause_o(mcause_o)
    );

    assign next_mode_o = (int_i != 32'b0) ? int_mode_i : exp_mode_i;

endmodule