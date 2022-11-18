// alu operations

`define ALU_ADD   4'b0000
`define ALU_SUB   4'b1000
`define ALU_SLL   4'b0001
`define ALU_SLT   4'b0010
`define ALU_SLTU  4'b0011
`define ALU_XOR   4'b0100
`define ALU_SRL   4'b0101
`define ALU_SRA   4'b1101
`define ALU_OR    4'b0110
`define ALU_AND   4'b0111
`define ALU_XNOR  4'b1100

// non-standard ALU operations
`define ALU_CLZ   4'b1001
`define ALU_MIN   4'b1010
`define ALU_CLR   4'b1011
`define ALU_USE_A 4'b1110

// comparator operations
`define CMP_EQ   3'b000
`define CMP_NE   3'b001
`define CMP_LT   3'b100
`define CMP_GE   3'b101
`define CMP_LTU  3'b110
`define CMP_GEU  3'b111
`define CMP_NONE 3'b011

// alu_a mux
`define ALU_A_RS1  2'b00
`define ALU_A_PC   2'b01
`define ALU_A_UIMM 2'b10

// alu_b mux
`define ALU_B_RS2 2'b00
`define ALU_B_IMM 2'b01
`define ALU_B_CSR 2'b10

// data mux
`define DATA_RD_ALU      2'b00
`define DATA_RD_MEM      2'b01
`define DATA_RD_PC_NEXT  2'b10
`define DATA_RD_CSR      2'b11

// csr
`define CSR_CYCLE    12'hC00 // read-only
`define CSR_TIME     12'hC01 // read-only
`define CSR_CYCLEH   12'hC80 // read-only
`define CSR_TIMEH    12'hC81 // read-only


`define CSR_SSTATUS  12'h100
`define CSR_SIE      12'h104
`define CSR_STVEC    12'h105

`define CSR_SSCRATCH 12'h140
`define CSR_SEPC     12'h141
`define CSR_SCAUSE   12'h142
`define CSR_STVAL    12'h143
`define CSR_SIP      12'h144

`define CSR_SATP     12'h180

`define CSR_MHARTID  12'hf14 // read-only

`define CSR_MSTATUS  12'h300
`define CSR_MEDELEG  12'h302
`define CSR_MIDELEG  12'h303
`define CSR_MIE      12'h304
`define CSR_MTVEC    12'h305

`define CSR_MSCRATCH 12'h340
`define CSR_MEPC     12'h341
`define CSR_MCAUSE   12'h342
`define CSR_MTVAL    12'h343
`define CSR_MIP      12'h344

`define CSR_MCYCLE   12'hb00
`define CSR_MCYCLEH  12'hb80

`define CSR_PMPCFG0  12'h3a0
`define CSR_PMPADDR0 12'h3b0

// csr id(for register file)
`define CSR_ID_CYCLE    5'd0 // TODO: should not exist actually (shadow of mcycle)      // read only
`define CSR_ID_TIME     5'd1 // TODO: should not exist actually (shadow of mmio-mtime)  // read only
`define CSR_ID_CYCLEH   5'd2 // TODO: should not exist actually (shadow of mcycleh)     // read only
`define CSR_ID_TIMEH    5'd3 // TODO: should not exist actually (shadow of mmio-mtimeH) // read only

`define CSR_ID_SSTATUS  5'd4 // TODO: should not exist actually (shadow of mstatus)
`define CSR_ID_SIE      5'd5 // TODO: should not exist actually (shadow of mie)
`define CSR_ID_STVEC    5'd6

`define CSR_ID_SSCRATCH 5'd7
`define CSR_ID_SEPC     5'd8
`define CSR_ID_SCAUSE   5'd9
`define CSR_ID_STVAL    5'd10
`define CSR_ID_SIP      5'd11 // TODO: should not exist actually (shadow of mip)

`define CSR_ID_SATP     5'd12

`define CSR_ID_MHARTID  5'd13 // read-only

`define CSR_ID_MSTATUS  5'd14
`define CSR_ID_MEDELEG  5'd15
`define CSR_ID_MIDELEG  5'd16
`define CSR_ID_MIE      5'd17
`define CSR_ID_MTVEC    5'd18

`define CSR_ID_MSCRATCH 5'd19
`define CSR_ID_MEPC     5'd20
`define CSR_ID_MCAUSE   5'd21
`define CSR_ID_MTVAL    5'd22
`define CSR_ID_MIP      5'd23

`define CSR_ID_MCYCLE   5'd24
`define CSR_ID_MCYCLEH  5'd25

`define CSR_ID_PMPCFG0  5'd26
`define CSR_ID_PMPADDR0 5'd27

`define CSR_ID_UNKNOWN  5'd31

`ifndef MODE_T_DEFINE
`define MODE_T_DEFINE

typedef enum logic [1:0] {
    M_MODE = 2'b11,
    H_MODE = 2'b10,
    S_MODE = 2'b01,
    U_MODE = 2'b00
} mode_t;

`endif

`define EXP_MRET        32'd24
`define EXP_SRET        32'd25


`define MSTATUS_SIE    5'd1
`define MSTATUS_SPIE   5'd5
`define MSTATUS_MIE    5'd3
`define MSTATUS_MPIE   5'd7
`define MSTATUS_SPP    5'd8
`define MSTATUS_MPP_H  5'd12
`define MSTATUS_MPP_L  5'd11

`define CSR_MTIP_MASK           32'b0000_0000_0000_0000_0000_0000_1000_0010

`define CSR_SHADOW_SIP_MASK     32'b0000_0000_0000_0000_0000_0010_0010_0010
`define CSR_SHADOW_SIE_MASK     32'b0000_0000_0000_0000_0000_0010_0010_0010
`define CSR_SHADOW_SSTATUS_MASK 32'b1000_0000_0000_1101_1110_0111_0110_0010