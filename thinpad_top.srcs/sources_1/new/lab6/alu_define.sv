`define ALU_ADD  4'b0000
`define ALU_SUB  4'b1000
`define ALU_SLL  4'b0001
`define ALU_SLT  4'b0010
`define ALU_SLTU 4'b0011
`define ALU_XOR  4'b0100
`define ALU_SRL  4'b0101
`define ALU_SRA  4'b1101
`define ALU_OR   4'b0110
`define ALU_AND  4'b0111
`define ALU_XNOR 4'b1100

// non-standard ALU operations
`define ALU_CLZ 4'b1001
`define ALU_MIN 4'b1010


`define CMP_EQ  3'b000
`define CMP_NE  3'b001
`define CMP_LT  3'b100
`define CMP_GE  3'b101
`define CMP_LTU 3'b110
`define CMP_GEU 3'b111
`define CMP_NONE 3'b011


`define DATA_RD_ALU      2'b00
`define DATA_RD_MEM      2'b01
`define DATA_RD_PC_NEXT  2'b10