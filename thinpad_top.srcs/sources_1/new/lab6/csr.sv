`include "alu_define.sv"

module csr_shadow_register_forward(
    input  wire  [ 4:0] addr_i,
    input  wire  [31:0] data_i,
    output logic [31:0] data_o
);

    always_comb begin
        case (addr_i)
            `CSR_ID_SIP: begin
                data_o = `CSR_SHADOW_SIP_MASK     & data_i;
            end
            `CSR_ID_SIE: begin
                data_o = `CSR_SHADOW_SIE_MASK     & data_i;
            end
            `CSR_ID_SSTATUS: begin
                data_o = `CSR_SHADOW_SSTATUS_MASK & data_i;
            end
            default : begin
                data_o = data_i;
            end
        endcase
    end

endmodule