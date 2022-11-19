`include "alu_define.sv"


module csr_shadow_register_translation(
    input  wire  [11:0] addr_i,
    output logic [ 4:0] addr_o,
    output logic [31:0] mask_o // writable bits
);

    always_comb begin
        mask_o = 32'hffffffff;
        
        case(addr_i)
            `CSR_CYCLE    : begin addr_o = `CSR_ID_MCYCLE    ; end
            `CSR_TIME     : begin addr_o = `CSR_ID_TIME      ; end // mmio
            `CSR_CYCLEH   : begin addr_o = `CSR_ID_MCYCLEH   ; end
            `CSR_TIMEH    : begin addr_o = `CSR_ID_TIMEH     ; end // mmio

            `CSR_SSTATUS  : begin addr_o = `CSR_ID_MSTATUS  ; mask_o = `CSR_SHADOW_SSTATUS_MASK;end
            `CSR_SIE      : begin addr_o = `CSR_ID_MIE      ; mask_o = `CSR_SHADOW_SIE_MASK    ;end
            `CSR_SIP      : begin addr_o = `CSR_ID_MIP      ; mask_o = `CSR_SHADOW_SIP_MASK    ;end

            `CSR_STVEC    : begin addr_o = `CSR_ID_STVEC    ; end
            `CSR_SSCRATCH : begin addr_o = `CSR_ID_SSCRATCH ; end
            `CSR_SEPC     : begin addr_o = `CSR_ID_SEPC     ; end
            `CSR_SCAUSE   : begin addr_o = `CSR_ID_SCAUSE   ; end
            `CSR_STVAL    : begin addr_o = `CSR_ID_STVAL    ; end
            `CSR_SATP     : begin addr_o = `CSR_ID_SATP     ; end

            `CSR_MHARTID  : begin addr_o = `CSR_ID_MHARTID  ; end

            `CSR_MSTATUS  : begin addr_o = `CSR_ID_MSTATUS  ; end
            `CSR_MIE      : begin addr_o = `CSR_ID_MIE      ; end
            `CSR_MIP      : begin addr_o = `CSR_ID_MIP      ; end

            `CSR_MEDELEG  : begin addr_o = `CSR_ID_MEDELEG  ; end
            `CSR_MIDELEG  : begin addr_o = `CSR_ID_MIDELEG  ; end

            `CSR_MTVEC    : begin addr_o = `CSR_ID_MTVEC    ; end
            `CSR_MSCRATCH : begin addr_o = `CSR_ID_MSCRATCH ; end
            `CSR_MEPC     : begin addr_o = `CSR_ID_MEPC     ; end
            `CSR_MCAUSE   : begin addr_o = `CSR_ID_MCAUSE   ; end
            `CSR_MTVAL    : begin addr_o = `CSR_ID_MTVAL    ; end
            

            `CSR_MCYCLE   : begin addr_o = `CSR_ID_MCYCLE   ; end
            `CSR_MCYCLEH  : begin addr_o = `CSR_ID_MCYCLEH  ; end

            `CSR_PMPCFG0  : begin addr_o = `CSR_ID_PMPCFG0  ; end
            `CSR_PMPADDR0 : begin addr_o = `CSR_ID_PMPADDR0 ; end
            default       : begin addr_o = `CSR_ID_UNKNOWN  ; end
        endcase
    end


endmodule