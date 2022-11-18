`include "alu_define.sv"

`define PAGESIZE 4096

`define PTESIZE  4
`define LEVELS   2

`define PTE_V    0
`define PTE_R    1
`define PTE_W    2
`define PTE_X    3
`define PTE_U    4
`define PTE_PPN  31:10


module mmu_sv32(
    // clk and reset
    input  wire         clk_i,
    input  wire         rst_i,
 
    input  wire  [31:0] satp_i, // IM and DM will not request with different modes
    input  wire  [ 1:0] mode_i,
    output logic        page_fault_o,
    

    // from IM-DM-bus
    // wishbone slave interface
    input  wire         wbs_cyc_i,
    input  wire         wbs_stb_i,
    output logic        wbs_ack_o,
    input  wire  [31:0] wbs_adr_i,
    input  wire  [31:0] wbs_dat_i, // if this is not zero, then it is a execution request
    output logic [31:0] wbs_dat_o,
    input  wire  [ 3:0] wbs_sel_i,
    input  wire         wbs_we_i,

    // to shared bus
    // wishbone master interface
    output logic        wbm_cyc_o,
    output logic        wbm_stb_o,
    input  wire         wbm_ack_i,
    output logic [31:0] wbm_adr_o,
    output logic [31:0] wbm_dat_o,
    input  wire  [31:0] wbm_dat_i,
    output logic [ 3:0] wbm_sel_o,
    output logic        wbm_we_o
);

    typedef enum logic {
        BARE = 1'b0,
        SV32 = 1'b1
    } satp_mode_t;

    satp_mode_t satp_mode;
    assign satp_mode = satp_mode_t'(satp_i[31]);

    logic sv32_enable;
    assign sv32_enable = !(satp_mode == BARE | mode_i == M_MODE);

    logic         sv32_wbm_cyc_o;
    logic         sv32_wbm_stb_o;
    logic         sv32_wbs_ack_o;
    logic  [31:0] sv32_wbm_adr_o;
    logic  [31:0] sv32_wbm_dat_o;
    logic  [31:0] sv32_wbs_dat_o;
    logic  [ 3:0] sv32_wbm_sel_o;
    logic         sv32_wbm_we_o;

    always_comb begin
        if (!sv32_enable) begin
            // direct mapping
            wbm_cyc_o = wbs_cyc_i;
            wbm_stb_o = wbs_stb_i;
            wbs_ack_o = wbm_ack_i;
            wbm_adr_o = wbs_adr_i;
            wbm_dat_o = wbs_dat_i;
            wbs_dat_o = wbm_dat_i;
            wbm_sel_o = wbs_sel_i;
            wbm_we_o  = wbs_we_i;
        end else begin
            // do sv32 translation
            wbm_cyc_o = sv32_wbm_cyc_o;
            wbm_stb_o = sv32_wbm_stb_o;
            wbs_ack_o = sv32_wbs_ack_o;
            wbm_adr_o = sv32_wbm_adr_o;
            wbm_dat_o = sv32_wbm_dat_o;
            wbs_dat_o = sv32_wbs_dat_o;
            wbm_sel_o = sv32_wbm_sel_o;
            wbm_we_o  = sv32_wbm_we_o;
        end

    end

    logic [31:0] sv32_pt;
    // assign sv32_pt = satp_i[21:0]*PAGESIZE;

    logic [9:0] sv32_vpn [0:1];
    assign sv32_vpn[1] = wbs_adr_i[31:22];
    assign sv32_vpn[0] = wbs_adr_i[21:12];

    logic [1:0] sv32_level;
    



    logic check_bit;
    assign check_bit = wbs_we_i ? `PTE_W : wbs_dat_i[0] ? `PTE_X : `PTE_R;

    // Note: this is not optimized for simplicity
    typedef enum logic [3:0] {
        IDLE,
        FETCH_PTE,
        WAIT_FETCH_PTE,
        MEM_OPERATION,
        WAIT_MEM_OPERATION,
        DONE

    } sv32_state_t;

    sv32_state_t state;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            state <= IDLE;
            page_fault_o <= 0;
            sv32_wbm_cyc_o <= 0;
            sv32_wbm_stb_o <= 0;
            sv32_wbm_adr_o <= 0;
            sv32_wbm_dat_o <= 0;
            sv32_wbs_dat_o <= 0;
            sv32_wbm_sel_o <= 0;
            sv32_wbm_we_o  <= 0;

        end else begin
            case (state)
                IDLE: begin
                    if (wbs_cyc_i && wbs_stb_i && sv32_enable) begin
                        state      <= FETCH_PTE;
                        sv32_pt    <= satp_i[21:0] * `PAGESIZE;
                        sv32_level <= `LEVELS-1;
                    end
                end
                FETCH_PTE: begin
                    state          <= WAIT_FETCH_PTE;
                    sv32_wbm_cyc_o <= 1'b1;
                    sv32_wbm_stb_o <= 1'b1;

                    sv32_wbm_adr_o <= sv32_pt + sv32_vpn[sv32_level] * `PTESIZE;
                    sv32_wbm_dat_o <= 32'b0;

                    sv32_wbm_sel_o <= 4'b1111;
                    sv32_wbm_we_o  <= 1'b0;
                end
                WAIT_FETCH_PTE: begin
                    if (wbm_ack_i) begin
                        if (wbm_dat_i[`PTE_V] == 1'b0 | (~wbm_dat_i[`PTE_R] & wbm_dat_i[`PTE_W])) begin
                            state        <= DONE;
                            page_fault_o <= 1'b1;
                        end else begin
                            if (wbm_dat_i[`PTE_R] | wbm_dat_i[`PTE_X]) begin // leaf
                                if (wbm_dat_i[check_bit] == 1'b0) begin
                                    state        <= DONE;
                                    page_fault_o <= 1'b1;
                                end else if (~wbm_dat_i[`PTE_U] & mode_i == U_MODE) begin
                                    state        <= DONE;
                                    page_fault_o <= 1'b1;
                                end else begin
                                    state        <= MEM_OPERATION;
                                end
                            end else begin
                                if (sv32_level == 0) begin
                                    state        <= DONE;
                                    page_fault_o <= 1'b1;
                                end else begin
                                    state      <= FETCH_PTE;
                                    sv32_level <= sv32_level - 1;
                                    sv32_pt    <= wbm_dat_i[`PTE_PPN] * `PAGESIZE;
                                end
                            end

                        end
                    end
                end
                MEM_OPERATION: begin
                    // TODO: not check misaligned superpage
                    // TODO: not check A/D bits
                    state          <= WAIT_MEM_OPERATION;
                    sv32_wbm_cyc_o <= 1'b1;
                    sv32_wbm_stb_o <= 1'b1;
                    sv32_wbm_adr_o <= {wbm_dat_i[`PTE_PPN] , wbs_adr_i[11:0]};
                    sv32_wbm_dat_o <= wbs_dat_i;
                    sv32_wbm_sel_o <= wbs_sel_i;
                    sv32_wbm_we_o  <= wbs_we_i;
                end
                WAIT_MEM_OPERATION: begin
                    if (wbm_ack_i) begin
                        sv32_wbs_dat_o <= wbm_dat_i;
                        state          <= DONE;
                    end
                end
                DONE: begin
                    state <= IDLE;
                    page_fault_o <= 1'b0;
                end
            endcase
        end
    end

    always_comb begin
        if (state == DONE) begin
            sv32_wbs_ack_o = 1'b1;
        end else begin
            sv32_wbs_ack_o = 1'b0;
        end
    end

endmodule