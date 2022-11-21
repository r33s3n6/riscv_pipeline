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


module simple_buffer(
    input  wire         clk_i,
    input  wire         rst_i,

    input  wire  [31:0] data_i,
    input  wire         write_enable_i,
    output wire  [31:0] data_o

);
    
    reg [31:0] temp_data;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            temp_data <= 32'h0;
        end else if (write_enable_i) begin
            temp_data <= data_i;
        end
    end

    assign data_o = write_enable_i ? data_i : temp_data;

endmodule

// simple, read in 1 cycle
module tlb_sv32 #(
    parameter ENTRIES   = 4,
    parameter IDX_WIDTH = $clog2(ENTRIES)
) (
    input  wire         clk_i,
    input  wire         rst_i,

    input  wire         enable_i,
    input  wire         write_enable_i,
    input  wire         clear_i,

    input  wire  [19:0] vpn_i,
    input  wire  [21:0] ppn_i, // for write
    input  wire  [ 7:0] perm_i, // for write

    output logic [21:0] ppn_o,
    output logic [ 7:0] perm_o,
    output logic        hit_o
);
    
    logic [19:0] vpn   [0:ENTRIES-1];
    logic [21:0] ppn   [0:ENTRIES-1];
    logic [ 7:0] perm  [0:ENTRIES-1];
    logic        valid [0:ENTRIES-1];

    logic [IDX_WIDTH-1:0] hit_idx;
    
    assign ppn_o  = ppn[hit_idx];
    assign perm_o = perm[hit_idx];

    always_comb begin
        hit_idx = {IDX_WIDTH{1'b1}};
        hit_o   = 1'b0;
        for (int i = 0; i < ENTRIES; i = i + 1) begin
            if (vpn_i == vpn[i] & valid[i]) begin
                hit_idx = i;
                hit_o   = 1'b1;
                break;
            end
        end
    end

    // put hit entry at the top
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            vpn   <= '{default: '0};
            ppn   <= '{default: '0};
            perm  <= '{default: '0};
            valid <= '{default: '0};
        end else begin
            if (clear_i) begin
                vpn   <= '{default: '0};
                ppn   <= '{default: '0};
                perm  <= '{default: '0};
                valid <= '{default: '0};
            end else if (enable_i | write_enable_i) begin // LRU
                if (hit_o | write_enable_i) begin
                    for (int i = 1; i <= hit_idx; i = i + 1) begin
                        vpn[i]   <= vpn[i-1];
                        ppn[i]   <= ppn[i-1];
                        perm[i]  <= perm[i-1];
                        valid[i] <= valid[i-1];
                    end

                    if (write_enable_i) begin 
                        vpn [0]  <= vpn_i;
                        ppn [0]  <= ppn_i;
                        perm[0]  <= perm_i;
                        valid[0] <= 1'b1;
                    end else begin // hit
                        vpn [0]  <= vpn[hit_idx];
                        ppn [0]  <= ppn[hit_idx];
                        perm[0]  <= perm[hit_idx];
                        valid[0] <= 1'b1;
                    end

                end
            end

            
        end
    end



endmodule


// TODO: rewrite logic, pass through if tlb hit, or fall back to memory
module mmu_sv32(
    // clk and reset
    input  wire         clk_i,
    input  wire         rst_i,
 
    input  wire  [31:0] satp_i, // IM and DM will not request with different modes
    input  wire         sum_i, 
    input  wire  [ 1:0] mode_i,

    input  wire         tlb_clear_i,
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
    output logic        wbm_we_o,

    output logic [ 1:0] debug_mmu_state_o,
    output logic [31:0] debug_mmu_pf_pte_o,
    output logic [ 3:0] debug_mmu_pf_cause_o,
    input  wire         debug_tlb_enable_i,

    output logic [31:0] debug_paddr_o,
    output logic        debug_tlb_hit_o,
    output logic [ 1:0] debug_pte_state_o
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

    // Note: this is not optimized for simplicity
    typedef enum logic [1:0] {
        IDLE,
        FETCH_PTE,
        MEM_OPERATION

    } sv32_state_t;
    sv32_state_t state, state_next;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            state <= IDLE;
        end else begin
            state <= state_next;
        end
    end
    
    // sv32 parameters
    logic [ 9:0] sv32_vpn [0:1];
    logic [19:0] sv32_all_vpn;
    assign sv32_vpn[1] = wbs_adr_i[31:22];
    assign sv32_vpn[0] = wbs_adr_i[21:12];

    assign sv32_all_vpn = wbs_adr_i[31:12];


    // pte check signals
    logic [3:0] sv32_check_bit;
    assign sv32_check_bit = wbs_we_i ? `PTE_W : wbs_dat_i[0] ? `PTE_X : `PTE_R;

    logic  sv32_request_from_master;
    logic  sv32_response_from_slave;

    assign sv32_request_from_master = sv32_enable & wbs_cyc_i & wbs_stb_i;

    assign sv32_response_from_slave  = sv32_enable & wbm_ack_i;


    // tlb signals
    logic        sv32_tlb_enable;
    logic [19:0] sv32_tlb_vpn;
    logic [21:0] sv32_tlb_ppn_i;
    logic [ 7:0] sv32_tlb_perm_i;
    logic [21:0] sv32_tlb_ppn_o;
    logic [ 7:0] sv32_tlb_perm_o;
    logic        sv32_tlb_hit_o;
    logic        sv32_tlb_write_enable;

    tlb_sv32 tlb_sv32_inst (
        .clk_i          (clk_i),
        .rst_i          (rst_i),
        .enable_i       (sv32_tlb_enable),
        .clear_i        (tlb_clear_i),
        .vpn_i          (sv32_tlb_vpn),
        .ppn_i          (sv32_tlb_ppn_i),
        .perm_i         (sv32_tlb_perm_i),
        .hit_o          (sv32_tlb_hit_o),
        .ppn_o          (sv32_tlb_ppn_o),
        .perm_o         (sv32_tlb_perm_o),
        .write_enable_i (sv32_tlb_write_enable)
    );
    logic  sv32_tlb_hit;
    assign sv32_tlb_hit =  debug_tlb_enable_i & sv32_tlb_hit_o;

    assign sv32_tlb_vpn    = sv32_all_vpn;
    assign sv32_tlb_enable = (state == IDLE) & sv32_request_from_master;


    // walk status
    logic [31:0] sv32_pt;
    logic [ 1:0] sv32_level;

    // sv32_wbm_* signals

    logic        sv32_wbm_cyc_buf;
    logic        sv32_wbm_stb_buf;
    logic [31:0] sv32_wbm_adr_buf;
    logic [31:0] sv32_wbm_dat_buf;
    logic [ 3:0] sv32_wbm_sel_buf;
    logic        sv32_wbm_we_buf;

    logic [31:0] sv32_pt_buf;   
    logic [ 1:0] sv32_level_buf;

    logic        sv32_wbm_cyc_next;
    logic        sv32_wbm_stb_next;
    logic [31:0] sv32_wbm_adr_next;
    logic [31:0] sv32_wbm_dat_next;
    logic [ 3:0] sv32_wbm_sel_next;
    logic        sv32_wbm_we_next;

    logic [31:0] sv32_pt_next;   
    logic [ 1:0] sv32_level_next;

    logic        sv32_wbm_forward;

    assign sv32_wbm_forward = 0;

    // if we at state idle, we can immediately start new request by forwarding
    assign sv32_wbm_cyc_o = sv32_wbm_forward ? sv32_wbm_cyc_next : sv32_wbm_cyc_buf;
    assign sv32_wbm_stb_o = sv32_wbm_forward ? sv32_wbm_stb_next : sv32_wbm_stb_buf;
    assign sv32_wbm_adr_o = sv32_wbm_forward ? sv32_wbm_adr_next : sv32_wbm_adr_buf;
    assign sv32_wbm_dat_o = sv32_wbm_forward ? sv32_wbm_dat_next : sv32_wbm_dat_buf;
    assign sv32_wbm_sel_o = sv32_wbm_forward ? sv32_wbm_sel_next : sv32_wbm_sel_buf;
    assign sv32_wbm_we_o  = sv32_wbm_forward ? sv32_wbm_we_next  : sv32_wbm_we_buf;

    assign sv32_pt        = sv32_wbm_forward ? sv32_pt_next      : sv32_pt_buf;
    assign sv32_level     = sv32_wbm_forward ? sv32_level_next   : sv32_level_buf;


    logic sv32_wbm_write_enable;
    
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            sv32_wbm_cyc_buf <= 1'b0;
            sv32_wbm_stb_buf <= 1'b0;
            sv32_wbm_adr_buf <= 32'h0;
            sv32_wbm_dat_buf <= 32'h0;
            sv32_wbm_sel_buf <= 4'h0;
            sv32_wbm_we_buf  <= 1'b0;

            sv32_pt_buf      <= 32'h0;
            sv32_level_buf   <= 0;
        end else begin

            if (sv32_wbm_write_enable) begin
                sv32_wbm_cyc_buf <= sv32_wbm_cyc_next;
                sv32_wbm_stb_buf <= sv32_wbm_stb_next;
                sv32_wbm_adr_buf <= sv32_wbm_adr_next;
                sv32_wbm_dat_buf <= sv32_wbm_dat_next;
                sv32_wbm_sel_buf <= sv32_wbm_sel_next;
                sv32_wbm_we_buf  <= sv32_wbm_we_next;
            end

            sv32_pt_buf      <= sv32_pt_next;
            sv32_level_buf   <= sv32_level_next;

        end
    end


    logic [31:0] sv32_pte;
    assign sv32_pte = (sv32_enable & sv32_tlb_hit) ? {sv32_tlb_ppn_o, 2'b00, sv32_tlb_perm_o} : wbm_dat_i;


    /* pte_state begin */
    // for permission check
    logic [7:0] sv32_pte_perm;
    assign sv32_pte_perm = sv32_pte[7:0];

    typedef enum logic [1:0] {
        OK,  // permission check ok
        PF,  // page fault
        NEXT // next level
    } sv32_pte_state_t;

    sv32_pte_state_t sv32_pte_state;

    logic [3:0] debug_sv32_pte_state_cause;


    always_comb begin
        // TODO: not check misaligned superpage
        // TODO: not check A/D bits
        sv32_pte_state = PF;
        debug_sv32_pte_state_cause = 4'hf;
        
        if (sv32_pte_perm[`PTE_V] == 1'b0 | (~sv32_pte_perm[`PTE_R] & sv32_pte_perm[`PTE_W])) begin
            sv32_pte_state = PF;
            debug_sv32_pte_state_cause = 4'h1;

        end else begin
            if (sv32_pte_perm[`PTE_R] | sv32_pte_perm[`PTE_X]) begin // leaf
                if (sv32_pte_perm[sv32_check_bit] == 1'b0) begin
                    sv32_pte_state = PF;
                    debug_sv32_pte_state_cause = 4'h2;

                end else if (~sv32_pte_perm[`PTE_U] & mode_i == U_MODE) begin
                    sv32_pte_state = PF;
                    debug_sv32_pte_state_cause = 4'h3;

                end else if (sv32_pte_perm[`PTE_U] & mode_i == S_MODE & ~sum_i) begin
                    sv32_pte_state = PF;
                    debug_sv32_pte_state_cause = 4'h4;

                end else begin
                    sv32_pte_state = OK;
                end
            end else begin
                if (sv32_level == 0) begin
                    sv32_pte_state = PF;
                    debug_sv32_pte_state_cause = 4'h5;

                end else begin
                    sv32_pte_state = NEXT;
                end
            end

        end
    end

    /* pte_state end*/

    // as slave

    simple_buffer sv32_output_buffer(
        .clk_i          (clk_i),
        .rst_i          (rst_i),
        .data_i         (wbm_dat_i),
        .data_o         (sv32_wbs_dat_o),
        .write_enable_i (wbs_ack_o) // TODO: & ~sv32_wbm_we_o
    );

    

    // state_next, page_fault
    always_comb begin

        page_fault_o          = 1'b0;
        state_next            = state;
        sv32_wbs_ack_o        = 1'b0;
        sv32_wbm_write_enable = 1'b1;

        case (state)
            IDLE: begin
                if (sv32_request_from_master) begin // new request
                    if(sv32_tlb_hit) begin
                        if (sv32_pte_state == OK) begin
                            state_next = MEM_OPERATION; 
                        end else if (sv32_pte_state == PF) begin
                            state_next = IDLE;
                        end
                    end else begin
                        state_next = FETCH_PTE;
                    end  

                end else begin
                    sv32_wbm_write_enable = 1'b0;
                end
            end
            FETCH_PTE: begin
                
                if (wbm_ack_i) begin
                    if(sv32_pte_state == NEXT) begin
                        state_next = FETCH_PTE;
                    end else if (sv32_pte_state == OK) begin
                        sv32_wbm_write_enable = 1'b1;
                        state_next = MEM_OPERATION;
                    end else if (sv32_pte_state == PF) begin
                        state_next     = IDLE;
                        sv32_wbs_ack_o = 1'b1;
                        page_fault_o   = 1'b1;
                    end
                end else begin
                    sv32_wbm_write_enable = 1'b0;
                end
            end

            MEM_OPERATION: begin
                if (wbm_ack_i) begin
                    state_next = IDLE;
                    sv32_wbs_ack_o = 1'b1;
                end else begin 
                    sv32_wbm_write_enable = 1'b0;
                end
            end


        endcase
    end

     

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            sv32_tlb_ppn_i        <= 22'h0;
            sv32_tlb_perm_i       <= 8'h0; 
            sv32_tlb_write_enable <= 1'b0;   
        end else begin
            if ((state == FETCH_PTE) & (sv32_pte_state == OK) & wbm_ack_i) begin
                sv32_tlb_ppn_i        <= wbm_dat_i[`PTE_PPN];
                sv32_tlb_perm_i       <= wbm_dat_i[7:0];
                sv32_tlb_write_enable <= 1'b1; // write tlb
            end else 
                sv32_tlb_write_enable <= 1'b0;
            
            
        end
    end


    always_comb begin
        sv32_pt_next    = sv32_pt_buf;
        sv32_level_next = sv32_level_buf;

        case (state)
            IDLE: begin
                if (sv32_request_from_master) begin // new request
                    if(!sv32_tlb_hit) begin
                        sv32_pt_next    = satp_i[21:0] * `PAGESIZE;
                        sv32_level_next = `LEVELS-1;
                    end
                end
            end
            FETCH_PTE: begin
                if (wbm_ack_i) begin
                    if(sv32_pte_state == NEXT) begin
                        sv32_pt_next    = wbm_dat_i[`PTE_PPN] * `PAGESIZE;
                        sv32_level_next = sv32_level - 1;
                    end 
                end
            end
        endcase

    end




    // wbm signals
    always_comb begin
        sv32_wbm_cyc_next =  1'b0;
        sv32_wbm_stb_next =  1'b0;
        sv32_wbm_adr_next = 32'b0;
        sv32_wbm_dat_next = 32'b0;
        sv32_wbm_sel_next =  4'b0;
        sv32_wbm_we_next  =  1'b0;

        case (state_next)
            IDLE: begin

            end
            FETCH_PTE: begin
                sv32_wbm_cyc_next = 1'b1;
                sv32_wbm_stb_next = 1'b1;
                sv32_wbm_adr_next = sv32_pt_next + sv32_vpn[sv32_level_next] * `PTESIZE;
                sv32_wbm_dat_next = 32'b0;
                sv32_wbm_sel_next = 4'b1111;
                sv32_wbm_we_next  = 1'b0;
            end
            MEM_OPERATION: begin
                sv32_wbm_cyc_next = 1'b1;
                sv32_wbm_stb_next = 1'b1;
                sv32_wbm_adr_next = {sv32_pte[`PTE_PPN] , wbs_adr_i[11:0]};
                sv32_wbm_dat_next = wbs_dat_i;
                sv32_wbm_sel_next = wbs_sel_i;
                sv32_wbm_we_next  = wbs_we_i;
            end
        endcase

    end

    assign debug_mmu_state_o    = state;
    assign debug_mmu_pf_cause_o = debug_sv32_pte_state_cause;
    assign debug_mmu_pf_pte_o   = sv32_pte;

    assign debug_paddr_o        = {sv32_pte[`PTE_PPN] , wbs_adr_i[11:0]};
    assign debug_tlb_hit_o      = sv32_tlb_hit;
    assign debug_pte_state_o    = sv32_pte_state;

endmodule



// global
// TODO: you may want forward if you change design(using cache)
module mmu_vm_status_regs(
    input  wire         clk_i,
    input  wire         rst_i,

    input  wire         write_enable_i,

    input  wire  [31:0] satp_i,
    input  wire  [ 1:0] mode_i,
    input  wire         sum_i,

    output wire  [31:0] satp_o,
    output wire  [ 1:0] mode_o,
    output wire         sum_o

);

    logic [31:0] satp_buf;
    logic [ 1:0] mode_buf;
    logic        sum_buf;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            satp_buf <= 32'b0;
            mode_buf <= M_MODE;
            sum_buf  <= 1'b0;
        end else begin
            if (write_enable_i) begin
                satp_buf <= satp_i;
                mode_buf <= mode_i;
                sum_buf  <= sum_i;
            end
        end
    end

    // assign satp_o = write_enable_i ? satp_i : satp_buf;
    // assign mode_o = write_enable_i ? mode_i : mode_buf;
    // assign sum_o  = write_enable_i ? sum_i  : sum_buf;

    assign satp_o = satp_buf;
    assign mode_o = mode_buf;
    assign sum_o  = sum_buf;

endmodule

/*
// you MUST provide valid signals BEFORE posedge clk_i
// and you can get valid data AFTER posedge clk_i
module mmu_memory_cache #(
    parameter CACHE_DATA_SIZE = 32, // bytes
    parameter CACHE_SETS      = 32  
    parameter CACHE_WAYS      = 4   
)
(
    input  wire         clk_i,
    input  wire         rst_i,

    input  wire         clear_i,

    input  wire         enable_i,
    input  wire         write_enable_i,
    input  wire  [31:0] addr_i,
    input  wire  [31:0] data_i,
    input  wire  [ 3:0] data_sel_i,
    
    output wire  [31:0] addr_o,
    output wire  [31:0] data_o,
    output wire         hit_o // when write, not hit means write back
);  

    // TODO: implement cache
    assign hit_o = 1'b0;
    assign addr_o = addr_i;
    assign data_o = data_i;
endmodule

// access memory with PHYSICAL address (mmu is placed above this module)
// you MUST provide valid signals BEFORE posedge clk_i
// and you can get valid data AFTER posedge clk_i
module mmu_physical_memory_interface #(
    parameter CACHE_DATA_SIZE = 32, // bytes
    parameter CACHE_SETS      = 32  
    parameter CACHE_WAYS      = 4   
)

(
    input  wire         clk_i,
    input  wire         rst_i,

    // control signals
    input  wire         no_cache_i,
    input  wire         enable_i,
    input  wire         write_enable_i,
    input  wire  [31:0] addr_i,
    input  wire  [31:0] data_i, // read request would also pass data_i to bus
    input  wire  [ 3:0] data_sel_i,
 
    output wire  [31:0] ack_addr_o,
    output wire         ack_o,
    output wire  [31:0] data_o,

    // wbm signals (as master) (fall back to wish bone bus)
    output wire         wb_cyc_o,
    output wire         wb_stb_o,
    input  wire         wb_ack_i,
    output wire  [31:0] wb_adr_o,
    output wire  [31:0] wb_dat_o,
    input  wire  [31:0] wb_dat_i,
    output wire  [ 3:0] wb_sel_o,
    output wire         wb_we_o
);
    
    typedef enum logic [2:0] {
        IDLE,
        CACHE_MISS,
        WAIT
    } state_t;

    state_t state, state_next;

    // input buffer
    logic        no_cache_i_buf;
    logic        enable_i_buf;
    logic        write_enable_i_buf;
    logic [31:0] addr_i_buf;
    logic [31:0] data_i_buf;
    logic [ 3:0] data_sel_i_buf;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            no_cache_i_buf     <= 1'b0;
            enable_i_buf       <= 1'b0;
            write_enable_i_buf <= 1'b0;
            addr_i_buf         <= 32'b0;
            data_i_buf         <= 32'b0;
            data_sel_i_buf     <= 4'b0;
        end else begin
            no_cache_i_buf     <= no_cache_i;
            enable_i_buf       <= enable_i;
            write_enable_i_buf <= write_enable_i;
            addr_i_buf         <= addr_i;
            data_i_buf         <= data_i;
            data_sel_i_buf     <= data_sel_i;
        end
    end

    // request buffer
    logic        wb_cyc_o_next;
    logic        wb_stb_o_next;
    logic [31:0] wb_adr_o_next;
    logic [31:0] wb_dat_o_next;
    logic [ 3:0] wb_sel_o_next;
    logic         wb_we_o_next;

    logic        wb_cyc_o_buf;
    logic        wb_stb_o_buf;
    logic [31:0] wb_adr_o_buf;
    logic [31:0] wb_dat_o_buf;
    logic [ 3:0] wb_sel_o_buf;
    logic         wb_we_o_buf;

    logic        wb_write_enable;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            wb_cyc_o_buf <= 1'b0;
            wb_stb_o_buf <= 1'b0;
            wb_adr_o_buf <= 32'b0;
            wb_dat_o_buf <= 32'b0;
            wb_sel_o_buf <= 4'b0;
            wb_we_o_buf  <= 1'b0;
        end else begin
            if (wb_write_enable) begin
                wb_cyc_o_buf <= wb_cyc_o_next;
                wb_stb_o_buf <= wb_stb_o_next;
                wb_adr_o_buf <= wb_adr_o_next;
                wb_dat_o_buf <= wb_dat_o_next;
                wb_sel_o_buf <= wb_sel_o_next;
                wb_we_o_buf  <= wb_we_o_next;
            end
        end
    end

    // forward signals
    assign wb_cyc_o = wb_write_enable ? wb_cyc_o_next : wb_cyc_o_buf;
    assign wb_stb_o = wb_write_enable ? wb_stb_o_next : wb_stb_o_buf;
    assign wb_adr_o = wb_write_enable ? wb_adr_o_next : wb_adr_o_buf;
    assign wb_dat_o = wb_write_enable ? wb_dat_o_next : wb_dat_o_buf;
    assign wb_sel_o = wb_write_enable ? wb_sel_o_next : wb_sel_o_buf;
    assign wb_we_o  = wb_write_enable ?  wb_we_o_next :  wb_we_o_buf;

    logic        cache_hit;
    logic [31:0] cache_data;


    // not using buffer
    mmu_memory_cache # (
        .CACHE_DATA_SIZE(CACHE_DATA_SIZE),
        .CACHE_SETS     (CACHE_SETS),
        .CACHE_WAYS     (CACHE_WAYS)
    ) cache (
        .clk_i          (clk_i),
        .rst_i          (rst_i),

        .clear_i        (clear_i),

        .enable_i       (enable_i),
        .write_enable_i (write_enable_i),
        .addr_i         (addr_i),
        .data_i         (data_i),
        .data_sel_i     (data_sel_i),

        .data_o         (cache_data),
        .hit_o          (cache_hit)
    );

    // logic wb_request;
    // assign wb_request = enable_i && !cache_hit;

    always_comb begin
        if (state == IDLE && wb_request) begin
            wb_write_enable = 1'b1;

            wb_cyc_o_next = 1'b1;
            wb_stb_o_next = 1'b1;
            wb_adr_o_next = addr_i_buf;
            wb_dat_o_next = data_i_buf;
            wb_sel_o_next = data_sel_i_buf;
            wb_we_o_next  = write_enable_i_buf;
        end else begin
            wb_cyc_o_next = wb_cyc_o_buf;
            wb_stb_o_next = wb_stb_o_buf;
            wb_adr_o_next = wb_adr_o_buf;
            wb_dat_o_next = wb_dat_o_buf;
            wb_sel_o_next = wb_sel_o_buf;
            wb_we_o_next  = wb_we_o_buf;
        end

    end
    

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            state <= IDLE;
        end else begin
            case(state)
                IDLE: begin
                    if (enable_i) begin
                        // cache got result next cycle
                    end
                end
                WAIT: begin
                    if (wb_ack_i) begin // no new request
                        if (wb_request) begin
                            state <= WAIT; // these request is sent
                        end else begin
                            state <= IDLE;
                        end
                    end
                end
            endcase
        end

    end

    // output buffer
    logic [31:0] ack_addr_o_buf;
    logic [31:0] data_o_buf;

    logic [31:0] ack_addr_o_next;
    logic [31:0] data_o_next;

    logic output_write_enable;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            ack_addr_o_buf <= 32'b0;
            data_o_buf     <= 32'b0;
        end else begin
            if (output_write_enable) begin
                ack_addr_o_buf <= ack_addr_o_next;
                data_o_buf     <= data_o_next;
            end
        end
    end

    assign ack_addr_o = output_write_enable ? ack_addr_o_next : ack_addr_o_buf;
    assign data_o     = output_write_enable ? data_o_next     : data_o_buf;

    always_comb begin
        if (wb_ack_i) begin
            output_write_enable = 1'b1;
            ack_addr_o_next     = wb_adr_i;
            data_o_next         = wb_dat_i;
        end else begin
            output_write_enable = 1'b0;
            ack_addr_o_next     = ack_addr_o_buf;
            data_o_next         = data_o_buf;
        end
    end




endmodule

// TODO: rewrite logic, pass through if tlb hit, or fall back to memory
module mmu_sv32(
    // clk and reset
    input  wire         clk_i,
    input  wire         rst_i,
 
    input  wire  [31:0] satp_i, // IM and DM will not request with different modes
    input  wire         sum_i, 
    input  wire  [ 1:0] mode_i,
    input  wire         tlb_clear_i,

    output logic        page_fault_o,



);



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

    // Note: this is not optimized for simplicity

    
    // sv32 parameters
    logic [ 9:0] sv32_vpn [0:1];
    logic [19:0] sv32_all_vpn;
    assign sv32_vpn[1] = wbs_adr_i[31:22];
    assign sv32_vpn[0] = wbs_adr_i[21:12];

    assign sv32_all_vpn = wbs_adr_i[31:12];


    // pte check signals
    logic [3:0] sv32_check_bit;
    assign sv32_check_bit = wbs_we_i ? `PTE_W : wbs_dat_i[0] ? `PTE_X : `PTE_R;

    logic  sv32_request_from_master;
    logic  sv32_response_from_slave;

    assign sv32_request_from_master = sv32_enable & wbs_cyc_i & wbs_stb_i;

    // Note: dangerous
    // assign sv32_response_from_slave  = sv32_enable & wbm_cyc_o & wbm_stb_o & wbm_ack_i;
    assign sv32_response_from_slave  = sv32_enable & wbm_ack_i;




    
    // buffer wbs result
    logic [31:0] sv32_sram_data;
    simple_buffer sv32_sram_buffer(
        .clk_i          (clk_i),
        .rst_i          (rst_i),
        .data_i         (wbm_dat_i),
        .data_o         (sv32_sram_data),
        .write_enable_i (sv32_response_from_slave ) // TODO: & ~sv32_wbm_we_o
    );



    always_comb begin
        sv32_tlb_vpn    = sv32_all_vpn;
        sv32_tlb_ppn_i  = sv32_sram_data[`PTE_PPN];
        sv32_tlb_perm_i = sv32_sram_data[7:0];
        sv32_tlb_enable = (state == IDLE) & sv32_request_from_master;
    end



    // walk status
    logic [31:0] sv32_pt;
    logic [ 1:0] sv32_level;

    // sv32_wbm_* signals

    logic        sv32_wbm_cyc_buf;
    logic        sv32_wbm_stb_buf;
    logic [31:0] sv32_wbm_adr_buf;
    logic [31:0] sv32_wbm_dat_buf;
    logic [ 3:0] sv32_wbm_sel_buf;
    logic        sv32_wbm_we_buf;

    logic [31:0] sv32_pt_buf;   
    logic [ 1:0] sv32_level_buf;

    logic        sv32_wbm_cyc_next;
    logic        sv32_wbm_stb_next;
    logic [31:0] sv32_wbm_adr_next;
    logic [31:0] sv32_wbm_dat_next;
    logic [ 3:0] sv32_wbm_sel_next;
    logic        sv32_wbm_we_next;

    logic [31:0] sv32_pt_next;   
    logic [ 1:0] sv32_level_next;

    logic        sv32_wbm_forward;


    // if we at state idle, we can immediately start new request by forwarding
    assign sv32_wbm_cyc_o = sv32_wbm_forward ? sv32_wbm_cyc_next : sv32_wbm_cyc_buf;
    assign sv32_wbm_stb_o = sv32_wbm_forward ? sv32_wbm_stb_next : sv32_wbm_stb_buf;
    assign sv32_wbm_adr_o = sv32_wbm_forward ? sv32_wbm_adr_next : sv32_wbm_adr_buf;
    assign sv32_wbm_dat_o = sv32_wbm_forward ? sv32_wbm_dat_next : sv32_wbm_dat_buf;
    assign sv32_wbm_sel_o = sv32_wbm_forward ? sv32_wbm_sel_next : sv32_wbm_sel_buf;
    assign sv32_wbm_we_o  = sv32_wbm_forward ? sv32_wbm_we_next  : sv32_wbm_we_buf;

    assign sv32_pt        = sv32_wbm_forward ? sv32_pt_next      : sv32_pt_buf;
    assign sv32_level     = sv32_wbm_forward ? sv32_level_next   : sv32_level_buf;

    
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            sv32_wbm_cyc_buf <= 1'b0;
            sv32_wbm_stb_buf <= 1'b0;
            sv32_wbm_adr_buf <= 32'h0;
            sv32_wbm_dat_buf <= 32'h0;
            sv32_wbm_sel_buf <= 4'h0;
            sv32_wbm_we_buf  <= 1'b0;

            sv32_pt_buf      <= 32'h0;
            sv32_level_buf   <= 0;
        end else begin

            sv32_wbm_cyc_buf <= sv32_wbm_cyc_next;
            sv32_wbm_stb_buf <= sv32_wbm_stb_next;
            sv32_wbm_adr_buf <= sv32_wbm_adr_next;
            sv32_wbm_dat_buf <= sv32_wbm_dat_next;
            sv32_wbm_sel_buf <= sv32_wbm_sel_next;
            sv32_wbm_we_buf  <= sv32_wbm_we_next;

            sv32_pt_buf      <= sv32_pt_next;
            sv32_level_buf   <= sv32_level_next;

        end
    end


    logic [31:0] sv32_pte;
    assign sv32_pte = sv32_tlb_hit ? {sv32_tlb_ppn_o, 2'b00, sv32_tlb_perm_o} : sv32_sram_data;

    // for permission check
    logic [7:0] sv32_pte_perm;
    assign sv32_pte_perm = sv32_pte[7:0];

    typedef enum logic [1:0] {
        OK,  // permission check ok
        PF,  // page fault
        NEXT // next level
    } sv32_pte_state_t;

    sv32_pte_state_t sv32_pte_state;

    logic [3:0] debug_sv32_pte_state_cause;


    always_comb begin
        // TODO: not check misaligned superpage
        // TODO: not check A/D bits
        sv32_pte_state = PF;
        debug_sv32_pte_state_cause = 4'hf;
        
        if (sv32_pte_perm[`PTE_V] == 1'b0 | (~sv32_pte_perm[`PTE_R] & sv32_pte_perm[`PTE_W])) begin
            sv32_pte_state = PF;
            debug_sv32_pte_state_cause = 4'h1;

        end else begin
            if (sv32_pte_perm[`PTE_R] | sv32_pte_perm[`PTE_X]) begin // leaf
                if (sv32_pte_perm[sv32_check_bit] == 1'b0) begin
                    sv32_pte_state = PF;
                    debug_sv32_pte_state_cause = 4'h2;

                end else if (~sv32_pte_perm[`PTE_U] & mode_i == U_MODE) begin
                    sv32_pte_state = PF;
                    debug_sv32_pte_state_cause = 4'h3;

                end else if (sv32_pte_perm[`PTE_U] & mode_i == S_MODE & ~sum_i) begin
                    sv32_pte_state = PF;
                    debug_sv32_pte_state_cause = 4'h4;

                end else begin
                    sv32_pte_state = OK;
                end
            end else begin
                if (sv32_level == 0) begin
                    sv32_pte_state = PF;
                    debug_sv32_pte_state_cause = 4'h5;

                end else begin
                    sv32_pte_state = NEXT;
                end
            end

        end
    end

    // as slave
    assign sv32_wbs_dat_o = sv32_sram_data;

    // state_next, page_fault
    always_comb begin

        page_fault_o          = 1'b0;
        sv32_tlb_write_enable = 1'b0;
        state_next            = state;
        sv32_wbs_ack_o        = 1'b0;

        case (state)
            IDLE: begin
                if (sv32_request_from_master) begin // new request
                    if(sv32_tlb_hit) begin
                        if (sv32_pte_state == OK) begin
                            state_next = MEM_OPERATION; 
                        end else if (sv32_pte_state == PF) begin
                            state_next = REPORT_PF;
                        end
                    end else begin
                        state_next = FETCH_PTE;
                    end  

                end
            end
            FETCH_PTE: begin
                if (wbm_ack_i) begin
                    if(sv32_pte_state == NEXT) begin
                        state_next = FETCH_PTE;
                    end else if (sv32_pte_state == OK) begin
                        sv32_tlb_write_enable = 1'b1; // write tlb
                        state_next = MEM_OPERATION;
                    end else if (sv32_pte_state == PF) begin
                        state_next = REPORT_PF;
                        // sv32_wbs_ack_o = 1'b1;
                        // page_fault_o = 1'b1;
                    end
                end
            end

            MEM_OPERATION: begin
                if (wbm_ack_i) begin
                    state_next = IDLE;
                    sv32_wbs_ack_o = 1'b1;
                end
            end

            REPORT_PF: begin
                state_next = IDLE;
                sv32_wbs_ack_o = 1'b1;
                page_fault_o = 1'b1;

            end

        endcase
    end


    always_comb begin
        sv32_pt_next    = sv32_pt_buf;
        sv32_level_next = sv32_level_buf;

        case (state)
            IDLE: begin
                if (sv32_request_from_master) begin // new request
                    if(!sv32_tlb_hit) begin
                        sv32_pt_next    = satp_i[21:0] * `PAGESIZE;
                        sv32_level_next = `LEVELS-1;
                    end
                end
            end
            FETCH_PTE: begin
                if (wbm_ack_i) begin
                    if(sv32_pte_state == NEXT) begin
                        sv32_pt_next    = wbm_dat_i[`PTE_PPN] * `PAGESIZE;
                        sv32_level_next = sv32_level - 1;
                    end 
                end
            end
        endcase

    end




    // sv32_wbm_forward
    always_comb begin
        if (state == IDLE & sv32_request_from_master) begin
            sv32_wbm_forward = 1'b0; // TODO: we are not forward right now because it cause logic loop
        end else begin
            sv32_wbm_forward = 1'b0;
        end
    end



    // wbm signals
    always_comb begin
        case (state_next)
            IDLE: begin
                sv32_wbm_cyc_next =  1'b0;
                sv32_wbm_stb_next =  1'b0;

                sv32_wbm_adr_next = 32'b0;
                sv32_wbm_dat_next = 32'b0;
                sv32_wbm_sel_next =  4'b0;
                sv32_wbm_we_next  =  1'b0;
            end
            FETCH_PTE: begin
                sv32_wbm_cyc_next = 1'b1;
                sv32_wbm_stb_next = 1'b1;
                sv32_wbm_adr_next = sv32_pt_next + sv32_vpn[sv32_level_next] * `PTESIZE;
                sv32_wbm_dat_next = 32'b0;
                sv32_wbm_sel_next = 4'b1111;
                sv32_wbm_we_next  = 1'b0;
            end
            MEM_OPERATION: begin
                sv32_wbm_cyc_next = 1'b1;
                sv32_wbm_stb_next = 1'b1;
                sv32_wbm_adr_next = {sv32_pte[`PTE_PPN] , wbs_adr_i[11:0]};
                sv32_wbm_dat_next = wbs_dat_i;
                sv32_wbm_sel_next = wbs_sel_i;
                sv32_wbm_we_next  = wbs_we_i;
            end
            REPORT_PF: begin
                sv32_wbm_cyc_next = 1'b0;
                sv32_wbm_stb_next = 1'b0;
                sv32_wbm_adr_next = 32'b0;
                sv32_wbm_dat_next = 32'b0;
                sv32_wbm_sel_next = 4'b0;
                sv32_wbm_we_next  = 1'b0;
            end


        endcase

    end

    assign debug_mmu_state_o    = state;
    assign debug_mmu_pf_cause_o = debug_sv32_pte_state_cause;
    assign debug_mmu_pf_pte_o   = sv32_pte;

    assign debug_paddr_o        = {sv32_pte[`PTE_PPN] , wbs_adr_i[11:0]};
    assign debug_tlb_hit_o      = sv32_tlb_hit;
    assign debug_pte_state_o    = sv32_pte_state;

endmodule

// TODO: you cannot connect wires directly from exe, you need to consider exception
// you need provide address before posedge
module mmu_virtual_memory_interface #(
    parameter CACHE_DATA_SIZE = 32, // bytes
    parameter CACHE_SETS      = 32  
    parameter CACHE_WAYS      = 4   
) 
(
    input  wire         clk_i,
    input  wire         rst_i,

    // memory interface signals
    input  wire         vmi_no_cache_i,
    input  wire         vmi_enable_i,
    input  wire         vmi_write_enable_i,
    input  wire  [31:0] vmi_addr_i,
    input  wire  [31:0] vmi_data_i, // read request would also pass data_i to bus
    input  wire  [ 3:0] vmi_data_sel_i,
 
    output wire  [31:0] vmi_ack_addr_o,
    output wire         vmi_ack_o,
    output wire  [31:0] vmi_data_o,

    // status signals (it is upper module's responsibility to keep these signals stable)
    input  wire  [31:0] satp_i, // IM and DM will not request with different modes
    input  wire         sum_i, 
    input  wire  [ 1:0] mode_i,
    input  wire         tlb_clear_i,
    
    output logic        page_fault_o,

    // wbm signals (as master) (fall back to wish bone bus)
    output wire         wb_cyc_o,
    output wire         wb_stb_o,
    input  wire         wb_ack_i,
    output wire  [31:0] wb_adr_o,
    output wire  [31:0] wb_dat_o,
    input  wire  [31:0] wb_dat_i,
    output wire  [ 3:0] wb_sel_o,
    output wire         wb_we_o,

    // debug signals

    input  wire         debug_sv32_tlb_enable_i,
    
    output logic [ 1:0] debug_sv32_state_o,
    output logic [31:0] debug_sv32_pf_pte_o,
    output logic [ 3:0] debug_sv32_pf_cause_o,
    output logic        debug_sv32_tlb_hit_o,
    output logic [ 1:0] debug_sv32_pte_state_o,

    output logic [31:0] debug_paddr_o
);

    logic         pmi_no_cache_i;
    logic         pmi_enable_i;
    logic         pmi_write_enable_i;
    logic  [31:0] pmi_addr_i;
    logic  [31:0] pmi_data_i;
    logic  [ 3:0] pmi_data_sel_i;
    logic  [31:0] pmi_ack_addr_o;
    logic  [31:0] pmi_data_o;

    mmu_physical_memory_interface # (
        .CACHE_DATA_SIZE (CACHE_DATA_SIZE),
        .CACHE_SETS      (CACHE_SETS),
        .CACHE_WAYS      (CACHE_WAYS)
    ) pmi (
        .clk_i           (clk_i),
        .rst_i           (rst_i),
 
        .no_cache_i      (pmi_no_cache_i),
        .enable_i        (pmi_enable_i),
        .write_enable_i  (pmi_write_enable_i),
        .addr_i          (pmi_addr_i),
        .data_i          (pmi_data_i),
        .data_sel_i      (pmi_data_sel_i),
 
        .ack_addr_o      (pmi_ack_addr_o),
        .data_o          (pmi_data_o),
 
        .wb_cyc_o        (wb_cyc_o),
        .wb_stb_o        (wb_stb_o),
        .wb_ack_i        (wb_ack_i),
        .wb_adr_o        (wb_adr_o),
        .wb_dat_o        (wb_dat_o),
        .wb_dat_i        (wb_dat_i),
        .wb_sel_o        (wb_sel_o),
        .wb_we_o         (wb_we_o)
    );

    // TODO: related logic
    logic  [31:0] pmi_addr_i_buf; // used to check if ack
    logic         pmi_ack;

    assign pmi_ack = (pmi_addr_i_buf == pmi_ack_addr_o);

    // external signals translation
    typedef enum logic {
        BARE = 1'b0,
        SV32 = 1'b1
    } satp_mode_t;

    satp_mode_t satp_mode;
    assign satp_mode = satp_mode_t'(satp_i[31]);

    logic sv32_enable;
    assign sv32_enable = !(satp_mode == BARE | mode_i == M_MODE);

    // input buffer
    logic        vmi_no_cache_i_buf;
    logic        vmi_enable_i_buf;
    logic        vmi_write_enable_i_buf;
    logic [31:0] vmi_addr_i_buf;
    logic [31:0] vmi_data_i_buf;
    logic [ 3:0] vmi_data_sel_i_buf;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            vmi_no_cache_i_buf     <= 1'b0;
            vmi_enable_i_buf       <= 1'b0;
            vmi_write_enable_i_buf <= 1'b0;
            vmi_addr_i_buf         <= 32'b0;
            vmi_data_i_buf         <= 32'b0;
            vmi_data_sel_i_buf     <= 4'b0;
        end else begin
            vmi_no_cache_i_buf     <= vmi_no_cache_i;
            vmi_enable_i_buf       <= vmi_enable_i;
            vmi_write_enable_i_buf <= vmi_write_enable_i;
            vmi_addr_i_buf         <= vmi_addr_i;
            vmi_data_i_buf         <= vmi_data_i;
            vmi_data_sel_i_buf     <= vmi_data_sel_i;
        end
    end

    typedef enum logic [1:0] {
        DONE,
        FETCH_PTE,
        MEM_OPERATION

    } sv32_state_t;
    sv32_state_t state;



    // sv32 parameters (after posedge)
    logic [ 9:0] sv32_vpn [0:1];
    assign sv32_vpn[1] = vmi_addr_i_buf[31:22];
    assign sv32_vpn[0] = vmi_addr_i_buf[21:12];

    // this is used by tlb (before posedge)
    logic [19:0] sv32_all_vpn;
    assign sv32_all_vpn = vmi_addr_i[31:12];

    // tlb translation is before posedge
    // pmi signals is before posedge

    // tlb signals
    logic        sv32_tlb_enable;
    logic [19:0] sv32_tlb_vpn;
    logic [21:0] sv32_tlb_ppn_i;
    logic [ 7:0] sv32_tlb_perm_i;
    logic [21:0] sv32_tlb_ppn_o;
    logic [ 7:0] sv32_tlb_perm_o;
    logic        sv32_tlb_hit_o;
    logic        sv32_tlb_write_enable;

    tlb_sv32 tlb_sv32_inst (
        .clk_i          (clk_i),
        .rst_i          (rst_i),
        .enable_i       (sv32_tlb_enable),
        .clear_i        (tlb_clear_i),
        .vpn_i          (sv32_tlb_vpn),
        .ppn_i          (sv32_tlb_ppn_i),
        .perm_i         (sv32_tlb_perm_i),
        .hit_o          (sv32_tlb_hit_o),
        .ppn_o          (sv32_tlb_ppn_o),
        .perm_o         (sv32_tlb_perm_o),
        .write_enable_i (sv32_tlb_write_enable)
    );

    logic  sv32_tlb_hit;
    assign sv32_tlb_hit = debug_tlb_enable_i & sv32_tlb_hit_o;

    
    // state machine
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            state <= DONE;
        end else begin
            case (state)
                DONE: begin
                    if (vmi_enable_i) begin
                        // we got the correct physical address (in comb logic) // TODO: comb_logic
                        if ((sv32_enable & sv32_tlb_hit) | !sv32_enable) begin
                            state <= WAIT_MEM;
                        end else begin
                            state <= FETCH_PTE;
                        end
                    end
                end
                WAIT_FETCH_PTE: begin
                    if (pmi_ack) begin
                        if (sv32_pte_state == NEXT) begin
                            state <= WAIT_FETCH_PTE;
                        end else if (sv32_pte_state == OK) begin
                            sv32_tlb_write_enable = 1'b1; // write tlb
                            state <= WAIT_MEM;
                        end else if (sv32_pte_state == PF) begin
                            state <= DONE;
                            // sv32_wbs_ack_o = 1'b1; // TODO: report ack
                            // page_fault_o = 1'b1;  // TODO: report page fault
                        end
                    end
                end
                WAIT_MEM: begin
                    if (pmi_ack) begin
                        state <= DONE;
                    end
                end
            endcase
        end

    end

    // pmi comb logic

endmodule

*/