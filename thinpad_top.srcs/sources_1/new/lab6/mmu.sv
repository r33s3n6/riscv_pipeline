`include "alu_define.sv"

`define PAGESIZE 4096

`define PTESIZE  4
`define LEVELS   2

`define PTE_V    0
`define PTE_R    1
`define PTE_W    2
`define PTE_X    3
`define PTE_U    4

`define PTE_PERM 7:0
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

/*

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


    // pte_state begin 
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

    // pte_state end

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
*/


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

    assign satp_o = write_enable_i ? satp_i : satp_buf;
    assign mode_o = write_enable_i ? mode_i : mode_buf;
    assign sum_o  = write_enable_i ? sum_i  : sum_buf;

    // assign satp_o = satp_buf;
    // assign mode_o = mode_buf;
    // assign sum_o  = sum_buf;

endmodule


// you MUST provide valid signals BEFORE posedge clk_i
// and you can get valid data AFTER posedge clk_i
module mmu_memory_cache #(
    parameter CACHE_DATA_SIZE = 32, // bytes
    parameter CACHE_SETS      = 16,
    parameter CACHE_WAYS      = 4,
    parameter CACHE_SET_WIDTH = $clog2(CACHE_SETS) 
)
(
    input  wire         clk_i,
    input  wire         rst_i,

    input  wire         flush_i,
    input  wire  [CACHE_SET_WIDTH-1:0] flush_set_i,
    // output wire         flush_hit_o,

    input  wire         enable_i,
    input  wire         write_enable_i,
    input  wire         dirty_i,

    input  wire  [31:0] addr_i,
    input  wire  [31:0] data_i,
    input  wire  [ 3:0] data_sel_i,
    
    output logic [31:0] addr_o,
    output logic [31:0] data_o,
    output logic        hit_o // when write, not hit means write back
);  

    // TODO: implement cache

    // simple cache, test upper logic
    logic [31:0] cache_data; // just one data
    logic [31:0] cache_addr;
    logic        cache_valid;
    logic        cache_dirty;

        // select bit mask
    logic [31:0] sel_bit_mask;

    // generate bit mask
    generate 
        for (genvar i = 0; i < 32; i = i + 8) begin : gen_sel_bit_mask
            assign sel_bit_mask[i+7:i] = {8{data_sel_i[i/8]}};
        end
    endgenerate

    logic [31:0] data_write;
    assign data_write = (data_i & sel_bit_mask) | (cache_data & ~sel_bit_mask);

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            cache_data  <= 32'b0;
            cache_addr  <= 32'b0;
            cache_valid <= 1'b0;
            cache_dirty <= 1'b0;
            hit_o       <= 1'b0;

            addr_o      <= 32'b0;
            data_o      <= 32'b0;
        end else begin
            if (flush_i) begin
                if (cache_valid & cache_dirty) begin
                    addr_o      <= cache_addr;
                    data_o      <= cache_data;
                    cache_data  <= 32'b0;
                    cache_addr  <= 32'b0;
                    cache_valid <= 1'b0;
                    cache_dirty <= 1'b0;
                    hit_o       <= 1'b1;
                end else begin
                    addr_o      <= 32'b0;
                    data_o      <= 32'b0;
                    cache_data  <= 32'b0;
                    cache_addr  <= 32'b0;
                    cache_valid <= 1'b0;
                    cache_dirty <= 1'b0;
                    hit_o       <= 1'b0;
                end
                
            end else begin
                if (enable_i) begin
                    if (write_enable_i) begin
                        if (cache_valid) begin
                            if (cache_addr == addr_i) begin
                                cache_data  <= data_write;
                                cache_dirty <= dirty_i;
                                hit_o       <= 1'b1;
                            end else begin
                                cache_data  <= data_write;
                                cache_addr  <= addr_i;
                                cache_dirty <= dirty_i;
                                if (cache_dirty) begin
                                    addr_o <= cache_addr;
                                    data_o <= cache_data;
                                    hit_o  <= 1'b0;
                                end else begin
                                    addr_o <= 32'b0;
                                    data_o <= 32'b0;
                                    hit_o  <= 1'b1;
                                end
                            end
                        end else begin // not valid, just write
                            cache_data  <= data_write;
                            cache_addr  <= addr_i;
                            cache_valid <= 1'b1;
                            cache_dirty <= dirty_i;
                            hit_o       <= 1'b1;
                        end
                    end else begin
                        if (cache_valid && cache_addr == addr_i) begin
                            hit_o       <= 1'b1;
                            addr_o      <= cache_addr;
                            data_o      <= cache_data;
                        end else begin
                            hit_o       <= 1'b0;
                            addr_o      <= 32'b0;
                            data_o      <= 32'b0;
                        end
                    end
                end else begin
                    hit_o       <= 1'b0;
                    addr_o      <= 32'b0;
                    data_o      <= 32'b0;
                end
            end
        end
    end

endmodule

// access memory with PHYSICAL address (mmu is placed above this module)
// you MUST provide valid signals BEFORE posedge clk_i
// and you can get valid data AFTER posedge clk_i
module mmu_physical_memory_interface #(
    parameter CACHE_DATA_SIZE = 32, // bytes
    parameter CACHE_SETS      = 16,  
    parameter CACHE_WAYS      = 4,
    parameter CACHE_SET_WIDTH = $clog2(CACHE_SETS) 
)

(
    input  wire         clk_i,
    input  wire         rst_i,

    input  wire         flush_i,

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
    output wire         wb_we_o,

    output wire         sync_done_o
);
    
    typedef enum logic [2:0] {
        _IDLE,
        CACHE_HIT_OR_BUS,
        BUS_WAIT_OR_DONE,
        HIT_OR_SECOND_BUS,
        CACHE_FLUSH_BUS_REQUEST_OR_DONE,
        CACHE_FLUSH_BUS_WAIT_OR_DONE
    } raw_state_t;

    raw_state_t raw_state;

    assign sync_done_o = (raw_state != CACHE_FLUSH_BUS_REQUEST_OR_DONE) && (raw_state != CACHE_FLUSH_BUS_WAIT_OR_DONE);
    
    // TODO: note that no cache write is acked immediately (this may cause problems)
    typedef enum logic [3:0] {
        IDLE,                     // idle              : (no ack) (serve)    we can serve new request
        CACHE_HIT,                // cache hit or bus  : (ack)    (serve)    cache hit
        BUS_REQUEST_WRITE_MISS,   // cache hit or bus  : (ack)    (no serve) write cache miss 
        BUS_REQUEST_READ_MISS,    // cache hit or bus  : (no ack) (no serve) read cache miss
        BUS_WAIT,                 // bus wait or done  : (no ack) (no serve) wait for bus
        BUS_DONE,                 // bus wait or done  : (no ack) (serve)    no need to write back to cache (write miss) (acked before)
        BUS_DONE_READ,            // bus wait or done  : (ack)    (serve)    no cache (read miss)
        BUS_WRITE_CACHE,          // bus wait or done  : (ack)    (no serve) write cache (read miss)
        BUS_REQUEST_SECOND,       // hit or second     : (no ack) (no serve) second bus request (read miss, write miss, write back to mem)
        BUS_WAIT_SECOND,          // hit or second     : (no ack) (no serve) wait for second bus
        BUS_DONE_SECOND,          // hit or second     : (no ack) (serve)    done for second bus (acked before)
        CACHE_FLUSH_BUS_REQUEST,  // cf bus req/done   : (no ack) (no serve) cache flush on entry
        CACHE_FLUSH_DONE,         // cf bus req/done   : (no ack) (serve)    cache flush done
        CACHE_FLUSH_NEXT_SET,     // cf bus req/done   : (no ack) (no serve) cache flush next set
        CACHE_FLUSH_BUS_WAIT,     // cf bus wait/done  : (no ack) (no serve) wait for cache flush
        CACHE_FLUSH_BUS_DONE      // cf bus wait/done  : (no ack) (no serve) cache flush one entry bus done
       
    } state_t;

    state_t state;

    

    logic serve_ready;
    logic do_ack;

    always_comb begin

        case(state)
            IDLE: begin
                do_ack = 1'b0;
                serve_ready = 1'b1;
            end
            CACHE_HIT: begin
                do_ack = 1'b1;
                serve_ready = 1'b1;
            end
            BUS_REQUEST_WRITE_MISS: begin
                do_ack = 1'b1;
                serve_ready = 1'b0;
            end
            BUS_REQUEST_READ_MISS: begin
                do_ack = 1'b0;
                serve_ready = 1'b0;
            end
            BUS_WAIT: begin
                do_ack = 1'b0;
                serve_ready = 1'b0;
            end
            BUS_DONE: begin
                do_ack = 1'b0;
                serve_ready = 1'b1;
            end
            BUS_DONE_READ: begin
                do_ack = 1'b1;
                serve_ready = 1'b1;
            end
            BUS_WRITE_CACHE: begin
                do_ack = 1'b1;
                serve_ready = 1'b0;
            end
            BUS_REQUEST_SECOND: begin
                do_ack = 1'b0;
                serve_ready = 1'b0;
            end
            BUS_WAIT_SECOND: begin
                do_ack = 1'b0;
                serve_ready = 1'b0;
            end
            BUS_DONE_SECOND: begin
                do_ack = 1'b0;
                serve_ready = 1'b1;
            end
            CACHE_FLUSH_BUS_REQUEST, CACHE_FLUSH_BUS_WAIT,
            CACHE_FLUSH_BUS_DONE, CACHE_FLUSH_NEXT_SET: begin
                do_ack = 1'b0;
                serve_ready = 1'b0;
            end
            CACHE_FLUSH_DONE: begin
                do_ack = 1'b0;
                serve_ready = 1'b1;
            end
            default: begin
                do_ack = 1'bx;
                serve_ready = 1'bx;
            end
        endcase

    end

    logic serve;

    assign serve = enable_i & serve_ready;

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
            if (serve) begin
                no_cache_i_buf     <= no_cache_i;
                enable_i_buf       <= enable_i;
                write_enable_i_buf <= write_enable_i;
                addr_i_buf         <= addr_i;
                data_i_buf         <= data_i;
                data_sel_i_buf     <= data_sel_i;
            end
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

    logic        wb_forward;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            wb_cyc_o_buf <= 1'b0;
            wb_stb_o_buf <= 1'b0;
            wb_adr_o_buf <= 32'b0;
            wb_dat_o_buf <= 32'b0;
            wb_sel_o_buf <= 4'b0;
            wb_we_o_buf  <= 1'b0;
        end else begin
            wb_cyc_o_buf <= wb_cyc_o_next;
            wb_stb_o_buf <= wb_stb_o_next;
            wb_adr_o_buf <= wb_adr_o_next;
            wb_dat_o_buf <= wb_dat_o_next;
            wb_sel_o_buf <= wb_sel_o_next;
            wb_we_o_buf  <= wb_we_o_next;

        end
    end

    // forward signals
    assign wb_cyc_o = wb_forward ? wb_cyc_o_next : wb_cyc_o_buf;
    assign wb_stb_o = wb_forward ? wb_stb_o_next : wb_stb_o_buf;
    assign wb_adr_o = wb_forward ? wb_adr_o_next : wb_adr_o_buf;
    assign wb_dat_o = wb_forward ? wb_dat_o_next : wb_dat_o_buf;
    assign wb_sel_o = wb_forward ? wb_sel_o_next : wb_sel_o_buf;
    assign wb_we_o  = wb_forward ?  wb_we_o_next :  wb_we_o_buf;


    logic        cache_flush;
    logic [CACHE_SET_WIDTH-1:0]       cache_flush_set;
    logic [CACHE_SET_WIDTH-1:0] cache_flush_set_reg;

    logic        cache_enable;
    logic        cache_write_enable;
    logic [31:0] cache_addr_in;
    logic [31:0] cache_data_in;
    logic [ 3:0] cache_data_sel;

    logic        cache_hit;
    logic [31:0] cache_addr_out;
    logic [31:0] cache_data_out;

    logic        cache_dirty_in;

    // TODO: cache_flush, cache_flush_set

    // not using buf
    mmu_memory_cache # (
        .CACHE_DATA_SIZE (CACHE_DATA_SIZE),
        .CACHE_SETS      (CACHE_SETS),
        .CACHE_WAYS      (CACHE_WAYS)
    ) cache ( 
        .clk_i           (clk_i),
        .rst_i           (rst_i),
 
        .flush_i         (cache_flush),
        .flush_set_i     (cache_flush_set),
 
        .enable_i        (cache_enable),
        .write_enable_i  (cache_write_enable),
        .dirty_i         (cache_dirty_in),

        .addr_i          (cache_addr_in),
        .data_i          (cache_data_in),
        .data_sel_i      (cache_data_sel),

        .addr_o          (cache_addr_out),
        .data_o          (cache_data_out),
        .hit_o           (cache_hit)
    );

    always_comb begin
        cache_flush     = 1'b0;
        cache_flush_set = {CACHE_SET_WIDTH{1'b0}};
        cache_dirty_in  = 1'b1;
        if (serve) begin
            if (flush_i) begin
                cache_enable       = 1'b1;
                cache_write_enable = 1'b0;
                cache_addr_in      = 32'b0;
                cache_data_in      = 32'b0;
                cache_data_sel     = 4'b0;

                cache_flush        = 1'b1;
                cache_flush_set    = 1'b0;

            end else begin
                cache_enable       = enable_i & ~no_cache_i;
                cache_write_enable = write_enable_i;
                cache_addr_in      = addr_i;
                cache_data_in      = data_i;
                cache_data_sel     = data_sel_i;
            end
        end else if (state == BUS_WRITE_CACHE) begin
            cache_enable       = 1'b1;
            cache_write_enable = 1'b1;
            cache_addr_in      = addr_i_buf;
            cache_data_in      = wb_dat_i;
            cache_data_sel     = 4'hf; // write all
            cache_dirty_in     = 1'b0; // from mem, not dirty

        end else if (state == CACHE_FLUSH_BUS_DONE) begin
                
            cache_enable       = 1'b1;
            cache_write_enable = 1'b0;
            cache_addr_in      = 32'b0;
            cache_data_in      = 32'b0;
            cache_data_sel     = 4'b0;

            cache_flush        = 1'b1;
            cache_flush_set    = cache_flush_set_reg;
        end else if (state == CACHE_FLUSH_NEXT_SET) begin
            cache_enable       = 1'b1;
            cache_write_enable = 1'b0;
            cache_addr_in      = 32'b0;
            cache_data_in      = 32'b0;
            cache_data_sel     = 4'b0;

            cache_flush        = 1'b1;
            cache_flush_set = cache_flush_set_reg + 1;
            

        end else begin
            cache_enable       = 1'b0;
            cache_write_enable = 1'b0;
            cache_addr_in      = 32'b0;
            cache_data_in      = 32'b0;
            cache_data_sel     = 4'b0;
        end
    end

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            cache_flush_set_reg <= {CACHE_SET_WIDTH{1'b0}};
        end else begin
            cache_flush_set_reg <= cache_flush_set;
        end
    end


    logic second_bus_requested; // used to determine state
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            second_bus_requested <= 1'b0;
        end else begin
            if (state == BUS_REQUEST_SECOND) begin
                second_bus_requested <= 1'b1;
            end else if (state == BUS_DONE_SECOND) begin
                second_bus_requested <= 1'b0;
            end
        end
    end

    // state
    always_comb begin
        case(raw_state)
            _IDLE: begin
                state = IDLE;
            end
            CACHE_HIT_OR_BUS: begin
                if (~no_cache_i_buf & cache_hit) begin
                    state = CACHE_HIT;
                end else begin
                    if (write_enable_i_buf) begin
                        state = BUS_REQUEST_WRITE_MISS;
                    end else begin
                        state = BUS_REQUEST_READ_MISS;
                    end
                end
            end
            BUS_WAIT_OR_DONE: begin
                if (wb_ack_i) begin
                    if (write_enable_i_buf) begin
                        state = BUS_DONE;
                    end else begin
                        if (no_cache_i_buf) begin
                            state = BUS_DONE_READ;
                        end else begin
                            state = BUS_WRITE_CACHE;
                        end
                    end
                end else begin
                    state = BUS_WAIT;
                end
            end
            HIT_OR_SECOND_BUS: begin
                if (cache_hit) begin
                    state = BUS_DONE_SECOND; // no need to second bus
                end else if (wb_ack_i) begin
                    state = BUS_DONE_SECOND;
                end else begin
                    if (second_bus_requested) begin
                        state = BUS_WAIT_SECOND;
                    end else begin
                        state = BUS_REQUEST_SECOND;
                    end
                end
            end
            CACHE_FLUSH_BUS_REQUEST_OR_DONE: begin
                if (cache_hit) begin
                    state = CACHE_FLUSH_BUS_REQUEST;
                end else if (cache_flush_set_reg == CACHE_SETS-1) begin
                    state = CACHE_FLUSH_DONE;
                end else begin
                    state = CACHE_FLUSH_NEXT_SET; // request next flush
                end
            end
            CACHE_FLUSH_BUS_WAIT_OR_DONE: begin
                if (wb_ack_i) begin
                    state = CACHE_FLUSH_BUS_DONE; // request next flush
                end else begin
                    state = CACHE_FLUSH_BUS_WAIT;
                end
            end
            default: begin
                state = IDLE;
            end

        endcase
    end

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            raw_state <= _IDLE;
        end else begin
            case (state) 
                // serve ready state
                _IDLE, CACHE_HIT, BUS_DONE, BUS_DONE_READ, BUS_DONE_SECOND, CACHE_FLUSH_DONE: begin
                    if (serve) begin
                        if (flush_i) begin
                            raw_state <= CACHE_FLUSH_BUS_REQUEST_OR_DONE;
                        end else begin
                            raw_state <= CACHE_HIT_OR_BUS;
                        end
                        
                    end else begin
                        raw_state <= _IDLE;
                    end
                end
                BUS_REQUEST_READ_MISS: begin
                    raw_state <= BUS_WAIT_OR_DONE;
                end
                BUS_REQUEST_WRITE_MISS: begin
                    raw_state <= BUS_WAIT_OR_DONE;
                end
                BUS_WAIT: begin
                    raw_state <= BUS_WAIT_OR_DONE;
                end
                BUS_WRITE_CACHE: begin
                    raw_state <= HIT_OR_SECOND_BUS;
                end
                BUS_REQUEST_SECOND: begin
                    raw_state <= HIT_OR_SECOND_BUS;
                end
                BUS_WAIT_SECOND: begin
                    raw_state <= HIT_OR_SECOND_BUS;
                end
                CACHE_FLUSH_BUS_REQUEST: begin
                    raw_state <= CACHE_FLUSH_BUS_WAIT_OR_DONE;
                end
                CACHE_FLUSH_NEXT_SET: begin
                    raw_state <= CACHE_FLUSH_BUS_REQUEST_OR_DONE;
                end
                CACHE_FLUSH_BUS_WAIT: begin
                    raw_state <= CACHE_FLUSH_BUS_WAIT_OR_DONE;
                end
                CACHE_FLUSH_BUS_DONE: begin
                    raw_state <= CACHE_FLUSH_BUS_REQUEST_OR_DONE;
                end
                default: begin
                    raw_state <= _IDLE;
                end
            endcase
        end
    end


    // wb_* signals
    always_comb begin
        case (state)
            IDLE, CACHE_HIT, CACHE_FLUSH_NEXT_SET, CACHE_FLUSH_DONE: begin // clear wb signals
                wb_forward    = 1'b1;

                wb_cyc_o_next = 1'b0;
                wb_stb_o_next = 1'b0;
                wb_adr_o_next = 32'b0;
                wb_dat_o_next = 32'b0;
                wb_sel_o_next = 4'b0;
                wb_we_o_next  = 1'b0;    
            end
            BUS_REQUEST_READ_MISS: begin
                wb_forward = 1'b1;
                if (no_cache_i_buf) begin // just forward request
                    wb_cyc_o_next = 1'b1;
                    wb_stb_o_next = 1'b1;
                    wb_adr_o_next = addr_i_buf;
                    wb_dat_o_next = data_i_buf;
                    wb_sel_o_next = data_sel_i_buf;
                    wb_we_o_next  = 1'b0; // read
                end else begin
                    wb_cyc_o_next = 1'b1;
                    wb_stb_o_next = 1'b1;
                    wb_adr_o_next = addr_i_buf;
                    wb_dat_o_next = data_i_buf;
                    wb_sel_o_next = 4'hf; // cache miss, read all
                    wb_we_o_next  = 1'b0; // read
                end
            end
            BUS_REQUEST_WRITE_MISS: begin
                wb_forward = 1'b1;
                if (no_cache_i_buf) begin // just forward request
                    wb_cyc_o_next = 1'b1;
                    wb_stb_o_next = 1'b1;
                    wb_adr_o_next = addr_i_buf;
                    wb_dat_o_next = data_i_buf;
                    wb_sel_o_next = data_sel_i_buf;
                    wb_we_o_next  = 1'b1; // write
                end else begin
                    wb_cyc_o_next = 1'b1;
                    wb_stb_o_next = 1'b1;
                    wb_adr_o_next = cache_addr_out;
                    wb_dat_o_next = cache_data_out;
                    wb_sel_o_next = 4'hf; // cache miss, write back all
                    wb_we_o_next  = 1'b1; // write
                end
            end
            BUS_WAIT, BUS_WAIT_SECOND, CACHE_FLUSH_BUS_WAIT: begin // keep values
                wb_forward = 1'b0;

                wb_cyc_o_next = wb_cyc_o_buf;
                wb_stb_o_next = wb_stb_o_buf;
                wb_adr_o_next = wb_adr_o_buf;
                wb_dat_o_next = wb_dat_o_buf;
                wb_sel_o_next = wb_sel_o_buf;
                wb_we_o_next  = wb_we_o_buf;
            end
            BUS_DONE, BUS_DONE_READ, BUS_DONE_SECOND, BUS_WRITE_CACHE, CACHE_FLUSH_BUS_DONE: begin // keep values, and clear next tick
                wb_forward = 1'b0;

                wb_cyc_o_next = 1'b0;
                wb_stb_o_next = 1'b0;
                wb_adr_o_next = 32'b0;
                wb_dat_o_next = 32'b0;
                wb_sel_o_next = 4'b0;
                wb_we_o_next  = 1'b0; 
            end
            CACHE_FLUSH_BUS_REQUEST: begin
                wb_forward = 1'b1;

                wb_cyc_o_next = 1'b1;
                wb_stb_o_next = 1'b1;
                wb_adr_o_next = cache_addr_out;
                wb_dat_o_next = cache_data_out;
                wb_sel_o_next = 4'hf; // flush all
                wb_we_o_next  = 1'b1; // write
            end
            default: begin
                wb_forward = 1'b1;

                wb_cyc_o_next = 1'bx;
                wb_stb_o_next = 1'bx;
                wb_adr_o_next = {32{1'bx}};
                wb_dat_o_next = {32{1'bx}};
                wb_sel_o_next = {4{1'bx}};
                wb_we_o_next  = 1'bx;
            end

        endcase

    end

    // ack logic
    assign ack_o = do_ack;
    logic [31:0] ack_addr_buf;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            ack_addr_buf <= 32'b0;
        end else begin
            if (ack_o) begin
                ack_addr_buf <= wb_adr_o_buf;
            end
        end
    end
    assign ack_addr_o = ack_o ? addr_i_buf : ack_addr_buf;

    logic [31:0] data_o_buf;
    logic [31:0] data_o_next;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            data_o_buf <= 32'b0;
        end else begin
            if (ack_o) begin
                data_o_buf <= data_o_next;
            end
        end
    end

    // forward
    assign data_o = ack_o ? data_o_next : data_o_buf;

    // ack data
    always_comb begin
        case (state)
            CACHE_HIT: begin
                data_o_next         = cache_data_out;
            end
            BUS_REQUEST_WRITE_MISS: begin
                data_o_next         = 32'b0;
            end
            BUS_DONE_READ, BUS_WRITE_CACHE: begin
                data_o_next         = wb_dat_i;
            end
            default: begin
                data_o_next         = 32'b0;
            end

                
        endcase
    end


    // cache flush control signals



endmodule



// TODO: you cannot connect wires directly from exe, you need to consider exception
// you need provide address before posedge
module mmu_virtual_memory_interface #(
    parameter CACHE_DATA_SIZE = 32, // bytes
    parameter CACHE_SETS      = 16,
    parameter CACHE_WAYS      = 4
) 
(
    input  wire         clk_i,
    input  wire         rst_i,

    input  wire         no_flush_cache_i, // TODO: not implemented fence.i

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
    input  wire         flush_i,
    
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

    output wire         sync_done_o,

    // debug signals

    input  wire         debug_sv32_tlb_enable_i,
    
    output logic [ 2:0] debug_state_o,
    output logic [31:0] debug_sv32_pte_o,
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
    logic         pmi_ack_o;
    logic  [31:0] pmi_data_o;

    mmu_physical_memory_interface # (
        .CACHE_DATA_SIZE (CACHE_DATA_SIZE),
        .CACHE_SETS      (CACHE_SETS),
        .CACHE_WAYS      (CACHE_WAYS)
    ) pmi (
        .clk_i           (clk_i),
        .rst_i           (rst_i),

        .flush_i         (no_flush_cache_i? 1'b0 : flush_i),
 
        .no_cache_i      (pmi_no_cache_i),
        .enable_i        (pmi_enable_i),
        .write_enable_i  (pmi_write_enable_i),
        .addr_i          (pmi_addr_i),
        .data_i          (pmi_data_i),
        .data_sel_i      (pmi_data_sel_i),
 
        .ack_addr_o      (pmi_ack_addr_o),
        .ack_o           (pmi_ack_o),
        .data_o          (pmi_data_o),
 
        .wb_cyc_o        (wb_cyc_o),
        .wb_stb_o        (wb_stb_o),
        .wb_ack_i        (wb_ack_i),
        .wb_adr_o        (wb_adr_o),
        .wb_dat_o        (wb_dat_o),
        .wb_dat_i        (wb_dat_i),
        .wb_sel_o        (wb_sel_o),
        .wb_we_o         (wb_we_o),

        .sync_done_o     (sync_done_o)
    );


    logic  pmi_ack;
    assign pmi_ack = pmi_ack_o; // no check ack_addr because we do not cancel any requests

    // state

    typedef enum logic [1:0] {
        _IDLE,            
        FETCH_PTE,     
        MEM_OPERATION,
        _REPORT_PF
    } raw_state_t;

    raw_state_t raw_state, raw_state_next;

    // TODO: note that no cache write is acked immediately (this may cause problem)
    typedef enum logic [2:0] {
        IDLE,                 // (no ack) (serve)
        FETCH_PTE_WAIT,       // (no ack) (no serve)
        FETCH_PTE_DONE_NEXT,  // (no ack) (no serve)
        FETCH_PTE_DONE_LEAF,  // (no ack) (no serve)
        FETCH_PTE_DONE_FAULT, // (no ack) (no serve) // NO SERVE to break logic loop
        MEM_WAIT,             // (no ack) (no serve)
        MEM_DONE,             // (ack)    (serve)
        REPORT_PF             // (ack)    (no serve) // NO SERVE to break logic loop // TODO: may optimize
    } state_t;

    state_t state;

    logic serve_ready;
    logic do_ack;

    always_comb begin
        case (state)
            IDLE: begin
                serve_ready = 1'b1;
                do_ack = 1'b0;
            end
            FETCH_PTE_WAIT: begin
                serve_ready = 1'b0;
                do_ack = 1'b0;
            end
            FETCH_PTE_DONE_NEXT: begin
                serve_ready = 1'b0;
                do_ack = 1'b0;
            end
            FETCH_PTE_DONE_LEAF: begin
                serve_ready = 1'b0;
                do_ack = 1'b0;
            end
            FETCH_PTE_DONE_FAULT: begin
                serve_ready = 1'b0;
                do_ack = 1'b0;
            end
            MEM_WAIT: begin
                serve_ready = 1'b0;
                do_ack = 1'b0;
            end
            MEM_DONE: begin
                serve_ready = 1'b1;
                do_ack = 1'b1;
            end
            REPORT_PF: begin
                serve_ready = 1'b0;
                do_ack = 1'b1;
            end
            default: begin
                serve_ready = 1'bx;
                do_ack = 1'bx;
            end
        endcase
    end

    logic serve;

    assign serve = vmi_enable_i & serve_ready;

    always_comb begin
    end

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

    logic        sv32_enable_buf;
    satp_mode_t  satp_buf;
    logic        sum_buf;
    logic        mode_buf;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            vmi_no_cache_i_buf     <= 1'b0;
            vmi_enable_i_buf       <= 1'b0;
            vmi_write_enable_i_buf <= 1'b0;
            vmi_addr_i_buf         <= 32'b0;
            vmi_data_i_buf         <= 32'b0;
            vmi_data_sel_i_buf     <= 4'b0;

            sv32_enable_buf        <= 1'b0;
            satp_buf               <= BARE;
            sum_buf                <= 1'b0;
            mode_buf               <= M_MODE;
        end else begin
            if (serve) begin
                vmi_no_cache_i_buf     <= vmi_no_cache_i;
                vmi_enable_i_buf       <= vmi_enable_i;
                vmi_write_enable_i_buf <= vmi_write_enable_i;
                vmi_addr_i_buf         <= vmi_addr_i;
                vmi_data_i_buf         <= vmi_data_i;
                vmi_data_sel_i_buf     <= vmi_data_sel_i;

                sv32_enable_buf        <= sv32_enable;
                satp_buf               <= satp_mode;
                sum_buf                <= sum_i;
                mode_buf               <= mode_i;
            end
        end
    end

    // request buffer is not needed


    // sv32 parameters (after posedge)
    logic [ 9:0] sv32_vpn [0:1];
    assign sv32_vpn[1] = vmi_addr_i_buf[31:22];
    assign sv32_vpn[0] = vmi_addr_i_buf[21:12];

    logic [ 9:0] sv32_vpn_next [0:1];
    assign sv32_vpn_next[1] = vmi_addr_i[31:22];
    assign sv32_vpn_next[0] = vmi_addr_i[21:12];

    // this is used by tlb (before posedge)
    logic [19:0] sv32_all_vpn;
    assign sv32_all_vpn = vmi_addr_i[31:12];

    logic [19:0] sv32_all_vpn_buf;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            sv32_all_vpn_buf <= 20'b0;
        end else begin
            if (serve) begin
                sv32_all_vpn_buf <= sv32_all_vpn;
            end
        end
    end

    logic [31:0] sv32_pte;
    logic [31:0] sv32_pte_buf;
    logic [31:0] sv32_pte_next;
    logic        sv32_pte_forward;

    logic [ 7:0] sv32_pte_perm;

    assign sv32_pte_perm = sv32_pte[`PTE_PERM];

    logic vmi_we;
    logic vmi_x;

    assign vmi_we = serve ? vmi_write_enable_i_buf : vmi_write_enable_i;
    assign vmi_x  = serve ? vmi_data_i_buf[0]      : vmi_data_i[0];

    logic  sv32_check_bit;
    assign sv32_check_bit = vmi_we ? sv32_pte_perm[`PTE_W] 
                           : vmi_x ? sv32_pte_perm[`PTE_X] 
                                   : sv32_pte_perm[`PTE_R];
    
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            sv32_pte_buf <= 32'b0;
        end else begin
            sv32_pte_buf <= sv32_pte_next;
        end
    end

    assign sv32_pte = sv32_pte_forward ? sv32_pte_next : sv32_pte_buf;
    
    // TODO: move all tlb wires together
    logic        sv32_tlb_hit;
    logic [21:0] sv32_tlb_ppn_o;
    logic [ 7:0] sv32_tlb_perm_o;

    always_comb begin
        sv32_pte_forward = 1'b0;
        sv32_pte_next = sv32_pte_buf;

        if (serve) begin
            sv32_pte_forward = 1'b1;
            sv32_pte_next = {sv32_tlb_ppn_o, 2'b00, sv32_tlb_perm_o}; // invalid is just ok (we are not checking pte_state when not hit)

        end else if (raw_state == FETCH_PTE && pmi_ack) begin
            sv32_pte_forward = 1'b1;
            sv32_pte_next = pmi_data_o;
        end
    end



    // walk status
    logic [31:0] sv32_pt;
    logic [ 1:0] sv32_level;

    logic [31:0] sv32_pt_next;
    logic [ 1:0] sv32_level_next;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            sv32_pt      <= 32'b0;
            sv32_level   <= 2'b0;
        end else begin
            sv32_pt      <= sv32_pt_next;
            sv32_level   <= sv32_level_next;
        end
    end


    /* ===================== pte_state begin ===================== */
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

                end else if (~sv32_pte_perm[`PTE_U] & mode_buf == U_MODE) begin
                    sv32_pte_state = PF;
                    debug_sv32_pte_state_cause = 4'h3;

                end else if (sv32_pte_perm[`PTE_U] & mode_buf == S_MODE & ~sum_buf) begin
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

    /* ===================== pte_state end ===================== */

    // tlb translation is before posedge
    // pmi signals is before posedge
    // tlb signals
    logic        sv32_tlb_enable;
    logic        sv32_tlb_write_enable;

    logic [19:0] sv32_tlb_vpn;
    logic [21:0] sv32_tlb_ppn_i;
    logic [ 7:0] sv32_tlb_perm_i;



    
    logic debug_sv32_enable_overwrite;
    assign debug_sv32_enable_overwrite = sv32_tlb_enable & debug_sv32_tlb_enable_i;

    tlb_sv32 tlb_sv32_inst (
        .clk_i          (clk_i),
        .rst_i          (rst_i),
        .enable_i       (debug_sv32_enable_overwrite),
        .write_enable_i (sv32_tlb_write_enable),
        .clear_i        (flush_i),

        .vpn_i          (sv32_tlb_vpn),
        .ppn_i          (sv32_tlb_ppn_i),
        .perm_i         (sv32_tlb_perm_i),

        .hit_o          (sv32_tlb_hit),
        .ppn_o          (sv32_tlb_ppn_o),
        .perm_o         (sv32_tlb_perm_o)
    );

    // tlb related logic
    always_comb begin
        if (serve) begin
            sv32_tlb_enable       = 1'b1;
            sv32_tlb_vpn          = sv32_all_vpn;
            sv32_tlb_ppn_i        = 22'b0;
            sv32_tlb_perm_i       = 8'b0;
            sv32_tlb_write_enable = 1'b0;
        end else begin
            sv32_tlb_enable       = 1'b0;
            sv32_tlb_vpn          = sv32_all_vpn_buf;
            sv32_tlb_ppn_i        = pmi_data_o[`PTE_PPN];
            sv32_tlb_perm_i       = pmi_data_o[`PTE_PERM];
            if (state == FETCH_PTE_DONE_LEAF || state == FETCH_PTE_DONE_FAULT) begin
                sv32_tlb_write_enable = 1'b1;
            end else begin
                sv32_tlb_write_enable = 1'b0;
            end
        end 
    end


    // state

    always_comb begin
        case (raw_state)
            _IDLE: begin
                state = IDLE;
            end
            FETCH_PTE: begin
                if (!pmi_ack) begin
                    state = FETCH_PTE_WAIT;
                end else begin
                    if (sv32_pte_state == OK) begin
                        state = FETCH_PTE_DONE_LEAF;
                    end else if (sv32_pte_state == NEXT) begin
                        state = FETCH_PTE_DONE_NEXT;
                    end else begin
                        state = FETCH_PTE_DONE_FAULT;
                    end
                end
            end
            MEM_OPERATION: begin
                if (!pmi_ack) begin
                    state = MEM_WAIT;
                end else begin
                    state = MEM_DONE;
                end
            end
            _REPORT_PF: begin
                state = REPORT_PF;
            end
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            raw_state <= _IDLE;
        end else begin
            raw_state <= raw_state_next;
        end
    end

    always_comb begin
        raw_state_next = _IDLE;
        case (state) 
            // serve ready state
            IDLE, MEM_DONE: begin
                if (serve) begin

                    if (!sv32_enable) begin
                        raw_state_next = MEM_OPERATION;
                    end else if (sv32_tlb_hit) begin
                        if (sv32_pte_state == OK) begin
                            raw_state_next = MEM_OPERATION;
                        end else begin
                            raw_state_next = _REPORT_PF;
                        end
                    end else begin
                        raw_state_next = FETCH_PTE;
                    end

                end else begin
                    raw_state_next = _IDLE;
                end
            end
            FETCH_PTE_DONE_NEXT, FETCH_PTE_WAIT: begin
                raw_state_next = FETCH_PTE;
            end
            FETCH_PTE_DONE_LEAF, MEM_WAIT: begin
                raw_state_next = MEM_OPERATION;
            end
            FETCH_PTE_DONE_FAULT: begin
                raw_state_next = _REPORT_PF;
            end
            REPORT_PF: begin
                raw_state_next = _IDLE;
            end
            default: begin
                raw_state_next = _IDLE;
            end
            
        endcase
 
    end

    logic [31:0] physical_addr;
    assign physical_addr = sv32_enable ? {sv32_pte[`PTE_PPN], serve ? vmi_addr_i[11:0] : vmi_addr_i_buf[11:0]} : vmi_addr_i;
    
    //0b10000000000000000000000000000000 - 0b10000000011111111111111111111111
    //0x80000000 - 0x807fffff
    logic in_sram;
    
    assign in_sram = physical_addr[31:23] == 9'b1_0000_0000;
    // pmi comb logic
    always_comb begin
        if (state == IDLE && flush_i && ~no_flush_cache_i) begin // we can assert it when sfence.vma
            pmi_enable_i       = 1'b1; // clear cache

            pmi_write_enable_i = 1'b0;
            pmi_no_cache_i     = 1'b0;
            pmi_addr_i         = 32'b0;
            pmi_data_i         = 32'b0;
            pmi_data_sel_i     = 4'b0;
        end else begin
            case (raw_state_next)
                _IDLE, _REPORT_PF: begin
                    pmi_enable_i       = 1'b0;
                    pmi_write_enable_i = 1'b0;
                    pmi_no_cache_i     = 1'b0;
                    pmi_addr_i         = 32'b0;
                    pmi_data_i         = 32'b0;
                    pmi_data_sel_i     = 4'b0;
                end
                FETCH_PTE: begin
                    pmi_enable_i       = 1'b1;
                    pmi_write_enable_i = 1'b0;
                    pmi_no_cache_i     = 1'b0;
                    if (serve) begin
                        pmi_addr_i     = sv32_pt_next + sv32_vpn_next[sv32_level_next] * `PTESIZE;
                    end else begin
                        pmi_addr_i     = sv32_pt_next + sv32_vpn[sv32_level_next] * `PTESIZE;
                    end
                    pmi_data_i         = 32'b0;
                    pmi_data_sel_i     = 4'hf;
                end
                MEM_OPERATION: begin
                    pmi_enable_i       = 1'b1;
                    pmi_write_enable_i = serve ? vmi_write_enable_i : vmi_write_enable_i_buf;
                    pmi_no_cache_i     = !in_sram | (serve ? vmi_no_cache_i : vmi_no_cache_i_buf);
                    pmi_addr_i         = physical_addr;
                    pmi_data_i         = serve ? vmi_data_i : vmi_data_i_buf;
                    pmi_data_sel_i     = serve ? vmi_data_sel_i : vmi_data_sel_i_buf;
                end

            endcase
        end
    end

    always_comb begin
        sv32_pt_next = sv32_pt;
        sv32_level_next = sv32_level;

        if (serve & !sv32_tlb_hit) begin
            sv32_pt_next = satp_i[21:0] * `PAGESIZE;
            sv32_level_next = `LEVELS - 1;
        end else if (state == FETCH_PTE_DONE_NEXT) begin
            sv32_pt_next = pmi_data_o[`PTE_PPN] * `PAGESIZE;
            sv32_level_next = sv32_level - 1;
        end 
    end

    // ack & page fault

    logic [31:0] ack_data_buf;
    logic [31:0] ack_data_next;

    logic [31:0] ack_addr_buf;
    logic [31:0] ack_addr_next;

    assign vmi_ack_o = do_ack;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            ack_data_buf <= 32'b0;
            ack_addr_buf <= 32'b0;
        end else begin
            if (do_ack) begin
                ack_data_buf <= ack_data_next;
                ack_addr_buf <= vmi_addr_i_buf;
            end
        end
    end

    always_comb begin
        ack_data_next = pmi_data_o;
        ack_addr_next = vmi_addr_i_buf;
    end

    assign vmi_data_o     = do_ack ? ack_data_next : ack_data_buf;
    assign vmi_ack_addr_o = do_ack ? ack_addr_next : ack_addr_buf;

    assign page_fault_o = raw_state == _REPORT_PF; // TODO: may optimize

    assign debug_state_o = state;
    assign debug_sv32_pte_o = sv32_pte;

    assign debug_sv32_pf_cause_o = debug_sv32_pte_state_cause;
    assign debug_sv32_tlb_hit_o = sv32_tlb_hit;
    assign debug_sv32_pte_state_o = sv32_pte_state;

    assign debug_paddr_o = physical_addr;

endmodule

