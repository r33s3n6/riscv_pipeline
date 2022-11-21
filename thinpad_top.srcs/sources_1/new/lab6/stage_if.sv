`define BUBBLE_INST 32'h0000_0013
`define BUBBLE_INST_PC 32'h0000_0000


module adder4 #(
    parameter ADDR_WIDTH = 32
) (
    input wire [ADDR_WIDTH-1:0] data_i,
    output wire [ADDR_WIDTH-1:0] data_plus4_o
);

    assign data_plus4_o = data_i + 4;

endmodule



module if_pc_reg #(    
    parameter ADDR_WIDTH = 32,
    parameter PC_INIT = 32'h8000_0000
) (
    input wire clk_i,
    input wire rst_i,

    input  wire [ADDR_WIDTH-1:0] next_pc_i,
    input  wire stall_i,
    output reg [ADDR_WIDTH-1:0] pc_o
);

    always @(posedge clk_i) begin
        if (rst_i) begin
            pc_o <= PC_INIT;
        end else if (!stall_i) begin
            pc_o <= next_pc_i;
        end
    end


endmodule

module if_next_pc_mux (
    input wire [31:0] pc_plus4_i,
    input wire [31:0] pc_branch_target_i,
    input wire [31:0] pc_tvec_i,

    input wire do_branch_i,
    input wire wb_prev_exception_i,

    output wire [31:0] next_pc_o
);

    assign next_pc_o = wb_prev_exception_i ? pc_tvec_i : 
                              do_branch_i  ? pc_branch_target_i
                                           : pc_plus4_i;

endmodule


module if_pc_controller (
    // pc_valid related signals`

    // if last instruction was a branch, then we need to
    // wait for branch result, because next instruction may
    // be invalid or pagefault, so pc is not valid
    input  wire         branch_last_i,

    // if exe-stage report branch not taken, then we can fetch 
    // next instruction, which is pc_now, so pc is valid
    // if branch taken, pc_next is valid, and at this time
    // pc is still invalid
    input  wire         branch_take_i,

    // if exception occurs, pc is not valid, 
    // note that we implement sfence.vma as exception, 
    // so when tlb updates, we are not doing any request
    // we implement mret/sret as exceptions too, therefore 
    // instructions that follows mret/sret will not be fetched,
    // causing no problems
    input  wire         id_next_exception_i,
    input  wire         exe_prev_exception_i,
    input  wire         mem_prev_exception_i,
    input  wire         wb_prev_exception_i,

    // if last instruction stalls, we need to keep the same pc
    input  wire         if_regs_stall_i,
  
    // if instruction fetch not complete, we stall pc until
    // it is complete
    input  wire  [31:0] inst_pc_i,
    input  wire  [31:0] inst_addr_i,

    // Note: pc is not definitely stall when pc is not valid,
    // because we may update pc to valid value this cycle
 
    output logic        pc_stall_o,
    output wire         pc_valid_o

);  
    // not including wb_prev_exception_i
    logic exception_pending;
    assign exception_pending = id_next_exception_i 
                             | exe_prev_exception_i 
                             | mem_prev_exception_i;


    assign pc_valid_o = !(  branch_last_i 
                        |   branch_take_i
                        |   exception_pending
                        |   wb_prev_exception_i
                        );
    
    // TODO: this logic may change if branch prediction is implemented
    always_comb begin
        if (wb_prev_exception_i) begin
            pc_stall_o = 1'b0;
        end else if (branch_take_i) begin // when branch taken, if_regs is bubble, so it 
            pc_stall_o = 1'b0;
        end else if (if_regs_stall_i) begin
            pc_stall_o = 1'b1;
        end else if (branch_last_i) begin
            pc_stall_o = 1'b1;
        end else if (inst_pc_i != inst_addr_i) begin
            pc_stall_o = 1'b1;
        end else if (exception_pending) begin
            pc_stall_o = 1'b1;
        end else begin
            pc_stall_o = 1'b0;
        end

    end


endmodule

// access instruction memory and buffer the result
module if_im_controller #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input  wire                   clk_i,
    input  wire                   rst_i,

    // control signals
    input  wire                   do_new_req_i,
    input  wire  [ADDR_WIDTH-1:0] addr_i,
    input  wire  [DATA_WIDTH-1:0] satp_i,
    input  wire  [           1:0] mode_i,
    input  wire                   sum_i,
 
    output wire  [ADDR_WIDTH-1:0] data_addr_o,
    output wire  [DATA_WIDTH-1:0] data_o,



    // wb signals
    output wire                     wb_cyc_o,
    output wire                     wb_stb_o,
    input  wire                     wb_ack_i,
    output wire  [ADDR_WIDTH-1:0]   wb_adr_o,
    output wire  [DATA_WIDTH-1:0]   wb_dat_o,
    input  wire  [DATA_WIDTH-1:0]   wb_dat_i,
    output wire  [DATA_WIDTH/8-1:0] wb_sel_o,
    output wire                     wb_we_o,

    output wire  [DATA_WIDTH-1:0]   satp_o,
    output wire  [           1:0]   mode_o,
    output wire                     sum_o
);

    // hard wired signals
    assign wb_stb_o = wb_cyc_o;
    assign wb_dat_o = {{(DATA_WIDTH-1){1'b0}}, 1'b1}; // notify that we are instruction fetch
    assign wb_sel_o = {DATA_WIDTH/8{1'b1}};
    assign wb_we_o = 1'b0;

    
    typedef enum logic [2:0] {
      ST_REQUEST,
      ST_WAIT
    } state_t;

    state_t state_reg, state_next;

    // request buffer
    logic [ADDR_WIDTH-1:0] wb_addr_reg;
    logic [ADDR_WIDTH-1:0] satp_buffer; 
    logic [           1:0] mode_buffer; 
    logic                  sum_buffer;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            wb_addr_reg <= `BUBBLE_INST_PC;
            satp_buffer <= 32'h0000_0000;
            mode_buffer <= 2'b11;
            sum_buffer  <= 1'b0;
        end else if (state_reg == ST_REQUEST & do_new_req_i) begin
            wb_addr_reg <= addr_i;
            satp_buffer <= satp_i;
            mode_buffer <= mode_i;
            sum_buffer  <= sum_i;
        end
    end

    // we are at request cycle 1, we update the output signals
    // when we are at request cycle >= 2 or we are not in request state, we keep the output signals the same by buffer
    assign wb_adr_o = (state_reg == ST_REQUEST & do_new_req_i) ? addr_i : wb_addr_reg;
    assign satp_o   = (state_reg == ST_REQUEST & do_new_req_i) ? satp_i : satp_buffer;
    assign mode_o   = (state_reg == ST_REQUEST & do_new_req_i) ? mode_i : mode_buffer;
    assign sum_o    = (state_reg == ST_REQUEST & do_new_req_i) ? sum_i  : sum_buffer;

    // output buffer
    logic [DATA_WIDTH-1:0] data_reg;
    logic [ADDR_WIDTH-1:0] addr_reg;

    assign data_addr_o = wb_ack_i ? wb_adr_o : addr_reg;
    assign data_o      = wb_ack_i ? wb_dat_i : data_reg;

    


    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            data_reg <= `BUBBLE_INST;
            addr_reg <= `BUBBLE_INST_PC;
        end else if (wb_ack_i) begin
            data_reg <= wb_dat_i;
            addr_reg <= wb_adr_o;
        end
    end

    assign wb_cyc_o = (state_reg == ST_REQUEST & do_new_req_i) | (state_reg == ST_WAIT);


    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            state_reg <= ST_REQUEST;
        end else begin
            state_reg <= state_next;
        end
    end

    always_comb begin
        state_next = state_reg;

        case (state_reg)
            ST_REQUEST: begin
                if (do_new_req_i) begin
                    state_next = ST_WAIT;
                end else begin
                    state_next = ST_REQUEST;
                end
            end
            ST_WAIT: begin
                if (wb_ack_i) begin
                    state_next = ST_REQUEST;
                end
            end
        endcase
    end

endmodule

module if_pipeline_regs (
    input wire        clk_i,
    input wire        rst_i,

    input wire        bubble_i,
    input wire        stall_i,

    output reg        nop_o,

    input wire [ 1:0] mode_i,
    output reg [ 1:0] mode_o,

    input wire [31:0] inst_pc_i,
    output reg [31:0] inst_pc_o,

    input wire [31:0] inst_i,
    output reg [31:0] inst_o,

    input wire [31:0] pc_plus4_i,
    output reg [31:0] pc_plus4_o,

    input wire        exception_i,
    output reg        exception_o,

    input wire [ 1:0] trap_mode_i,
    output reg [ 1:0] trap_mode_o,

    input wire [31:0] mcause_i,
    output reg [31:0] mcause_o,

    input wire [31:0] mtval_i,
    output reg [31:0] mtval_o



);

    

    always @(posedge clk_i) begin
        if (rst_i) begin
            inst_pc_o   <= `BUBBLE_INST_PC;
            pc_plus4_o  <= `BUBBLE_INST_PC;
            inst_o      <= `BUBBLE_INST;
            mode_o      <= 2'b00; // user mode bubble
            exception_o <= 1'b0;
            trap_mode_o <= 2'b00;
            mcause_o    <= 32'b0;
            mtval_o     <= 32'b0;
            nop_o       <= 1'b1;
        end else if (stall_i) begin
            // do nothing
        end else if (bubble_i) begin
            inst_pc_o   <= `BUBBLE_INST_PC;
            pc_plus4_o  <= `BUBBLE_INST_PC;
            inst_o      <= `BUBBLE_INST;
            // mode_o      <= 2'b00; // user mode bubble
            exception_o <= 1'b0;
            trap_mode_o <= 2'b00;
            mcause_o    <= 32'b0;
            mtval_o     <= 32'b0;
            nop_o       <= 1'b1;
        end else begin
            inst_pc_o   <= inst_pc_i;
            pc_plus4_o  <= pc_plus4_i;
            inst_o      <= inst_i;
            mode_o      <= mode_i;
            exception_o <= exception_i;
            trap_mode_o <= trap_mode_i;
            mcause_o    <= mcause_i;
            mtval_o     <= mtval_i;
            nop_o       <= 1'b0;
        end
    end

endmodule

module if_bubble_controller (
    input wire [31:0] inst_pc_i,
    input wire [31:0] inst_addr_i,

    input wire        id_next_exception_i,
    input wire        exe_next_exception_i,
    input wire        mem_next_exception_i,
    input wire        wb_prev_exception_i,

    output wire if_bubble_o
);

    assign if_bubble_o = (inst_pc_i != inst_addr_i) 
                        || id_next_exception_i
                        || exe_next_exception_i
                        || mem_next_exception_i
                        || wb_prev_exception_i;

endmodule



module if_stall_controller #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input wire id_stall_i,
    input wire id_wait_reg_i,

    output wire if_stall_o
);

    assign if_stall_o = id_stall_i || id_wait_reg_i;              

endmodule



