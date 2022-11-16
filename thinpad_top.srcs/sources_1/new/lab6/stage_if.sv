`define BUBBLE_INST 32'h0000_0013
`define BUBBLE_INST_PC 32'h0000_0000


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

module if_next_pc_mux #(
    parameter ADDR_WIDTH = 32
) (
    input wire [ADDR_WIDTH-1:0] pc_plus4_i,
    input wire [ADDR_WIDTH-1:0] pc_branch_target_i,

    input wire do_branch_i,

    output wire [ADDR_WIDTH-1:0] next_pc_o
);

    assign next_pc_o = do_branch_i ? pc_branch_target_i : pc_plus4_i;

endmodule

module adder4 #(
    parameter ADDR_WIDTH = 32
) (
    input wire [ADDR_WIDTH-1:0] data_i,
    output wire [ADDR_WIDTH-1:0] data_plus4_o
);

    assign data_plus4_o = data_i + 4;

endmodule

module if_pipeline_regs (
    input wire        clk_i,
    input wire        rst_i,

    input wire [31:0] inst_pc_i,
    input wire [31:0] inst_i,
    input wire [ 1:0] mode_i,

    input wire        bubble_i,
    input wire        stall_i,

    output reg [31:0] inst_pc_o,
    output reg [31:0] inst_o,
    output reg [ 1:0] mode_o


);

    always @(posedge clk_i) begin
        if (rst_i) begin
            inst_pc_o   <= `BUBBLE_INST_PC;
            inst_o      <= `BUBBLE_INST;
            mode_o      <= 2'b00; // user mode bubble
        end else if (stall_i) begin
            // do nothing
        end else if (bubble_i) begin
            inst_pc_o   <= `BUBBLE_INST_PC;
            inst_o      <= `BUBBLE_INST;
            // mode_o      <= 2'b00; // user mode bubble
        end else begin
            inst_pc_o   <= inst_pc_i;
            inst_o      <= inst_i;
            mode_o      <= mode_i;
        end
    end

endmodule

module if_pc_controller #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32

) (

    input wire branch_last_i,
    input wire branch_take_i,
    input wire if_regs_stall_i,

    input wire [31:0] inst_pc_i,
    input wire [31:0] inst_addr_i,

    output wire pc_stall_o,
    output wire pc_valid_o

);
    assign pc_valid_o = !(branch_last_i | branch_take_i);
    
    logic pc_stall_o_comb;
    
    assign pc_stall_o = pc_stall_o_comb;

    always_comb begin
        if (if_regs_stall_i) begin
            pc_stall_o_comb = 1'b1;
        end else if (branch_last_i) begin
            pc_stall_o_comb = 1'b1;
        end else if (branch_take_i) begin
            pc_stall_o_comb = 1'b0;
        end else if (inst_pc_i != inst_addr_i) begin
            pc_stall_o_comb = 1'b1;
        end else begin
            pc_stall_o_comb = 1'b0;
        end

    end


endmodule

// access instruction memory and buffer the result
module if_im_controller #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input wire clk_i,
    input wire rst_i,

    // control signals
    input wire do_new_req_i,
    input wire [ADDR_WIDTH-1:0] addr_i,

    output wire [ADDR_WIDTH-1:0] data_addr_o,
    output wire [DATA_WIDTH-1:0] data_o,


    // wb signals
    output wire                     wb_cyc_o,
    output wire                     wb_stb_o,
    input  wire                     wb_ack_i,
    output wire  [ADDR_WIDTH-1:0]   wb_adr_o,
    output wire  [DATA_WIDTH-1:0]   wb_dat_o,
    input  wire  [DATA_WIDTH-1:0]   wb_dat_i,
    output wire  [DATA_WIDTH/8-1:0] wb_sel_o,
    output wire                     wb_we_o
);

    // hard wired signals
    assign wb_stb_o = wb_cyc_o;
    assign wb_dat_o = {DATA_WIDTH{1'b0}};
    assign wb_sel_o = {DATA_WIDTH/8{1'b1}};
    assign wb_we_o = 1'b0;

    
    typedef enum logic [2:0] {
      ST_REQUEST,
      ST_WAIT
    } state_t;

    state_t state_reg, state_next;

    // request buffer
    logic [ADDR_WIDTH-1:0] wb_addr_reg;

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            wb_addr_reg <= `BUBBLE_INST_PC;
        end else if (state_reg == ST_REQUEST & do_new_req_i) begin
            wb_addr_reg <= addr_i;
        end
    end
    assign wb_adr_o = state_reg == ST_REQUEST ? addr_i : wb_addr_reg;

    // output buffer
    logic [DATA_WIDTH-1:0] data_reg;
    logic [ADDR_WIDTH-1:0] addr_reg;

    assign data_addr_o = wb_ack_i? wb_adr_o: addr_reg;
    assign data_o = wb_ack_i? wb_dat_i: data_reg;


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

module if_bubble_controller #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input wire [ADDR_WIDTH-1:0] inst_pc_i,
    input wire [ADDR_WIDTH-1:0] inst_addr_i,

    output wire if_bubble_o
);

    assign if_bubble_o = (inst_pc_i != inst_addr_i);

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



