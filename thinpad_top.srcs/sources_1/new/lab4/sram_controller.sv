// Note: you must ensure that wb_* signals input delay is not too long
module sram_controller #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,

    parameter SRAM_ADDR_WIDTH = 20,
    parameter SRAM_DATA_WIDTH = 32,

    localparam SRAM_BYTES = SRAM_DATA_WIDTH / 8,
    localparam SRAM_BYTE_WIDTH = $clog2(SRAM_BYTES)
) (
    // clk and reset
    input  wire clk_i,
    input  wire rst_i,

    // wishbone slave interface
    input  wire wb_cyc_i,
    input  wire wb_stb_i,
    output wire wb_ack_o,
    input  wire [ADDR_WIDTH-1:0] wb_adr_i,
    input  wire [DATA_WIDTH-1:0] wb_dat_i,
    output wire [DATA_WIDTH-1:0] wb_dat_o,
    input  wire [DATA_WIDTH/8-1:0] wb_sel_i,
    input  wire wb_we_i,

    // sram interface
    output wire [SRAM_ADDR_WIDTH-1:0] sram_addr,
    inout  wire [SRAM_DATA_WIDTH-1:0] sram_data,
    output wire sram_ce_n,
    output wire sram_oe_n,
    output wire sram_we_n,
    output wire [SRAM_BYTES-1:0] sram_be_n,

    output wire [SRAM_DATA_WIDTH-1:0] debug_data_out,
    output wire [SRAM_DATA_WIDTH-1:0] debug_data_in,
    output wire                       debug_data_is_in
);

  // set up inout
  logic [SRAM_DATA_WIDTH-1:0] sram_data_to_comb;
  logic [SRAM_DATA_WIDTH-1:0] sram_data_from_comb;
  logic sram_data_t_comb; // z

  // select bit mask
  logic [DATA_WIDTH-1:0] sel_bit_mask_comb;

  // generate bit mask
  generate 
    for (genvar i = 0; i < DATA_WIDTH; i = i + 8) begin : gen_sel_bit_mask
      assign sel_bit_mask_comb[i+7:i] = {8{wb_sel_i[i/8]}};
    end
  endgenerate
  

  assign sram_data = sram_data_t_comb ? {SRAM_DATA_WIDTH{1'bz}} : sram_data_to_comb;
  assign sram_data_from_comb = sel_bit_mask_comb & sram_data;



  // define states
  typedef enum logic [1:0] {
    STATE_RW_1 = 0,
    STATE_READ_2 = 1,
    STATE_WRITE_2 = 2,
    STATE_WRITE_3 = 3
  } state_t;

  // define state register
  state_t state_reg, next_state_comb;
  logic ready_reg, next_ready_comb;

  assign wb_ack_o = ready_reg;

  // define output buffers
  logic [DATA_WIDTH-1:0] wb_dat_o_reg;
  logic [DATA_WIDTH-1:0] next_wb_dat_o_comb;
  // assign wb_dat_o = wb_dat_o_reg;
  assign wb_dat_o = next_wb_dat_o_comb;
  

  // sram control

  // translate address
  assign sram_addr = wb_adr_i[SRAM_BYTE_WIDTH+SRAM_ADDR_WIDTH:SRAM_BYTE_WIDTH];

  assign sram_data_to_comb = wb_dat_i;

  // byte select
  assign sram_be_n = ~wb_sel_i;


  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_reg <= STATE_RW_1;
      ready_reg <= 1'b0;
      wb_dat_o_reg <= 1'b0;
    end else begin
      state_reg <= next_state_comb;
      ready_reg <= next_ready_comb;
      wb_dat_o_reg <= next_wb_dat_o_comb;
    end
  end

  logic sram_ce_n_comb;
  logic sram_oe_n_comb;
  logic sram_we_n_comb;

  assign sram_ce_n = sram_ce_n_comb;
  assign sram_oe_n = sram_oe_n_comb;
  assign sram_we_n = sram_we_n_comb;

  // state machine
  always_comb begin
    // default behaviour for buffers
    next_wb_dat_o_comb = wb_dat_o_reg;

    case(state_reg)
      STATE_RW_1: begin
        if (wb_cyc_i && wb_stb_i) begin
          if (wb_we_i) begin // current state is STATE_WRITE_1
            next_state_comb = STATE_WRITE_2;
            sram_ce_n_comb = 1'b0;
            sram_we_n_comb = 1'b1;
            sram_oe_n_comb = 1'b1;
            sram_data_t_comb = 1'b0; // we put data on the inout
            next_ready_comb = 1'b0; // clear ready signal
          end else begin // current state is STATE_READ_1
            next_state_comb = STATE_READ_2;
            sram_ce_n_comb = 1'b0;
            sram_we_n_comb = 1'b1;
            sram_oe_n_comb = 1'b0;
            sram_data_t_comb = 1'b1; 
            next_ready_comb = 1'b1; // set ready signal
          end
        end else begin
          next_state_comb = STATE_RW_1;
          sram_ce_n_comb = 1'b1;
          sram_we_n_comb = 1'b1;
          sram_oe_n_comb = 1'b1;
          sram_data_t_comb = 1'b1;
          next_ready_comb = 1'b0; // clear ready signal
        end
        
      end
      STATE_READ_2: begin
        next_state_comb = STATE_RW_1;
        next_ready_comb = 1'b0; 
        sram_ce_n_comb = 1'b0;
        sram_we_n_comb = 1'b1;
        sram_oe_n_comb = 1'b0;
        sram_data_t_comb = 1'b1; 
        next_wb_dat_o_comb = sram_data_from_comb;
      end
      STATE_WRITE_2: begin
        next_state_comb = STATE_WRITE_3;
        next_ready_comb = 1'b1; // set ready signal
        sram_ce_n_comb = 1'b0;
        sram_we_n_comb = 1'b0;
        sram_oe_n_comb = 1'b1;
        sram_data_t_comb = 1'b0;
      end
      STATE_WRITE_3: begin
        next_state_comb = STATE_RW_1;
        next_ready_comb = 1'b0; 
        sram_ce_n_comb = 1'b0;
        sram_we_n_comb = 1'b1;
        sram_oe_n_comb = 1'b1;
        sram_data_t_comb = 1'b0;
      end
      default: begin
        next_state_comb = STATE_RW_1;
        next_ready_comb = 1'b0;
        sram_ce_n_comb = 1'b1;
        sram_oe_n_comb = 1'b1;
        sram_we_n_comb = 1'b1;
        sram_data_t_comb = 1'b1;
      end
    endcase
  
  end

  assign debug_data_in = sram_data;
  assign debug_data_out = sram_data_to_comb;
  assign debug_data_is_in = sram_data_t_comb;

endmodule
