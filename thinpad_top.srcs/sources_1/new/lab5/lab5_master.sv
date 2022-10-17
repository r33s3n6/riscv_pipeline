module lab5_master #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input wire clk_i,
    input wire rst_i,

    input wire [ADDR_WIDTH-1:0] base_addr_i,

    // wishbone master
    output reg  wb_cyc_o,
    output reg  wb_stb_o,
    input  wire wb_ack_i,
    output reg  [ADDR_WIDTH-1:0] wb_adr_o,
    output reg  [DATA_WIDTH-1:0] wb_dat_o,
    input  wire [DATA_WIDTH-1:0] wb_dat_i,
    output reg  [DATA_WIDTH/8-1:0] wb_sel_o,
    output reg  wb_we_o
);


localparam UART_DATA_ADDR =   32'h10000000;
localparam UART_STATUS_ADDR = 32'h10000005;
localparam UART_DATA_ADDR_SEL =   4'b0001;
localparam UART_STATUS_ADDR_SEL = 4'b0010;
localparam UART_FLAG_READ_READY =   0 + 8;
localparam UART_FLAG_WRITE_READY =  5 + 8;

localparam REPEAT = 4'd10;

// select bit mask
logic [DATA_WIDTH-1:0] sel_bit_mask_comb;

// generate bit mask
generate 
  for (genvar i = 0; i < DATA_WIDTH; i = i + 8) begin : gen_sel_bit_mask
    assign sel_bit_mask_comb[i+7:i] = {8{wb_sel_o[i/8]}};
  end
endgenerate
  

// next state logic
logic wb_cyc_o_next;
logic wb_stb_o_next;
logic [ADDR_WIDTH-1:0] wb_adr_o_next;
logic [DATA_WIDTH-1:0] wb_dat_o_next;
logic [DATA_WIDTH/8-1:0] wb_sel_o_next;
logic wb_we_o_next;

always_ff @(posedge clk_i) begin
    if (rst_i) begin
        wb_cyc_o <= 1'b0;
        wb_stb_o <= 1'b0;
        wb_adr_o <= 32'h0;
        wb_dat_o <= 32'h0;
        wb_sel_o <= 4'b0;
        wb_we_o <= 1'b0;
    end else begin
        wb_cyc_o <= wb_cyc_o_next;
        wb_stb_o <= wb_stb_o_next;
        wb_adr_o <= wb_adr_o_next;
        wb_dat_o <= wb_dat_o_next;
        wb_sel_o <= wb_sel_o_next;
        wb_we_o <= wb_we_o_next;
    end
end


// loop for 10 times
logic [3:0] count_reg;
logic [3:0] count_next;

logic [ADDR_WIDTH-1:0] addr_reg;
logic [ADDR_WIDTH-1:0] addr_next;

logic [DATA_WIDTH-1:0] data_buf_reg;
logic [DATA_WIDTH-1:0] data_buf_next;

// define states
typedef enum logic [2:0] {
  ST_UART_WAIT_READ,
  ST_UART_READ,
  ST_UART_WAIT_WRITE,
  ST_UART_WRITE,

  ST_SRAM_WRITE,

  ST_DONE
} state_t;

// define state register
state_t state_reg;
state_t state_next;

// wishbone output
always_comb begin
  wb_cyc_o_next = wb_cyc_o;
  wb_stb_o_next = wb_stb_o;
  wb_adr_o_next = wb_adr_o;
  wb_dat_o_next = wb_dat_o;
  wb_sel_o_next = wb_sel_o;
  wb_we_o_next = wb_we_o;

  case (state_next)
    // read status register
    ST_UART_WAIT_READ, ST_UART_WAIT_WRITE: begin
      wb_cyc_o_next = 1'b1;
      wb_stb_o_next = 1'b1;
      wb_adr_o_next = UART_STATUS_ADDR;
      wb_sel_o_next = UART_STATUS_ADDR_SEL;
      wb_we_o_next  = 1'b0;
    end

    ST_UART_READ: begin
      wb_cyc_o_next = 1'b1;
      wb_stb_o_next = 1'b1;
      wb_adr_o_next = UART_DATA_ADDR;
      wb_sel_o_next = UART_DATA_ADDR_SEL;
      wb_we_o_next  = 1'b0;
    end

    ST_UART_WRITE: begin
      wb_cyc_o_next = 1'b1;
      wb_stb_o_next = 1'b1;
      wb_adr_o_next = UART_DATA_ADDR;
      wb_sel_o_next = UART_DATA_ADDR_SEL;
      wb_we_o_next  = 1'b1;
      wb_dat_o_next = data_buf_next;
    end

    ST_SRAM_WRITE: begin
      // do write sram request
      wb_cyc_o_next = 1'b1;
      wb_stb_o_next = 1'b1;
      wb_adr_o_next = addr_reg;
      wb_sel_o_next = 4'b0001; // write only 1 byte
      wb_we_o_next  = 1'b1;
      wb_dat_o_next = data_buf_next;
    end

    ST_DONE: begin
      wb_cyc_o_next = 1'b0;
      wb_stb_o_next = 1'b0;
    end
  endcase

end

// state machine
always_comb begin
  state_next = state_reg;
  data_buf_next = data_buf_reg;

  case (state_reg)
    ST_UART_WAIT_READ: begin
      if (wb_ack_i) begin
        if (wb_dat_i[UART_FLAG_READ_READY]) begin
          state_next = ST_UART_READ;
        end else begin
          state_next = ST_UART_WAIT_READ;
        end
      end
    end

    ST_UART_READ: begin
      if (wb_ack_i) begin
        data_buf_next = wb_dat_i & sel_bit_mask_comb;
        state_next = ST_SRAM_WRITE;
      end
    end


    ST_SRAM_WRITE: begin
      if (wb_ack_i) begin
        state_next = ST_UART_WAIT_WRITE;
      end
    end

    ST_UART_WAIT_WRITE: begin
      if (wb_ack_i) begin
        if (wb_dat_i[UART_FLAG_WRITE_READY]) begin
          state_next = ST_UART_WRITE;
        end else begin
          state_next = ST_UART_WAIT_WRITE;
        end
      end
    end

    ST_UART_WRITE: begin
      if (wb_ack_i) begin
        if (count_next == REPEAT) begin
          state_next = ST_DONE;
        end else begin
          state_next = ST_UART_WAIT_READ;
        end
      end
    end

    ST_DONE: begin
      state_next = ST_DONE;
    end
  endcase
end

// count and addr register next
always_comb begin
  count_next = count_reg;
  addr_next = addr_reg;
  if (state_reg == ST_UART_WRITE && wb_ack_i) begin
      count_next = count_reg + 1'd1;
      addr_next = addr_reg + 3'd4;
  end
end

// register
always_ff @(posedge clk_i) begin
  if (rst_i) begin
    state_reg <= ST_UART_WAIT_READ;
    count_reg <= 4'd0;
    addr_reg <= {base_addr_i[ADDR_WIDTH-1:2], 2'b00}; // align to 4 bytes
    data_buf_reg <= {DATA_WIDTH{1'b0}};
  end else begin
    state_reg <= state_next;
    count_reg <= count_next;
    addr_reg <= addr_next;
    data_buf_reg <= data_buf_next;
  end
end




endmodule
