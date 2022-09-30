`default_nettype none


module instruction_decoder(
    input  wire [31:0] instruction,
    output wire [15:0] imm16,
    output wire [ 4:0] rd,
    output wire [ 4:0] rs1,
    output wire [ 4:0] rs2,
    output wire [ 3:0] op,
    output wire [ 2:0] op_type
);
  
  assign      rd = instruction[11: 7];
  assign     rs1 = instruction[19:15];
  assign     rs2 = instruction[24:20];
  assign      op = instruction[ 6: 3];
  assign op_type = instruction[ 2: 0];
  assign   imm16 = instruction[31:16];
endmodule

module reg_file(
  input  wire         clk,
  input  wire         rst,
  input  wire [ 4: 0] waddr,
  input  wire [15: 0] wdata,
  input  wire         we,
  input  wire [ 4: 0] raddr_a,
  input  wire [ 4: 0] raddr_b,
  output wire [15: 0] rdata_a,
  output wire [15: 0] rdata_b
);
  // real registers
  reg [15:0] reg_file [0:31];


  // data buffer

  //logic [15:0] rdata_a_reg;
  //logic [15:0] rdata_b_reg;

  //assign rdata_a = rdata_a_reg;
  //assign rdata_b = rdata_b_reg;
  assign rdata_a = reg_file[raddr_a];
  assign rdata_b = reg_file[raddr_b];

  

  always_ff @(posedge clk) begin
    if (rst) begin
      reg_file <= '{default: '0};
    end else begin
      //rdata_a_reg <= reg_file[raddr_a];
      //rdata_b_reg <= reg_file[raddr_b];
      // dont write to zero register
      if (we && waddr) begin
        reg_file[waddr] <= wdata;
      end
    end
  end

endmodule

module alu(
  input  wire signed [15:0] a,
  input  wire        [15:0] b,
  input  wire        [ 3:0] op,
  output logic       [15:0] result
);
  parameter ADD = 4'd1;
  parameter SUB = 4'd2;
  parameter AND = 4'd3;
  parameter OR  = 4'd4;
  parameter XOR = 4'd5;
  parameter NOT = 4'd6;
  parameter SLL = 4'd7;
  parameter SRL = 4'd8;
  parameter SRA = 4'd9;
  parameter ROL = 4'd10;

  always_comb begin
    case (op)
      ADD: result = a + b;
      SUB: result = a - b;
      AND: result = a & b;
      OR:  result = a | b;
      XOR: result = a ^ b;
      NOT: result = ~a;
      SLL: result = a << b[3:0];
      SRL: result = a >> b[3:0];
      SRA: result = a >>> b[3:0];
      ROL: result = a << b[3:0] | a >> (16 - b[3:0]);
      default: result = 16'b0;
    endcase
  end
    
endmodule


module controller (
    input wire clk,
    input wire rst,

    // to register file
    output reg  [ 4:0]  rf_raddr_a,
    input  wire [15:0]  rf_rdata_a,
    output reg  [ 4:0]  rf_raddr_b,
    input  wire [15:0]  rf_rdata_b,
    output reg  [ 4:0]  rf_waddr,
    output reg  [15:0]  rf_wdata,
    output reg          rf_we,

    // to alu
    output wire  [15:0] alu_a,
    output wire  [15:0] alu_b,
    output wire  [ 3:0] alu_op,
    input  wire  [15:0] alu_y,

    input  wire        step,     // confirm
    input  wire [31:0] instruction,
    output reg  [15:0] leds
);
  parameter rtype = 3'b001;
  parameter itype = 3'b010;

  parameter iop_poke = 4'b0001;
  parameter iop_peek = 4'b0010;

  logic [ 4:0] next_rf_raddr_a_comb;
  logic [ 4:0] next_rf_raddr_b_comb;
  logic [ 4:0] next_rf_waddr_comb;
  logic [15:0] next_rf_wdata_comb;
  logic        next_rf_we_comb;

  logic [15:0] next_leds_comb;


  logic [31:0] inst_reg; //register buffer
  logic [31:0] next_inst_comb;


  logic [15:0] imm16_comb;
  logic [ 4:0] rd_comb;
  logic [ 4:0] rs1_comb;
  logic [ 4:0] rs2_comb;
  logic [ 3:0] op_comb;
  logic [ 2:0] op_type_comb;

  // hardwire the alu with the data read from the register file
  assign alu_a  = rf_rdata_a;
  assign alu_b  = rf_rdata_b;
  assign alu_op = op_comb;

  instruction_decoder inst_decoder(
    .instruction(inst_reg),
    .imm16(imm16_comb),
    .rd(rd_comb),
    .rs1(rs1_comb),
    .rs2(rs2_comb),
    .op(op_comb),
    .op_type(op_type_comb)
  );

  typedef enum logic [3:0] {
    ST_INIT,
    ST_DECODE,
    ST_CALC,
    ST_READ_REG,
    ST_WRITE_REG
  } state_t;

  state_t state_reg;
  state_t next_state_comb;

  always_ff @(posedge clk) begin
    if (rst) begin
      state_reg            <= ST_INIT;
      inst_reg             <= 16'b0;
      rf_raddr_a           <= 5'b0;
      rf_raddr_b           <= 5'b0;
      rf_waddr             <= 5'b0;
      rf_wdata             <= 16'b0;
      rf_we                <= 1'b0;
      leds                 <= 16'b0;

    end else begin
      state_reg  <= next_state_comb;
      inst_reg   <= next_inst_comb;
      rf_raddr_a <= next_rf_raddr_a_comb;
      rf_raddr_b <= next_rf_raddr_b_comb;
      rf_waddr   <= next_rf_waddr_comb;
      rf_wdata   <= next_rf_wdata_comb;
      rf_we      <= next_rf_we_comb;
      leds       <= next_leds_comb;

    end
  end

  always_comb begin
    // default set to themselves
    next_state_comb      = state_reg;
    next_inst_comb       = inst_reg;
    next_rf_raddr_a_comb = rf_raddr_a;
    next_rf_raddr_b_comb = rf_raddr_b;
    next_rf_waddr_comb   = rf_waddr;
    next_rf_wdata_comb   = rf_wdata;
    next_rf_we_comb      = rf_we;
    next_leds_comb       = leds;

    case(state_reg)
      ST_INIT: begin
        if(step) begin
          next_state_comb = ST_DECODE;
          next_inst_comb  = instruction;
        end
      end
      ST_DECODE: begin
        // we got the decoder output
        if (op_type_comb == rtype) begin
          next_state_comb      = ST_CALC;
          next_rf_raddr_a_comb = rs1_comb;
          next_rf_raddr_b_comb = rs2_comb;
        end else if (op_type_comb == itype) begin
          if(op_comb == iop_peek) begin
            next_state_comb      = ST_READ_REG;
            next_rf_raddr_a_comb = rd_comb;
          end else if (op_comb == iop_poke) begin
            next_state_comb      = ST_WRITE_REG;
            next_rf_waddr_comb   = rd_comb;
            next_rf_wdata_comb   = imm16_comb;
            next_rf_we_comb      = 1'b1;
          end else begin
            // invalid instruction
            next_state_comb = ST_INIT;
          end
        end else begin
          // invalid instruction
          next_state_comb = ST_INIT;
        end
      end
      ST_CALC: begin
        // we got the result, write back
        next_state_comb      = ST_WRITE_REG;
        next_rf_waddr_comb   = rd_comb;
        next_rf_wdata_comb   = alu_y;
        next_rf_we_comb      = 1'b1;
      end
      ST_READ_REG: begin
        // display the value
        next_state_comb = ST_INIT;
        next_leds_comb  = rf_rdata_a;
      end
      ST_WRITE_REG: begin
        next_state_comb = ST_INIT;
      end
      default: begin
        next_state_comb = ST_INIT;
      end
    endcase
  end


endmodule

module lab3_top (
    input wire clk_50M,     // 50MHz 时钟输入
    input wire clk_11M0592, // 11.0592MHz 时钟输入（备用，可不用）

    input wire push_btn,  // BTN5 按钮开关，带消抖电路，按下时为 1
    input wire reset_btn, // BTN6 复位按钮，带消抖电路，按下时为 1

    input  wire [ 3:0] touch_btn,  // BTN1~BTN4，按钮开关，按下时为 1
    input  wire [31:0] dip_sw,     // 32 位拨码开关，拨到“ON”时为 1
    output wire [15:0] leds,       // 16 位 LED，输出时 1 点亮
    output wire [ 7:0] dpy0,       // 数码管低位信号，包括小数点，输出 1 点亮
    output wire [ 7:0] dpy1,       // 数码管高位信号，包括小数点，输出 1 点亮

    // CPLD 串口控制器信号
    output wire uart_rdn,        // 读串口信号，低有效
    output wire uart_wrn,        // 写串口信号，低有效
    input  wire uart_dataready,  // 串口数据准备好
    input  wire uart_tbre,       // 发送数据标志
    input  wire uart_tsre,       // 数据发送完毕标志

    // BaseRAM 信号
    inout wire [31:0] base_ram_data,  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共享
    output wire [19:0] base_ram_addr,  // BaseRAM 地址
    output wire [3:0] base_ram_be_n,  // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire base_ram_ce_n,  // BaseRAM 片选，低有效
    output wire base_ram_oe_n,  // BaseRAM 读使能，低有效
    output wire base_ram_we_n,  // BaseRAM 写使能，低有效

    // ExtRAM 信号
    inout wire [31:0] ext_ram_data,  // ExtRAM 数据
    output wire [19:0] ext_ram_addr,  // ExtRAM 地址
    output wire [3:0] ext_ram_be_n,  // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire ext_ram_ce_n,  // ExtRAM 片选，低有效
    output wire ext_ram_oe_n,  // ExtRAM 读使能，低有效
    output wire ext_ram_we_n,  // ExtRAM 写使能，低有效

    // 直连串口信号
    output wire txd,  // 直连串口发送端
    input  wire rxd,  // 直连串口接收端

    // Flash 存储器信号，参考 JS28F640 芯片手册
    output wire [22:0] flash_a,  // Flash 地址，a0 仅在 8bit 模式有效，16bit 模式无意义
    inout wire [15:0] flash_d,  // Flash 数据
    output wire flash_rp_n,  // Flash 复位信号，低有效
    output wire flash_vpen,  // Flash 写保护信号，低电平时不能擦除、烧写
    output wire flash_ce_n,  // Flash 片选信号，低有效
    output wire flash_oe_n,  // Flash 读使能信号，低有效
    output wire flash_we_n,  // Flash 写使能信号，低有效
    output wire flash_byte_n, // Flash 8bit 模式选择，低有效。在使用 flash 的 16 位模式时请设为 1

    // USB 控制器信号，参考 SL811 芯片手册
    output wire sl811_a0,
    // inout  wire [7:0] sl811_d,     // USB 数据线与网络控制器的 dm9k_sd[7:0] 共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    // 网络控制器信号，参考 DM9000A 芯片手册
    output wire dm9k_cmd,
    inout wire [15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input wire dm9k_int,

    // 图像输出信号
    output wire [2:0] video_red,    // 红色像素，3 位
    output wire [2:0] video_green,  // 绿色像素，3 位
    output wire [1:0] video_blue,   // 蓝色像素，2 位
    output wire       video_hsync,  // 行同步（水平同步）信号
    output wire       video_vsync,  // 场同步（垂直同步）信号
    output wire       video_clk,    // 像素时钟输出
    output wire       video_de      // 行数据有效信号，用于区分消隐区
);

  /* =========== Demo code begin =========== */

  // PLL 分频示例
  logic locked, clk_10M, clk_20M;
  pll_example clock_gen (
      // Clock in ports
      .clk_in1(clk_50M),  // 外部时钟输入
      // Clock out ports
      .clk_out1(clk_10M),  // 时钟输出 1，频率在 IP 配置界面中设置
      .clk_out2(clk_20M),  // 时钟输出 2，频率在 IP 配置界面中设置
      // Status and control signals
      .reset(reset_btn),  // PLL 复位输入
      .locked(locked)  // PLL 锁定指示输出，"1"表示时钟稳定，
                       // 后级电路复位信号应当由它生成（见下）
  );

  logic reset_of_clk10M;
  // 异步复位，同步释放，将 locked 信号转为后级电路的复位 reset_of_clk10M
  always_ff @(posedge clk_10M or negedge locked) begin
    if (~locked) reset_of_clk10M <= 1'b1;
    else reset_of_clk10M <= 1'b0;
  end

  /* =========== Demo code end =========== */
  logic [ 4:0] waddr_comb;
  logic [15:0] wdata_comb;
  logic        we_comb;
  logic [ 4:0] raddr_a_comb;
  logic [ 4:0] raddr_b_comb;
  logic [15:0] rdata_a_comb;
  logic [15:0] rdata_b_comb;
  reg_file reg_file_inst (
      .clk(clk_10M),
      .rst(reset_of_clk10M),
      .waddr(waddr_comb),
      .wdata(wdata_comb),
      .we(we_comb),
      .raddr_a(raddr_a_comb),
      .raddr_b(raddr_b_comb),
      .rdata_a(rdata_a_comb),
      .rdata_b(rdata_b_comb)
  );

  logic [15:0] alu_a_comb;
  logic [15:0] alu_b_comb;
  logic [3: 0] alu_op_comb;
  logic [15:0] alu_result_comb;

  alu alu_inst (
      .a(alu_a_comb),
      .b(alu_b_comb),
      .op(alu_op_comb),
      .result(alu_result_comb)
  );

  logic push_btn_trigger_comb;

   trigger trigger_inst (
      .clk    (clk_10M),
      .rst    (reset_of_clk10M),
      .btn    (push_btn),
      .trigger(push_btn_trigger_comb)
  );

  controller controller_inst (
      .clk(clk_10M),
      .rst(reset_of_clk10M),

      // register file
      .rf_waddr(waddr_comb),
      .rf_wdata(wdata_comb),
      .rf_we(we_comb),
      .rf_raddr_a(raddr_a_comb),
      .rf_raddr_b(raddr_b_comb),
      .rf_rdata_a(rdata_a_comb),
      .rf_rdata_b(rdata_b_comb),

      //alu
      .alu_a(alu_a_comb),
      .alu_b(alu_b_comb),
      .alu_op(alu_op_comb),
      .alu_y(alu_result_comb),

      // io
      .step(push_btn_trigger_comb),
      .instruction(dip_sw),
      .leds(leds));





endmodule
