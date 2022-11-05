`default_nettype none



`timescale 1ns / 1ns
module lab6_top (
    input wire clk_50M,     // 50MHz 时钟输入
    input wire clk_11M0592, // 11.0592MHz 时钟输入（备用，可不用）

    input wire push_btn,  // BTN5 按钮�?????关，带消抖电路，按下时为 1
    input wire reset_btn, // BTN6 复位按钮，带消抖电路，按下时�????? 1

    input  wire [ 3:0] touch_btn,  // BTN1~BTN4，按钮开关，按下时为 1
    input  wire [31:0] dip_sw,     // 32 位拨码开关，拨到“ON”时�????? 1
    output wire [15:0] leds,       // 16 �????? LED，输出时 1 点亮
    output wire [ 7:0] dpy0,       // 数码管低位信号，包括小数点，输出 1 点亮
    output wire [ 7:0] dpy1,       // 数码管高位信号，包括小数点，输出 1 点亮

    // CPLD 串口控制器信�?????
    output wire uart_rdn,        // 读串口信号，低有�?????
    output wire uart_wrn,        // 写串口信号，低有�?????
    input  wire uart_dataready,  // 串口数据准备�?????
    input  wire uart_tbre,       // 发�?�数据标�?????
    input  wire uart_tsre,       // 数据发�?�完毕标�?????

    // BaseRAM 信号
    inout wire [31:0] base_ram_data,  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共�?????
    output wire [19:0] base_ram_addr,  // BaseRAM 地址
    output wire [3:0] base_ram_be_n,  // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持�????? 0
    output wire base_ram_ce_n,  // BaseRAM 片�?�，低有�?????
    output wire base_ram_oe_n,  // BaseRAM 读使能，低有�?????
    output wire base_ram_we_n,  // BaseRAM 写使能，低有�?????

    // ExtRAM 信号
    inout wire [31:0] ext_ram_data,  // ExtRAM 数据
    output wire [19:0] ext_ram_addr,  // ExtRAM 地址
    output wire [3:0] ext_ram_be_n,  // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持�????? 0
    output wire ext_ram_ce_n,  // ExtRAM 片�?�，低有�?????
    output wire ext_ram_oe_n,  // ExtRAM 读使能，低有�?????
    output wire ext_ram_we_n,  // ExtRAM 写使能，低有�?????

    // 直连串口信号
    output wire txd,  // 直连串口发�?�端
    input  wire rxd,  // 直连串口接收�?????

    // Flash 存储器信号，参�?? JS28F640 芯片手册
    output wire [22:0] flash_a,  // Flash 地址，a0 仅在 8bit 模式有效�?????16bit 模式无意�?????
    inout wire [15:0] flash_d,  // Flash 数据
    output wire flash_rp_n,  // Flash 复位信号，低有效
    output wire flash_vpen,  // Flash 写保护信号，低电平时不能擦除、烧�?????
    output wire flash_ce_n,  // Flash 片�?�信号，低有�?????
    output wire flash_oe_n,  // Flash 读使能信号，低有�?????
    output wire flash_we_n,  // Flash 写使能信号，低有�?????
    output wire flash_byte_n, // Flash 8bit 模式选择，低有效。在使用 flash �????? 16 位模式时请设�????? 1

    // USB 控制器信号，参�?? SL811 芯片手册
    output wire sl811_a0,
    // inout  wire [7:0] sl811_d,     // USB 数据线与网络控制器的 dm9k_sd[7:0] 共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    // 网络控制器信号，参�?? DM9000A 芯片手册
    output wire dm9k_cmd,
    inout wire [15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input wire dm9k_int,

    // 图像输出信号
    output wire [2:0] video_red,    // 红色像素�?????3 �?????
    output wire [2:0] video_green,  // 绿色像素�?????3 �?????
    output wire [1:0] video_blue,   // 蓝色像素�?????2 �?????
    output wire       video_hsync,  // 行同步（水平同步）信�?????
    output wire       video_vsync,  // 场同步（垂直同步）信�?????
    output wire       video_clk,    // 像素时钟输出
    output wire       video_de      // 行数据有效信号，用于区分消隐�?????
);

  /* =========== Demo code begin =========== */

  // PLL 分频示例
  logic locked, clk_10M, clk_60M;
  pll_example clock_gen (
      // Clock in ports
      .clk_in1(clk_50M),  // 外部时钟输入
      // Clock out ports
      .clk_out1(clk_10M),  // 时钟输出 1，频率在 IP 配置界面中设�?????
      .clk_out2(clk_60M),  // 时钟输出 2，频率在 IP 配置界面中设�?????
      // Status and control signals
      .reset(reset_btn),  // PLL 复位输入
      .locked(locked)  // PLL 锁定指示输出�?????"1"表示时钟稳定�?????
                       // 后级电路复位信号应当由它生成（见下）
  );

  logic reset_of_clk10M;
  always_ff @(posedge clk_10M or negedge locked) begin
    if (~locked) reset_of_clk10M <= 1'b1;
    else reset_of_clk10M <= 1'b0;
  end

  logic reset_of_clk60M;
  always_ff @(posedge clk_60M or negedge locked) begin
    if (~locked) reset_of_clk60M <= 1'b1;
    else reset_of_clk60M <= 1'b0;
  end

  /* =========== Demo code end =========== */

  logic sys_clk;
  logic sys_rst;

  assign sys_clk = clk_60M;
  assign sys_rst = reset_of_clk60M;

  // 本实验不使用 CPLD 串口，禁用防止�?�线冲突
  assign uart_rdn = 1'b1;
  assign uart_wrn = 1'b1;

  /* =========== Wishbone Arbiter begin =========== */
  logic        wbm0_cyc_o;
  logic        wbm0_stb_o;
  logic        wbm0_ack_i;
  logic [31:0] wbm0_adr_o;
  logic [31:0] wbm0_dat_o;
  logic [31:0] wbm0_dat_i;
  logic [ 3:0] wbm0_sel_o;
  logic        wbm0_we_o;

  logic        wbm1_cyc_o;
  logic        wbm1_stb_o;
  logic        wbm1_ack_i;
  logic [31:0] wbm1_adr_o;
  logic [31:0] wbm1_dat_o;
  logic [31:0] wbm1_dat_i;
  logic [ 3:0] wbm1_sel_o;
  logic        wbm1_we_o;

  logic        wbs_cyc_o;
  logic        wbs_stb_o;
  logic        wbs_ack_i;
  logic [31:0] wbs_adr_o;
  logic [31:0] wbs_dat_o;
  logic [31:0] wbs_dat_i;
  logic [ 3:0] wbs_sel_o;
  logic        wbs_we_o;


  wb_arbiter_2 wb_arbiter (
    .clk(sys_clk),
    .rst(sys_rst),

    .wbm0_adr_i(wbm0_adr_o),    
    .wbm0_dat_i(wbm0_dat_o),    
    .wbm0_dat_o(wbm0_dat_i),    
    .wbm0_we_i (wbm0_we_o ),    
    .wbm0_sel_i(wbm0_sel_o),    
    .wbm0_stb_i(wbm0_stb_o),    
    .wbm0_ack_o(wbm0_ack_i),    
    .wbm0_err_o(),    
    .wbm0_rty_o(),    
    .wbm0_cyc_i(wbm0_cyc_o),    

    .wbm1_adr_i(wbm1_adr_o),    
    .wbm1_dat_i(wbm1_dat_o),    
    .wbm1_dat_o(wbm1_dat_i),    
    .wbm1_we_i (wbm1_we_o ),    
    .wbm1_sel_i(wbm1_sel_o),    
    .wbm1_stb_i(wbm1_stb_o),    
    .wbm1_ack_o(wbm1_ack_i),    
    .wbm1_err_o(),    
    .wbm1_rty_o(),    
    .wbm1_cyc_i(wbm1_cyc_o),   

    .wbs_adr_o(wbs_adr_o),
    .wbs_dat_i(wbs_dat_i),
    .wbs_dat_o(wbs_dat_o),
    .wbs_we_o (wbs_we_o ), 
    .wbs_sel_o(wbs_sel_o),
    .wbs_stb_o(wbs_stb_o),
    .wbs_ack_i(wbs_ack_i),
    .wbs_err_i('0),
    .wbs_rty_i('0),
    .wbs_cyc_o(wbs_cyc_o) 

  );
  /* =========== Wishbone Arbiter end =========== */


  /* =========== Wishbone MUX begin =========== */
  // Wishbone MUX (Masters) => bus slaves
  logic wbs0_cyc_o;
  logic wbs0_stb_o;
  logic wbs0_ack_i;
  logic [31:0] wbs0_adr_o;
  logic [31:0] wbs0_dat_o;
  logic [31:0] wbs0_dat_i;
  logic [3:0] wbs0_sel_o;
  logic wbs0_we_o;

  logic wbs1_cyc_o;
  logic wbs1_stb_o;
  logic wbs1_ack_i;
  logic [31:0] wbs1_adr_o;
  logic [31:0] wbs1_dat_o;
  logic [31:0] wbs1_dat_i;
  logic [3:0] wbs1_sel_o;
  logic wbs1_we_o;

  logic wbs2_cyc_o;
  logic wbs2_stb_o;
  logic wbs2_ack_i;
  logic [31:0] wbs2_adr_o;
  logic [31:0] wbs2_dat_o;
  logic [31:0] wbs2_dat_i;
  logic [3:0] wbs2_sel_o;
  logic wbs2_we_o;

  wb_mux_3 wb_mux (
      .clk(sys_clk),
      .rst(sys_rst),

      // Master interface (to Arbiter)
      .wbm_adr_i(wbs_adr_o),
      .wbm_dat_i(wbs_dat_o),
      .wbm_dat_o(wbs_dat_i),
      .wbm_we_i (wbs_we_o),
      .wbm_sel_i(wbs_sel_o),
      .wbm_stb_i(wbs_stb_o),
      .wbm_ack_o(wbs_ack_i),
      .wbm_err_o(),
      .wbm_rty_o(),
      .wbm_cyc_i(wbs_cyc_o),

      // Slave interface 0 (to BaseRAM controller)
      // Address range: 0x8000_0000 ~ 0x803F_FFFF
      .wbs0_addr    (32'h8000_0000),
      .wbs0_addr_msk(32'hFFC0_0000),

      .wbs0_adr_o(wbs0_adr_o),
      .wbs0_dat_i(wbs0_dat_i),
      .wbs0_dat_o(wbs0_dat_o),
      .wbs0_we_o (wbs0_we_o),
      .wbs0_sel_o(wbs0_sel_o),
      .wbs0_stb_o(wbs0_stb_o),
      .wbs0_ack_i(wbs0_ack_i),
      .wbs0_err_i('0),
      .wbs0_rty_i('0),
      .wbs0_cyc_o(wbs0_cyc_o),

      // Slave interface 1 (to ExtRAM controller)
      // Address range: 0x8040_0000 ~ 0x807F_FFFF
      .wbs1_addr    (32'h8040_0000),
      .wbs1_addr_msk(32'hFFC0_0000),

      .wbs1_adr_o(wbs1_adr_o),
      .wbs1_dat_i(wbs1_dat_i),
      .wbs1_dat_o(wbs1_dat_o),
      .wbs1_we_o (wbs1_we_o),
      .wbs1_sel_o(wbs1_sel_o),
      .wbs1_stb_o(wbs1_stb_o),
      .wbs1_ack_i(wbs1_ack_i),
      .wbs1_err_i('0),
      .wbs1_rty_i('0),
      .wbs1_cyc_o(wbs1_cyc_o),

      // Slave interface 2 (to UART controller)
      // Address range: 0x1000_0000 ~ 0x1000_FFFF
      .wbs2_addr    (32'h1000_0000),
      .wbs2_addr_msk(32'hFFFF_0000),

      .wbs2_adr_o(wbs2_adr_o),
      .wbs2_dat_i(wbs2_dat_i),
      .wbs2_dat_o(wbs2_dat_o),
      .wbs2_we_o (wbs2_we_o),
      .wbs2_sel_o(wbs2_sel_o),
      .wbs2_stb_o(wbs2_stb_o),
      .wbs2_ack_i(wbs2_ack_i),
      .wbs2_err_i('0),
      .wbs2_rty_i('0),
      .wbs2_cyc_o(wbs2_cyc_o)
  );

  /* =========== Wishbone MUX end =========== */

  /* =========== Wishbone Slaves begin =========== */
  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) sram_controller_base (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      // Wishbone slave (to MUX)
      .wb_cyc_i(wbs0_cyc_o),
      .wb_stb_i(wbs0_stb_o),
      .wb_ack_o(wbs0_ack_i),
      .wb_adr_i(wbs0_adr_o),
      .wb_dat_i(wbs0_dat_o),
      .wb_dat_o(wbs0_dat_i),
      .wb_sel_i(wbs0_sel_o),
      .wb_we_i (wbs0_we_o),

      // To SRAM chip
      .sram_addr(base_ram_addr),
      .sram_data(base_ram_data),
      .sram_ce_n(base_ram_ce_n),
      .sram_oe_n(base_ram_oe_n),
      .sram_we_n(base_ram_we_n),
      .sram_be_n(base_ram_be_n)
  );

  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) sram_controller_ext (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      // Wishbone slave (to MUX)
      .wb_cyc_i(wbs1_cyc_o),
      .wb_stb_i(wbs1_stb_o),
      .wb_ack_o(wbs1_ack_i),
      .wb_adr_i(wbs1_adr_o),
      .wb_dat_i(wbs1_dat_o),
      .wb_dat_o(wbs1_dat_i),
      .wb_sel_i(wbs1_sel_o),
      .wb_we_i (wbs1_we_o),

      // To SRAM chip
      .sram_addr(ext_ram_addr),
      .sram_data(ext_ram_data),
      .sram_ce_n(ext_ram_ce_n),
      .sram_oe_n(ext_ram_oe_n),
      .sram_we_n(ext_ram_we_n),
      .sram_be_n(ext_ram_be_n)
  );

  // 串口控制器模�?????
  // NOTE: 如果修改系统时钟频率，也�?????要修改此处的时钟频率参数
  uart_controller #(
      .CLK_FREQ(80_000_000),
      .BAUD    (115200)
  ) uart_controller (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      .wb_cyc_i(wbs2_cyc_o),
      .wb_stb_i(wbs2_stb_o),
      .wb_ack_o(wbs2_ack_i),
      .wb_adr_i(wbs2_adr_o),
      .wb_dat_i(wbs2_dat_o),
      .wb_dat_o(wbs2_dat_i),
      .wb_sel_i(wbs2_sel_o),
      .wb_we_i (wbs2_we_o),

      // to UART pins
      .uart_txd_o(txd),
      .uart_rxd_i(rxd)
  );

  /* =========== Wishbone Slaves end =========== */

  /* =========== pipeline begin =========== */

  // wires
    wire id_is_branch;
  
    wire exe_use_alu_pc;
    wire [31:0] exe_alu_y;
  
    wire id_stall;
    wire id_wait_reg;
    wire exe_stall;
    wire mem_stall;
    wire [31:0] mem_data_rd;
    wire mem_done;

    // register wires

    wire [31:0] if_reg_inst_pc;
    wire [31:0] if_reg_inst;

    wire [ 3:0]  id_reg_alu_op;
    wire [ 2:0]  id_reg_cmp_op;
    wire [31:0]  id_reg_imm;
    wire [31:0]  id_reg_data_rs1;
    wire [31:0]  id_reg_data_rs2;
    wire [ 4:0]  id_reg_reg_rd;
    wire         id_reg_is_branch;
    wire [31:0]  id_reg_inst_pc;
    wire         id_reg_mem_operation;
    wire         id_reg_mem_write_enable;
    wire         id_reg_mem_unsigned_ext;
    wire         id_reg_rf_write_enable;
    wire [ 1:0]  id_reg_data_rd_mux;
    wire [ 3:0]  id_reg_byte_sel;
    wire         id_reg_alu_a_use_pc;
    wire         id_reg_alu_b_use_imm;

    wire [31:0] exe_reg_inst_pc;
    wire [31:0] exe_reg_alu_y;
    wire        exe_reg_mem_operation;
    wire        exe_reg_mem_write_enable;
    wire        exe_reg_mem_unsigned_ext;
    wire        exe_reg_rf_write_enable;
    wire [ 1:0] exe_reg_data_rd_mux;
    wire [ 3:0] exe_reg_byte_sel; 
    wire [31:0] exe_reg_data_rs2;
    wire [ 4:0] exe_reg_reg_rd;


    wire [31:0]  mem_reg_data_rd;
    wire [ 4:0]  mem_reg_reg_rd;
    wire         mem_reg_rf_write_enable;



    stage_if stage_if_inst (
        .clk_i(sys_clk),
        .rst_i(sys_rst),

        .id_is_branch_i(id_is_branch),
        .id_stall_i(id_stall),
        .id_wait_reg_i(id_wait_reg),

        .id_reg_is_branch_i(id_reg_is_branch),
        .exe_use_alu_pc_i(exe_use_alu_pc),
        .exe_alu_y_i(exe_alu_y),

        .mem_operation_i(exe_reg_mem_operation),

        .wb_cyc_o(wbm1_cyc_o),
        .wb_stb_o(wbm1_stb_o),
        .wb_ack_i(wbm1_ack_i),
        .wb_adr_o(wbm1_adr_o),
        .wb_dat_o(wbm1_dat_o),
        .wb_dat_i(wbm1_dat_i),
        .wb_sel_o(wbm1_sel_o),
        .wb_we_o (wbm1_we_o ),

        .reg_inst_pc_o(if_reg_inst_pc),
        .reg_inst_o(if_reg_inst)
    );

    stage_id stage_id_inst(
        .clk_i                     (sys_clk),
        .rst_i                     (sys_rst),

        .exe_stall_i               (exe_stall),
        .stall_o                   (id_stall),
        .wait_reg_o                (id_wait_reg),

        .id_reg_reg_rd_i           (id_reg_reg_rd),
        .id_reg_rf_write_enable_i  (id_reg_rf_write_enable),
        .exe_alu_y_i               (exe_alu_y),
        .exe_load_data_i           (id_reg_mem_operation && !id_reg_mem_write_enable),
   
        .exe_reg_reg_rd_i          (exe_reg_reg_rd),
        .exe_reg_rf_write_enable_i (exe_reg_rf_write_enable),
        .exe_reg_alu_y_i           (exe_reg_alu_y),
        .mem_load_data_i           (exe_reg_mem_operation && !exe_reg_mem_write_enable),

        .mem_data_rd_i             (mem_data_rd),
        .mem_done_i                (mem_done),

        .mem_reg_data_rd_i         (mem_reg_data_rd),
        .mem_reg_reg_rd_i          (mem_reg_reg_rd),
        .mem_reg_rf_write_enable_i (mem_reg_rf_write_enable),

        .if_reg_inst_i             (if_reg_inst),
        .if_reg_inst_pc_i          (if_reg_inst_pc),
        .is_branch_o               (id_is_branch),

        .reg_alu_op_o              (id_reg_alu_op),
        .reg_cmp_op_o              (id_reg_cmp_op),
        .reg_imm_o                 (id_reg_imm),
        .reg_data_rs1_o            (id_reg_data_rs1),
        .reg_data_rs2_o            (id_reg_data_rs2),
        .reg_reg_rd_o              (id_reg_reg_rd),
        .reg_is_branch_o           (id_reg_is_branch),
        .reg_inst_pc_o             (id_reg_inst_pc),
        .reg_mem_operation_o       (id_reg_mem_operation),
        .reg_mem_write_enable_o    (id_reg_mem_write_enable),
        .reg_mem_unsigned_ext_o    (id_reg_mem_unsigned_ext),
        .reg_rf_write_enable_o     (id_reg_rf_write_enable),
        .reg_data_rd_mux_o         (id_reg_data_rd_mux),
        .reg_byte_sel_o            (id_reg_byte_sel),
        .reg_alu_a_use_pc_o        (id_reg_alu_a_use_pc),
        .reg_alu_b_use_imm_o       (id_reg_alu_b_use_imm)
    );

    stage_exe stage_exe_inst(
    .clk_i                             (sys_clk),
    .rst_i                             (sys_rst),

    .mem_stall_i                       (mem_stall),
    .stall_o                           (exe_stall),
    .mem_done_i                         (mem_done),

    .use_alu_pc_o                      (exe_use_alu_pc),
    .alu_y_o                           (exe_alu_y),

    .id_reg_alu_op_i                   (id_reg_alu_op),
    .id_reg_cmp_op_i                   (id_reg_cmp_op),
    .id_reg_imm_i                      (id_reg_imm),
    .id_reg_data_rs1_i                 (id_reg_data_rs1),
    .id_reg_data_rs2_i                 (id_reg_data_rs2),
    .id_reg_reg_rd_i                   (id_reg_reg_rd),
//    .id_reg_is_branch_i                (id_reg_is_branch),
    .id_reg_inst_pc_i                  (id_reg_inst_pc),
    .id_reg_mem_operation_i            (id_reg_mem_operation),
    .id_reg_mem_write_enable_i         (id_reg_mem_write_enable),
    .id_reg_mem_unsigned_ext_i         (id_reg_mem_unsigned_ext),
    .id_reg_rf_write_enable_i          (id_reg_rf_write_enable),
    .id_reg_data_rd_mux_i              (id_reg_data_rd_mux),
    .id_reg_byte_sel_i                 (id_reg_byte_sel),
    .id_reg_alu_a_use_pc_i             (id_reg_alu_a_use_pc),
    .id_reg_alu_b_use_imm_i            (id_reg_alu_b_use_imm),


    .reg_inst_pc_o                     (exe_reg_inst_pc),
    .reg_alu_y_o                       (exe_reg_alu_y),
    .reg_mem_operation_o               (exe_reg_mem_operation),
    .reg_mem_write_enable_o            (exe_reg_mem_write_enable),
    .reg_mem_unsigned_ext_o            (exe_reg_mem_unsigned_ext),
    .reg_rf_write_enable_o             (exe_reg_rf_write_enable),
    .reg_data_rd_mux_o                 (exe_reg_data_rd_mux),
    .reg_byte_sel_o                    (exe_reg_byte_sel), 
    .reg_data_rs2_o                    (exe_reg_data_rs2),
    .reg_reg_rd_o                      (exe_reg_reg_rd)
    );

    stage_mem stage_mem_inst(
        .clk_i                      (sys_clk),
        .rst_i                      (sys_rst),
        
        .stall_o                    (mem_stall),
        .done_o                     (mem_done),
        .data_rd_o                  (mem_data_rd),
        
        .exe_reg_inst_pc_i          (exe_reg_inst_pc),
        .exe_reg_alu_y_i            (exe_reg_alu_y),
        .exe_reg_mem_operation_i    (exe_reg_mem_operation),
        .exe_reg_mem_write_enable_i (exe_reg_mem_write_enable),
        .exe_reg_mem_unsigned_ext_i (exe_reg_mem_unsigned_ext),
        .exe_reg_rf_write_enable_i  (exe_reg_rf_write_enable),
        .exe_reg_data_rd_mux_i      (exe_reg_data_rd_mux),
        .exe_reg_byte_sel_i         (exe_reg_byte_sel), 
        .exe_reg_data_rs2_i         (exe_reg_data_rs2),
        .exe_reg_reg_rd_i           (exe_reg_reg_rd),
        
        .wb_cyc_o                   (wbm0_cyc_o),
        .wb_stb_o                   (wbm0_stb_o),
        .wb_ack_i                   (wbm0_ack_i),
        .wb_adr_o                   (wbm0_adr_o),
        .wb_dat_o                   (wbm0_dat_o),
        .wb_dat_i                   (wbm0_dat_i),
        .wb_sel_o                   (wbm0_sel_o),
        .wb_we_o                    (wbm0_we_o),
        
        .reg_data_rd_o              (mem_reg_data_rd),
        .reg_reg_rd_o               (mem_reg_reg_rd),
        .reg_rf_write_enable_o      (mem_reg_rf_write_enable)
    );



endmodule
