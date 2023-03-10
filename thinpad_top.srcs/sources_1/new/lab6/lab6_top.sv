`default_nettype none

`include "alu_define.sv"


`timescale 1ns / 1ns
module lab6_top (
    input wire clk_50M,     // 50MHz 时钟输入
    input wire clk_11M0592, // 11.0592MHz 时钟输入（备用，可不用）

    input wire push_btn,  // BTN5 按钮�????????????????????????????????????关，带消抖电路，按下时为 1
    input wire reset_btn, // BTN6 复位按钮，带消抖电路，按下时�???????????????????????????????????? 1

    input  wire [ 3:0] touch_btn,  // BTN1~BTN4，按钮开关，按下时为 1
    input  wire [31:0] dip_sw,     // 32 位拨码开关，拨到“ON”时�???????????????????????????????????? 1
    output wire [15:0] leds,       // 16 �???????????????????????????????????? LED，输出时 1 点亮
    output wire [ 7:0] dpy0,       // 数码管低位信号，包括小数点，输出 1 点亮
    output wire [ 7:0] dpy1,       // 数码管高位信号，包括小数点，输出 1 点亮

    // CPLD 串口控制器信�????????????????????????????????????
    output wire uart_rdn,        // 读串口信号，低有�????????????????????????????????????
    output wire uart_wrn,        // 写串口信号，低有�????????????????????????????????????
    input  wire uart_dataready,  // 串口数据准备�????????????????????????????????????
    input  wire uart_tbre,       // 发�?�数据标�????????????????????????????????????
    input  wire uart_tsre,       // 数据发�?�完毕标�????????????????????????????????????

    // BaseRAM 信号
    inout  wire [31:0] base_ram_data,  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共�????????????????????????????????????
    output wire [19:0] base_ram_addr,  // BaseRAM 地址
    output wire [ 3:0] base_ram_be_n,  // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持�???????????????????????????????????? 0
    output wire base_ram_ce_n,  // BaseRAM 片�?�，低有�????????????????????????????????????
    output wire base_ram_oe_n,  // BaseRAM 读使能，低有�????????????????????????????????????
    output wire base_ram_we_n,  // BaseRAM 写使能，低有�????????????????????????????????????

    // ExtRAM 信号
    inout  wire [31:0] ext_ram_data,  // ExtRAM 数据
    output wire [19:0] ext_ram_addr,  // ExtRAM 地址
    output wire [ 3:0] ext_ram_be_n,  // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持�???????????????????????????????????? 0
    output wire ext_ram_ce_n,  // ExtRAM 片�?�，低有�????????????????????????????????????
    output wire ext_ram_oe_n,  // ExtRAM 读使能，低有�????????????????????????????????????
    output wire ext_ram_we_n,  // ExtRAM 写使能，低有�????????????????????????????????????

    // 直连串口信号
    output wire txd,  // 直连串口发�?�端
    input  wire rxd,  // 直连串口接收�????????????????????????????????????

    // Flash 存储器信号，参�?? JS28F640 芯片手册
    output wire [22:0] flash_a,  // Flash 地址，a0 仅在 8bit 模式有效�????????????????????????????????????16bit 模式无意�????????????????????????????????????
    inout wire [15:0] flash_d,  // Flash 数据
    output wire flash_rp_n,  // Flash 复位信号，低有效
    output wire flash_vpen,  // Flash 写保护信号，低电平时不能擦除、烧�????????????????????????????????????
    output wire flash_ce_n,  // Flash 片�?�信号，低有�????????????????????????????????????
    output wire flash_oe_n,  // Flash 读使能信号，低有�????????????????????????????????????
    output wire flash_we_n,  // Flash 写使能信号，低有�????????????????????????????????????
    output wire flash_byte_n, // Flash 8bit 模式选择，低有效。在使用 flash �???????????????????????????????????? 16 位模式时请设�???????????????????????????????????? 1

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
    inout  wire [15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input  wire dm9k_int,

    // 图像输出信号
    output wire [2:0] video_red,    // 红色像素�????????????????????????????????????3 �????????????????????????????????????
    output wire [2:0] video_green,  // 绿色像素�????????????????????????????????????3 �????????????????????????????????????
    output wire [1:0] video_blue,   // 蓝色像素�????????????????????????????????????2 �????????????????????????????????????
    output wire       video_hsync,  // 行同步（水平同步）信�????????????????????????????????????
    output wire       video_vsync,  // 场同步（垂直同步）信�????????????????????????????????????
    output wire       video_clk,    // 像素时钟输出
    output wire       video_de      // 行数据有效信号，用于区分消隐�????????????????????????????????????
);

    /* =========== Demo code begin =========== */

    // PLL 分频示例
    logic locked, clk_10M, clk_70M;
    pll_example clock_gen (
        // Clock in ports
        .clk_in1    (clk_50M),  // 外部时钟输入
        // Clock out ports
        .clk_out1   (clk_10M),  // 时钟输出 1，频率在 IP 配置界面中设�????????????????????????????????????
        .clk_out2   (clk_70M),  // 时钟输出 2，频率在 IP 配置界面中设�????????????????????????????????????
        // Status and control signals
        .reset      (reset_btn),  // PLL 复位输入
        .locked     (locked)  // PLL 锁定指示输出�????????????????????????????????????"1"表示时钟稳定�????????????????????????????????????
                         // 后级电路复位信号应当由它生成（见下）
    );

    logic reset_of_clk10M;
    always_ff @(posedge clk_10M or negedge locked) begin
        if (~locked) reset_of_clk10M <= 1'b1;
        else reset_of_clk10M <= 1'b0;
    end

    logic reset_of_clk70M;
    always_ff @(posedge clk_70M or negedge locked) begin
        if (~locked) reset_of_clk70M <= 1'b1;
        else reset_of_clk70M <= 1'b0;
    end

    /* =========== Demo code end =========== */

    logic sys_clk;
    logic sys_rst;

    assign sys_clk = clk_70M;
    assign sys_rst = reset_of_clk70M;

    // 本实验不使用 CPLD 串口，禁用防止�?�线冲突
    assign uart_rdn = 1'b1;
    assign uart_wrn = 1'b1;

    // shared with bus
    (* MARK_DEBUG = "TRUE" *) logic        core_time_irq;
    (* MARK_DEBUG = "TRUE" *) logic [63:0] core_time;
    (* MARK_DEBUG = "TRUE" *) logic [63:0] core_timecmp;

    (* MARK_DEBUG = "TRUE" *) logic [ 1:0] core_mode;
    // (* MARK_DEBUG = "TRUE" *) logic        core_page_fault;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] core_satp;
    (* MARK_DEBUG = "TRUE" *) logic        core_sum;

    (* MARK_DEBUG = "TRUE" *) logic        core_tlb_clear;

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


    // shared bus
    (* MARK_DEBUG = "TRUE" *) logic        wbs_cyc_o;
    (* MARK_DEBUG = "TRUE" *) logic        wbs_stb_o;
    (* MARK_DEBUG = "TRUE" *) logic        wbs_ack_i;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] wbs_adr_o;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] wbs_dat_o;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] wbs_dat_i;
    (* MARK_DEBUG = "TRUE" *) logic [ 3:0] wbs_sel_o;
    (* MARK_DEBUG = "TRUE" *) logic        wbs_we_o;

    (* MARK_DEBUG = "TRUE" *) logic [ 2:0] debug_if_vmi_state;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_if_vmi_pte;
    (* MARK_DEBUG = "TRUE" *) logic [ 3:0] debug_if_vmi_pf_cause;

    (* MARK_DEBUG = "TRUE" *) logic        debug_if_vmi_tlb_hit;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_if_vmi_paddr;
    (* MARK_DEBUG = "TRUE" *) logic [ 1:0] debug_if_vmi_pte_state;

    (* MARK_DEBUG = "TRUE" *) logic [ 2:0] debug_mem_vmi_state;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_mem_vmi_pte;
    (* MARK_DEBUG = "TRUE" *) logic [ 3:0] debug_mem_vmi_pf_cause;

    (* MARK_DEBUG = "TRUE" *) logic        debug_mem_vmi_tlb_hit;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_mem_vmi_paddr;
    (* MARK_DEBUG = "TRUE" *) logic [ 1:0] debug_mem_vmi_pte_state;


    (* MARK_DEBUG = "TRUE" *) logic debug_mem_vmi_tlb_write_enable;
    (* MARK_DEBUG = "TRUE" *) logic [21:0] debug_mem_vmi_tlb_ppn_in;
    (* MARK_DEBUG = "TRUE" *) logic [19:0] debug_mem_vmi_tlb_vpn_in;


    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_mem_pmi_addr_in_o     ;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_mem_pmi_data_out_o    ;
    (* MARK_DEBUG = "TRUE" *) logic        debug_mem_pmi_ack_o         ;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_mem_pmi_ack_addr_o    ;

    (* MARK_DEBUG = "TRUE" *) logic [ 4:0] debug_mem_pmi_state;
    (* MARK_DEBUG = "TRUE" *) logic        debug_mem_pmi_cache_hit;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_mem_pmi_cache_addr_out;

    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_eram_data_in;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_eram_data_out;
    (* MARK_DEBUG = "TRUE" *) logic        debug_eram_data_is_in;
    (* MARK_DEBUG = "TRUE" *) logic [31:0] debug_eram_addr;
    (* MARK_DEBUG = "TRUE" *) logic        debug_eram_we_n;
    (* MARK_DEBUG = "TRUE" *) logic        debug_eram_ce_n;
    (* MARK_DEBUG = "TRUE" *) logic        debug_eram_oe_n;
    (* MARK_DEBUG = "TRUE" *) logic [ 3:0] debug_eram_be_n;

    assign debug_eram_addr = ext_ram_addr;
    assign debug_eram_we_n = ext_ram_we_n;
    assign debug_eram_ce_n = ext_ram_ce_n;
    assign debug_eram_oe_n = ext_ram_oe_n;
    assign debug_eram_be_n = ext_ram_be_n;


    /* =========== Wishbone MUX begin =========== */
    // Wishbone MUX (Masters) => bus slaves
    logic        wbs0_cyc_o;
    logic        wbs0_stb_o;
    logic        wbs0_ack_i;
    logic [31:0] wbs0_adr_o;
    logic [31:0] wbs0_dat_o;
    logic [31:0] wbs0_dat_i;
    logic [ 3:0] wbs0_sel_o;
    logic        wbs0_we_o;

    logic        wbs1_cyc_o;
    logic        wbs1_stb_o;
    logic        wbs1_ack_i;
    logic [31:0] wbs1_adr_o;
    logic [31:0] wbs1_dat_o;
    logic [31:0] wbs1_dat_i;
    logic [ 3:0] wbs1_sel_o;
    logic        wbs1_we_o;

    logic        wbs2_cyc_o;
    logic        wbs2_stb_o;
    logic        wbs2_ack_i;
    logic [31:0] wbs2_adr_o;
    logic [31:0] wbs2_dat_o;
    logic [31:0] wbs2_dat_i;
    logic [ 3:0] wbs2_sel_o;
    logic        wbs2_we_o;

    logic        wbs3_cyc_o;
    logic        wbs3_stb_o;
    logic        wbs3_ack_i;
    logic [31:0] wbs3_adr_o;
    logic [31:0] wbs3_dat_o;
    logic [31:0] wbs3_dat_i;
    logic [ 3:0] wbs3_sel_o;
    logic        wbs3_we_o;

    wb_mux_4 wb_mux (
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
        .wbs2_cyc_o(wbs2_cyc_o),

        // Slave interface 3 (to CLINT device)
        // Address range: 0x0200_0000 ~ 0x0200_FFFF
        .wbs3_addr    (32'h0200_0000),
        .wbs3_addr_msk(32'hFFFF_0000),

        .wbs3_adr_o(wbs3_adr_o),
        .wbs3_dat_i(wbs3_dat_i),
        .wbs3_dat_o(wbs3_dat_o),
        .wbs3_we_o (wbs3_we_o),
        .wbs3_sel_o(wbs3_sel_o),
        .wbs3_stb_o(wbs3_stb_o),
        .wbs3_ack_i(wbs3_ack_i),
        .wbs3_err_i('0),
        .wbs3_rty_i('0),
        .wbs3_cyc_o(wbs3_cyc_o)

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
        .sram_be_n(ext_ram_be_n),

        .debug_data_out(debug_eram_data_out),
        .debug_data_in(debug_eram_data_in),
        .debug_data_is_in(debug_eram_data_is_in)
    );

    // 串口控制器模�????????????????????????????????????
    // NOTE: 如果修改系统时钟频率，也�????????????????????????????????????要修改此处的时钟频率参数
    uart_controller #(
        .CLK_FREQ(40_000_000),
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

    // TODO: we use a slow device to lower the delay
    clint_slow_device clint_device (
        .clk_i     (sys_clk),
        .rst_i     (sys_rst),

        .wb_cyc_i  (wbs3_cyc_o),
        .wb_stb_i  (wbs3_stb_o),
        .wb_ack_o  (wbs3_ack_i),
        .wb_adr_i  (wbs3_adr_o),
        .wb_dat_i  (wbs3_dat_o),
        .wb_dat_o  (wbs3_dat_i),
        .wb_sel_i  (wbs3_sel_o),
        .wb_we_i   (wbs3_we_o),

        // to CPU
        .irq_o     (core_time_irq),
        .time_o    (core_time),
        .timecmp_o (core_timecmp)
    );

    /* =========== Wishbone Slaves end =========== */

    wire clk;
    wire rst;

    assign clk = sys_clk;
    assign rst = sys_rst;
    /* =========== pipeline begin =========== */

    /* shared wires */

    // stage if
    logic [ 1:0] if_mode;
    wire  [31:0] if_pc_plus4;

    wire         if_exception;
    wire  [ 1:0] if_trap_mode;
    wire  [31:0] if_mcause;
    wire  [31:0] if_mtval;

    wire  [30:0] if_exp_exception_code;
    logic [31:0] if_request_addr;



    // if-id-registers
    wire  [ 1:0] id_mode;
    wire  [31:0] id_inst;
    (* MARK_DEBUG = "TRUE" *) wire  [31:0] id_inst_pc;
    wire  [31:0] id_pc_plus4;
    wire         id_nop;

    wire         id_prev_exception;
    wire  [ 1:0] id_prev_trap_mode;
    wire  [31:0] id_prev_mcause;
    wire  [31:0] id_prev_mtval;

    // stage id
    wire         id_stall;
    wire         id_wait_reg;
    wire         id_is_branch;

    wire         id_exception;
    wire  [ 1:0] id_trap_mode;
    wire  [31:0] id_mcause;
    wire  [31:0] id_mtval;

    wire         id_next_exception;
    wire  [ 1:0] id_next_trap_mode;
    wire  [31:0] id_next_mcause;
    wire  [31:0] id_next_mtval;

    wire  [ 1:0] id_int_trap_mode;
    wire  [31:0] id_int_int;
    
    logic        id_exp_exception;
    logic [ 1:0] id_exp_trap_mode;
    logic [30:0] id_exp_exception_code;
    logic [31:0] id_exp_mtval;

    wire  [31:0] id_satp;
    wire         id_mstatus_sum;

    // id-exe-registers
    wire  [ 1:0] exe_mode;
    wire  [31:0] exe_inst_pc;
    wire  [31:0] exe_pc_plus4;
    wire         exe_nop;

    wire  [ 1:0] exe_alu_a_mux;
    wire  [ 1:0] exe_alu_b_mux;
    wire  [ 3:0] exe_alu_op;

    wire  [ 2:0] exe_cmp_op;

    wire  [31:0] exe_imm;
    wire  [31:0] exe_uimm;

    wire  [31:0] exe_data_rs1;
    wire  [31:0] exe_data_rs2;
    
    wire         exe_is_branch;
    
    wire  [ 4:0] exe_reg_rd;
    wire  [ 1:0] exe_data_rd_mux;
    wire         exe_rf_write_enable;

    wire  [ 3:0] exe_byte_sel;
    wire         exe_mem_operation;
    wire         exe_mem_write_enable;
    wire         exe_mem_unsigned_ext;

    wire  [ 4:0] exe_id_csr;
    wire  [31:0] exe_data_csr;
    wire         exe_csr_write_enable;
    wire  [31:0] exe_csr_mask;


    wire         exe_prev_exception;
    wire  [ 1:0] exe_prev_trap_mode;
    wire  [31:0] exe_prev_mcause;
    wire  [31:0] exe_prev_mtval;

    wire  [31:0] exe_medeleg;

    // stage exe
    wire         exe_stall;
    wire         exe_branch_take;
    wire  [31:0] exe_alu_y;

   

    wire         exe_exception;
    wire  [ 1:0] exe_trap_mode;
    wire  [31:0] exe_mcause;
    wire  [31:0] exe_mtval;

    wire  [30:0] exe_exp_exception_code;

    wire         exe_next_exception;
    wire  [ 1:0] exe_next_trap_mode;
    wire  [31:0] exe_next_mcause;
    wire  [31:0] exe_next_mtval;

    
    // exe-mem-registers
    wire  [ 1:0] mem_mode;
    (* MARK_DEBUG = "TRUE" *) wire  [31:0] mem_inst_pc;
    wire  [31:0] mem_pc_plus4;
    wire         mem_nop;

    wire  [31:0] mem_data_rs2; // address

    wire  [ 3:0] mem_byte_sel; 
    wire         mem_mem_operation;
    wire         mem_mem_write_enable;
    wire         mem_mem_unsigned_ext;
    
    wire  [31:0] mem_alu_y;
    wire  [31:0] mem_data_csr;
    
    
    wire  [ 4:0] mem_reg_rd;
    wire  [31:0] mem_data_rd;
    wire  [ 1:0] mem_data_rd_mux;
    wire         mem_rf_write_enable;

    wire  [ 4:0] mem_id_csr;
    wire  [31:0] mem_new_data_csr;
    wire         mem_csr_write_enable;



    wire         mem_prev_exception;
    wire  [ 1:0] mem_prev_trap_mode;
    wire  [31:0] mem_prev_mcause;
    wire  [31:0] mem_prev_mtval;

    wire  [31:0] mem_medeleg;
    

    // stage mem
    wire         mem_stall;
    wire         mem_done;

    wire         mem_exception;
    wire  [ 1:0] mem_trap_mode;
    wire  [31:0] mem_mcause;
    wire  [31:0] mem_mtval;

    wire  [30:0] mem_exp_exception_code;

    wire         mem_next_exception;
    wire  [ 1:0] mem_next_trap_mode;
    wire  [31:0] mem_next_mcause;
    wire  [31:0] mem_next_mtval;

    logic        mem_cache_sync_done;
    logic        mem_pmi_wait_bus;

    // mem-wb-registers
    // rf
    wire  [31:0] wb_data_rd;
    wire  [ 4:0] wb_reg_rd;
    wire         wb_rf_write_enable;

    // csr
    wire  [31:0] wb_data_csr;
    wire  [ 4:0] wb_id_csr;
    wire         wb_csr_write_enable;

    wire  [ 1:0] wb_mode;
    (* MARK_DEBUG = "TRUE" *) wire  [31:0] wb_inst_pc;
    (* MARK_DEBUG = "TRUE" *) wire         wb_prev_exception;
    (* MARK_DEBUG = "TRUE" *) wire  [ 1:0] wb_prev_trap_mode;
    (* MARK_DEBUG = "TRUE" *) wire  [31:0] wb_prev_mcause;
    (* MARK_DEBUG = "TRUE" *) wire  [31:0] wb_prev_mtval;

    logic [31:0] wb_tvec;

    logic [31:0] wb_mstatus;

    logic [31:0] wb_mepc;
    logic [31:0] wb_mtval;
    logic [31:0] wb_mcause;

    logic [31:0] wb_sepc;
    logic [31:0] wb_stval;
    logic [31:0] wb_scause;

    // global wires

    mmu_vm_status_regs mmu_vm_status_regs(
        .clk_i          (clk),
        .rst_i          (rst),

        .mode_i         (wb_prev_trap_mode),
        .satp_i         (id_satp),
        .sum_i          (id_mstatus_sum),

        .mode_o         (core_mode),
        .satp_o         (core_satp),
        .sum_o          (core_sum),

        .write_enable_i (wb_prev_exception)
    );



    /* =========== stage if begin =========== */

    // internal wires

    
    wire  [31:0] _if_next_pc;
    (* MARK_DEBUG = "TRUE" *) wire  [31:0] _if_inst_pc;
    wire         _if_pc_stall;
    wire         _if_pc_valid;
    wire  [31:0] _if_inst;
    (* MARK_DEBUG = "TRUE" *) wire  [31:0] _if_inst_addr;
    wire         _if_bubble;
    wire         _if_stall;

    wire         _if_branch_last;
    wire         _if_branch_take;
    logic        _if_do_next_req;

    wire         _if_pc_misaligned;
    wire         _if_pagefault;

    wire         _if_if_ack; 

    wire         _if_control_flow_change;

    wire         _if_req_accepted;



    assign _if_pc_misaligned = (_if_inst_pc[1:0] != 2'b00);

    // assign _if_do_req = _if_pc_valid & ~mem_mem_operation & ~_if_pc_misaligned;
    // assign _if_do_next_req = ~rst & ~mem_mem_operation & ~_if_pc_misaligned 
    //                         & ~(_if_if_ack & _if_control_flow_change); 

    // always_comb begin
    //     _if_do_next_req = 1'b0;
    //     if (~rst & ~mem_mem_operation & ~_if_pc_misaligned 
    //              & ~core_tlb_clear & mem_cache_sync_done) begin
    //         if (_if_if_ack) begin
    //             if (_if_control_flow_change) begin
    //                 _if_do_next_req = 1'b0;
    //             end else begin
    //                 _if_do_next_req = 1'b1;
    //             end
    //         end else begin
    //             _if_do_next_req = _if_pc_valid;
    //         end
    //     end
    // end

    // branch
    assign _if_branch_take = exe_is_branch & exe_branch_take & ~id_stall; // TODO: stall delay is so long
    assign _if_branch_last = id_is_branch | (exe_is_branch & id_stall);


    always_comb begin
        _if_do_next_req = 1'b0;
        // TODO: this orignally use pc_valid and some other signals simply, but it seems cause vivado to fail
        if (~rst & ~_if_pc_misaligned & ~core_tlb_clear & ~_if_branch_last
            & mem_cache_sync_done & ~id_exp_exception & ~exe_prev_exception 
            & ~mem_prev_exception & ~wb_prev_exception 
            & ~(_if_control_flow_change &(_if_inst_addr == _if_inst_pc))) begin
            _if_do_next_req = 1'b1; // always request next instruction
        end
    end


    always_comb begin
        // id stall, then we don't care the influence of exe
        // TODO: actually this can be optimized :)
        if (  _if_branch_take
            | wb_prev_exception 
            | (_if_inst_addr == _if_inst_pc)) begin

            if_request_addr = _if_next_pc;
        end else begin
            if_request_addr = _if_inst_pc;
        end
    end

    // just mret, sret, sfence.vma
    id_control_flow_change_detector if_cfcd(
        .inst_i(_if_inst),
        .change_o(_if_control_flow_change)
    );
    

    wire [31:0] _if_temp_mcause;

    exp_mcause_encoder if_eme(
        .int_i(32'b0),
        .exception_code_i(if_exp_exception_code),
        .mcause_o(if_mcause)
    );


    exp_exception_encoder if_eee (
        .mode_i             (if_mode),
        .medeleg_i          (_id_medeleg), 

        .inst_addr_bp       (1'b0), // not implemented
        .inst_access_fault  (1'b0), // not implemented: if stage
        .inst_page_fault    (_if_pagefault), // if stage
        .inst_illegal       (1'b0), // id stage
        .inst_misaligned    (_if_pc_misaligned), // if stage
        .ecall              (1'b0), // id stage
        .ebreak             (1'b0), // id stage
        .lsa_addr_bp        (1'b0), // not implemented, 
        .load_misaligned    (1'b0), // exe stage
        .sa_misaligned      (1'b0), // exe stage
        .load_page_fault    (1'b0), // mem stage
        .sa_page_fault      (1'b0), // mem stage
        .load_access_fault  (1'b0), // mem stage
        .sa_access_fault    (1'b0), // mem stage

        .addr_i             (_if_inst_pc),
        .inst_i             (_if_inst),

        .exception_o        (if_exception),
        .exception_code_o   (if_exp_exception_code),
        .exp_mode_o         (if_trap_mode),
        .mtval_o            (if_mtval)
    );

    

    // mode
    always_ff @(posedge clk) begin
        if (rst) begin
            if_mode <= M_MODE;
        end else if (wb_prev_exception) begin
            if_mode <= wb_prev_trap_mode;
        end
    end

    // pc related
    adder4 pc_adder (
        .data_i         (_if_inst_pc),
        .data_plus4_o   (if_pc_plus4)
    );

    if_pc_reg if_pc_reg_inst (
        .clk_i      (clk),
        .rst_i      (rst),
        .stall_i    (_if_pc_stall),
        .next_pc_i  (_if_next_pc),
        .pc_o       (_if_inst_pc)
    );

    if_next_pc_mux if_next_pc_mux_inst (
        .pc_plus4_i          (if_pc_plus4),
        .pc_branch_target_i  (exe_alu_y),
        .pc_tvec_i           (wb_tvec),

        .do_branch_i         (_if_branch_take),
        .wb_prev_exception_i (wb_prev_exception),


        .next_pc_o           (_if_next_pc)
    );



    if_pc_controller if_pc_controller_inst (
        .branch_last_i          (_if_branch_last),
        .branch_take_i          (_if_branch_take), // target address computed
        .if_regs_stall_i        (_if_stall),

        // .id_next_exception_i    (id_next_exception),
        // .exe_prev_exception_i   (exe_prev_exception),
        // .mem_prev_exception_i   (mem_prev_exception),
        .id_next_exception_i    (1'b0),
        .exe_prev_exception_i   (1'b0),
        .mem_prev_exception_i   (1'b0),
        .wb_prev_exception_i    (wb_prev_exception),

        .inst_addr_i            (_if_inst_addr),
        .inst_pc_i              (_if_inst_pc),

        .pc_stall_o             (_if_pc_stall),
        .pc_valid_o             (_if_pc_valid)
    );

    
    // instruction memory controller
    mmu_virtual_memory_interface if_im_fetch (
        .clk_i              (clk),
        .rst_i              (rst),

        .no_flush_cache_i   (1'b0), 

        // memory interface signals
        .vmi_no_cache_i     (dip_sw[1]),
        .vmi_enable_i       (_if_do_next_req),
        .vmi_write_enable_i (1'b0),
        .vmi_addr_i         (if_request_addr),
        .vmi_data_i         (32'b1),
        .vmi_data_sel_i     (4'b1111),
    
        .vmi_ack_addr_o     (_if_inst_addr),
        .vmi_ack_o          (_if_if_ack),
        .vmi_data_o         (_if_inst),

        .pmi_yield_i        (mem_pmi_wait_bus),
        .pmi_wait_bus_o     (),


        .satp_i             (core_satp),
        .sum_i              (core_sum),
        .mode_i             (core_mode),
        .flush_i            (core_tlb_clear),

        .page_fault_o       (_if_pagefault),

        // wbm signals (as master) (fall back to wish bone bus)
        .wb_ack_i           (wbm1_ack_i),
        .wb_dat_i           (wbm1_dat_i),
        .wb_cyc_o           (wbm1_cyc_o),
        .wb_stb_o           (wbm1_stb_o),
        .wb_adr_o           (wbm1_adr_o),
        .wb_dat_o           (wbm1_dat_o),
        .wb_sel_o           (wbm1_sel_o),
        .wb_we_o            (wbm1_we_o ),

        .sync_done_o        (),

        // debug signals

        .debug_sv32_tlb_enable_i (~dip_sw[0]),

        .debug_state_o           (debug_if_vmi_state),
        .debug_sv32_pte_o        (debug_if_vmi_pte),
        .debug_sv32_pf_cause_o   (debug_if_vmi_pf_cause),
        .debug_sv32_tlb_hit_o    (debug_if_vmi_tlb_hit),
        .debug_sv32_pte_state_o  (debug_if_vmi_pte_state),

        .debug_paddr_o           (debug_if_vmi_paddr)

    );



    // stall controller
    if_stall_controller if_stall_controller_inst(
        .id_wait_reg_i  (id_wait_reg),
        .id_stall_i     (id_stall),
        .if_stall_o     (_if_stall)
    );

    // bubble controller
    if_bubble_controller if_bubble_controller_inst(
        .inst_pc_i              (_if_inst_pc),
        .inst_addr_i            (_if_inst_addr),

        .id_is_branch_i         (id_is_branch),
        .exe_is_branch_i        (exe_is_branch),

        .id_next_exception_i    (id_next_exception),
        .exe_next_exception_i   (exe_next_exception),
        .mem_next_exception_i   (mem_next_exception),
        .wb_prev_exception_i    (wb_prev_exception),

        .if_bubble_o            (_if_bubble)
    );

    if_pipeline_regs if_pipeline_regs_inst(
        .clk_i(clk),
        .rst_i(rst),

        .bubble_i(_if_bubble),
        .stall_i(_if_stall),

        .nop_o(id_nop),

        .mode_i(if_mode),
        .mode_o(id_mode),

        .inst_i(_if_inst),
        .inst_o(id_inst),

        .inst_pc_i(_if_inst_pc),
        .inst_pc_o(id_inst_pc),

        .pc_plus4_i(if_pc_plus4),
        .pc_plus4_o(id_pc_plus4),

        .exception_i(if_exception),
        .exception_o(id_prev_exception),

        .trap_mode_i(if_trap_mode),
        .trap_mode_o(id_prev_trap_mode),

        .mcause_i(if_mcause),
        .mcause_o(id_prev_mcause),

        .mtval_i(if_mtval),
        .mtval_o(id_prev_mtval)

    );

    /* =========== stage if end =========== */

    /* =========== stage id begin =========== */

    wire [4:0] _id_reg_rs1;
    wire [4:0] _id_reg_rs2;
    
    wire [31:0] _id_data_rs1_old;
    wire [31:0] _id_data_rs2_old;

    wire [31:0] _id_data_rs1;
    wire [31:0] _id_data_rs2;

    wire [ 4:0] _id_id_csr;
    wire [31:0] _id_data_csr;
    
    wire [31:0] _id_imm;
    wire [31:0] _id_uimm;
    wire [3:0]  _id_alu_op;
    wire [2:0]  _id_cmp_op;
    wire [4:0]  _id_reg_rd;

    wire        _id_mem_operation;
    wire        _id_mem_write_enable;
    wire        _id_mem_unsigned_ext;

    wire        _id_rf_write_enable;
    wire        _id_csr_write_enable;

    wire [1:0]  _id_data_rd_mux;
    wire [3:0]  _id_byte_sel;
    wire [1:0]  _id_alu_a_mux;
    wire [1:0]  _id_alu_b_mux;

    wire        _id_invalid_inst;
    wire        _id_bubble;


    // important csr

    wire [31:0] _id_stvec;
    wire [31:0] _id_sepc;

    wire [31:0] _id_scause;
    wire [31:0] _id_stval;

    (* MARK_DEBUG = "TRUE" *) wire [31:0] _id_mstatus;
    (* MARK_DEBUG = "TRUE" *) wire [31:0] _id_medeleg;
    (* MARK_DEBUG = "TRUE" *) wire [31:0] _id_mideleg;
    (* MARK_DEBUG = "TRUE" *) wire [31:0] _id_mie;
    wire [31:0] _id_mtvec;
    wire [31:0] _id_mepc;
    (* MARK_DEBUG = "TRUE" *) wire [31:0] _id_mip;
    wire [31:0] _id_mcause;
    wire [31:0] _id_mtval;

    wire [31:0] _id_old_mstatus;

    wire [ 1:0] _id_mstatus_mpp;
    wire [ 1:0] _id_mstatus_spp;

    wire [31:0] _id_csr_mask;

    

    assign _id_mstatus_spp[1] = 1'b0;

    exp_mstatus_decoder id_emd(
        .mstatus(_id_mstatus),
        .mpp(_id_mstatus_mpp),
        .spp(_id_mstatus_spp[0]),
        .sum(id_mstatus_sum)

    );
    

    assign id_next_exception = id_prev_exception ? id_prev_exception : id_exception;
    assign id_next_trap_mode = id_prev_exception ? id_prev_trap_mode : id_trap_mode;
    assign id_next_mcause    = id_prev_exception ? id_prev_mcause    : id_mcause;
    assign id_next_mtval     = id_prev_exception ? id_prev_mtval     : id_mtval;

    wire _id_int_exception;
    assign _id_int_exception = (id_int_int != 32'b0);
    
    wire [31:0] _id_temp_mcause;

    // bubble is not waiting for interrupt
    assign id_exception = ~id_nop & (_id_int_exception | id_exp_exception);
    assign id_trap_mode = _id_int_exception ? id_int_trap_mode : id_exp_trap_mode;
    assign id_mtval     =  id_exp_exception ? id_exp_mtval     : 32'b0;
    assign id_mcause    =  id_exception     ? _id_temp_mcause  : 32'b0;

    exp_mcause_encoder id_eme(
        .int_i(id_int_int),
        .exception_code_i(id_exp_exception_code),
        .mcause_o(_id_temp_mcause)
    );

    wire _id_inst_decode_ecall;
    wire _id_inst_decode_ebreak;
    wire _id_inst_decode_mret;
    wire _id_inst_decode_sret;
    wire _id_inst_decode_tlb_clear;

    

    wire        _id_exp_normal_exception;
    wire [30:0] _id_exp_normal_exception_code;
    wire [ 1:0] _id_exp_normal_trap_mode;
    // exception controller
    exp_exception_encoder id_eee (
        .mode_i             (id_mode),
        .medeleg_i          (_id_medeleg),

        .inst_addr_bp       (1'b0), // not implemented
        .inst_access_fault  (1'b0), // if stage
        .inst_page_fault    (1'b0), // if stage
        .inst_illegal       (_id_invalid_inst),
        .inst_misaligned    (1'b0), // if stage
        .ecall              (_id_inst_decode_ecall),
        .ebreak             (_id_inst_decode_ebreak),
        .lsa_addr_bp        (1'b0), // not implemented, 
        .load_misaligned    (1'b0), // exe stage
        .sa_misaligned      (1'b0), // exe stage
        .load_page_fault    (1'b0), // mem stage
        .sa_page_fault      (1'b0), // mem stage
        .load_access_fault  (1'b0), // mem stage
        .sa_access_fault    (1'b0), // mem stage

        .addr_i             (id_inst_pc),
        .inst_i             (id_inst),

        .exception_o        (_id_exp_normal_exception),
        .exception_code_o   (_id_exp_normal_exception_code),
        .exp_mode_o         (_id_exp_normal_trap_mode),
        .mtval_o            (id_exp_mtval)
    );

    // id_exp_exception_code
    always_comb begin
        // exception occurs before actually execute mret/sret
        if (_id_exp_normal_exception) begin
            id_exp_exception        = _id_exp_normal_exception;
            id_exp_exception_code   = _id_exp_normal_exception_code;
            id_exp_trap_mode        = _id_exp_normal_trap_mode;
        end else if (_id_inst_decode_mret) begin
            id_exp_exception        = 1'b1;
            id_exp_exception_code   = `EXP_MRET;
            id_exp_trap_mode        = _id_mstatus_mpp;
        end else if (_id_inst_decode_sret) begin
            id_exp_exception        = 1'b1;
            id_exp_exception_code   = `EXP_SRET;
            id_exp_trap_mode        = _id_mstatus_spp;
        end else if(_id_inst_decode_tlb_clear) begin
            id_exp_exception        = 1'b1;
            id_exp_exception_code   = `EXP_SFENCE_VMA;
            id_exp_trap_mode        = id_mode; // DO not change anything
        end else begin
            id_exp_exception        = 1'b0;
            id_exp_exception_code   = 31'b0;
            id_exp_trap_mode        = 2'b0;
        end
    end
    // trap mode is useless

    // interrupt controller
    exp_interrupt_encoder id_eie (
        .mode_i     (id_mode),

        .mideleg_i  (_id_mideleg),
        .mstatus_i  (_id_mstatus),
        .mip_i      (_id_mip),
        .mie_i      (_id_mie),

        .int_o      (id_int_int),
        .int_mode_o (id_int_trap_mode)
    );


    // instruction decoder
    id_instruction_decoder id_instruction_decoder_inst (
        .inst_i             (id_inst),
        .mode_i             (id_mode),

        .invalid_inst_o     (_id_invalid_inst),
        .ecall_o            (_id_inst_decode_ecall),
        .ebreak_o           (_id_inst_decode_ebreak),
        .mret_o             (_id_inst_decode_mret),
        .sret_o             (_id_inst_decode_sret),
        .tlb_clear_o        (_id_inst_decode_tlb_clear),

        .is_branch_o        (id_is_branch),

        .reg_rs1_o          (_id_reg_rs1),
        .reg_rs2_o          (_id_reg_rs2),
        .reg_rd_o           (_id_reg_rd),
        .rf_write_enable_o  (_id_rf_write_enable),
        .data_rd_mux_o      (_id_data_rd_mux),

        .id_csr_o           (_id_id_csr),
        .csr_write_enable_o (_id_csr_write_enable),
        .csr_mask_o         (_id_csr_mask),

        .imm_o              (_id_imm),
        .uimm_o             (_id_uimm),

        .alu_op_o           (_id_alu_op),
        .alu_a_mux_o        (_id_alu_a_mux),
        .alu_b_mux_o        (_id_alu_b_mux),

        .cmp_op_o           (_id_cmp_op),

        .byte_sel_o         (_id_byte_sel),
        .mem_operation_o    (_id_mem_operation),
        .mem_write_enable_o (_id_mem_write_enable),
        .mem_unsigned_ext_o (_id_mem_unsigned_ext)
    );

    // register file
    id_register_file id_register_file_inst (
        .clk_i      (clk),
        .rst_i      (rst),

        .waddr_i    (wb_reg_rd),
        .wdata_i    (wb_data_rd),
        .we_i       (wb_rf_write_enable),
        .raddr_a_i  (_id_reg_rs1),
        .raddr_b_i  (_id_reg_rs2),

        .rdata_a_o  (_id_data_rs1_old),
        .rdata_b_o  (_id_data_rs2_old)
    );

    // csr
    id_csr_file id_csr_file_inst (
        .clk_i           (clk    ),
        .rst_i           (rst    ),

        .raddr_a_i       (_id_id_csr             ),
        .rdata_a_o       (_id_data_csr           ),

        .waddr_i         (wb_id_csr              ),
        .wdata_i         (wb_data_csr            ),
        .we_i            (wb_csr_write_enable    ),

        .mem_waddr_i     (mem_id_csr             ),
        .mem_wdata_i     (mem_new_data_csr       ),
        .mem_we_i        (mem_csr_write_enable   ),

        .exe_waddr_i     (exe_id_csr             ),
        .exe_wdata_i     (exe_alu_y              ),
        .exe_we_i        (exe_csr_write_enable   ),

        .core_time_irq_i (core_time_irq          ),
        .core_time_i     (core_time              ),

        .wb_exception_i  (wb_prev_exception      ),

        .mstatus_i       (wb_mstatus             ),

        .mepc_i          (wb_mepc                ),
        .mtval_i         (wb_mtval               ),
        .mcause_i        (wb_mcause              ),

        .sepc_i          (wb_sepc                ),
        .stval_i         (wb_stval               ),
        .scause_i        (wb_scause              ),

        //.sstatus_o       (_id_sstatus  ),
        //.sie_o           (_id_sie      ),
        .stvec_o         (_id_stvec    ),
        .sepc_o          (_id_sepc     ),
        //.sip_o           (_id_sip      ),
        .satp_o          (id_satp      ),
        .scause_o        (_id_scause   ),
        .stval_o         (_id_stval    ),

        .mstatus_o       (_id_mstatus  ),
        .medeleg_o       (_id_medeleg  ),
        .mideleg_o       (_id_mideleg  ),
        .mie_o           (_id_mie      ),
        .mtvec_o         (_id_mtvec    ),
        .mepc_o          (_id_mepc     ),
        .mip_o           (_id_mip      ),
        .mcause_o        (_id_mcause   ),
        .mtval_o         (_id_mtval    ),

        .old_mstatus_o   (_id_old_mstatus)
    );

    // stall controller
    // TODO: optimization: branch instruction is no need to stall
    id_stall_controller id_stall_controller_inst (
        .exe_stall_i    (exe_stall),
        .id_stall_o     (id_stall)
    );

    // bubble controller
    id_bubble_controller id_bubble_controller_inst (
        .wait_reg_i     (id_wait_reg),

        .exe_next_exception_i   (exe_next_exception),
        .mem_next_exception_i   (mem_next_exception),
        .wb_prev_exception_i    (wb_prev_exception),

        .id_bubble_o    (_id_bubble)
    );

    // wait register controller

    id_wait_reg_controller id_wait_reg_controller_inst (
        .reg_rs1_i              (_id_reg_rs1),
        .reg_rs2_i              (_id_reg_rs2),

        .data_rs1_i             (_id_data_rs1_old),
        .data_rs2_i             (_id_data_rs2_old),

        .exe_reg_rd_i           (exe_reg_rd),
        .exe_rf_write_enable_i  (exe_rf_write_enable),
        .exe_alu_y_i            (exe_alu_y),        // (arithmetic instruction)
        .exe_data_csr_i         (_exe_masked_data_csr),     // (csr instruction)
        .exe_pc_plus4_i         (exe_pc_plus4),     // (jal/jalr instruction)
        .exe_data_rd_mux_i      (exe_data_rd_mux), 

        .mem_reg_rd_i           (mem_reg_rd),
        .mem_rf_write_enable_i  (mem_rf_write_enable),
        .mem_data_rd_i          (mem_data_rd),
        .mem_load_data_i        (mem_data_rd_mux == `DATA_RD_MEM),
        .mem_done_i             (mem_done),

        .data_rs1_o             (_id_data_rs1),
        .data_rs2_o             (_id_data_rs2),
        
        .wait_reg_o             (id_wait_reg)
    );

    // pipeline registers
    id_pipeline_regs id_pipeline_regs_inst (
        .clk_i                  (clk),
        .rst_i                  (rst),

        .bubble_i               (_id_bubble),
        .stall_i                (id_stall),
        .prev_exception_i       (exe_prev_exception | mem_prev_exception | wb_prev_exception),

        .nop_o                  (exe_nop),

        .mode_i                 (id_mode),

        .alu_op_i               (_id_alu_op),
        .cmp_op_i               (_id_cmp_op),
        .imm_i                  (_id_imm),
        .uimm_i                 (_id_uimm),
        .data_rs1_i             (_id_data_rs1),
        .data_rs2_i             (_id_data_rs2),

        .reg_rd_i               (_id_reg_rd),
        .id_csr_i               (_id_id_csr),
        .data_csr_i             (_id_data_csr),
        .csr_write_enable_i     (_id_csr_write_enable),
        .csr_mask_i             (_id_csr_mask),

        .is_branch_i            (id_is_branch),
        .inst_pc_i              (id_inst_pc),
        .pc_plus4_i             (id_pc_plus4),

        .mem_operation_i        (_id_mem_operation),
        .mem_write_enable_i     (_id_mem_write_enable),
        .mem_unsigned_ext_i     (_id_mem_unsigned_ext),

        .rf_write_enable_i      (_id_rf_write_enable),
        .data_rd_mux_i          (_id_data_rd_mux),

        .byte_sel_i             (_id_byte_sel),
        .alu_a_mux_i            (_id_alu_a_mux),
        .alu_b_mux_i            (_id_alu_b_mux),

        .mode_o                 (exe_mode),
        
        .alu_op_o               (exe_alu_op),
        .cmp_op_o               (exe_cmp_op),
        .imm_o                  (exe_imm),
        .uimm_o                 (exe_uimm),
        .data_rs1_o             (exe_data_rs1),
        .data_rs2_o             (exe_data_rs2),

        .reg_rd_o               (exe_reg_rd),
        .id_csr_o               (exe_id_csr),
        .data_csr_o             (exe_data_csr),
        .csr_write_enable_o     (exe_csr_write_enable),
        .csr_mask_o             (exe_csr_mask),

        .is_branch_o            (exe_is_branch),
        .inst_pc_o              (exe_inst_pc),
        .pc_plus4_o             (exe_pc_plus4),

        .mem_operation_o        (exe_mem_operation),
        .mem_write_enable_o     (exe_mem_write_enable),
        .mem_unsigned_ext_o     (exe_mem_unsigned_ext),

        .rf_write_enable_o      (exe_rf_write_enable),
        .data_rd_mux_o          (exe_data_rd_mux),

        .byte_sel_o             (exe_byte_sel),
        .alu_a_mux_o            (exe_alu_a_mux),
        .alu_b_mux_o            (exe_alu_b_mux),

        .exception_i            (id_next_exception),
        .exception_o            (exe_prev_exception),

        .trap_mode_i            (id_next_trap_mode),
        .trap_mode_o            (exe_prev_trap_mode),

        .mcause_i               (id_next_mcause),
        .mcause_o               (exe_prev_mcause),

        .mtval_i                (id_next_mtval),
        .mtval_o                (exe_prev_mtval),

        
        .medeleg_i              (_id_medeleg),
        .medeleg_o              (exe_medeleg)

    );

    /* =========== stage id end =========== */

    /* =========== stage exe begin =========== */

    // internal wires
    logic [31:0] _exe_alu_a;
    logic [31:0] _exe_alu_b;
    wire         _exe_bubble;

    wire  [31:0] _exe_masked_data_csr;

    logic        exe_next_mem_operation;

    // not doing memory operation when exception occurs
    assign exe_next_mem_operation = (exe_next_exception | mem_next_exception | wb_prev_exception) ? 1'b0: exe_mem_operation;

    assign _exe_masked_data_csr = exe_data_csr & exe_csr_mask;


    assign exe_next_exception = exe_prev_exception ? exe_prev_exception : exe_exception;
    assign exe_next_trap_mode = exe_prev_exception ? exe_prev_trap_mode : exe_trap_mode;
    assign exe_next_mcause    = exe_prev_exception ? exe_prev_mcause    : exe_mcause;
    assign exe_next_mtval     = exe_prev_exception ? exe_prev_mtval     : exe_mtval;


    wire exe_load_misaligned   ;  
    wire exe_sa_misaligned     ;


    wire exe_misaligned_addr  ;
    assign exe_misaligned_addr = (exe_alu_y[1:0] != 2'b00);

    assign exe_load_misaligned = exe_mem_operation && !exe_mem_write_enable && exe_byte_sel == 4'b1111  && exe_misaligned_addr;
    assign exe_sa_misaligned   = exe_mem_operation && exe_mem_write_enable  && exe_byte_sel == 4'b1111  && exe_misaligned_addr;


    exp_mcause_encoder exe_eme(
        .int_i(32'b0),
        .exception_code_i(exe_exp_exception_code),
        .mcause_o(exe_mcause)
    );


    exp_exception_encoder exe_eee (
        .mode_i             (exe_mode),
        .medeleg_i          (exe_medeleg), 

        .inst_addr_bp       (1'b0), // not implemented
        .inst_access_fault  (1'b0), // not implemented: if stage
        .inst_page_fault    (1'b0), // not implemented: if stage
        .inst_illegal       (1'b0), // id stage
        .inst_misaligned    (1'b0), // if stage
        .ecall              (1'b0), // id stage
        .ebreak             (1'b0), // id stage
        .lsa_addr_bp        (1'b0), // not implemented, 
        .load_misaligned    (exe_load_misaligned), // exe stage
        .sa_misaligned      (exe_sa_misaligned), // exe stage
        .load_page_fault    (1'b0), // mem stage
        .sa_page_fault      (1'b0), // mem stage
        .load_access_fault  (1'b0), // mem stage
        .sa_access_fault    (1'b0), // mem stage

        .addr_i             (exe_alu_y),
        .inst_i             (32'b0),

        .exception_o        (exe_exception),
        .exception_code_o   (exe_exp_exception_code),
        .exp_mode_o         (exe_trap_mode),
        .mtval_o            (exe_mtval)
    );
    
    
    // _exe_alu_a
    always_comb begin
        case (exe_alu_a_mux)
            `ALU_A_PC   : _exe_alu_a = exe_inst_pc;
            `ALU_A_RS1  : _exe_alu_a = exe_data_rs1;
            `ALU_A_UIMM : _exe_alu_a = exe_uimm;
            default     : _exe_alu_a = 32'b0;
        endcase
    end
    
    // _exe_alu_b
    always_comb begin
        case (exe_alu_b_mux)
            `ALU_B_RS2  : _exe_alu_b = exe_data_rs2;
            `ALU_B_IMM  : _exe_alu_b = exe_imm;
            `ALU_B_CSR  : _exe_alu_b = exe_data_csr;
            default     : _exe_alu_b = 32'b0;
        endcase
    end
    
    exe_alu exe_alu_inst (
        .op  (exe_alu_op),
        .mask(exe_csr_mask),
        .a   (_exe_alu_a),
        .b   (_exe_alu_b),
        .y   (exe_alu_y)
    );
    
    exe_comparator exe_comparator_inst (
        .op  (exe_cmp_op),
        .a   (exe_data_rs1),
        .b   (exe_data_rs2),
        .y   (exe_branch_take)
    );
    

    exe_stall_controller exe_stall_controller_inst (
        .mem_stall_i        (mem_stall),
        .mem_operation_i    (mem_mem_operation),
        .mem_done_i         (mem_done),
        .exe_stall_o        (exe_stall)
    );
    
    exe_bubble_controller exe_bubble_controller_inst (
        .mem_next_exception_i   (mem_next_exception),
        .wb_prev_exception_i    (wb_prev_exception),
        .exe_bubble_o(_exe_bubble)
    );
    
    // register outputs
    exe_pipeline_regs exe_pipeline_regs_inst (
        .clk_i                  (clk),
        .rst_i                  (rst),
    
        .stall_i                (exe_stall),
        .bubble_i               (_exe_bubble),
        .prev_exception_i       (mem_prev_exception | wb_prev_exception),
        .nop_o                  (mem_nop),

        .mode_i                 (exe_mode),
        .inst_pc_i              (exe_inst_pc),
        .pc_plus4_i             (exe_pc_plus4),
        .alu_y_i                (exe_alu_y),
        .mem_operation_i        (exe_next_mem_operation),
        .mem_write_enable_i     (exe_mem_write_enable),
        .mem_unsigned_ext_i     (exe_mem_unsigned_ext),
        .rf_write_enable_i      (exe_rf_write_enable),
        .data_rd_mux_i          (exe_data_rd_mux),
        .byte_sel_i             (exe_byte_sel),
        .data_rs2_i             (exe_data_rs2),
        .reg_rd_i               (exe_reg_rd),
        .id_csr_i               (exe_id_csr),
        .data_csr_i             (_exe_masked_data_csr), // just allow masked bits
        .csr_write_enable_i     (exe_csr_write_enable),

        .mode_o                 (mem_mode),
        .inst_pc_o              (mem_inst_pc),
        .pc_plus4_o             (mem_pc_plus4),
        .alu_y_o                (mem_alu_y),
        .mem_operation_o        (mem_mem_operation),
        .mem_write_enable_o     (mem_mem_write_enable),
        .mem_unsigned_ext_o     (mem_mem_unsigned_ext),
        .rf_write_enable_o      (mem_rf_write_enable),
        .data_rd_mux_o          (mem_data_rd_mux),
        .byte_sel_o             (mem_byte_sel),
        .data_rs2_o             (mem_data_rs2),
        .reg_rd_o               (mem_reg_rd),
        .id_csr_o               (mem_id_csr),
        .data_csr_o             (mem_data_csr),
        .csr_write_enable_o     (mem_csr_write_enable),

        .exception_i            (exe_next_exception),
        .exception_o            (mem_prev_exception),

        .trap_mode_i            (exe_next_trap_mode),
        .trap_mode_o            (mem_prev_trap_mode),

        .mcause_i               (exe_next_mcause),
        .mcause_o               (mem_prev_mcause),

        .mtval_i                (exe_next_mtval),
        .mtval_o                (mem_prev_mtval),

        .medeleg_i              (exe_medeleg),
        .medeleg_o              (mem_medeleg)
    );

    /* =========== stage exe end =========== */

    /* =========== stage mem begin =========== */

    // internal wires
    wire [31:0] _mem_data_read_final;
    wire [31:0] _mem_data_read;

    wire        _mem_bubble;

    wire        mem_dm_ack;
    wire [31:0] mem_dm_ack_addr;

    assign mem_done = mem_dm_ack & (mem_dm_ack_addr == mem_alu_y);

    assign mem_next_exception = mem_prev_exception ? mem_prev_exception : mem_exception;
    assign mem_next_trap_mode = mem_prev_exception ? mem_prev_trap_mode : mem_trap_mode;
    assign mem_next_mcause    = mem_prev_exception ? mem_prev_mcause    : mem_mcause;
    assign mem_next_mtval     = mem_prev_exception ? mem_prev_mtval     : mem_mtval;

    wire mem_load_page_fault   ;  
    wire mem_sa_page_fault     ;
    wire mem_load_access_fault ;
    wire mem_sa_access_fault   ;
    
    wire mem_pagefault;
    
    // wire mem_pagefault = wbm0_ack_i & mem_mem_operation & core_page_fault;

    
    assign mem_load_page_fault = mem_pagefault & ~mem_mem_write_enable;
    assign mem_sa_page_fault   = mem_pagefault & mem_mem_write_enable;

    // TODO: implement access fault
    assign mem_load_access_fault = 1'b0;
    assign mem_sa_access_fault   = 1'b0;


    exp_mcause_encoder mem_eme(
        .int_i(32'b0),
        .exception_code_i(mem_exp_exception_code),
        .mcause_o(mem_mcause)
    );


    exp_exception_encoder mem_eee (
        .mode_i             (mem_mode),
        .medeleg_i          (mem_medeleg), 

        .inst_addr_bp       (1'b0), // not implemented
        .inst_access_fault  (1'b0), // not implemented: if stage
        .inst_page_fault    (1'b0), // not implemented: if stage
        .inst_illegal       (1'b0), // id stage
        .inst_misaligned    (1'b0), // if stage
        .ecall              (1'b0), // id stage
        .ebreak             (1'b0), // id stage
        .lsa_addr_bp        (1'b0), // not implemented, 
        .load_misaligned    (1'b0), // exe stage
        .sa_misaligned      (1'b0), // exe stage
        .load_page_fault    (mem_load_page_fault), // mem stage
        .sa_page_fault      (mem_sa_page_fault), // mem stage
        .load_access_fault  (mem_load_access_fault), // mem stage
        .sa_access_fault    (mem_sa_access_fault), // mem stage

        .addr_i             (mem_alu_y),
        .inst_i             (32'b0),

        .exception_o        (mem_exception),
        .exception_code_o   (mem_exp_exception_code),
        .exp_mode_o         (mem_trap_mode),
        .mtval_o            (mem_mtval)
    );
    
    assign mem_new_data_csr = mem_alu_y;

    mem_imm_gen mem_imm_gen_inst(
        .imm_i          (_mem_data_read),
        .byte_sel_i     (mem_byte_sel),
        .unsigned_ext_i (mem_mem_unsigned_ext),
        .imm_o          (_mem_data_read_final)
    );

    mem_data_rd_mux mem_data_rd_mux_inst(
        .pc_i           (mem_pc_plus4),
        .alu_i          (mem_alu_y),
        .mem_i          (_mem_data_read_final),
        .csr_i          (mem_data_csr),
        .data_rd_mux_i  (mem_data_rd_mux),
        .data_rd_o      (mem_data_rd)
    );

    
    logic [31:0] _mem_vmi_data_o;
    logic [31:0] _exe_vmi_data_i;
    logic [ 3:0] _exe_vmi_sel_o;


    assign _exe_vmi_data_i = exe_mem_write_enable ? exe_data_rs2 << (exe_alu_y[1:0] * 8) : 32'b0; // not aligned
    assign _exe_vmi_sel_o = exe_byte_sel << exe_alu_y[1:0]; // not aligned

    assign _mem_data_read = _mem_vmi_data_o >> (mem_alu_y[1:0] * 8);  // not aligned

    logic mem_cache_sync_done_reg;
    always_ff @(posedge clk) begin
        if (rst) begin
            mem_cache_sync_done_reg <= 1'b0;
        end else begin
            mem_cache_sync_done_reg <= mem_cache_sync_done;
        end
    end

    mmu_virtual_memory_interface mem_dm_interface (
        .clk_i              (clk),
        .rst_i              (rst),

        .no_flush_cache_i   (1'b0),

        // memory interface signals ()
        .vmi_no_cache_i     (dip_sw[1]),
        .vmi_enable_i       (exe_next_mem_operation),
        .vmi_write_enable_i (exe_mem_write_enable),
        .vmi_addr_i         (exe_alu_y),
        .vmi_data_i         (_exe_vmi_data_i),
        .vmi_data_sel_i     (_exe_vmi_sel_o),
    
        .vmi_ack_addr_o     (mem_dm_ack_addr),
        .vmi_ack_o          (mem_dm_ack),
        .vmi_data_o         (_mem_vmi_data_o),

        // status signals (it is upper module's responsibility to keep these signals stable)
        .satp_i             (core_satp),
        .sum_i              (core_sum),
        .mode_i             (core_mode),
        .flush_i            (core_tlb_clear),

        .page_fault_o       (mem_pagefault),

        // wbm signals (as master) (fall back to wish bone bus)
        .wb_ack_i           (wbm0_ack_i),
        .wb_dat_i           (wbm0_dat_i),
        .wb_cyc_o           (wbm0_cyc_o),
        .wb_stb_o           (wbm0_stb_o),
        .wb_adr_o           (wbm0_adr_o),
        .wb_dat_o           (wbm0_dat_o),
        .wb_sel_o           (wbm0_sel_o),
        .wb_we_o            (wbm0_we_o ),

        .pmi_yield_i        (1'b0),
        .pmi_wait_bus_o     (mem_pmi_wait_bus),
        .sync_done_o        (mem_cache_sync_done),

        // debug signals

        .debug_sv32_tlb_enable_i (~dip_sw[0]),

        .debug_state_o           (debug_mem_vmi_state),
        .debug_sv32_pte_o        (debug_mem_vmi_pte),
        .debug_sv32_pf_cause_o   (debug_mem_vmi_pf_cause),
        .debug_sv32_tlb_hit_o    (debug_mem_vmi_tlb_hit),
        .debug_sv32_pte_state_o  (debug_mem_vmi_pte_state),

        .debug_paddr_o           (debug_mem_vmi_paddr),

        .debug_tlb_write_enable_o (debug_mem_vmi_tlb_write_enable),
        .debug_tlb_ppn_in_o      (debug_mem_vmi_tlb_ppn_in),
        .debug_tlb_vpn_in_o      (debug_mem_vmi_tlb_vpn_in),

        .debug_pmi_addr_in_o     (debug_mem_pmi_addr_in_o),
        .debug_pmi_data_out_o    (debug_mem_pmi_data_out_o),
        .debug_pmi_ack_o         (debug_mem_pmi_ack_o),
        .debug_pmi_ack_addr_o    (debug_mem_pmi_ack_addr_o),

        .debug_pmi_state          (debug_mem_pmi_state),
        .debug_pmi_cache_hit      (debug_mem_pmi_cache_hit),
        .debug_pmi_cache_addr_out (debug_mem_pmi_cache_addr_out)



    );



    mem_stall_controller mem_stall_controller_inst(
        .mem_stall_o    (mem_stall)
    );

    mem_bubble_controller mem_bubble_controller_inst(
        .mem_operation_i(mem_mem_operation),
        .mem_done_i     (mem_done),

        .wb_prev_exception_i    (wb_prev_exception),

        .mem_bubble_o   (_mem_bubble)
    );

    mem_pipeline_regs mem_pipeline_regs_inst(
        .clk_i              (clk),
        .rst_i              (rst),

        .bubble_i           (_mem_bubble),
        .stall_i            (mem_stall),
        .prev_exception_i   (wb_prev_exception),

        .data_rd_i          (mem_data_rd),
        .reg_rd_i           (mem_reg_rd),
        .rf_write_enable_i  (mem_rf_write_enable),

        .data_csr_i         (mem_new_data_csr),
        .id_csr_i           (mem_id_csr),
        .csr_write_enable_i (mem_csr_write_enable),

        .data_rd_o          (wb_data_rd),
        .reg_rd_o           (wb_reg_rd),
        .rf_write_enable_o  (wb_rf_write_enable),

        .data_csr_o         (wb_data_csr),
        .id_csr_o           (wb_id_csr),
        .csr_write_enable_o (wb_csr_write_enable),

        .mode_i             (mem_mode),
        .mode_o             (wb_mode),

        .inst_pc_i          (mem_inst_pc),
        .inst_pc_o          (wb_inst_pc),

        .exception_i        (mem_next_exception),
        .exception_o        (wb_prev_exception),

        .trap_mode_i        (mem_next_trap_mode),
        .trap_mode_o        (wb_prev_trap_mode),

        .mcause_i           (mem_next_mcause),
        .mcause_o           (wb_prev_mcause),

        .mtval_i            (mem_next_mtval),
        .mtval_o            (wb_prev_mtval)
    );

    /* =========== stage mem end =========== */

    /* =========== stage wb begin =========== */
    // basically exception handling

    wire  [31:0] wb_old_mstatus;
    assign wb_old_mstatus = _id_old_mstatus; // don't use forwarded value

    logic [31:0] wb_new_mstatus;

    // internal wires
    wire wb_old_mstatus_mie;
    wire wb_old_mstatus_sie;
    wire wb_old_mstatus_mpie;
    wire wb_old_mstatus_spie;
    wire [1:0] wb_old_mstatus_mpp;
    wire wb_old_mstatus_spp;

    logic wb_new_mstatus_mie;
    logic wb_new_mstatus_sie;
    logic wb_new_mstatus_mpie;
    logic wb_new_mstatus_spie;

    logic [1:0] wb_new_mstatus_mpp;
    logic wb_new_mstatus_spp;

    exp_mstatus_decoder wb_emd(
        .mstatus            (wb_old_mstatus),
        .mie                (wb_old_mstatus_mie),
        .sie                (wb_old_mstatus_sie),
        .mpie               (wb_old_mstatus_mpie),
        .spie               (wb_old_mstatus_spie),
        .mpp                (wb_old_mstatus_mpp),
        .spp                (wb_old_mstatus_spp)
    );

    // set tvec as pc 
    always_comb begin
        // default
        wb_tvec = 32'b0;
        

        wb_mepc     = _id_mepc;
        wb_mtval    = _id_mtval;
        wb_mcause   = _id_mcause;

        wb_sepc     = _id_sepc;
        wb_stval    = _id_stval;
        wb_scause   = _id_scause;

        wb_new_mstatus_mie  = wb_old_mstatus_mie;
        wb_new_mstatus_sie  = wb_old_mstatus_sie;
        wb_new_mstatus_mpie = wb_old_mstatus_mpie;
        wb_new_mstatus_spie = wb_old_mstatus_spie;
        wb_new_mstatus_mpp  = wb_old_mstatus_mpp;
        wb_new_mstatus_spp  = wb_old_mstatus_spp;
        if (wb_prev_exception) begin
            if (wb_prev_mcause == `EXP_MRET) begin
                wb_tvec = _id_mepc;
                wb_new_mstatus_mie  = wb_old_mstatus_mpie;
                wb_new_mstatus_mpie = 1'b1;
                wb_new_mstatus_mpp  = 2'b00;
            end else if (wb_prev_mcause == `EXP_SRET) begin
                wb_tvec = _id_sepc;
                wb_new_mstatus_sie  = wb_old_mstatus_spie;
                wb_new_mstatus_spie = 1'b1;
                wb_new_mstatus_spp  = 1'b0;
            end else if (wb_prev_mcause == `EXP_SFENCE_VMA) begin
                wb_tvec = wb_inst_pc + 3'd4; // TODO: replace with reg

            end else begin
                if (wb_prev_trap_mode == M_MODE) begin
                    wb_tvec = _id_mtvec;
                    wb_new_mstatus_mie  = 1'b0;
                    wb_new_mstatus_mpie = wb_old_mstatus_mie;
                    wb_new_mstatus_mpp  = wb_mode;

                    wb_mepc = wb_inst_pc;
                    wb_mtval = wb_prev_mtval;
                    wb_mcause = wb_prev_mcause;
                end else if (wb_prev_trap_mode == S_MODE) begin
                    wb_tvec = _id_stvec;
                    wb_new_mstatus_sie  = 1'b0;
                    wb_new_mstatus_spie = wb_old_mstatus_sie;
                    wb_new_mstatus_spp  = wb_mode;

                    wb_sepc = wb_inst_pc;
                    wb_stval = wb_prev_mtval;
                    wb_scause = wb_prev_mcause;
                end
            end
        end
    end

    always_comb begin
        wb_new_mstatus = wb_old_mstatus;

        wb_new_mstatus[`MSTATUS_MIE]  = wb_new_mstatus_mie;
        wb_new_mstatus[`MSTATUS_SIE]  = wb_new_mstatus_sie;
        wb_new_mstatus[`MSTATUS_MPIE] = wb_new_mstatus_mpie;
        wb_new_mstatus[`MSTATUS_SPIE] = wb_new_mstatus_spie;
        wb_new_mstatus[`MSTATUS_MPP_H:`MSTATUS_MPP_L]  = wb_new_mstatus_mpp;
        wb_new_mstatus[`MSTATUS_SPP]  = wb_new_mstatus_spp;
    end


    assign wb_mstatus  = wb_new_mstatus;


    assign core_tlb_clear = (wb_prev_mcause == `EXP_SFENCE_VMA);
    /* =========== stage wb end =========== */

    ila_0 ila(
        .clk(clk),
        .probe0(core_mode),
        .probe1(core_time),
        .probe2(core_timecmp),
        .probe3(core_time_irq),
        .probe4(core_satp),
        
        .probe5(_id_mstatus),
        .probe6(_id_medeleg),
        .probe7(_id_mideleg),
        .probe8(_id_mie),
        .probe9(_id_mip),
        .probe10(_id_mtvec),
        .probe11(_id_stvec),

        .probe12(wbs_cyc_o),
        .probe13(wbs_stb_o),
        .probe14(wbs_ack_i),
        .probe15(wbs_adr_o),
        .probe16(wbs_dat_o),
        .probe17(wbs_dat_i),
        .probe18(wbs_sel_o),
        .probe19(wbs_we_o ),

        .probe20(_if_inst_addr),
        .probe21(_if_inst_pc),
        .probe22(id_inst_pc),
        .probe23(mem_inst_pc),
        .probe24(wb_inst_pc),
        .probe25(wb_prev_mcause),
        .probe26(wb_prev_exception),
        .probe27(wb_prev_trap_mode),
        
        .probe28(debug_if_vmi_state),
        .probe29(debug_if_vmi_pte),
        .probe30(debug_if_vmi_pf_cause),
        .probe31(debug_if_vmi_tlb_hit),
        .probe32(debug_if_vmi_paddr),
        .probe33(debug_if_vmi_pte_state),

        .probe34(debug_mem_vmi_state),
        .probe35(debug_mem_vmi_pte),
        .probe36(debug_mem_vmi_pf_cause),
        .probe37(debug_mem_vmi_tlb_hit),
        .probe38(debug_mem_vmi_paddr),
        .probe39(debug_mem_vmi_pte_state),
        .probe40(debug_mem_vmi_tlb_write_enable),
        .probe41(debug_mem_vmi_tlb_ppn_in),
        

        .probe42(exe_next_mem_operation),
        .probe43(exe_alu_y),

        .probe44(debug_mem_vmi_tlb_vpn_in),
        .probe45(_if_inst),

        .probe46(debug_mem_pmi_addr_in_o),
        .probe47(debug_mem_pmi_data_out_o),
        .probe48(debug_mem_pmi_ack_o),
        .probe49(debug_mem_pmi_ack_addr_o),

        .probe50(debug_mem_pmi_state),
        .probe51(debug_mem_pmi_cache_hit),
        .probe52(debug_mem_pmi_cache_addr_out),

        .probe53(debug_eram_data_in),
        .probe54(debug_eram_data_out),
        .probe55(debug_eram_addr),
        .probe56(debug_eram_we_n),
        .probe57(debug_eram_ce_n),
        .probe58(debug_eram_oe_n),
        .probe59(debug_eram_be_n),

        .probe60(debug_eram_data_is_in)


    );

    ila_pc ila_pc_inst( 
        .clk(clk),
        .probe0(wb_inst_pc) // not wasted instructions
    );



endmodule
