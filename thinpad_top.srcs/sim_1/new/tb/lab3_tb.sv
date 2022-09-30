`timescale 1ns / 1ns
module lab3_tb;

  wire clk_50M, clk_11M0592;

  reg push_btn;   // BTN5 按钮开关，带消抖电路，按下时为 1
  reg reset_btn;  // BTN6 复位按钮，带消抖电路，按下时为 1

  reg [3:0] touch_btn; // BTN1~BTN4，按钮开关，按下时为 1
  reg [31:0] dip_sw;   // 32 位拨码开关，拨到“ON”时为 1

  wire [15:0] leds;  // 16 位 LED，输出时 1 点亮
  wire [7:0] dpy0;   // 数码管低位信号，包括小数点，输出 1 点亮
  wire [7:0] dpy1;   // 数码管高位信号，包括小数点，输出 1 点亮

  // 实验 3 用到的指令格式
  `define inst_rtype(rd, rs1, rs2, op) \
    {7'b0, rs2, rs1, 3'b0, rd, op, 3'b001}

  `define inst_itype(rd, imm, op) \
    {imm, 4'b0, rd, op, 3'b010}
  
  `define inst_poke(rd, imm) `inst_itype(rd, imm, 4'b0001)
  `define inst_peek(rd, imm) `inst_itype(rd, imm, 4'b0010)

  // opcode table
  typedef enum logic [3:0] {
    ADD = 4'b0001,
    SUB = 4'b0010,
    AND = 4'b0011,
    OR  = 4'b0100,
    XOR = 4'b0101,
    NOT = 4'b0110,
    SLL = 4'b0111,
    SRL = 4'b1000,
    SRA = 4'b1001,
    ROL = 4'b1010
  } opcode_t;

  logic is_rtype, is_itype, is_load, is_store, is_unknown;
  logic [15:0] imm;
  logic [4:0] rd, rs1, rs2;
  logic [3:0] opcode;

  logic [15:0] random_value[32];
  logic [15:0] expect_reg[32];

  task execute_instruction(input [31:0] instruction);
    dip_sw = instruction;
    push_btn = 1;
    #100;
    push_btn = 0;
    // wait
    #1000;
  endtask

  task read_and_assert_register(input [4:0] addr, string info);
    execute_instruction(`inst_peek(addr, 0));
    assert(leds == expect_reg[addr]) else $display("(%s) reg[%0d] = %0d, expect %0d",info, addr, dip_sw, expect_reg[addr]);
  endtask

  task write_register(input [4:0] addr, input [15:0] value);
    execute_instruction(`inst_poke(addr, value));
    if (addr != 0) begin
      expect_reg[addr] = value;
    end
    else begin
      expect_reg[addr] = 0;
    end
  endtask

  initial begin
    // 在这里可以自定义测试输入序列，例如：
    dip_sw = 32'h0;
    touch_btn = 0;
    reset_btn = 0;
    push_btn = 0;


    #100;
    reset_btn = 1;
    #100;
    reset_btn = 0;
    #2500;  // 等待复位结束

    // use POKE/PEEK instruction to test register IO
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;
      imm = $urandom_range(0, 65535);
      write_register(rd, imm);
      read_and_assert_register(rd,"POKE/PEEK");
    end


    // use ADD instruction to add two registers
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);
      rs2 = $urandom_range(0, 31);

      execute_instruction(`inst_rtype(rd, rs1, rs2, ADD));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = expect_reg[rs1] + expect_reg[rs2];
      end

      read_and_assert_register(rd,"ADD");
    end

    // use SUB instruction to subtract two registers
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);
      rs2 = $urandom_range(0, 31);

      execute_instruction(`inst_rtype(rd, rs1, rs2, SUB));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = expect_reg[rs1] - expect_reg[rs2];
      end

      read_and_assert_register(rd,"SUB");
    end

    // use AND instruction to and two registers
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);
      rs2 = $urandom_range(0, 31);

      execute_instruction(`inst_rtype(rd, rs1, rs2, AND));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = expect_reg[rs1] & expect_reg[rs2];
      end

      read_and_assert_register(rd,"AND");
    end

    // use OR instruction to or two registers
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);
      rs2 = $urandom_range(0, 31);

      execute_instruction(`inst_rtype(rd, rs1, rs2, OR));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = expect_reg[rs1] | expect_reg[rs2];
      end

      read_and_assert_register(rd,"OR");
    end

    // use XOR instruction to xor two registers
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);
      rs2 = $urandom_range(0, 31);

      execute_instruction(`inst_rtype(rd, rs1, rs2, XOR));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = expect_reg[rs1] ^ expect_reg[rs2];
      end

      read_and_assert_register(rd,"XOR");
    end

    // use NOT instruction to not a register
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);

      execute_instruction(`inst_rtype(rd, rs1, 0, NOT));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = ~expect_reg[rs1];
      end

      read_and_assert_register(rd,"NOT");
    end

    // use SLL instruction to shift left a register
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);
      rs2 = $urandom_range(0, 31);

      // write_register(rs2, $urandom_range(1, 15));

      execute_instruction(`inst_rtype(rd, rs1, rs2, SLL));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = expect_reg[rs1] << expect_reg[rs2][3:0];
      end

      read_and_assert_register(rd,"SLL");
    end

    // use SRL instruction to shift right a register
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);
      rs2 = $urandom_range(0, 31);

      // write_register(rs2, $urandom_range(1, 15));

      execute_instruction(`inst_rtype(rd, rs1, rs2, SRL));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = expect_reg[rs1] >> expect_reg[rs2][3:0];
      end

      read_and_assert_register(rd,"SRL");
    end

    // use SRA instruction to shift right a register
    for (int i = 0; i < 1000; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);
      rs2 = $urandom_range(0, 31);

      write_register(rs1, $urandom_range(0, 65535));
      write_register(rs2, $urandom_range(0, 65535));


      execute_instruction(`inst_rtype(rd, rs1, rs2, SRA));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = ({16 {expect_reg[rs1][15]}} << (16-expect_reg[rs2][3:0])) | (expect_reg[rs1] >> expect_reg[rs2][3:0]);
      end

      read_and_assert_register(rd, "SRA");
    end

    // use ROL instruction to rotate left a register
    for (int i = 0; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      rs1 = $urandom_range(0, 31);
      rs2 = $urandom_range(0, 31);

      // write_register(rs2, $urandom_range(1, 15));

      execute_instruction(`inst_rtype(rd, rs1, rs2, ROL));

      // wait instruction to complete
      #1000;
      if( rd==0 ) begin
        expect_reg[rd] = 0;
      end else begin
        expect_reg[rd] = (expect_reg[rs1] << expect_reg[rs2][3:0] )| (expect_reg[rs1] >> (16-expect_reg[rs2][3:0]));
      end

      read_and_assert_register(rd,"ROL");
    end





    






    #10000 $finish;
  end

  // 待测试用户设计
  lab3_top dut (
      .clk_50M(clk_50M),
      .clk_11M0592(clk_11M0592),
      .push_btn(push_btn),
      .reset_btn(reset_btn),
      .touch_btn(touch_btn),
      .dip_sw(dip_sw),
      .leds(leds),
      .dpy1(dpy1),
      .dpy0(dpy0),

      .txd(),
      .rxd(1'b1),
      .uart_rdn(),
      .uart_wrn(),
      .uart_dataready(1'b0),
      .uart_tbre(1'b0),
      .uart_tsre(1'b0),
      .base_ram_data(),
      .base_ram_addr(),
      .base_ram_ce_n(),
      .base_ram_oe_n(),
      .base_ram_we_n(),
      .base_ram_be_n(),
      .ext_ram_data(),
      .ext_ram_addr(),
      .ext_ram_ce_n(),
      .ext_ram_oe_n(),
      .ext_ram_we_n(),
      .ext_ram_be_n(),
      .flash_d(),
      .flash_a(),
      .flash_rp_n(),
      .flash_vpen(),
      .flash_oe_n(),
      .flash_ce_n(),
      .flash_byte_n(),
      .flash_we_n()
  );

  // 时钟源
  clock osc (
      .clk_11M0592(clk_11M0592),
      .clk_50M    (clk_50M)
  );

endmodule
