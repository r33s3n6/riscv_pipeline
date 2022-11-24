// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2019.2 (win64) Build 2708876 Wed Nov  6 21:40:23 MST 2019
// Date        : Thu Nov 24 10:08:16 2022
// Host        : DESKTOP-J3OH9BI running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               G:/Workspaces/2022F/vivado/cod22-fuzx20/thinpad_top.srcs/sources_1/ip/ila_pc/ila_pc_stub.v
// Design      : ila_pc
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a100tfgg676-2L
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "ila,Vivado 2019.2" *)
module ila_pc(clk, probe0)
/* synthesis syn_black_box black_box_pad_pin="clk,probe0[31:0]" */;
  input clk;
  input [31:0]probe0;
endmodule
