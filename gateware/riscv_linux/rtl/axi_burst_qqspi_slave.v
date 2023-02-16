/*
 * axi_burst_qqspi_slave taken from mem_sim.v
 *
 * vim: ts=4 sw=4
 *
 * Copyright (C) 2020-2021  Sylvain Munaut <tnt@246tNt.com>
 * Copyright (c) 2023 Hirosh Dabui <hirosh@dabui.de>
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 */

`default_nettype none

module axi_burst_qqspi_slave #(
    parameter INIT_FILE = "",
    parameter integer AW = 20,

    // auto
    parameter integer AL = AW - 1
) (
    // Memory controller interface
    input  wire [AL:0] mi_addr,
    input  wire [ 6:0] mi_len,
    input  wire        mi_rw,
    input  wire        mi_valid,
    output wire        mi_ready,

    input  wire [31:0] mi_wdata,
    output wire        mi_wack,
    output wire        mi_wlast,

    output wire [31:0] mi_rdata,
    output wire        mi_rstb,
    output wire        mi_rlast,

    output wire cen,
    output wire sclk,
    inout wire sio1_so_miso,
    inout wire sio0_si_mosi,
    inout wire sio2,
    inout wire sio3,
    output wire [3:0] cs,
    // Common
    input wire clk,
    input wire rst
);

  localparam [1:0] ST_IDLE = 0, ST_PAUSE = 1, ST_WRITE = 2, ST_READ = 3;


  // Signals
  // -------

  // Memory array

  wire                                 [AL:0] mem_addr;
  wire                                 [31:0] mem_wdata;
  wire                                 [31:0] mem_rdata;
  wire mem_we = state_cur == ST_WRITE;
  reg                                         mem_valid;
  wire                                        mem_ready;

  // FSM
  reg                                  [ 1:0] state_cur;
  reg                                  [ 1:0] state_nxt;

  // Command counters
  reg                                  [AL:0] cmd_addr;
  reg                                  [ 7:0] cmd_len;
  wire                                        cmd_last;

  // Addresses
  wire                                 [AL:0] rd_addr;
  wire                                 [AL:0] wr_addr;


  // Memory content
  // --------------

  `define QQSPI
`ifndef QQSPI
  memsim_data #(
      .INIT_FILE(INIT_FILE),
      .AW(AW)
  ) data_I (
      .mem_addr (mem_addr),
      .mem_wdata(mem_wdata),
      .mem_rdata(mem_rdata),
      .mem_we   (mem_we),
      .mem_valid(mem_valid),
      .mem_ready(mem_ready),
      .clk      (clk)
  );
`else
  qqspi #(
      .QUAD_MODE(1'b1)
  ) qqspi_i (
      .addr        (mem_addr),
      .rdata       (mem_rdata),
      .wdata       (mem_wdata),
      .wstrb       ({4{mem_we}}),
      .ready       (mem_ready),
      .valid       (mem_valid),
      .clk         (clk),
      .resetn      (rst),
      .cen         (cen),
      .sclk        (sclk),
      .sio1_so_miso(sio1_so_miso),
      .sio0_si_mosi(sio0_si_mosi),
      .sio2        (sio2),
      .sio3        (sio3),
      .cs          (cs)
  );
`endif
  // Main FSM
  // --------

  always @(posedge clk)
    if (rst) state_cur <= ST_IDLE;
    else state_cur <= state_nxt;

  always @(*) begin
    state_nxt = state_cur;

    case (state_cur)
      ST_IDLE:  if (mi_valid) state_nxt = mi_rw ? ST_READ : ST_WRITE;
      ST_PAUSE: if (!mem_we) state_nxt = ST_IDLE;
      ST_READ:  if (mi_rlast) state_nxt = ST_PAUSE;
      ST_WRITE: if (mi_wlast) state_nxt = ST_PAUSE;
    endcase
  end


  // Command channel
  // ---------------

  // Register command
  always @(posedge clk) begin
    if (state_cur == ST_IDLE) begin
      cmd_addr <= mi_addr;
      //cmd_len  <= !mi_rw ? {1'b0, mi_len} : {1'b0, mi_len} +1;// - 1;
      //cmd_len  <= {1'b0, mi_len} -1;
      //cmd_len  <= !mi_rw ? {1'b0, mi_len} -1: {1'b0, mi_len} +0;// - 1;
      cmd_len  <= {1'b0, mi_len};  // works

    end else begin
      if (xfer_state && mem_valid) begin
        cmd_addr <= cmd_addr + 1;
        cmd_len  <= cmd_len - 1;
      end
    end
  end

  assign cmd_last  = cmd_len[7];
  assign mi_ready  = (state_cur == ST_IDLE);
  assign mem_addr  = mem_we ? wr_addr : rd_addr;


  // Write data channel
  // ------------------

  assign mem_wdata = mi_wdata;
  //assign mi_wack   = mem_we & mem_ready;
  assign mi_wack   = mem_we & mem_ready;

  // Read data channel
  // -----------------

  assign wr_addr   = cmd_addr;
  assign rd_addr   = cmd_addr;

  assign mi_rstb   = !mem_we && mem_ready;
  assign mi_rdata  = mi_rstb ? mem_rdata : 32'hxxxxxxxx;
  assign mi_rlast  = !mem_we && cmd_last && mem_ready;
  assign mi_wlast  = mem_we && cmd_last && mem_ready;

  wire is_xfer = (state_cur == ST_WRITE || state_cur == ST_READ);
  reg mem_valid_nxt;

  reg [0:0] xfer_state;
  reg [0:0] xfer_state_nxt;
  always @(posedge clk) begin
    if (rst) begin
      mem_valid  <= 1'b0;
      xfer_state <= 0;
    end else begin
      xfer_state <= xfer_state_nxt;
      mem_valid  <= mem_valid_nxt;
    end
  end

  always @* begin
    xfer_state_nxt = xfer_state;
    mem_valid_nxt  = mem_valid;

    case (xfer_state)
      0: begin
        if (!mem_ready && is_xfer) begin
          xfer_state_nxt = 1;
          mem_valid_nxt  = 1'b1;
        end
      end
      1: begin
        mem_valid_nxt = 1'b0;
        if (mem_ready) begin
          xfer_state_nxt = 0;
        end
      end
      default: xfer_state_nxt = 0;
    endcase
  end


endmodule

`ifndef QQSPI
module memsim_data #(parameter INIT_FILE = "",
    parameter integer AW = 20,

    // auto
    parameter integer AL = AW - 1
) (
    input  wire [AL:0] mem_addr,
    input  wire [31:0] mem_wdata,
    output reg  [31:0] mem_rdata,
    input  wire        mem_valid,
    output reg         mem_ready,
    input  wire        mem_we,
    input  wire        clk
);

  // Memory array
  reg [31:0] mem[0:(1<<AW)-1];
  reg [10:0] p = 0;

  // Init
  initial begin : mem_init
    integer a;

    if (INIT_FILE == "") begin
      for (a = 0; a < (1 << AW) - 1; a = a + 1) mem[a] = 32'hbaadc0de;
    end else begin
      $readmemh(INIT_FILE, mem);
    end

    mem_ready = 0;
  end

  always @(posedge clk) begin
    if (mem_we && mem_valid) mem[mem_addr] <= mem_wdata;
    /*   if (!mem_we && mem_valid) */ mem_rdata <= mem[mem_addr];

    p = {p[9:0], mem_valid};
    mem_ready <= p[10];
  end

endmodule
`endif
