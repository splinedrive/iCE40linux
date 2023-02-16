/*
 * top.v
 *
 * vim: ts=4 sw=4
 *
 * Copyright (C) 2021  Sylvain Munaut <tnt@246tNt.com>
 * Copyright (c) 2023 Hirosh Dabui <hirosh@dabui.de>
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 */

`default_nettype none `timescale 1 ns / 100 ps

module top (
    // SPI
    inout  wire spi_mosi,
    inout  wire spi_miso,
    inout  wire spi_clk,
    output wire spi_flash_cs_n,

    // Debug UART
    input  wire uart_rx,
    output wire uart_tx,

    /*
    input  wire uart_rx1,
    output wire uart_tx1,
    */

    // Button
//    input wire btn,

    // Clock
    //input wire clk_in,
 //   output wire cen,
    output wire sclk,
    inout wire sio1_so_miso,
    inout wire sio0_si_mosi,
    inout wire sio2,
    inout wire sio3,
    output wire [3:0] cs,
    output wire [2:0] rgb
);

  localparam integer WB_N = 4;
  localparam integer WB_DW = 32;
  localparam integer WB_AW = 22;
  localparam integer WB_RW = WB_DW * WB_N;
  localparam integer WB_MW = WB_DW / 8;

    assign rgb = {1'b1, 1'b1, ~cnt[25]};

  genvar i;


  // Signals
  // -------

  // Vex Misc
  wire [     31:0] vex_externalResetVector;
  wire             vex_timerInterrupt;
  wire             vex_softwareInterrupt;
  wire [     31:0] vex_externalInterruptArray;

  // Vex busses
  wire             i_axi_ar_valid;
  wire             i_axi_ar_ready;
  wire [     31:0] i_axi_ar_payload_addr;
  wire [      7:0] i_axi_ar_payload_len;
  wire [      1:0] i_axi_ar_payload_burst;
  wire [      3:0] i_axi_ar_payload_cache;
  wire [      2:0] i_axi_ar_payload_prot;
  wire             i_axi_r_valid;
  wire             i_axi_r_ready;
  wire [     31:0] i_axi_r_payload_data;
  wire [      1:0] i_axi_r_payload_resp;
  wire             i_axi_r_payload_last;

  wire             d_wb_cyc;
  wire             d_wb_stb;
  wire             d_wb_ack;
  wire             d_wb_we;
  wire [     29:0] d_wb_adr;
  wire [     31:0] d_wb_dat_miso;
  wire [     31:0] d_wb_dat_mosi;
  wire [      3:0] d_wb_sel;
  wire             d_wb_err;
  wire [      1:0] d_wb_bte;
  wire [      2:0] d_wb_cti;

  // RAM
  wire [     27:0] ram_addr;
  wire [     31:0] ram_rdata;
  wire [     31:0] ram_wdata;
  wire [      3:0] ram_wmsk;
  wire             ram_we;

  // Cache Request / Response interface
  wire [     27:0] cache_req_addr_pre;
  wire             cache_req_valid;
  wire             cache_req_write;
  wire [     31:0] cache_req_wdata;
  wire [      3:0] cache_req_wmsk;

  wire             cache_resp_ack;
  wire             cache_resp_nak;
  wire [     31:0] cache_resp_rdata;

  // Memory interface
  wire [     23:0] mi_addr;
  wire [      6:0] mi_len;
  wire             mi_rw;
  wire             mi_linear;
  wire             mi_valid;
  wire             mi_ready;

  wire [     31:0] mi_wdata;
  wire [      3:0] mi_wmsk;
  wire             mi_wack;
  wire             mi_wlast;

  wire [     31:0] mi_rdata;
  wire             mi_rstb;
  wire             mi_rlast;

  // HyperRAM PHY
  wire [      1:0] phy_ck_en;

  wire [      3:0] phy_rwds_in;
  wire [      3:0] phy_rwds_out;
  wire [      1:0] phy_rwds_oe;

  wire [     31:0] phy_dq_in;
  wire [     31:0] phy_dq_out;
  wire [      1:0] phy_dq_oe;

  wire [      3:0] phy_cs_n;
  wire             phy_rst_n;

  wire [      7:0] phy_cfg_wdata;
  wire [      7:0] phy_cfg_rdata;
  wire             phy_cfg_stb;

  // Wishbone
  wire [WB_AW-1:0] wb_addr;
  wire [WB_DW-1:0] wb_rdata                   [0:WB_N-1];
  wire [WB_RW-1:0] wb_rdata_flat;
  wire [WB_DW-1:0] wb_wdata;
  wire [WB_MW-1:0] wb_wmsk;
  wire             wb_we;
  wire [WB_N -1:0] wb_cyc;
  wire [WB_N -1:0] wb_ack;

  // Clock / Reset logic
  wire [      3:0] clk_rd_delay;
  wire             clk_1x;
  wire             clk_4x;
  wire             clk_rd;
  wire             sync_4x;
  wire             sync_rd;
  wire             rst;


  // SoC
  // ---

  // CPU
  VexRiscv cpu_I (
      .externalResetVector     (vex_externalResetVector),
      .timerInterrupt          (vex_timerInterrupt),
      .softwareInterrupt       (vex_softwareInterrupt),
      .externalInterruptArray  (vex_externalInterruptArray),
      .iBusAXI_ar_valid        (i_axi_ar_valid),
      .iBusAXI_ar_ready        (i_axi_ar_ready),
      .iBusAXI_ar_payload_addr (i_axi_ar_payload_addr),
      .iBusAXI_ar_payload_len  (i_axi_ar_payload_len),
      .iBusAXI_ar_payload_burst(i_axi_ar_payload_burst),
      .iBusAXI_ar_payload_cache(i_axi_ar_payload_cache),
      .iBusAXI_ar_payload_prot (i_axi_ar_payload_prot),
      .iBusAXI_r_valid         (i_axi_r_valid),
      .iBusAXI_r_ready         (i_axi_r_ready),
      .iBusAXI_r_payload_data  (i_axi_r_payload_data),
      .iBusAXI_r_payload_resp  (i_axi_r_payload_resp),
      .iBusAXI_r_payload_last  (i_axi_r_payload_last),
      .dBusWishbone_CYC        (d_wb_cyc),
      .dBusWishbone_STB        (d_wb_stb),
      .dBusWishbone_ACK        (d_wb_ack),
      .dBusWishbone_WE         (d_wb_we),
      .dBusWishbone_ADR        (d_wb_adr),
      .dBusWishbone_DAT_MISO   (d_wb_dat_miso),
      .dBusWishbone_DAT_MOSI   (d_wb_dat_mosi),
      .dBusWishbone_SEL        (d_wb_sel),
      .dBusWishbone_ERR        (d_wb_err),
      .dBusWishbone_BTE        (d_wb_bte),
      .dBusWishbone_CTI        (d_wb_cti),
      .clk                     (clk_1x),
      .reset                   (rst)
  );

  // CPU interrupt wiring
  assign vex_externalResetVector    = 32'h00000000;
  assign vex_softwareInterrupt      = 1'b0;
  assign vex_externalInterruptArray = 32'h00000000;

  // Cache bus interface / bridge

  mc_bus_vex #(
      .WB_N(WB_N)
  ) cache_bus_I (
      .i_axi_ar_valid        (i_axi_ar_valid),
      .i_axi_ar_ready        (i_axi_ar_ready),
      .i_axi_ar_payload_addr (i_axi_ar_payload_addr),
      .i_axi_ar_payload_len  (i_axi_ar_payload_len),
      .i_axi_ar_payload_burst(i_axi_ar_payload_burst),
      .i_axi_ar_payload_cache(i_axi_ar_payload_cache),
      .i_axi_ar_payload_prot (i_axi_ar_payload_prot),
      .i_axi_r_valid         (i_axi_r_valid),
      .i_axi_r_ready         (i_axi_r_ready),
      .i_axi_r_payload_data  (i_axi_r_payload_data),
      .i_axi_r_payload_resp  (i_axi_r_payload_resp),
      .i_axi_r_payload_last  (i_axi_r_payload_last),
      .d_wb_cyc              (d_wb_cyc),
      .d_wb_stb              (d_wb_stb),
      .d_wb_ack              (d_wb_ack),
      .d_wb_we               (d_wb_we),
      .d_wb_adr              (d_wb_adr),
      .d_wb_dat_miso         (d_wb_dat_miso),
      .d_wb_dat_mosi         (d_wb_dat_mosi),
      .d_wb_sel              (d_wb_sel),
      .d_wb_err              (d_wb_err),
      .d_wb_bte              (d_wb_bte),
      .d_wb_cti              (d_wb_cti),
      .wb_addr               (wb_addr),
      .wb_wdata              (wb_wdata),
      .wb_wmsk               (wb_wmsk),
      .wb_rdata              (wb_rdata_flat),
      .wb_cyc                (wb_cyc),
      .wb_we                 (wb_we),
      .wb_ack                (wb_ack),
      .ram_addr              (ram_addr),
      .ram_wdata             (ram_wdata),
      .ram_wmsk              (ram_wmsk),
      .ram_rdata             (ram_rdata),
      .ram_we                (ram_we),
      .req_addr_pre          (cache_req_addr_pre),
      .req_valid             (cache_req_valid),
      .req_write             (cache_req_write),
      .req_wdata             (cache_req_wdata),
      .req_wmsk              (cache_req_wmsk),
      .resp_ack              (cache_resp_ack),
      .resp_nak              (cache_resp_nak),
      .resp_rdata            (cache_resp_rdata),
      .clk                   (clk_1x),
      .rst                   (rst)
  );

  for (i = 0; i < WB_N; i = i + 1) assign wb_rdata_flat[i*WB_DW+:WB_DW] = wb_rdata[i];

  // Boot memory
  soc_bram #(
      .AW(10),
`ifdef SIM
      .INIT_FILE("boot-sim.hex")
`else
      .INIT_FILE("boot.hex")
`endif
  ) bram_I (
      .addr (ram_addr[9:0]),
      .rdata(ram_rdata),
      .wdata(ram_wdata),
      .wmsk (ram_wmsk),
      .we   (ram_we),
      .clk  (clk_1x)
  );

  // Cache

  mc_core #(
      .N_WAYS(4),
      .ADDR_WIDTH(24),
      .CACHE_LINE(64),
      .CACHE_SIZE(128)
  ) cache_I (
      .req_addr_pre(cache_req_addr_pre[23:0]),
      .req_valid   (cache_req_valid),
      .req_write   (cache_req_write),
      .req_wdata   (cache_req_wdata),
      .req_wmsk    (cache_req_wmsk),
      .resp_ack    (cache_resp_ack),
      .resp_nak    (cache_resp_nak),
      .resp_rdata  (cache_resp_rdata),
      .mi_addr     (mi_addr),
      .mi_len      (mi_len),
      .mi_rw       (mi_rw),
      .mi_valid    (mi_valid),
      .mi_ready    (mi_ready),
      .mi_wdata    (mi_wdata),
      .mi_wack     (mi_wack),
      .mi_wlast    (mi_wlast),
      .mi_rdata    (mi_rdata),
      .mi_rstb     (mi_rstb),
      .mi_rlast    (mi_rlast),
      .clk         (clk_1x),
      .rst         (rst)
  );

  wire [3:0] cs_qqspi;
  assign cs = ~cs_qqspi;
  axi_burst_qqspi_slave #(
      .AW(23)
  ) qqspi_i (
      .mi_addr     (mi_addr[22:0]),
      .mi_len      (mi_len),
      .mi_rw       (mi_rw),
      .mi_valid    (mi_valid),
      .mi_ready    (mi_ready),
      .mi_wdata    (mi_wdata),
      .mi_wack     (mi_wack),
      .mi_wlast    (mi_wlast),
      .mi_rdata    (mi_rdata),
      .mi_rstb     (mi_rstb),
      .mi_rlast    (mi_rlast),
      .clk         (clk_1x),
      .rst         (rst),
      .cen         (),
      .sclk        (sclk),
      .sio1_so_miso(sio1_so_miso),
      .sio0_si_mosi(sio0_si_mosi),
      .sio2        (sio2),
      .sio3        (sio3),
      .cs          (cs_qqspi)
  );

  assign wb_ack[0]   = wb_cyc[0];
  assign wb_rdata[0] = 32'h00000000;

  // UART [1]
  // ----

  uart_wb #(
      .DIV_WIDTH(12),
      .DW(WB_DW)
  ) uart_I1 (
      .uart_tx (uart_tx),
      .uart_rx (uart_rx),
      .wb_addr (wb_addr[1:0]),
      .wb_rdata(wb_rdata[1]),
      .wb_we   (wb_we),
      .wb_wdata(wb_wdata),
      .wb_cyc  (wb_cyc[1]),
      .wb_ack  (wb_ack[1]),
      .clk     (clk_1x),
      .rst     (rst)
  );

  /*
uart_wb #(
      .DIV_WIDTH(12),
      .DW(WB_DW)
  ) uart_I0 (
      .uart_tx (uart_tx1),
      .uart_rx (uart_rx1),
      .wb_addr (wb_addr[1:0]),
      .wb_rdata(wb_rdata[3]),
      .wb_we   (wb_we),
      .wb_wdata(wb_wdata),
      .wb_cyc  (wb_cyc[3]),
      .wb_ack  (wb_ack[3]),
      .clk     (clk_1x),
      .rst     (rst)
  );
  */


  assign wb_rdata[3] = 0;
  assign wb_ack[3]   = wb_cyc[3];

  // Platform [2]  (SPI + Timer)
  // --------

  platform platform_I (
      .spi_pad_mosi(spi_mosi),
      .spi_pad_miso(spi_miso),
      .spi_pad_clk (spi_clk),
      .spi_pad_cs_n(spi_flash_cs_n),
      .irqo_timer  (vex_timerInterrupt),
      .wb_addr     (wb_addr[1:0]),
      .wb_rdata    (wb_rdata[2]),
      .wb_wdata    (wb_wdata),
      .wb_we       (wb_we),
      .wb_cyc      (wb_cyc[2]),
      .wb_ack      (wb_ack[2]),
      .clk         (clk_1x),
      .rst         (rst)
  );

  // Clock / Reset
  // -------------

  /* PLL CRG */
  /*
  sysmgr sys_mgr_I (
      .delay  (),
      .clk_in (clk_in),
      .clk_1x (clk_1x),
      .clk_4x (),
      .clk_rd (),
      .sync_4x(),
      .sync_rd(),
      .rst    (rst)
  );
  */
/*  */

  SB_HFOSC clk24mhz_i (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk_1x));
  defparam clk24mhz_i.CLKHF_DIV = "0b01";

  reg [25:0] cnt;
  always @(posedge clk_1x) begin
      cnt <= cnt + 1;
  end
  /*
  ice40up5k_pll #(
      .freq(24)
  ) pll_i (
      clk_in,
      clk_1x
  );
  */
//  assign clk_1x = clk_in;
  wire clk = clk_1x;

  reg [8:0] rst_cnt = 0;
  always @(posedge clk_1x) begin
    if (!rst_cnt[8]) rst_cnt <= rst_cnt + 1;
  end

  assign rst = !rst_cnt[8];


endmodule  // top
