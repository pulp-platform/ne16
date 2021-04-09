/*
 * ne16_accumulator_scm.sv
 *
 * Copyright (C) 2019-2021 ETH Zurich, University of Bologna and GreenWaves Technologies
 *
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the "License"); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */

/*
 * Authors (RBE):  Gianna Paulin <pauling@iis.ee.ethz.ch>
 *                 Francesco Conti <f.conti@unibo.it>
 * Authors (NE16): Francesco Conti <francesco.conti@greenwaves-technologies.com>
 */

module ne16_accumulator_scm
#(
  parameter int unsigned ADDR_WIDTH   = 5,
  parameter int unsigned DATA_WIDTH   = 32,
  parameter int unsigned NUM_WORDS    = 2**ADDR_WIDTH,
  parameter int unsigned WIDTH_FACTOR = 4
)
(
  input  logic                               clk_i,
  input  logic                               rst_ni,
  input  logic                               clear_i,
  input  logic                               test_mode_i,
  input  logic [WIDTH_FACTOR-1:0]            wide_enable_i,

  // Read port
  input  logic                               re_i,
  input  logic [ADDR_WIDTH-1:0]              raddr_i,
  output logic [DATA_WIDTH-1:0]              rdata_o,
  output logic [WIDTH_FACTOR*DATA_WIDTH-1:0] rdata_wide_o,

  // Write port
  input  logic                               we_i,
  input  logic                               we_all_i,
  input  logic [ADDR_WIDTH-1:0]              waddr_i,
  input  logic [DATA_WIDTH-1:0]              wdata_i,
  input  logic [WIDTH_FACTOR*DATA_WIDTH-1:0] wdata_wide_i,

  output logic [NUM_WORDS-1:0][DATA_WIDTH-1:0] accumulators_o
);

  // Read address register, located at the input of the address decoder
  logic [NUM_WORDS-1:0][DATA_WIDTH-1:0] accumulators;
  logic [NUM_WORDS-1:0]  waddr_onehot;
  logic [NUM_WORDS-1:0]  clk_we;

  logic [WIDTH_FACTOR*DATA_WIDTH-1:0]      rdata_q;
  logic [WIDTH_FACTOR-1:0][DATA_WIDTH-1:0] wdata_q;

  logic clk_gated;

  // ========================================================================
  // CLK GATE
  // ========================================================================
  cluster_clock_gating i_cg_we_global
  (
    .clk_o     ( clk_gated      ),
    .en_i      ( we_i | clear_i ),
    .test_en_i ( test_mode_i    ),
    .clk_i     ( clk_i          )
  );

  // ========================================================================
  // WDATA SAMPLING
  // ========================================================================

  logic [WIDTH_FACTOR-1:0][DATA_WIDTH-1:0] wdata_d;
  generate

    for(genvar ii=0; ii<WIDTH_FACTOR; ii++) begin

      localparam ii_rem2 = ii % 2;
      localparam ii_rem4 = ii % 4;
      localparam ii_rem8 = ii % 8;

      assign wdata_d[ii] = (wide_enable_i == 8'h1) || (wide_enable_i == 8'h2) || (wide_enable_i == 8'h4) || (wide_enable_i == 8'h8) || (wide_enable_i == 8'h10) || (wide_enable_i == 8'h20) || (wide_enable_i == 8'h40) || (wide_enable_i == 8'h80) ? wdata_wide_i[DATA_WIDTH-1:0] :
                           (wide_enable_i == 8'h3) || (wide_enable_i == 8'hc) || (wide_enable_i == 8'h30) || (wide_enable_i == 8'hc0) ? wdata_wide_i[(ii_rem2+1)*DATA_WIDTH-1:ii_rem2*DATA_WIDTH] :
                           (wide_enable_i == 8'hf) || (wide_enable_i == 8'hf0) ? wdata_wide_i[(ii_rem4+1)*DATA_WIDTH-1:ii_rem4*DATA_WIDTH] :
                           (wide_enable_i == 8'hff) ? wdata_wide_i[(ii_rem8+1)*DATA_WIDTH-1:ii_rem8*DATA_WIDTH] : wdata_i;

      always_ff @(posedge clk_i or negedge rst_ni)
      begin
        if(~rst_ni)
          wdata_q[ii] <= '0;
        else if(clear_i)
          wdata_q[ii] <= '0;
        else if(we_i | we_all_i)
          wdata_q[ii] <= wdata_d[ii];
      end
    end
  endgenerate

  // ========================================================================
  // SCM (LATCHES)
  // ========================================================================

  // use the sampled address to select the correct rdata_o
  generate
    always_ff @(posedge clk_i or negedge rst_ni)
    begin
      if(~rst_ni)
        rdata_q[DATA_WIDTH-1:0] <= '0;
      else if(clear_i)
        rdata_q[DATA_WIDTH-1:0] <= '0;
      else if(re_i) begin
        rdata_q[DATA_WIDTH-1:0] <= accumulators[raddr_i];
      end
    end
    for(genvar ii=1; ii<WIDTH_FACTOR; ii++) begin

      logic [ADDR_WIDTH-1:0] raddr_wide;
      assign raddr_wide = raddr_i + ii;

      always_ff @(posedge clk_i or negedge rst_ni)
      begin
        if(~rst_ni)
          rdata_q[(ii+1)*DATA_WIDTH-1:ii*DATA_WIDTH] <= '0;
        else if(clear_i)
          rdata_q[(ii+1)*DATA_WIDTH-1:ii*DATA_WIDTH] <= '0;
        else if(re_i & (|wide_enable_i)) begin
          rdata_q[(ii+1)*DATA_WIDTH-1:ii*DATA_WIDTH] <= accumulators[raddr_wide];
        end
      end
    end
  endgenerate

  assign rdata_o      = rdata_q[DATA_WIDTH-1:0];
  assign rdata_wide_o = rdata_q;

  // decode

  // three modes:
  //  - broadcast: all waddr_onehot are enabled
  //  - normal:    the waddr_onehot is the decoded version of waddr_i
  //  - wide:      decoded on wide words, masked by wide_enable

  logic [NUM_WORDS-1:0] waddr_decoded_normal;
  logic [NUM_WORDS-1:0] waddr_decoded_wide;

  generate
    for(genvar ii=0; ii<NUM_WORDS/WIDTH_FACTOR; ii++) begin : WADDR_DECODE_ITER_1

      logic [ADDR_WIDTH-1:0] idx_ii;
      assign idx_ii = ii*WIDTH_FACTOR;

      for(genvar jj=0; jj<WIDTH_FACTOR; jj++) begin : WADDR_DECODE_ITER_0

        logic [ADDR_WIDTH-1:0] idx_ii_jj;
        assign idx_ii_jj = ii*WIDTH_FACTOR+jj;

        always_comb
        begin : waddr_decoding
          if((we_i==1'b1) && (waddr_i == idx_ii_jj))
            waddr_decoded_normal[ii*WIDTH_FACTOR+jj] = 1'b1;
          else
            waddr_decoded_normal[ii*WIDTH_FACTOR+jj] = 1'b0;
        end

      end

      assign waddr_decoded_wide[(ii+1)*WIDTH_FACTOR-1:ii*WIDTH_FACTOR] = (we_i==1'b1) && (waddr_i == idx_ii) ? wide_enable_i : '0;

    end
  endgenerate

  assign waddr_onehot = clear_i | we_all_i ? '1 :
                        (|(wide_enable_i)) ? waddr_decoded_wide :
                                             waddr_decoded_normal;

  // generate one clock-gating cell for each register element
  generate
    for(genvar ii=0; ii<NUM_WORDS; ii++) begin : CG_CELL_WORD_ITER

      cluster_clock_gating i_cg
      (
        .clk_o     ( clk_we[ii]       ),
        .en_i      ( waddr_onehot[ii] ),
        .test_en_i ( test_mode_i      ),
        .clk_i     ( clk_i            )
      );

    end
  endgenerate

  generate

    for(genvar ii=0; ii<NUM_WORDS/WIDTH_FACTOR; ii++) begin : LATCH_ITER_1
      for(genvar jj=0; jj<WIDTH_FACTOR; jj++) begin : LATCH_ITER_0

        always_latch
        begin : latch_wdata
          if( clk_we[ii*WIDTH_FACTOR+jj] ) begin
            accumulators[ii*WIDTH_FACTOR+jj] = clear_i ? '0 : wdata_q[jj];
          end
        end

      end
    end

  endgenerate

  assign accumulators_o = accumulators;

endmodule // ne16_accumulator_scm
