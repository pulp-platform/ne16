/*
 * ne16_binconv_array.sv
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

import ne16_package::*;

module ne16_binconv_array #(
  parameter int unsigned COLUMN_SIZE    = NE16_COLUMN_SIZE, // number of BinConv blocks per column (default 9)
  parameter int unsigned NR_COLUMN      = NE16_COLUMN_SIZE, // number of BinConv columns (default 9 -- same of size of BinConv columns!)
  parameter int unsigned BLOCK_SIZE     = NE16_BLOCK_SIZE,  // number of SoP's per BinConv block (default 4)
  parameter int unsigned NR_ACTIVATIONS = 400,              // 25 * BLOCK_SIZE, FIXME
  parameter int unsigned TP_IN             = NE16_TP_IN     // number of input elements processed per cycle
) (
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // local enable & clear
  input  logic                   enable_i,
  input  logic                   clear_i,
  // input activation stream + handshake
  hwpe_stream_intf_stream.sink   activation_i  [NR_ACTIVATIONS-1:0],
  // input weight stream + handshake
  hwpe_stream_intf_stream.sink   weight_conv_i   [COLUMN_SIZE-1:0],
  hwpe_stream_intf_stream.sink   weight_linear_i [31:0],
  // output features + handshake
  hwpe_stream_intf_stream.source pres_o        [NR_COLUMN-1:0],
  // control channel
  input  ctrl_binconv_array_t    ctrl_i,
  output flags_binconv_array_t   flags_o
);


  ///////////////////////////////////////////
  // Local Params, Interfaces, and Signals //
  ///////////////////////////////////////////

  localparam BLOCK_PRES_SIZE  = NE16_QA_IN+NE16_QA_16BIT+8+$clog2(BLOCK_SIZE);
  localparam COLUMN_PRES_SIZE = BLOCK_PRES_SIZE+$clog2(COLUMN_SIZE);

  logic [NR_ACTIVATIONS-1:0][NE16_QA_IN-1  :0] activation_data;
  logic [NR_ACTIVATIONS-1:0]                   activation_valid;
  logic [NR_ACTIVATIONS-1:0][NE16_QA_IN/8-1:0] activation_strb;

  logic [NR_COLUMN-1:0][COLUMN_SIZE-1:0][BLOCK_SIZE-1:0][NE16_QA_IN-1  :0] activation_mapped_fs1_data;
  logic [NR_COLUMN-1:0][COLUMN_SIZE-1:0][BLOCK_SIZE-1:0]                   activation_mapped_fs1_valid;
  logic [NR_COLUMN-1:0][COLUMN_SIZE-1:0][BLOCK_SIZE-1:0][NE16_QA_IN/8-1:0] activation_mapped_fs1_strb;

  logic [NR_COLUMN-1:0][COLUMN_SIZE-1:0][BLOCK_SIZE-1:0][NE16_QA_IN-1  :0] activation_mapped_fs3_data;
  logic [NR_COLUMN-1:0][COLUMN_SIZE-1:0][BLOCK_SIZE-1:0]                   activation_mapped_fs3_valid;
  logic [NR_COLUMN-1:0][COLUMN_SIZE-1:0][BLOCK_SIZE-1:0][NE16_QA_IN/8-1:0] activation_mapped_fs3_strb;

  logic [NR_COLUMN-1:0][COLUMN_SIZE-1:0][BLOCK_SIZE-1:0][NE16_QA_IN-1  :0] activation_mapped_linear_data;
  logic [NR_COLUMN-1:0][COLUMN_SIZE-1:0][BLOCK_SIZE-1:0]                   activation_mapped_linear_valid;
  logic [NR_COLUMN-1:0][COLUMN_SIZE-1:0][BLOCK_SIZE-1:0][NE16_QA_IN/8-1:0] activation_mapped_linear_strb;

  // block-level counter, moved here to be shared!
  logic block_cnt_en, block_clear;
  logic [$clog2(NE16_QA_IN):0] block_cnt_q, block_cnt_d;

  // depthwise counter
  logic depthwise_cnt_en;
  logic [$clog2(NE16_TP_IN):0] depthwise_cnt_q, depthwise_cnt_d;
  logic [NE16_TP_IN:0] depthwise_cnt_oh_q, depthwise_cnt_oh_d;

  logic block_invalidate_d, block_invalidate_q;

  // interfaces
  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_QA_IN )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) activation_mapped [NR_COLUMN*COLUMN_SIZE*BLOCK_SIZE-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH       ( TP_IN )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) weight_int [NR_COLUMN*COLUMN_SIZE-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( COLUMN_PRES_SIZE )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) pres_int [NR_COLUMN-1:0] (
    .clk ( clk_i )
  );

  //////////////////////////////////////
  // Column, Row and Block generation //
  //////////////////////////////////////

  // weight ready assignment
  for (genvar jj=0; jj<COLUMN_SIZE; jj++) begin : weight_conv_ready_gen
    assign weight_conv_i[jj].ready = weight_int[jj].ready & ~ctrl_i.weight_offset;
  end
  for (genvar jj=0; jj<32; jj++) begin : weight_linear_ready_gen
    assign weight_linear_i[jj].ready = weight_int[jj].ready & ~ctrl_i.weight_offset;
  end

  generate
    // activation extraction from interface
    for(genvar jj=0; jj<NR_ACTIVATIONS; jj++) begin : activation_assignment_gen
      assign activation_data[jj]    = activation_i[jj].data;
      assign activation_valid[jj]   = activation_i[jj].valid;
      assign activation_strb[jj]    = activation_i[jj].strb;
      assign activation_i[jj].ready = activation_mapped[0].ready;
    end

    logic [80:0][15:0] weight_linear_data_mapped;

    // weight assignment
    for (genvar kk=0; kk<81; kk++) begin : weight_linear_data_mapped_gen
      if(NE16_LINEAR_MAP[kk] == -1) begin : weight_linear_data_mapped_null_gen
        assign weight_linear_data_mapped[kk] = '0;
      end
      else begin : weight_linear_data_mapped_non_null_gen
        assign weight_linear_data_mapped[kk] = weight_linear_i[NE16_LINEAR_MAP[kk]].data;
      end
    end
    
    for(genvar ii=0; ii<NR_COLUMN; ii++) begin : column_gen

      for (genvar jj=0; jj<COLUMN_SIZE; jj++) begin : row_w_gen
        localparam ii_jj = ii*COLUMN_SIZE+jj;
        assign weight_int[ii_jj].data  = ctrl_i.mode_linear ? weight_linear_data_mapped[ii_jj] : weight_conv_i[jj].data;
        assign weight_int[ii_jj].strb  = ctrl_i.mode_linear ? weight_linear_i[0].strb          : weight_conv_i[jj].strb;
        assign weight_int[ii_jj].valid = ctrl_i.mode_linear ? weight_linear_i[0].valid         : weight_conv_i[jj].valid;
      end // row_w_gen

      //  CONV MODES:
      //                                 width 5 pixels
      //               +----------+----------+----------+----------+----------+
      //  16 channels /          /          /          /          /          /|
      //             /          /          /          /          /          / |
      //            +----------+----------+----------+----------+----------+  |
      //            |          |          |          |          |          |  +
      //            |   0,0    |   0,1    |   0,2    |   0,3    |   0,4    | /|
      //            |          |          |          |          |          |/ |
      //            +----------+----------+----------+----------+----------+  |
      //            |          |          |          |          |          |  +
      //            |   1,0    |   1,1    |   1,2    |   1,3    |   1,4    | /|
      //            |          |          |          |          |          |/ |
      //            +----------+----------+----------+----------+----------+  |
      //  height    |          |          |          |          |          |  +
      // 5 pixels   |   2,0    |   2,1    |   2,2    |   2,3    |   2,4    | /|
      //            |          |          |          |          |          |/ |
      //            +----------+----------+----------+----------+----------+  |
      //            |          |          |          |          |          |  +
      //            |   3,0    |   3,1    |   3,2    |   3,3    |   3,4    | /|
      //            |          |          |          |          |          |/ |
      //            +----------+----------+----------+----------+----------+  |
      //            |          |          |          |          |          |  +
      //            |   4,0    |   4,1    |   4,2    |   4,3    |   4,4    | /
      //            |          |          |          |          |          |/
      //            +----------+----------+----------+----------+----------+

      // array activation mapping (3x3 mode)
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,0 | 0,1 | 0,2 | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,1 | 0,2 | 0,3 | 1,1 | 1,2 | 1,3 | 2,1 | 2,2 | 2,3 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,2 | 0,3 | 0,4 | 1,2 | 1,3 | 1,4 | 2,2 | 2,3 | 2,4 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 | 3,0 | 3,1 | 3,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 1,1 | 1,2 | 1,3 | 2,1 | 2,2 | 2,3 | 3,1 | 3,2 | 3,3 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 1,2 | 1,3 | 1,4 | 2,2 | 2,3 | 2,4 | 3,2 | 3,3 | 3,4 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 2,0 | 2,1 | 2,2 | 3,0 | 3,1 | 3,2 | 4,0 | 4,1 | 4,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 2,1 | 2,2 | 2,3 | 3,1 | 3,2 | 3,3 | 4,1 | 4,2 | 4,3 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 2,2 | 2,3 | 2,4 | 3,2 | 3,3 | 3,4 | 4,2 | 4,3 | 4,4 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+

      // array activation mapping (1x1 mode)
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,0 | 0,1 | 0,2 | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,0 | 0,1 | 0,2 | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,0 | 0,1 | 0,2 | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,0 | 0,1 | 0,2 | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,0 | 0,1 | 0,2 | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,0 | 0,1 | 0,2 | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,0 | 0,1 | 0,2 | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            | 0,0 | 0,1 | 0,2 | 1,0 | 1,1 | 1,2 | 2,0 | 2,1 | 2,2 |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  -  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+

      // array activation mapping (linear mode 8bit)
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  0  |  8  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  1  |  9  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  2  | 10  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  3  | 11  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  4  | 12  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  5  | 13  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  6  | 14  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  7  | 15  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  -  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+

      // array activation mapping (linear mode 16bit)
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  0  |  8  | 16  | 24  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  1  |  9  | 17  | 25  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  2  | 10  | 18  | 26  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  3  | 11  | 19  | 27  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  4  | 12  | 20  | 28  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  5  | 13  | 21  | 29  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  6  | 14  | 22  | 30  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  7  | 15  | 23  | 31  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+
      //            |  -  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |  -  |
      //            +-----+-----+-----+-----+-----+-----+-----+-----+-----+

      for(genvar rr=0; rr<COLUMN_SIZE; rr++) begin : row_a_gen

        localparam i=0;
        localparam j=0;

        // filter size = 1
        localparam j_fs1  = (ii % 3);
        localparam i_fs1  = ((ii-(ii%3)) / 3);

        // filter size = 3
        localparam fj_fs3 = rr % 3;
        localparam fi_fs3 = (rr-fj_fs3)/ 3;
        localparam j_fs3  = ii % 3 + fj_fs3;
        localparam i_fs3  = (ii-(ii%3)) / 3 + fi_fs3;

        for(genvar bb=0; bb<BLOCK_SIZE; bb++) begin : block_gen

          // FIXME parameterize 5 (also NR_ACTIVATIONS)
          assign activation_mapped_fs3_data [ii][rr][bb][NE16_QA_IN-1:0]   = activation_data [i_fs3*NE16_TP_IN*5 + j_fs3*NE16_TP_IN + bb][NE16_QA_IN-1:0];
          assign activation_mapped_fs3_valid[ii][rr][bb]                   = activation_valid[0];
          assign activation_mapped_fs3_strb [ii][rr][bb][NE16_QA_IN/8-1:0] = activation_strb [0][NE16_QA_IN/8-1:0];

          assign activation_mapped_fs1_data [ii][rr][bb][NE16_QA_IN-1:0]   = activation_data [i_fs1*NE16_TP_IN*5 + j_fs1*NE16_TP_IN + bb][NE16_QA_IN-1:0];
          assign activation_mapped_fs1_valid[ii][rr][bb]                   = activation_valid[0];
          assign activation_mapped_fs1_strb [ii][rr][bb][NE16_QA_IN/8-1:0] = activation_strb [0][NE16_QA_IN/8-1:0];

          if(ii<4 && rr<8) begin
            assign activation_mapped_linear_data [ii][rr][bb][NE16_QA_IN-1:0]   = activation_data [ii*8*NE16_TP_IN+rr*NE16_TP_IN+bb][NE16_QA_IN-1:0];
            assign activation_mapped_linear_valid[ii][rr][bb]                   = activation_valid[0];
            assign activation_mapped_linear_strb [ii][rr][bb][NE16_QA_IN/8-1:0] = activation_strb [0][NE16_QA_IN/8-1:0];
          end
          else begin
            assign activation_mapped_linear_data [ii][rr][bb][NE16_QA_IN-1:0]   = '0;
            assign activation_mapped_linear_valid[ii][rr][bb]                   = activation_valid[0];
            assign activation_mapped_linear_strb [ii][rr][bb][NE16_QA_IN/8-1:0] = activation_strb [0][NE16_QA_IN/8-1:0];
          end

          localparam ii_rr_bb = ii*(COLUMN_SIZE*BLOCK_SIZE) + rr*(BLOCK_SIZE) + bb;

          assign activation_mapped[ii_rr_bb].valid = ctrl_i.mode_linear ? activation_mapped_linear_valid[ii][rr][bb] : (ctrl_i.filter_mode == NE16_FILTER_MODE_1X1) ? activation_mapped_fs1_valid[ii][rr][bb] : activation_mapped_fs3_valid[ii][rr][bb];
          assign activation_mapped[ii_rr_bb].data  = ctrl_i.mode_linear ? activation_mapped_linear_data[ii][rr][bb]  : (ctrl_i.filter_mode == NE16_FILTER_MODE_1X1) ? activation_mapped_fs1_data[ii][rr][bb]  : activation_mapped_fs3_data[ii][rr][bb];
          assign activation_mapped[ii_rr_bb].strb  = ctrl_i.mode_linear ? activation_mapped_linear_strb[ii][rr][bb]  : (ctrl_i.filter_mode == NE16_FILTER_MODE_1X1) ? activation_mapped_fs1_strb[ii][rr][bb]  : activation_mapped_fs3_strb[ii][rr][bb];

        end // block_gen
      end // row_a_gen

      ctrl_binconv_column_t ctrl_column;
      always_comb
      begin
        ctrl_column = ctrl_i.ctrl_column;
        ctrl_column.enable_block = ctrl_i.mode_linear ? ctrl_i.ctrl_column.enable_block_linear[ii] : ctrl_i.ctrl_column.enable_block;
        ctrl_column.ctrl_block.block_cnt = block_cnt_q;
        // in depthwise mode, MACs are enabled sequentially in the channel in dimension
        ctrl_column.ctrl_block.enable_mac = ctrl_i.ctrl_column.ctrl_block.filter_mode == NE16_FILTER_MODE_3X3_DW ? ctrl_i.ctrl_column.ctrl_block.enable_mac & depthwise_cnt_oh_q :
                                                                                                                   ctrl_i.ctrl_column.ctrl_block.enable_mac;
        // in depthwise mode, partial sum valid signals have to be invalidated in some cases (check)
        ctrl_column.ctrl_block.invalidate = ctrl_i.ctrl_column.ctrl_block.filter_mode == NE16_FILTER_MODE_3X3_DW & ctrl_i.ctrl_column.ctrl_block.weight_offset ? block_invalidate_q : 1'b0;
      end

      // column instantiation
      logic clk_gated;
      cluster_clock_gating i_hier_column_gate (
        .clk_i     ( clk_i                                         ),
        .en_i      ( enable_i & ctrl_i.enable_column[ii] | clear_i ),
        .test_en_i ( test_mode_i                                   ),
        .clk_o     ( clk_gated                                     )
      );

      ne16_binconv_column #(
        .COLUMN_SIZE ( COLUMN_SIZE ),
        .BLOCK_SIZE  ( BLOCK_SIZE ),
        .TP_IN       ( TP_IN         )
      ) i_column (
        .clk_i         ( clk_gated                                                                     ),
        .rst_ni        ( rst_ni                                                                        ),
        .test_mode_i   ( test_mode_i                                                                   ),
        .enable_i      ( enable_i & ctrl_i.enable_column[ii]                                           ),
        .clear_i       ( clear_i                                                                       ),
        .activation_i  ( activation_mapped [(ii+1)*COLUMN_SIZE*BLOCK_SIZE-1:ii*COLUMN_SIZE*BLOCK_SIZE] ),
        .weight_i      ( weight_int [(ii+1)*COLUMN_SIZE-1:ii*COLUMN_SIZE]                              ),
        .column_pres_o ( pres_int [ii]                                                                 ),
        .ctrl_i        ( ctrl_column                                                                   ),
        .flags_o       ( flags_o.flags_column[ii]                                                      )
      );

      if(ii==0) begin : pres_0_gen
        // in linear mode, the first 4 columns are accumulated together, the others are ignored; in other modes, each column is accumulated separately
        assign pres_o[0].data = ctrl_i.mode_linear ? pres_int[0].data + pres_int[1].data + pres_int[2].data + pres_int[3].data :
                                                     pres_int[0].data;
        assign pres_o[0].valid = pres_int[0].valid;
        assign pres_o[0].strb  = pres_int[0].strb;
        assign pres_int[0].ready = pres_o[0].ready;
      end
      else begin : pres_non0_gen
        assign pres_o[ii].data = ctrl_i.mode_linear ? '0 : pres_int[ii].data;
        assign pres_o[ii].valid = pres_int[0].valid;
        assign pres_o[ii].strb  = pres_int[0].strb;
        assign pres_int[ii].ready = pres_o[0].ready;
      end

    end // column_gen
  endgenerate

  // counter used for scale control
  assign block_clear = clear_i | ctrl_i.ctrl_column.ctrl_block.clear;
  assign block_cnt_en = (ctrl_i.ctrl_column.ctrl_block.weight_offset==1'b0)                                       ? activation_mapped[0].valid & activation_mapped[0].ready & weight_int[0].valid & weight_int[0].ready :
                        (block_cnt_q=='0 || ctrl_i.ctrl_column.ctrl_block.filter_mode == NE16_FILTER_MODE_3X3_DW) ? activation_mapped[0].valid & (activation_mapped[0].ready | weight_int[0].ready) :
                                                                                                                    '0;

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni)
      block_cnt_q <= '0;
    else if(block_clear)
      block_cnt_q <= 0;
    else if(block_cnt_en)
      block_cnt_q <= block_cnt_d;
  end

  always_comb
  begin
    block_cnt_d = block_cnt_q;
    if (block_cnt_q == (ctrl_i.ctrl_column.ctrl_block.qw-1))
      block_cnt_d = '0;
    else
      block_cnt_d = block_cnt_q + 1;
  end

  // counter used for enable control
  assign depthwise_cnt_en = ctrl_i.ctrl_column.ctrl_block.filter_mode != NE16_FILTER_MODE_3X3_DW ? '0 :
                            ctrl_i.ctrl_column.ctrl_block.weight_offset                          ? block_cnt_en :
                                                                                                   block_cnt_en & (block_cnt_q == ctrl_i.ctrl_column.ctrl_block.qw-1);

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni)
      depthwise_cnt_q <= '0;
    else if(block_clear)
      depthwise_cnt_q <= '0;
    else if(depthwise_cnt_en)
      depthwise_cnt_q <= depthwise_cnt_d;
  end

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni)
      depthwise_cnt_oh_q <= '0;
    else if(block_clear)
      depthwise_cnt_oh_q <= 1;
    else if(depthwise_cnt_en)
      depthwise_cnt_oh_q <= depthwise_cnt_oh_d;
  end

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni)
      block_invalidate_q <= '0;
    else if(block_clear)
      block_invalidate_q <= '0;
    else if(block_cnt_en)
      block_invalidate_q <= block_invalidate_d;
  end

  always_comb
  begin
    block_invalidate_d = block_invalidate_q;
    depthwise_cnt_d = depthwise_cnt_q;
    if (depthwise_cnt_q == (ctrl_i.depthwise_len-1)) begin
      depthwise_cnt_d = '0;
      block_invalidate_d = '1;
    end
    else if(depthwise_cnt_en & ~block_invalidate_q) begin
      depthwise_cnt_d = depthwise_cnt_q + 1;
    end
  end

  always_comb
  begin
    depthwise_cnt_oh_d = depthwise_cnt_oh_q;
    if (depthwise_cnt_q == (ctrl_i.depthwise_len-1)) begin
      depthwise_cnt_oh_d = 1;
    end
    else if(depthwise_cnt_en & ~block_invalidate_q) begin
      depthwise_cnt_oh_d = '0;
      depthwise_cnt_oh_d[depthwise_cnt_q + 1] = 1'b1;
    end
  end

endmodule // ne16_binconv_array
