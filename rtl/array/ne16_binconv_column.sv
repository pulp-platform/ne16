/*
 * ne16_binconv_column.sv
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

module ne16_binconv_column #(
  parameter int unsigned COLUMN_SIZE      = NE16_COLUMN_SIZE,         // number of BinConv blocks per column (default 9)
  parameter int unsigned BLOCK_SIZE       = NE16_BLOCK_SIZE,          // number of SoP's per BinConv block (default 4)
  parameter int unsigned BC_COLBLOCK_SIZE = COLUMN_SIZE*BLOCK_SIZE,
  parameter int unsigned TP_IN            = NE16_TP_IN                   // number of input elements processed per cycle
) (
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // local enable & clear
  input  logic                   enable_i,
  input  logic                   clear_i,
  // input activation stream + handshake
  hwpe_stream_intf_stream.sink   activation_i  [BC_COLBLOCK_SIZE-1:0],
  // input weight stream + handshake
  hwpe_stream_intf_stream.sink   weight_i      [COLUMN_SIZE-1:0],
  // output features + handshake
  hwpe_stream_intf_stream.source column_pres_o,
  // control channel
  input  ctrl_binconv_column_t   ctrl_i,
  output flags_binconv_column_t  flags_o
);

  ///////////////////////////////////////////
  // Local Params, Interfaces, and Signals //
  ///////////////////////////////////////////

  localparam BLOCK_PRES_SIZE  = NE16_QA_IN+NE16_QA_16BIT+8+$clog2(BLOCK_SIZE);
  localparam COLUMN_PRES_SIZE = BLOCK_PRES_SIZE+$clog2(COLUMN_SIZE);

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( BLOCK_PRES_SIZE )
  `ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
  `endif
  ) block_pres [COLUMN_SIZE-1:0] (
    .clk ( clk_i )
  );

  logic signed [COLUMN_PRES_SIZE-1:0]   binconv_column_pres_d, binconv_column_pres_q;
  logic                                 binconv_column_pres_valid_d, binconv_column_pres_valid_q;
  logic        [COLUMN_PRES_SIZE/8-1:0] binconv_column_pres_strb_d, binconv_column_pres_strb_q;

  logic signed [COLUMN_SIZE-1:0][BLOCK_PRES_SIZE-1:0] block_pres_data;

  ///////////////////
  // Block Modules //
  ///////////////////
  generate
    for(genvar ii=0; ii<COLUMN_SIZE; ii++) begin : block_gen

      ctrl_binconv_block_t ctrl_block;

      always_comb
      begin
        ctrl_block = ctrl_i.ctrl_block;
        ctrl_block.scale_shift = ii; // used for 1x1
      end

      ne16_binconv_block #(
        .BLOCK_SIZE ( BLOCK_SIZE ),
        .TP_IN      ( TP_IN      )
      ) i_block (
        .clk_i        ( clk_i                                            ),
        .rst_ni       ( rst_ni                                           ),
        .test_mode_i  ( test_mode_i                                      ),
        .enable_i     ( ctrl_i.enable_block[ii]                          ),
        .clear_i      ( clear_i                                          ),
        .activation_i ( activation_i [(ii+1)*BLOCK_SIZE-1:ii*BLOCK_SIZE] ),
        .weight_i     ( weight_i [ii]                                    ),
        .block_pres_o ( block_pres [ii]                                  ),
        .ctrl_i       ( ctrl_block                                       ),
        .flags_o      ( flags_o.flags_block[ii]                          )
      );

      assign block_pres_data[ii] = ctrl_i.enable_block[ii] ? block_pres[ii].data : '0;

    end // block_gen
  endgenerate


  ///////////////////////////////////
  // Computation of Column Results //
  ///////////////////////////////////

  always_comb
  begin
    binconv_column_pres_d = '0;
    for(int i=0; i<COLUMN_SIZE; i++) begin
      binconv_column_pres_d += BLOCK_PRES_SIZE'(signed'(block_pres_data[i]));
    end
  end
  assign binconv_column_pres_valid_d = block_pres[0].valid;
  assign binconv_column_pres_strb_d  = block_pres[0].strb;


  ////////////////////////
  // Output Assignments //
  ////////////////////////

  assign column_pres_o.valid = binconv_column_pres_valid_q;
  assign column_pres_o.strb  = binconv_column_pres_strb_q;
  assign column_pres_o.data  = enable_i ? binconv_column_pres_q : ctrl_i.padding_value[COLUMN_PRES_SIZE-1:0];

  generate
    for(genvar ii=0; ii<COLUMN_SIZE; ii++) begin : ready_prop_gen
      assign block_pres[ii].ready = column_pres_o.ready;
    end // ready_prop_gen
  endgenerate


  ///////////////
  // Registers //
  ///////////////

  // registers for column results
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni)
      binconv_column_pres_q <= '0;
    else if(clear_i)
      binconv_column_pres_q <= '0;
    else if(enable_i & block_pres[0].valid & block_pres[0].ready)
      binconv_column_pres_q <= binconv_column_pres_d;
  end

  // registers for output valid signal
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      binconv_column_pres_valid_q <= '0;
      binconv_column_pres_strb_q  <= '0;
    end
    else if(clear_i) begin
      binconv_column_pres_valid_q <= '0;
      binconv_column_pres_strb_q  <= '0;
    end
    else if(block_pres[0].ready) begin
      binconv_column_pres_valid_q <= binconv_column_pres_valid_d;
      binconv_column_pres_strb_q  <= binconv_column_pres_strb_d;
    end

  end

endmodule // ne16_binconv_column
