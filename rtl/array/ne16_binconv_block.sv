/*
 * ne16_binconv_block.sv
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

module ne16_binconv_block #(
  parameter int unsigned BLOCK_SIZE = NE16_BLOCK_SIZE,           // number of SoP's per BinConv block (default 4)
  parameter int unsigned TP_IN      = NE16_TP_IN,                // number of input elements processed per cycle
  parameter int unsigned PIPELINE   = 1
) (
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // local enable & clear
  input  logic                   enable_i,
  input  logic                   clear_i,
  // input activation stream + handshake
  hwpe_stream_intf_stream.sink   activation_i [BLOCK_SIZE-1:0],
  // input weight stream + handshake
  hwpe_stream_intf_stream.sink   weight_i,
  // output features + handshake
  hwpe_stream_intf_stream.source block_pres_o,
  // control channel
  input  ctrl_binconv_block_t    ctrl_i,
  output flags_binconv_block_t   flags_o
);

  logic clk_gated;
  cluster_clock_gating i_hier_block_gate (
    .clk_i     ( clk_i              ),
    .en_i      ( enable_i | clear_i ),
    .test_en_i ( test_mode_i        ),
    .clk_o     ( clk_gated          )
  );

  ///////////////////////////////////////////
  // Local Params, Interfaces, and Signals //
  ///////////////////////////////////////////

  // internal weight interface
  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( 1 )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) weight_int [BLOCK_SIZE-1:0] (
    .clk ( clk_i )
  );

  // BinConv result interface
  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_QA_IN )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) popcount [BLOCK_SIZE-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_QA_IN+$clog2(BLOCK_SIZE)+NE16_QA_16BIT )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) pres_nonscaled (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_QA_IN+$clog2(BLOCK_SIZE)+NE16_QA_16BIT+8 )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) pres (
    .clk ( clk_i )
  );

  logic clear_int;

  logic [NE16_QA_IN+$clog2(BLOCK_SIZE)+NE16_QA_16BIT-1:0] binconv_block_pres_nonscaled_d, binconv_block_pres_nonscaled_q;
  logic [NE16_QA_IN+$clog2(BLOCK_SIZE)-2:0] binconv_block_pres_nonscaled_hi_d;
  logic [NE16_QA_IN+$clog2(BLOCK_SIZE)-2:0] binconv_block_pres_nonscaled_lo_d;
  logic                                     binconv_block_pres_nonscaled_valid_d, binconv_block_pres_nonscaled_valid_q;

  logic [NE16_QA_IN+NE16_QA_16BIT+8+$clog2(BLOCK_SIZE)-1:0] binconv_block_pres_q;
  logic                                                     binconv_block_pres_valid_q;

  ctrl_scale_t scale_ctrl;
  ctrl_scale_t scale_ctrl_q;

  logic [BLOCK_SIZE-1:0] [NE16_QA_IN-1:0]     popcount_data;

  assign clear_int = clear_i | ctrl_i.clear;

  ///////////////////////////////
  // BinConv and Scale Modules //
  ///////////////////////////////
  // iterate over all BLOCK_SIZE BinConvs in a singe block

  generate

    for(genvar ii=0; ii<BLOCK_SIZE; ii+=1) begin : sop_gen

      localparam ii_div2 = ii/2;

      assign weight_int[ii].data  = ctrl_i.weight_offset ? 1    : ctrl_i.mode_16 ? weight_i.data[ii_div2] : weight_i.data[ii];
      assign weight_int[ii].valid = ctrl_i.weight_offset ? 1'b1 : weight_i.valid;
      assign weight_int[ii].strb  = weight_i.strb;

      assign popcount[ii].valid = (ctrl_i.weight_offset==1'b0)                  ? activation_i[ii].valid & activation_i[ii].ready & weight_int[ii].valid & weight_int[ii].ready :
                                  (ctrl_i.filter_mode==NE16_FILTER_MODE_3X3_DW) ? activation_i[ii].valid & activation_i[ii].ready & ~ctrl_i.invalidate :
                                  (ctrl_i.block_cnt=='0)                        ? activation_i[ii].valid : '0;
      assign popcount[ii].strb  = '1;

      // 1x8bit "multipliers" (i.e., simple multiplexers)
      assign popcount[ii].data  = ctrl_i.enable_mac[ii] & weight_int[ii].data ? activation_i[ii].data : '0;

      // ========================================================================
      // INPUT STREAMER HANDSHAKING
      // ========================================================================

      always_comb
      begin : ready_propagation
        case({activation_i[ii].valid, weight_int[ii].valid})
          2'b00 : begin
            activation_i[ii].ready = popcount[ii].ready;
            weight_int[ii].ready   = popcount[ii].ready;
          end
          2'b01 : begin
            activation_i[ii].ready = popcount[ii].ready;
            weight_int[ii].ready   = 1'b0;
          end
          2'b10 : begin
            activation_i[ii].ready = 1'b0;
            weight_int[ii].ready   = popcount[ii].ready;
          end
          2'b11 : begin
            activation_i[ii].ready = popcount[ii].ready;
            weight_int[ii].ready   = popcount[ii].ready;
          end
        endcase
      end

      assign popcount_data[ii] = popcount[ii].data;
      assign popcount[ii].ready = pres_nonscaled.ready;

    end // sop_gen

    if (PIPELINE ==1 ) begin : pipe_stage_gen

      always_ff @(posedge clk_gated or negedge rst_ni)
      begin
        if(~rst_ni) begin
          binconv_block_pres_nonscaled_q       <= '0;
          scale_ctrl_q <= '0;
        end
        else if(clear_int) begin
          binconv_block_pres_nonscaled_q       <= '0;
          scale_ctrl_q <= '0;
        end
        else if(enable_i) begin
          binconv_block_pres_nonscaled_q       <= binconv_block_pres_nonscaled_d;
          scale_ctrl_q <= scale_ctrl;
        end
      end

      always_ff @(posedge clk_i or negedge rst_ni)
      begin
        if(~rst_ni) begin
          binconv_block_pres_nonscaled_valid_q <= '0;
        end
        else if(clear_int) begin
          binconv_block_pres_nonscaled_valid_q <= '0;
        end
        else begin
          binconv_block_pres_nonscaled_valid_q <= binconv_block_pres_nonscaled_valid_d;
        end
      end

    end
    else begin

      assign binconv_block_pres_nonscaled_q = binconv_block_pres_nonscaled_d;
      assign binconv_block_pres_nonscaled_valid_q = binconv_block_pres_nonscaled_valid_d;
      assign scale_ctrl_q = scale_ctrl;

    end

  endgenerate


  //////////////////////////////////
  // Block-level reduction
  //////////////////////////////////
  always_comb
  begin
    binconv_block_pres_nonscaled_hi_d = '0;
    for(int i=1; i<BLOCK_SIZE; i+=2) begin
      binconv_block_pres_nonscaled_hi_d += popcount_data[i];
    end
  end
  always_comb
  begin
    binconv_block_pres_nonscaled_lo_d = '0;
    for(int i=0; i<BLOCK_SIZE; i+=2) begin
      binconv_block_pres_nonscaled_lo_d += popcount_data[i];
    end
  end
  assign binconv_block_pres_nonscaled_d = ctrl_i.mode_16 ? {binconv_block_pres_nonscaled_hi_d, 8'b0} + binconv_block_pres_nonscaled_lo_d :
                                                           binconv_block_pres_nonscaled_hi_d         + binconv_block_pres_nonscaled_lo_d;

  assign binconv_block_pres_nonscaled_valid_d = popcount[0].valid;

  assign pres_nonscaled.strb  = '1;
  assign pres_nonscaled.data  = binconv_block_pres_nonscaled_q;
  assign pres_nonscaled.valid = binconv_block_pres_nonscaled_valid_q;

  //////////////////////////////////
  // Scaling factor
  //////////////////////////////////
  ne16_scale #(
    .INP_ACC     ( NE16_QA_IN + $clog2(BLOCK_SIZE) + NE16_QA_16BIT     ),
    .OUT_ACC     ( NE16_QA_IN + $clog2(BLOCK_SIZE) + NE16_QA_16BIT + 8 ),
    .N_SHIFTS    ( 8                                                   )
  ) i_binconv_scale (
    .clk_i       ( clk_gated      ),
    .rst_ni      ( rst_ni         ),
    .test_mode_i ( test_mode_i    ),
    .data_i      ( pres_nonscaled ),
    .data_o      ( pres           ),
    .ctrl_i      ( scale_ctrl_q   ),
    .flags_o     (                )
  );

  assign flags_o = '0; // FIXME

  ////////////////////////
  // Output Assignments //
  ////////////////////////
  assign weight_i.ready = weight_int[0].ready;

  assign block_pres_o.valid = binconv_block_pres_valid_q;
  assign block_pres_o.data  = binconv_block_pres_q;

  assign pres.ready = block_pres_o.ready;

  ///////////////
  // Registers //
  ///////////////
  // registers for block results
  always_ff @(posedge clk_gated or negedge rst_ni)
    begin
      if(~rst_ni)
        binconv_block_pres_q <= '0;
      else if(clear_int)
        binconv_block_pres_q <= '0;
      else if(pres.valid & pres.ready)
        binconv_block_pres_q <= pres.data;
    end

  // registers for output valid signal
  always_ff @(posedge clk_i or negedge rst_ni)
    begin
      if(~rst_ni)
        binconv_block_pres_valid_q <= '0;
      else if(clear_int)
        binconv_block_pres_valid_q <= '0;
      else if(pres.ready)
        binconv_block_pres_valid_q <= pres.valid;
    end

  generate
    assign scale_ctrl.shift_sel = (ctrl_i.filter_mode == NE16_FILTER_MODE_3X3_DW & ctrl_i.weight_offset) ? '0 :
                                  ~ctrl_i.mode_linear & (ctrl_i.filter_mode == NE16_FILTER_MODE_1X1)     ? ctrl_i.scale_shift :
                                                                                                           ctrl_i.block_cnt;
    assign scale_ctrl.invert = 1'b0;
  endgenerate

endmodule // ne16_binconv_block
