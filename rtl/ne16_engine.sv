/*
 * ne16_engine.sv
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

module ne16_engine #(
  parameter int unsigned COLUMN_SIZE    = NE16_COLUMN_SIZE, // number of BinConv blocks per column (default 9)
  parameter int unsigned NR_COLUMN      = NE16_COLUMN_SIZE, // number of BinConv columns (default 9 -- same of size of BinConv columns!)
  parameter int unsigned BLOCK_SIZE     = NE16_BLOCK_SIZE,  // number of SoP's per BinConv block (default 4)
  parameter int unsigned INPUT_BUF_SIZE = 32*BLOCK_SIZE,    // TODO FIXME
  parameter int unsigned TP_IN          = NE16_TP_IN,       // number of input elements processed per cycle
  parameter int unsigned TP_OUT         = NE16_TP_OUT
) (
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // local enable & clear
  input  logic                   enable_i,
  input  logic                   clear_i,
  // input streams + handshake
  hwpe_stream_intf_stream.sink   load_in,
  hwpe_stream_intf_stream.sink   load_weight,
  hwpe_stream_intf_stream.sink   load_norm,
  hwpe_stream_intf_stream.sink   load_streamin,
  hwpe_stream_intf_stream.source store_out,
  input  ctrl_engine_t           ctrl_i,
  output flags_engine_t          flags_o
);

  /* Local Params, Interfaces, and Signals */
  localparam BLOCK_PRES_SIZE  = NE16_QA_IN+NE16_QA_16BIT+8+$clog2(BLOCK_SIZE);
  localparam COLUMN_PRES_SIZE = BLOCK_PRES_SIZE+$clog2(COLUMN_SIZE);

  logic                      all_norm_ready;
  logic [NE16_NR_COLUMN-1:0] all_norm_ready_tree;

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_QA_IN )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_in_blocks [BLOCK_SIZE-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_weight_fifo (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_weight_fifo_demuxed [1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( TP_IN )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_weight_rows_mode8 [15:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( TP_IN/2 )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_weight_rows_mode16_8bit [31:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( TP_IN )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_weight_rows_mode16 [31:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( TP_IN )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_weight_rows_conv [COLUMN_SIZE-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( TP_IN )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_weight_rows_linear [31:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) store_out_cols [NR_COLUMN-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_streamin_cols [NR_COLUMN-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_QA_IN )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) in_from_buf [INPUT_BUF_SIZE-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( COLUMN_PRES_SIZE )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) pres [NR_COLUMN-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) norm [NR_COLUMN-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_norm_fifo (
    .clk ( clk_i )
  );

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH )
`ifndef SYNTHESIS
    ,
    .BYPASS_VCR_ASSERT( 1'b1  ),
    .BYPASS_VDR_ASSERT( 1'b1  )
`endif
  ) load_streamin_fifo (
    .clk ( clk_i )
  );

  // Infeat data from the input buffer is split in blocks of size 16bits
  //
  //            load_in[128b]
  //                 ||
  //                 \/
  //         +-----------------+
  //         |hwpe_stream_split|
  //         +-----------------+
  //                 ||
  //                 \/
  //        load_in_blocks[15:0][8b]

  hwpe_stream_split #(
    .NB_OUT_STREAMS ( BLOCK_SIZE            ),
    .DATA_WIDTH_IN  ( NE16_QA_IN*BLOCK_SIZE )
  ) i_split_load_in_blocks (
    .clk_i   ( clk_i          ),
    .rst_ni  ( rst_ni         ),
    .clear_i ( clear_i        ),
    .push_i  ( load_in        ),
    .pop_o   ( load_in_blocks )
  );
  
  // The following diagram explains the way that the weight stream is split in order to
  // support the various CONV modes and the LINEAR mode at 16 and 8 bits.
  // 
  //                               load_weight[256b]
  //                                      ||
  //                                      \/
  //                                    |____|
  //                                    |____| hwpe_stream_fifo
  //                                      ||
  //                                      \/
  //                             load_weight_fifo[256b]
  //                                      ||
  //                                      \/
  //                          /------------------------\
  // ctrl_i.mode16 ------->  /__________________________\
  //                          || 0                  1 ||
  //                          \/                      \/
  //       load_weight_fifo_demuxed[0][256b] load_weight_fifo_demuxed[1][256b]
  //                          ||                      ||
  //                          \/                      \/
  //                 +-----------------+      +-----------------+
  //                 |hwpe_stream_split|      |hwpe_stream_split|
  //                 +-----------------+      +-----------------+
  //                          ||                      ||
  //                          ||                      \/
  //                          ||                load_weight_rows_mode16_8bit[31:0][8b]
  //                          ||                      ||
  //                          ||                      \/
  //                          ||              +-----------------+
  //                          ||              |   zero-extend   |
  //                          ||              +-----------------+
  //                          ||                      ||
  //                          \/                      \/
  //       load_weight_rows_mode8[15:0][16b]    load_weight_rows_mode16[31:0][16b]
  //
  //
  // Convolutional modes actually use only 144 of the 256bits of memory interface:
  //       load_weight_rows_mode8[15:0][16b]    load_weight_rows_mode16[31:0][16b]
  //                          || [8:0]]               || [8:0]
  //                          \/ 0                  1 \/
  //                         \--------------------------/
  // ctrl_i.mode16 ---------->\________________________/
  //                                      ||
  //                                      \/
  //                          load_weight_rows_conv[8:0][16b]
  //
  // 
  // Linear mode uses 256 bits of bandwidth in both 8 and 16 bit modes -- with 16 bit mode using 2x the number of MACs
  //
  //       load_weight_rows_mode8[15:0][16b]    load_weight_rows_mode16[31:0][16b]                     256b zeros              load_weight_rows_mode16[31:0][16b]  
  //                          ||                      || [15:0]                                              ||                      || [31:16]           
  //                          \/ 0                  1 \/                                                     \/ 0                  1 \/                            
  //                         \--------------------------/                                                   \--------------------------/                           
  // ctrl_i.mode16 ---------->\________________________/                            ctrl_i.mode16 ---------->\________________________/                            
  //                                      ||                                                                             ||                                        
  //                                      \/                                                                             \/                                        
  //                          load_weight_rows_linear[15:0][16b]                                             load_weight_rows_linear[31:0][16b]                    

  hwpe_stream_fifo #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH ),
    .FIFO_DEPTH ( 2                  )
  ) i_fifo_load_weight (
    .clk_i   ( clk_i            ),
    .rst_ni  ( rst_ni           ),
    .clear_i ( clear_i          ),
    .flags_o (                  ),
    .push_i  ( load_weight      ),
    .pop_o   ( load_weight_fifo )
  );

  hwpe_stream_demux_static #(
    .NB_OUT_STREAMS ( 2 )
  ) i_fifo_load_weight_fifo_demux (
    .clk_i   ( clk_i                    ),
    .rst_ni  ( rst_ni                   ),
    .clear_i ( clear_i                  ),
    .sel_i   ( ctrl_i.mode_16           ),
    .push_i  ( load_weight_fifo         ),
    .pop_o   ( load_weight_fifo_demuxed )
  );

  hwpe_stream_split #(
    .NB_OUT_STREAMS ( 16                 ),
    .DATA_WIDTH_IN  ( NE16_MEM_BANDWIDTH )
  ) i_split_load_weight_rows_mode8 (
    .clk_i   ( clk_i                       ),
    .rst_ni  ( rst_ni                      ),
    .clear_i ( clear_i                     ),
    .push_i  ( load_weight_fifo_demuxed[0] ),
    .pop_o   ( load_weight_rows_mode8      )
  );

  hwpe_stream_split #(
    .NB_OUT_STREAMS ( 32                 ),
    .DATA_WIDTH_IN  ( NE16_MEM_BANDWIDTH )
  ) i_split_load_weight_rows_mode16 (
    .clk_i   ( clk_i                        ),
    .rst_ni  ( rst_ni                       ),
    .clear_i ( clear_i                      ),
    .push_i  ( load_weight_fifo_demuxed[1]  ),
    .pop_o   ( load_weight_rows_mode16_8bit )
  );

  generate

    for(genvar ii=0; ii<32; ii++) begin: load_weight_rows_mode16_adapt_gen
      assign load_weight_rows_mode16[ii].data  = { 8'b0, load_weight_rows_mode16_8bit[ii].data };
      assign load_weight_rows_mode16[ii].valid = load_weight_rows_mode16_8bit[0].valid;
      assign load_weight_rows_mode16[ii].strb  = load_weight_rows_mode16_8bit[0].strb;
      assign load_weight_rows_mode16_8bit[ii].ready = load_weight_rows_mode16[ii].ready;
    end

    logic ready_conv, ready_linear;
    assign ready_conv = load_weight_rows_conv[0].ready;
    assign ready_linear = load_weight_rows_linear[0].ready;

    for(genvar ii=0; ii<COLUMN_SIZE; ii++) begin: load_weight_rows_mode16_conv_mux_gen
      assign load_weight_rows_conv[ii].data  = ctrl_i.mode_16 ? load_weight_rows_mode16[ii].data : load_weight_rows_mode8[ii].data;
      assign load_weight_rows_conv[ii].valid = ctrl_i.mode_16 ? load_weight_rows_mode16[0].valid : load_weight_rows_mode8[0].valid;
      assign load_weight_rows_conv[ii].strb  = '1;
    end
    
    for(genvar ii=0; ii<16; ii++) begin: load_weight_rows_mode16_linear_mux_lo_gen
      assign load_weight_rows_linear[ii].data  = ctrl_i.mode_16 ? load_weight_rows_mode16[ii].data : load_weight_rows_mode8[ii].data;
      assign load_weight_rows_linear[ii].valid = ctrl_i.mode_16 ? load_weight_rows_mode16[0].valid : load_weight_rows_mode8[0].valid;
      assign load_weight_rows_linear[ii].strb  = '1;
    end
    
    for(genvar ii=16; ii<32; ii++) begin: load_weight_rows_mode16_linear_mux_hi_gen
      assign load_weight_rows_linear[ii].data  = ctrl_i.mode_16 ? load_weight_rows_mode16[ii].data : '0;
      assign load_weight_rows_linear[ii].valid = ctrl_i.mode_16 ? load_weight_rows_mode16[0].valid : '0;
      assign load_weight_rows_linear[ii].strb  = '1;
    end

    for(genvar ii=0; ii<16; ii++) begin: load_weight_rows_lo_ready_prop_gen
      assign load_weight_rows_mode16[ii].ready =  ctrl_i.mode_16 ? ready_conv | ready_linear : 1'b0;
      assign load_weight_rows_mode8[ii].ready  = ~ctrl_i.mode_16 ? ready_conv | ready_linear : 1'b0;
    end
    
    for(genvar ii=16; ii<32; ii++) begin: load_weight_rows_hi_ready_prop_gen
      assign load_weight_rows_mode16[ii].ready =  ctrl_i.mode_16 ? ready_conv | ready_linear : 1'b0;
    end

  endgenerate

  // Streamout data from the column accumulators is serialized one column after the other
  //
  //        store_out_cols[8:0][256b]
  //                 ||
  //                 \/
  //       +---------------------+
  //       |hwpe_stream_serialize|
  //       +---------------------+
  //                 ||
  //                 \/
  //           store_out[256b]

  hwpe_stream_serialize #(
    .NB_IN_STREAMS ( NR_COLUMN          ),
    .DATA_WIDTH    ( NE16_MEM_BANDWIDTH )
  ) i_serialize_store_out (
    .clk_i   ( clk_i                 ),
    .rst_ni  ( rst_ni                ),
    .clear_i ( clear_i               ),
    .ctrl_i  ( ctrl_i.ctrl_serialize ),
    .push_i  ( store_out_cols        ),
    .pop_o   ( store_out             )
  );

  // Streamin data goingo into the column accumulators comes per column and is deserialized
  //
  //          load_streamin[256b]
  //                 ||
  //                 \/
  //               |____|
  //               |____| hwpe_stream_fifo
  //                 ||
  //                 \/
  //        load_streamin_fifo[256b]
  //                 ||
  //                 \/
  //      +-----------------------+
  //      |hwpe_stream_deserialize|
  //      +-----------------------+
  //                 ||
  //                 \/
  //           load_streamin_cols[8:0][256b]

  hwpe_stream_fifo #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH ),
    .FIFO_DEPTH ( 2                  )
  ) i_fifo_load_streamin (
    .clk_i   ( clk_i              ),
    .rst_ni  ( rst_ni             ),
    .clear_i ( clear_i            ),
    .flags_o (                    ),
    .push_i  ( load_streamin      ),
    .pop_o   ( load_streamin_fifo )
  );

  hwpe_stream_deserialize #(
    .NB_OUT_STREAMS ( NR_COLUMN          ),
    .DATA_WIDTH     ( NE16_MEM_BANDWIDTH )
  ) i_deserialize_load_streamin (
    .clk_i   ( clk_i                      ),
    .rst_ni  ( rst_ni                     ),
    .clear_i ( clear_i | ctrl_i.clear_des ),
    .ctrl_i  ( ctrl_i.ctrl_serialize      ),
    .push_i  ( load_streamin_fifo         ),
    .pop_o   ( load_streamin_cols         )
  );

  // The same norm stream, coming simply from a FIFO, is shared between all columns.
  //
  //          load_norm[256b]
  //                 ||
  //                 \/
  //               |____|
  //               |____| hwpe_stream_fifo
  //                 ||
  //                 \/
  //          load_norm_fifo[256b]
  //                 || copy 9x
  //                 \/
  //             norm[8:0][256b]

  // enqueue norm stream
  hwpe_stream_fifo #(
    .DATA_WIDTH ( NE16_MEM_BANDWIDTH ),
    .FIFO_DEPTH ( 2                  )
  ) i_fifo_load_norm (
    .clk_i   ( clk_i          ),
    .rst_ni  ( rst_ni         ),
    .clear_i ( clear_i        ),
    .flags_o (                ),
    .push_i  ( load_norm      ),
    .pop_o   ( load_norm_fifo )
  );

  // duplicate norm stream
  generate
    for(genvar ii=0; ii<NE16_NR_COLUMN; ii++) begin
      assign all_norm_ready_tree[ii] = norm[ii].ready;
      assign norm[ii].data           = load_norm_fifo.data;
      assign norm[ii].valid          = load_norm_fifo.valid;
      assign norm[ii].strb           = load_norm_fifo.strb;
    end

    assign all_norm_ready = &(all_norm_ready_tree);
    assign load_norm_fifo.ready = all_norm_ready;
  endgenerate

  /* Input Buffer */
  ne16_input_buffer #(
    .INPUT_BUF_SIZE ( INPUT_BUF_SIZE ),
    .BLOCK_SIZE     ( BLOCK_SIZE     ),
    .DW             ( NE16_QA_IN     )
  ) i_input_buffer (
    .clk_i       ( clk_i                      ),
    .rst_ni      ( rst_ni                     ),
    .test_mode_i ( test_mode_i                ),
    .enable_i    ( enable_i                   ),
    .clear_i     ( clear_i                    ),
    .ctrl_i      ( ctrl_i.ctrl_input_buffer   ),
    .flags_o     ( flags_o.flags_input_buffer ),
    .feat_i      ( load_in_blocks             ),
    .feat_o      ( in_from_buf                )
  );

  /* BinConv Array */
  ne16_binconv_array #(
    .COLUMN_SIZE    ( COLUMN_SIZE    ),
    .NR_COLUMN      ( NR_COLUMN      ),
    .NR_ACTIVATIONS ( INPUT_BUF_SIZE ),
    .BLOCK_SIZE     ( BLOCK_SIZE     ),
    .TP_IN          ( TP_IN          )
  ) i_binconv_array (
    .clk_i             ( clk_i                          ),
    .rst_ni            ( rst_ni                         ),
    .test_mode_i       ( test_mode_i                    ),
    .enable_i          ( enable_i                       ),
    .clear_i           ( clear_i                        ),
    .activation_i      ( in_from_buf                    ),
    .weight_conv_i     ( load_weight_rows_conv          ),
    .weight_linear_i   ( load_weight_rows_linear        ),
    .pres_o            ( pres                           ),
    .ctrl_i            ( ctrl_i.ctrl_binconv_array      ),
    .flags_o           ( flags_o.flags_binconv_array    )
  );

  /* Accumulators + Normalization/Quantization */
  generate
    for (genvar ii=0; ii<NR_COLUMN; ii++) begin : accumulator_gen

      ctrl_aq_t ctrl_accumulator;
      always_comb
      begin
        ctrl_accumulator = ctrl_i.ctrl_accumulator;
        ctrl_accumulator.enable_streamout = ctrl_i.enable_accumulator[ii];
      end

      ne16_accumulator_normquant #(
        .TP  ( TP_IN  ),
        .AP  ( TP_OUT ),
        .ACC ( 32     )
      ) i_accumulator (
        .clk_i       ( clk_i                          ),
        .rst_ni      ( rst_ni                         ),
        .test_mode_i ( test_mode_i                    ),
        .enable_i    ( enable_i                       ),
        .clear_i     ( clear_i                        ),
        .conv_i      ( pres [ii]                      ),
        .norm_i      ( norm [ii]                      ),
        .streamin_i  ( load_streamin_cols [ii]        ),
        .conv_o      ( store_out_cols [ii]            ),
        .ctrl_i      ( ctrl_accumulator               ),
        .flags_o     ( flags_o.flags_accumulator [ii] )
      );

    end // accumulator_gen
  endgenerate

endmodule // ne16_engine
