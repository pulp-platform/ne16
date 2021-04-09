/*
 * ne16_accumulator_normquant.sv
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

module ne16_accumulator_normquant #(
  parameter int unsigned TP             = NE16_TP_IN, // output filter size in bits/cycle
  parameter int unsigned AP             = NE16_TP_OUT, // number of accumulators
  parameter int unsigned ACC            = NE16_ACCUM_SIZE,
  parameter int unsigned CNT            = VLEN_CNT_SIZE,
  parameter int unsigned PIPE_NORMQUANT = 1
) (
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // local enable & clear
  input  logic                   enable_i,
  input  logic                   clear_i,
  // incoming psums
  hwpe_stream_intf_stream.sink   conv_i,
  // incoming normalization parameters
  hwpe_stream_intf_stream.sink   norm_i,
  // incoming streamin accumulators
  hwpe_stream_intf_stream.sink   streamin_i,
  // output features + handshake
  hwpe_stream_intf_stream.source conv_o,
  // control channel
  input  ctrl_aq_t               ctrl_i,
  output flags_aq_t              flags_o
);

  localparam WIDTH_FACTOR = NE16_MEM_BANDWIDTH / ACC;

  logic clk_gated;

  logic signed [2**$clog2(AP)-1:0][ACC-1:0] all_accumulators;
  logic signed [ACC-1:0]                    accumulator_q;
  logic signed [WIDTH_FACTOR*ACC-1:0]       accumulator_wide_q;
  logic signed [4*ACC-1:0]                  normalized_q;
  logic signed [ACC-1:0]                    accumulator_plus_d;
  logic                                     addr_cnt_en_d, addr_cnt_en_q, addr_cnt_en_q2;
  logic                                     accumulator_clr;
  logic signed [ACC-1:0]                    accumulator_offset_d, accumulator_offset_q;

  logic [127:0] normquant_in;

  logic signed [ACC-1:0] accumulator_plus_q;

  logic [$clog2(AP)-1:0] waddr, waddr_streamin, waddr_streamout, waddr_normquant, waddr_normquant_bias, waddr_accum;
  logic [$clog2(AP)-1:0] raddr, raddr_streamin, raddr_streamout, raddr_normquant, raddr_normquant_bias, raddr_accum;
  logic signed [ACC-1:0] wdata, wdata_streamin, wdata_streamout, wdata_normquant, wdata_normquant_bias, wdata_accum;
  logic signed [WIDTH_FACTOR*ACC-1:0] wdata_wide, wdata_wide_streamin, wdata_wide_streamout, wdata_wide_normquant, wdata_wide_normquant_bias, wdata_wide_accum;

  logic we_all, we;

  // counter for accumulators
  logic [CNT-1:0] addr_cnt_q;
  logic [CNT-1:0] addr_cnt_q2, addr_cnt_q2_accum;
  logic [CNT-1:0] addr_cnt_d;
  logic           addr_cnt_clear;
  logic [CNT-1:0] full_accumulation_cnt;
  logic [CNT-1:0] qw_accumulation_cnt;

  logic conv_handshake_q, conv_handshake_q2, norm_handshake_q;

  cluster_clock_gating i_hier_accum_gate (
    .clk_i     ( clk_i                             ),
    .en_i      ( ctrl_i.enable_streamout | clear_i ),
    .test_en_i ( test_mode_i                       ),
    .clk_o     ( clk_gated                         )
  );

  // fsm state
  state_aq_t fsm_cs, fsm_ns;

  /* Ready propagation */
  assign conv_i.ready = (fsm_cs == AQ_ACCUM) ? 1'b1 : 1'b0;
  logic norm_ready_en;
  assign norm_ready_en = fsm_cs == AQ_NORMQUANT_BIAS ? (addr_cnt_q[2] & addr_cnt_q[1] & addr_cnt_q[0]) | (addr_cnt_q2 == ctrl_i.bias_len-1 && addr_cnt_en_q == 1'b1) :
                                                       (addr_cnt_q[2] & addr_cnt_q[1] & addr_cnt_q[0]) | (addr_cnt_q2 == ctrl_i.scale_len-1 && addr_cnt_en_q == 1'b1); // norm_ready when addr_cnt is full, or in the last cycle of AQ_NORMQUANT
  assign norm_i.ready = (fsm_cs == AQ_NORMQUANT) ? norm_i.valid & norm_ready_en :
                        (fsm_cs == AQ_NORMQUANT_SHIFT) ? norm_i.valid :
                        (fsm_cs == AQ_NORMQUANT_BIAS) ? norm_i.valid : 1'b0;
  assign streamin_i.ready = (fsm_cs == AQ_STREAMIN) ? streamin_i.valid : 1'b0;

  /* accumulation buffer */

  // (where written in 1 cycle in the next read, subtracted and written to all addresses
  logic [7:0] wide_enable;

  ne16_accumulator_scm_test_wrap #(
    .ADDR_WIDTH   ( $clog2(AP)   ),
    .DATA_WIDTH   ( ACC          ),
    .WIDTH_FACTOR ( WIDTH_FACTOR )
  ) i_accumulator (
    .clk_i             ( clk_gated          ),
    .rst_ni            ( rst_ni             ),
    .clear_i           ( accumulator_clr    ),
    .test_mode_i       ( test_mode_i        ),
    .wide_enable_i     ( wide_enable        ),
    .re_i              ( enable_i           ),
    .raddr_i           ( raddr              ), // can likely be incorportaed in all_accumulators...
    .rdata_o           ( accumulator_q      ), // can likely be incorportaed in all_accumulators...
    .rdata_wide_o      ( accumulator_wide_q ), // can likely be incorportaed in all_accumulators...
    .we_all_i          ( we_all             ),
    .we_i              ( we                 ),
    .waddr_i           ( waddr              ),
    .wdata_i           ( wdata              ),
    .wdata_wide_i      ( wdata_wide         ),
    .accumulators_o    ( all_accumulators   ),
    .BIST              (                    ),
    .CSN_T             (                    ),
    .WEN_T             (                    ),
    .A_T               (                    ),
    .D_T               (                    ),
    .Q_T               (                    )
  );

  /* ACCUM operation: adder + pipeline stage for accumulator */
  logic [1:0] forward_state_d, forward_state_q;
  logic accumulator_plus_forward;

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      forward_state_q <= 2'b00;
    end
    else if(clear_i | ctrl_i.clear | ctrl_i.goto_accum | (ctrl_i.weight_offset & ctrl_i.depthwise)) begin
      forward_state_q <= 2'b10;
    end
    else if(ctrl_i.qw == '0) begin
      forward_state_q <= 2'b10;
    end
    else if(enable_i) begin
      forward_state_q <= forward_state_d;
    end
  end

  always_comb
  begin
    forward_state_d = forward_state_q;
    case(forward_state_q)
      2'b00: begin
        if(addr_cnt_en_d & conv_i.valid) begin
          forward_state_d = 2'b10;
        end
        else if(addr_cnt_en_d & ~conv_i.valid) begin
          forward_state_d = 2'b01;
        end
      end
      2'b01: begin
        if(conv_i.valid) begin
          forward_state_d = 2'b10;
        end
      end
      2'b10: begin
        if(conv_i.valid) begin
          forward_state_d = 2'b00;
        end
      end
    endcase
  end

  assign accumulator_offset_d = ctrl_i.weight_offset & ~ctrl_i.depthwise ? $signed(accumulator_offset_q + $signed(normalized_q[31:0])) :
                                                                           '0;
  always_ff @(posedge clk_i or negedge rst_ni)
  begin : accumulator_offset_ff
    if(~rst_ni) begin
      accumulator_offset_q <= '0;
    end
    else if(clear_i | ctrl_i.clear | ctrl_i.clear_offset) begin
      accumulator_offset_q <= '0;
    end
    else if(enable_i & conv_handshake_q2 & ctrl_i.weight_offset& ~ctrl_i.depthwise) begin
      accumulator_offset_q <= accumulator_offset_d;
    end
  end

  assign accumulator_plus_forward = ctrl_i.weight_offset ? 1'b0 : forward_state_q != 2'b10 ? 1'b1 : 1'b0;

  assign accumulator_plus_d = ctrl_i.weight_offset     ? accumulator_q :
                              accumulator_plus_forward ? accumulator_plus_q + $signed(conv_i.data) :
                                                         accumulator_q + $signed(conv_i.data) ;
  always_ff @(posedge clk_i or negedge rst_ni)
  begin : accumulator_plus_ff
    if(~rst_ni) begin
      accumulator_plus_q <= '0;
    end
    else if(clear_i | ctrl_i.clear) begin
      accumulator_plus_q <= '0;
    end
    // else if(ctrl_i.goto_accum) begin
    //   accumulator_plus_q <= accumulator_q;
    // end
    else if(enable_i & conv_i.valid) begin
      accumulator_plus_q <= accumulator_plus_d;
    end
  end

  assign waddr_accum = (ctrl_i.weight_offset & ctrl_i.depthwise) ? addr_cnt_q :
                                                                   addr_cnt_q2_accum; // in 1x1 mode, no QW accumulation
  assign raddr_accum = addr_cnt_d;
  assign wdata_accum = ctrl_i.weight_offset & ctrl_i.depthwise ? wdata_normquant :
                                              ctrl_i.depthwise ? accumulator_plus_q :
                                                                 accumulator_plus_q + accumulator_offset_q;
  assign wdata_wide_accum = '0;

  /* STREAMIN operation: load */

  // 256 bit word -> 8x 32-bit accumulators

  assign waddr_streamin = addr_cnt_q << 3;
  assign raddr_streamin = '0;
  assign wdata_streamin = '0;
  assign wdata_wide_streamin = streamin_i.data;

  /* NORM operation */

  // 1 big-word:
  //  - parameters for all accumulators (8-bit)
  //  - parameters for 16 accumulators  (16-bit)
  //  - paramaters for 8 accumulators   (32-bit)

  localparam NMULT = 4;

  logic [NE16_MEM_BANDWIDTH/8 -1:0][ 7:0] norm_data_8b;
  logic [NE16_MEM_BANDWIDTH/16-1:0][15:0] norm_data_16b;
  logic [NE16_MEM_BANDWIDTH/32-1:0][31:0] norm_data_32b;
  logic [NMULT-1:0][NORM_MULT_SIZE-1:0] norm_mult;
  ctrl_normquant_t ctrl_normquant;
  flags_normquant_t [3:0] flags_normquant;

  localparam NMS = NORM_MULT_SIZE;

  // sample shift data
  logic [NE16_MEM_BANDWIDTH/8 -1:0][ 7:0] shift_data_q;
  logic [NMULT-1:0][ 7:0] shift_data_q2;
  logic [NMULT-1:0][ 7:0] norm_shift;
  logic [8-1:0][ 7:0] norm_shift_bias;

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      shift_data_q <= '0;
    end
    else if (clear_i) begin
      shift_data_q <= '0;
    end
    else if(ctrl_i.weight_offset) begin
      shift_data_q <= '0;
    end
    else if(~ctrl_i.norm_option_shift & ctrl_i.sample_shift) begin
      shift_data_q <= {(NE16_MEM_BANDWIDTH/8) {3'b0, ctrl_i.ctrl_normquant.right_shift}};
    end
    else if(fsm_cs==AQ_NORMQUANT_SHIFT & norm_i.valid & norm_i.ready) begin
      shift_data_q <= norm_data_8b;
    end
  end

  logic shift_data_q2_en_d, shift_data_q2_en_q;
  assign shift_data_q2_en_d = ctrl_i.weight_offset | (~ctrl_i.norm_option_shift & ctrl_i.sample_shift) | (fsm_cs==AQ_NORMQUANT_SHIFT) | (fsm_cs==AQ_NORMQUANT_BIAS) | (fsm_cs==AQ_NORMQUANT);

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      shift_data_q2_en_q <= '0;
    end
    else if(clear_i) begin
      shift_data_q2_en_q <= '0;
    end
    else begin
      shift_data_q2_en_q <= shift_data_q2_en_d;
    end
  end

  assign norm_data_8b  = norm_i.data;
  assign norm_data_16b = norm_i.data;
  assign norm_data_32b = ctrl_i.weight_offset ? {NE16_MEM_BANDWIDTH/32 {ctrl_i.weight_offset_scale}} : norm_i.data;
  generate
    for(genvar ii=0; ii<NMULT; ii++) begin : norm_mult_gen
      assign norm_shift[ii] = shift_data_q2[ii];
      always_comb
      begin
        norm_mult [ii] = '0;
        // norm_shift[ii] = '0;
        if(ctrl_i.norm_mode == NE16_MODE_32B || ctrl_i.weight_offset==1'b1) begin // actually 24 bits!
          norm_mult[ii]  = norm_data_32b[addr_cnt_q[2:0]][(ii+1)*8-1:ii*8];
          // norm_shift[ii] = shift_data_q [addr_cnt_q2[5:0]];
        end
        else if(ctrl_i.norm_mode == NE16_MODE_16B) begin
          localparam ii_div2 = ii/2;
          localparam ii_rem2 = ii%2;
          norm_mult[ii] = norm_data_16b[{addr_cnt_q[2:0], 1'b0} + ii_div2][(ii_rem2+1)*8-1:ii_rem2*8];
          // norm_shift[ii] = shift_data_q [{addr_cnt_q2[5:0], 1'b0} + ii_rem2];
        end
        else if(ctrl_i.norm_mode == NE16_MODE_8B) begin
          norm_mult[ii] = norm_data_8b[{addr_cnt_q[2:0], 2'b0} + ii];
          // norm_shift[ii] = shift_data_q [{addr_cnt_q2[5:0], 2'b0}+ii];
        end
      end
      always_ff @(posedge clk_i or negedge rst_ni)
      begin
        if(~rst_ni) begin
          shift_data_q2[ii] <= '0;
        end
        else if(clear_i) begin
          shift_data_q2[ii] <= '0;
        end
        else if(shift_data_q2_en_q) begin
          if(ctrl_i.norm_mode == NE16_MODE_32B || ctrl_i.weight_offset==1'b1) begin // actually 24 bits!
            shift_data_q2[ii] <= shift_data_q [addr_cnt_q[5:0]];
          end
          else if(ctrl_i.norm_mode == NE16_MODE_16B) begin
            localparam ii_div2 = ii/2;
            localparam ii_rem2 = ii%2;
            shift_data_q2[ii] <= shift_data_q [{addr_cnt_q[5:0], 1'b0} + ii_rem2];
          end
          else if(ctrl_i.norm_mode == NE16_MODE_8B) begin
            shift_data_q2[ii] <= shift_data_q [{addr_cnt_q[5:0], 2'b0}+ii];
          end
        end
      end
    end
    for(genvar ii=0; ii<8; ii++) begin : norm_add_gen
      assign norm_shift_bias[ii] = shift_data_q [{addr_cnt_q[5:0], 3'b0}+ii];
    end
  endgenerate

  assign normquant_in = ctrl_i.weight_offset ? { 96'b0, $signed(conv_i.data) } : accumulator_wide_q [4*32-1:0];
  ne16_normquant #(
    .NMULT           ( 4              ),
    .ACC             ( ACC            ),
    .PIPE            ( PIPE_NORMQUANT )
  ) i_normquant (
    .clk_i         ( clk_gated                              ),
    .rst_ni        ( rst_ni                                 ),
    .test_mode_i   ( test_mode_i                            ),
    .clear_i       ( clear_i                                ),
    .norm_mult_i   ( norm_mult                              ),
    .shift_i       ( norm_shift                             ),
    .accumulator_i ( normquant_in                           ),
    .accumulator_o ( normalized_q                           ),
    .ctrl_i        ( ctrl_normquant                         ),
    .flags_o       ( flags_normquant                        )
  );

  assign waddr_normquant = ctrl_i.norm_mode == NE16_MODE_8B  ? { addr_cnt_q2[CNT-3:1], 3'b0 } :
                           ctrl_i.norm_mode == NE16_MODE_16B ? { addr_cnt_q2[CNT-2:2], 3'b0 } :
                                                               addr_cnt_q2;
  assign raddr_normquant = ctrl_i.norm_mode == NE16_MODE_8B  ? addr_cnt_d << 2 :
                           ctrl_i.norm_mode == NE16_MODE_16B ? addr_cnt_d << 1 :
                                                               addr_cnt_d;
  assign wdata_normquant = normalized_q[31:0]; // used only in weight_offseting_with_scale stage
  assign wdata_wide_normquant = {{ACC{1'b0}}, normalized_q};

  always_comb
  begin
    ctrl_normquant = ctrl_i.ctrl_normquant;
    ctrl_normquant.start = fsm_cs == AQ_NORMQUANT ? 1'b1 : 1'b0;
  end

  /* NORMQUANT_BIAS operation */

  logic [255:0] norm_bias;
  logic [255:0] biased_data;
  assign norm_bias = norm_i.data;

  ne16_normquant_bias i_normquant_bias (
    .clk_i         ( clk_gated          ),
    .rst_ni        ( rst_ni             ),
    .test_mode_i   ( test_mode_i        ),
    .clear_i       ( clear_i            ),
    .norm_bias_i   ( norm_bias          ),
    .shift_i       ( norm_shift_bias    ),
    .accumulator_i ( accumulator_wide_q ),
    .accumulator_o ( biased_data        ),
    .ctrl_i        ( ctrl_normquant     )
  );

  assign waddr_normquant_bias = addr_cnt_q << 3;
  assign raddr_normquant_bias = addr_cnt_d << 3;
  assign wdata_normquant_bias = '0;
  assign wdata_wide_normquant_bias = biased_data;

  /* STREAMOUT operation */
  assign waddr_streamout = '0;
  assign raddr_streamout = addr_cnt_d << 3;
  assign wdata_streamout = '0;
  assign wdata_wide_streamout = '0;

  logic [255:0] conv_data_8b;
  logic [255:0] conv_data_32b;

  generate
    for(genvar ii=0; ii<ACC; ii++) begin : streamout_data_8b_gen
      assign conv_data_8b [(ii+1)*NE16_QA_OUT-1:ii*NE16_QA_OUT] = all_accumulators[ii][NE16_QA_OUT-1:0];
    end
    assign conv_data_32b = accumulator_wide_q;
  endgenerate
  assign conv_o.data = (ctrl_i.quant_mode == NE16_MODE_8B)  ? conv_data_8b  :
                       (ctrl_i.quant_mode == NE16_MODE_32B) ? conv_data_32b : '0; // 256 bit
  assign conv_o.strb = ~ctrl_i.enable_streamout             ? '0 :
                       (ctrl_i.quant_mode == NE16_MODE_8B)  ? (ctrl_i.streamout_len == 0     ? '1 : (1 << ( ctrl_i.streamout_len))   - 1)  :
                       (ctrl_i.quant_mode == NE16_MODE_32B && addr_cnt_q < (ctrl_i.streamout_len/WIDTH_FACTOR + (ctrl_i.streamout_len%WIDTH_FACTOR==0 ? 0 : 1) -1) ) ? '1 :
                       (ctrl_i.quant_mode == NE16_MODE_32B) ? (ctrl_i.streamout_len % 8 == 0 ? '1 : (1 << ((ctrl_i.streamout_len % 8)*4)) -1) : '1;

  // ========================================================================
  // Counters
  // ========================================================================

  // read and write address in Registers
  assign addr_cnt_d = (clear_i | addr_cnt_clear) ? '0 : addr_cnt_en_d ? addr_cnt_q + 1 : addr_cnt_q;

  // accumulator address counter
  always_ff @(posedge clk_i or negedge rst_ni)
  begin : address_counter
    if(~rst_ni) begin
      addr_cnt_q <= '0;
      addr_cnt_en_q <= '0;
      addr_cnt_en_q2 <= '0;
    end
    else if(enable_i | ctrl_i.clear) begin
      if(clear_i) begin
        addr_cnt_q <= '0;
        addr_cnt_en_q <= '0;
        addr_cnt_en_q2 <= '0;
      end
      else begin
        addr_cnt_q <= addr_cnt_d;
        addr_cnt_en_q <= addr_cnt_en_d;
        addr_cnt_en_q2 <= addr_cnt_en_q;
      end
    end
  end

  // if normquant is pipelined, delay addr_cnt_q2
  generate
    // if(PIPE_NORMQUANT) begin : pipe_normquant
      always_ff @(posedge clk_i or negedge rst_ni)
      begin : address_counter_q
        if(~rst_ni) begin
          addr_cnt_q2 <= '0;
        end
        else if(enable_i | ctrl_i.clear) begin
          if(clear_i | addr_cnt_clear)
            addr_cnt_q2 <= '0;
          else if((conv_i.valid & conv_i.ready) | (addr_cnt_en_q))
            addr_cnt_q2 <= addr_cnt_q;
        end
      end

      always_ff @(posedge clk_i or negedge rst_ni)
      begin : address_counter_accum_q
        if(~rst_ni) begin
          addr_cnt_q2_accum <= '0;
        end
        else if(enable_i) begin
          if(clear_i | ctrl_i.clear | addr_cnt_clear)
            addr_cnt_q2_accum <= '0;
          else if(conv_handshake_q)
            addr_cnt_q2_accum <= addr_cnt_q;
        end
      end
    // end
    // else begin : no_pipe_normquant
    //   assign addr_cnt_q2 = addr_cnt_q;
    // end
  endgenerate

  logic full_accumulation_cnt_en;
  assign full_accumulation_cnt_en = conv_handshake_q;

  // counter of number of total accumulations done
  always_ff @(posedge clk_i or negedge rst_ni)
  begin : accum_counter
    if(~rst_ni) begin
      full_accumulation_cnt <= '0;
    end
    else if(enable_i) begin
      if(clear_i | ctrl_i.clear)
        full_accumulation_cnt <= '0;
      else if(fsm_cs != AQ_ACCUM)
        full_accumulation_cnt <= '0;
      else if(full_accumulation_cnt_en)
        full_accumulation_cnt <= full_accumulation_cnt + 1;
    end
  end

  logic qw_accumulation_cnt_en;
  assign qw_accumulation_cnt_en = conv_handshake_q & (ctrl_i.qw != '0);

  // counter of number of qw accumulations done
  always_ff @(posedge clk_i or negedge rst_ni)
  begin : qw_counter
    if(~rst_ni) begin
      qw_accumulation_cnt <= '0;
    end
    else if(enable_i) begin
      if(clear_i | ctrl_i.clear)
        qw_accumulation_cnt <= '0;
      else if(fsm_cs != AQ_ACCUM)
        qw_accumulation_cnt <= '0;
      else if(qw_accumulation_cnt_en) begin
        if(qw_accumulation_cnt != ctrl_i.qw-1)
          qw_accumulation_cnt <= qw_accumulation_cnt + 1;
        else
          qw_accumulation_cnt <= '0;
      end
    end
  end

  // ========================================================================
  // FSM Registers
  // ========================================================================
  always_ff @(posedge clk_i or negedge rst_ni)
  begin : fsm_seq
    if(~rst_ni) begin
      fsm_cs <= AQ_IDLE;
    end
    else if(enable_i) begin
      if(clear_i)
        fsm_cs <= AQ_IDLE;
      else
        fsm_cs <= fsm_ns;
    end
  end

  // ========================================================================
  // FSM Code
  // ========================================================================

  always_comb
  begin : fsm_out_comb

    addr_cnt_en_d = 1'b0;
    addr_cnt_clear = 1'b0;

    waddr = '0;
    raddr = '0;
    wdata = '0;
    wdata_wide = '0;
    we = '0;
    we_all = '0;
    wide_enable = '0;

    // addr_cnt en/clear
    case(fsm_cs)
      AQ_IDLE, AQ_ACCUM_DONE, AQ_NORMQUANT_SHIFT, AQ_NORMQUANT_TOBIAS, AQ_NORMQUANT_DONE, AQ_STREAMIN_DONE, AQ_STREAMOUT_DONE : begin
        addr_cnt_clear = 1'b1;
      end
      AQ_ACCUM : begin
        addr_cnt_en_d = (ctrl_i.qw == '0) ? conv_i.valid & conv_i.ready :
                                            conv_handshake_q & ((qw_accumulation_cnt == ctrl_i.qw-2) | (ctrl_i.weight_offset & ctrl_i.depthwise));
      end
      AQ_NORMQUANT : begin
        addr_cnt_en_d = norm_i.valid; // normalization and stream-in use the same stream?
      end
      AQ_NORMQUANT_BIAS : begin
        addr_cnt_en_d = norm_i.valid; // normalization and stream-in use the same stream?
      end
      AQ_STREAMIN : begin
        addr_cnt_en_d = streamin_i.valid & streamin_i.ready;
      end
      AQ_STREAMOUT : begin
        addr_cnt_en_d = conv_o.valid & conv_o.ready;
      end
    endcase

    // selector for addresses
    if(fsm_cs == AQ_ACCUM || fsm_cs == AQ_ACCUM_DONE) begin
      waddr = waddr_accum;
      raddr = raddr_accum;
      wdata = wdata_accum;
      wdata_wide = wdata_wide_accum;
      we_all = 1'b0;
      we = (ctrl_i.depthwise | ~ctrl_i.weight_offset) & conv_handshake_q;
    end
    else if(fsm_cs == AQ_NORMQUANT) begin
      waddr = waddr_normquant;
      raddr = raddr_normquant;
      wdata = wdata_normquant;
      wdata_wide = wdata_wide_normquant;
      we = addr_cnt_en_q;
      we_all = 1'b0;
      wide_enable = ctrl_i.norm_mode==NE16_MODE_8B     ? 8'h0f << { addr_cnt_q2[0], 2'b0 } :
                    ctrl_i.norm_mode==NE16_MODE_16B    ? 8'h03 << { addr_cnt_q2[1:0], 1'b0 } : '0;
    end
    else if(fsm_cs == AQ_NORMQUANT_BIAS) begin
      waddr = waddr_normquant_bias;
      raddr = raddr_normquant_bias;
      wdata = wdata_normquant_bias;
      wdata_wide = wdata_wide_normquant_bias;
      we = norm_i.valid & norm_i.ready;
      we_all = 1'b0;
      wide_enable = '1;
    end
    else if(fsm_cs == AQ_STREAMIN || fsm_cs == AQ_STREAMIN_DONE) begin
      waddr = waddr_streamin;
      raddr = raddr_streamin;
      wdata = wdata_streamin;
      wdata_wide = wdata_wide_streamin;
      we = streamin_i.valid & streamin_i.ready;
      wide_enable = '1;
    end
    else begin // (fsm_cs == AQ_STREAMOUT || fsm_cs == AQ_STREAMOUT_DONE) + remaining cases
      waddr = waddr_streamout;
      raddr = raddr_streamout;
      wdata = wdata_streamout;
      wdata_wide = wdata_wide_streamout;
      wide_enable = '1;
    end
  end

  logic [VLEN_CNT_SIZE-1:0] norm_bias_lim;
  assign norm_bias_lim = ctrl_i.bias_len > 24 ? 3 :
                         ctrl_i.bias_len > 16 ? 2 :
                         ctrl_i.bias_len > 8  ? 1 : 0;

  always_comb
  begin : fsm_ns_comb
    fsm_ns = fsm_cs;

    case(fsm_cs)

      AQ_IDLE, AQ_ACCUM_DONE, AQ_NORMQUANT_DONE, AQ_STREAMIN_DONE, AQ_STREAMOUT_DONE : begin
        if(ctrl_i.goto_accum) begin
          fsm_ns = AQ_ACCUM;
        end
        else if(ctrl_i.goto_normquant) begin
          if(ctrl_i.norm_option_shift)
            fsm_ns = AQ_NORMQUANT_SHIFT;
          else
            fsm_ns = AQ_NORMQUANT;
        end
        else if(ctrl_i.goto_accum) begin
          fsm_ns = AQ_ACCUM;
        end
        else if(ctrl_i.goto_streamin) begin
          fsm_ns = AQ_STREAMIN;
        end
        else if(ctrl_i.goto_streamout) begin
          fsm_ns = AQ_STREAMOUT;
        end
        else begin
          fsm_ns = AQ_IDLE;
        end
      end

      AQ_ACCUM : begin
        if((full_accumulation_cnt == ctrl_i.full_accumulation_len-1) & full_accumulation_cnt_en) begin
          fsm_ns = AQ_ACCUM_DONE;
        end
      end

      AQ_NORMQUANT_SHIFT : begin
        if(norm_i.valid & norm_i.ready)
          fsm_ns = AQ_NORMQUANT;
      end

      AQ_NORMQUANT : begin
        if(addr_cnt_q2 == ctrl_i.scale_len-1 && addr_cnt_en_q == 1'b1) begin
          if(ctrl_i.norm_option_bias)
            fsm_ns = AQ_NORMQUANT_TOBIAS;
          else
            fsm_ns = AQ_NORMQUANT_DONE;
        end
      end

      AQ_NORMQUANT_TOBIAS : begin
        fsm_ns = AQ_NORMQUANT_BIAS;
      end

      AQ_NORMQUANT_BIAS : begin
        if(addr_cnt_q2 == norm_bias_lim && addr_cnt_en_q == 1'b1) begin
          fsm_ns = AQ_NORMQUANT_DONE;
        end
      end

      AQ_STREAMIN : begin
        if(addr_cnt_q == (ctrl_i.streamout_len/WIDTH_FACTOR + (ctrl_i.streamout_len%WIDTH_FACTOR==0 ? 0 : 1) -1) && (streamin_i.valid & streamin_i.ready)) begin
          fsm_ns = AQ_STREAMIN_DONE;
        end
      end

      AQ_STREAMOUT : begin
        if(ctrl_i.quant_mode == NE16_MODE_8B && addr_cnt_q == 0 && (conv_o.valid & conv_o.ready)) begin
          fsm_ns = AQ_STREAMOUT_DONE;
        end
        else if(ctrl_i.quant_mode == NE16_MODE_32B && addr_cnt_q == (ctrl_i.streamout_len/WIDTH_FACTOR + (ctrl_i.streamout_len%WIDTH_FACTOR==0 ? 0 : 1) -1) && (conv_o.valid & conv_o.ready)) begin
          fsm_ns = AQ_STREAMOUT_DONE;
        end
      end

    endcase
  end

  // ========================================================================
  // General Registers
  // ========================================================================

  // valid conv handshake register
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      conv_handshake_q <= '0;
    end
    else if(clear_i) begin
      conv_handshake_q <= '0;
    end
    else begin
      conv_handshake_q <= (conv_i.valid & conv_i.ready);
    end
  end

  // valid conv handshake register (2 cycles back)
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      conv_handshake_q2 <= '0;
    end
    else if(clear_i) begin
      conv_handshake_q2 <= '0;
    end
    else begin
      conv_handshake_q2 <= conv_handshake_q;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      norm_handshake_q <= '0;
    end
    else if(clear_i) begin
      norm_handshake_q <= '0;
    end
    else begin
      norm_handshake_q <= (norm_i.valid & norm_i.ready);
    end
  end

  // ========================================================================
  // FSM Registers
  // ========================================================================
  assign conv_o.valid     = (fsm_cs == AQ_STREAMOUT) ? 1'b1 : 1'b0;
  assign accumulator_clr  = clear_i | ctrl_i.clear;
  assign flags_o.state    = fsm_cs;
  assign flags_o.addr_cnt_en_q = addr_cnt_en_q;

// `ifndef SYNTHESIS
// `ifndef VERILATOR
//   property p_accumulator_accum;
//     @(posedge clk_i)
//     $past(conv_handshake_q) && $past(fsm_cs)==AQ_ACCUM && ~$past(ctrl_i.weight_offset)
//       |->
//     $past(all_accumulators[waddr]) + $past($signed(conv_i.data), 2) + $past(accumulator_offset_q) == all_accumulators[$past(waddr)];
//   endproperty;

//   property p_accumulator_streamin;
//     @(posedge clk_i)
//     $past(streamin_i.valid & streamin_i.ready) && $past(fsm_cs)==AQ_STREAMIN
//       |->
//     $past(streamin_i.data[ 31:  0]) == all_accumulators[$past(waddr)+0] and
//     $past(streamin_i.data[ 63: 32]) == all_accumulators[$past(waddr)+1] and
//     $past(streamin_i.data[ 95: 64]) == all_accumulators[$past(waddr)+2] and
//     $past(streamin_i.data[127: 96]) == all_accumulators[$past(waddr)+3] and
//     $past(streamin_i.data[159:128]) == all_accumulators[$past(waddr)+4] and
//     $past(streamin_i.data[191:160]) == all_accumulators[$past(waddr)+5] and
//     $past(streamin_i.data[223:192]) == all_accumulators[$past(waddr)+6] and
//     $past(streamin_i.data[255:224]) == all_accumulators[$past(waddr)+7];
//   endproperty;

//   property p_accumulator_streamout_32b(ii);
//     @(posedge clk_i)
//     $past(conv_o.valid & conv_o.ready) && $past(fsm_cs)==AQ_STREAMOUT && ctrl_i.quant_mode == NE16_MODE_32B
//       |->
//     $past(conv_o.data[(ii+1)*32-1:ii*32]) == all_accumulators[$past(raddr, 2)+ii];
//   endproperty;

//   property p_accumulator_streamout_8b(ii);
//     @(posedge clk_i)
//     $past(conv_o.valid & conv_o.ready) && $past(fsm_cs)==AQ_STREAMOUT && ctrl_i.quant_mode == NE16_MODE_8B
//       |->
//     $past(conv_o.data[(ii+1)*8-1:ii*8]) == all_accumulators[$past(raddr, 2)+ii][7:0];
//   endproperty;

//   generate
//     for(genvar ii=0; ii<32; ii++) begin : a_accumulator_streamout_8b_gen
//       assert property(p_accumulator_streamout_8b(ii));
//     end

//     for(genvar ii=0; ii<8; ii++) begin : a_accumulator_streamout_32b_gen
//       assert property(p_accumulator_streamout_32b(ii));
//     end
//   endgenerate

//   // assert property(p_accumulator_accum and p_accumulator_streamin);
// `endif
// `endif

endmodule
