/*
 * ne16_ctrl_fsm.sv
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
import hwpe_ctrl_package::*;
import hci_package::*;

module ne16_ctrl_fsm (
  // global signals
  input  logic             clk_i,
  input  logic             rst_ni,
  input  logic             test_mode_i,
  input  logic             clear_i,
  input  logic             start_i,
  // ctrl & flags
  input  flags_engine_t    flags_engine_i,
  input  flags_streamer_t  flags_streamer_i,
  input  config_ne16_t     config_i,
  output state_ne16_t      state_o,
  output logic             state_change_o,
  input  logic             uloop_ready_i,
  output index_ne16_t      index_o,
  output base_addr_ne16_t  base_addr_o
);

  /* signal declarations */
  state_ne16_t state_d, state_q;
  logic state_change_d, state_change_q;

  ctrl_uloop_t       ctrl_uloop;
  flags_uloop_t      flags_uloop;
  uloop_code_t       code_uloop;
  logic [17:0][31:0] ro_reg;

  index_ne16_t     index_d, index_q;
  index_update_ne16_t index_update_d, index_update_q;
  base_addr_ne16_t base_addr_d, base_addr_q;
  logic streamin_en;

  /* finite state machine */
  always_ff @(posedge clk_i or negedge rst_ni)
  begin : fsm_sequential
    if(~rst_ni) begin
      state_q <= IDLE;
      state_change_q <= '0;
    end
    else if(clear_i) begin
      state_q <= IDLE;
      state_change_q <= '0;
    end
    else begin
      state_q <= state_d;
      state_change_q <= state_change_d;
    end
  end

  always_comb
  begin: fsm_next_state
    state_d = state_q;
    state_change_d = 1'b0;

    case(state_q)

      IDLE: begin
        if(start_i) begin
          state_d = LOAD;
          state_change_d = 1'b1;
        end
      end

      LOAD: begin
        if(flags_engine_i.flags_input_buffer.state == IB_EXTRACT) begin
          state_d = WEIGHTOFFS;
          state_change_d = 1'b1;
        end
      end

      WEIGHTOFFS: begin
        if(flags_engine_i.flags_accumulator[8].state == AQ_ACCUM_DONE) begin
          if(streamin_en) begin
            state_d = STREAMIN;
            state_change_d = 1'b1;
          end
          else begin
            state_d = MATRIXVEC;
            state_change_d = 1'b1;
          end
        end
      end

      STREAMIN: begin
        if(flags_engine_i.flags_accumulator[8].state == AQ_STREAMIN_DONE) begin
          state_d = MATRIXVEC;
          state_change_d = 1'b1;
        end
      end

      MATRIXVEC: begin
        if(flags_engine_i.flags_accumulator[8].state == AQ_ACCUM_DONE) begin
          if(~uloop_ready_i) begin
            state_d = UPDATEIDX_WAIT;
            state_change_d = 1'b1;
          end
          else begin
            state_d = UPDATEIDX;
            state_change_d = 1'b1;
          end
        end
      end

      NORMQUANT_SHIFT: begin
        if(flags_engine_i.flags_accumulator[8].state == AQ_NORMQUANT) begin
          state_d = NORMQUANT;
          state_change_d = 1'b1;
        end
      end

      NORMQUANT: begin
        if(flags_engine_i.flags_accumulator[8].state == AQ_NORMQUANT_BIAS) begin
          state_d = NORMQUANT_BIAS;
          state_change_d = 1'b1;
        end
        else if(~config_i.norm_option_bias & flags_engine_i.flags_accumulator[8].state == AQ_NORMQUANT_DONE) begin
          state_d = STREAMOUT;
          state_change_d = 1'b1;
        end
      end

      NORMQUANT_BIAS: begin
        if(flags_engine_i.flags_accumulator[8].state == AQ_NORMQUANT_DONE) begin
          state_d = STREAMOUT;
          state_change_d = 1'b1;
        end
      end

      STREAMOUT: begin
        if(flags_engine_i.flags_accumulator[8].state == AQ_STREAMOUT_DONE) begin
          if(flags_uloop.done) begin
            state_d = DONE;
            state_change_d = 1'b1;
          end
          else begin
            state_d = STREAMOUT_DONE;
            state_change_d = 1'b1;
          end
        end
      end

      STREAMOUT_DONE: begin
        if(flags_streamer_i.tcdm_fifo_empty) begin
          state_d = LOAD;
          state_change_d = 1'b1;
        end
      end

      UPDATEIDX_WAIT: begin
        if(uloop_ready_i) begin
          state_d = UPDATEIDX;
          state_change_d = 1'b1;
        end
      end

      UPDATEIDX: begin
        if(flags_uloop.valid) begin
          if((config_i.filter_mode != NE16_FILTER_MODE_3X3_DW) && (flags_uloop.idx_update == 4'b0001) && (~flags_uloop.done)) begin
            state_d = LOAD;
            state_change_d = 1'b1;
          end
          else if(~config_i.streamout_quant) begin
            state_d = STREAMOUT;
            state_change_d = 1'b1;
          end
          else if(config_i.norm_option_shift) begin
            state_d = NORMQUANT_SHIFT;
            state_change_d = 1'b1;
          end
          else begin
            state_d = NORMQUANT;
            state_change_d = 1'b1;
          end
        end
      end

      DONE: begin
        state_d = IDLE;
        state_change_d = 1'b1;
      end

    endcase
  end

  /* uloop instantiation */
  always_comb
  begin
    code_uloop = '0;
    code_uloop.code     = config_i.filter_mode == NE16_FILTER_MODE_3X3_DW ? ULOOP_CODE_DEPTHWISE   : ULOOP_CODE_NORMAL;
    code_uloop.loops    = config_i.filter_mode == NE16_FILTER_MODE_3X3_DW ? ULOOP_LOOPS_DEPTHWISE  : ULOOP_LOOPS_NORMAL;
    code_uloop.range[0] = config_i.filter_mode == NE16_FILTER_MODE_3X3_DW ? config_i.subtile_nb_wo : config_i.subtile_nb_ki;
    code_uloop.range[1] = config_i.filter_mode == NE16_FILTER_MODE_3X3_DW ? config_i.subtile_nb_ho : config_i.subtile_nb_wo;
    code_uloop.range[2] = config_i.filter_mode == NE16_FILTER_MODE_3X3_DW ? config_i.subtile_nb_ko : config_i.subtile_nb_ho;
    code_uloop.range[3] = config_i.filter_mode == NE16_FILTER_MODE_3X3_DW ? 1                      : config_i.subtile_nb_ko;
  end

  assign ctrl_uloop.enable = (state_q == UPDATEIDX) & ~flags_uloop.valid;
  assign ctrl_uloop.clear  = (state_q == IDLE);
  assign ctrl_uloop.ready  = config_i.filter_mode == NE16_FILTER_MODE_1X1 ? 1'b1 : uloop_ready_i;

  hwpe_ctrl_uloop #(
    .LENGTH    ( 32 ),
    .NB_LOOPS  ( 4  ),
    .NB_RO_REG ( 18 ),
    .NB_REG    ( 4  ),
    .REG_WIDTH ( 32 ),
    .CNT_WIDTH ( 16 ),
    .SHADOWED  ( 1  )
`ifndef SYNTHESIS
    ,
    .DEBUG_DISPLAY ( 0 )
`endif
  ) i_uloop (
    .clk_i            ( clk_i                      ),
    .rst_ni           ( rst_ni                     ),
    .test_mode_i      ( test_mode_i                ),
    .clear_i          ( clear_i | ctrl_uloop.clear ),
    .ctrl_i           ( ctrl_uloop                 ),
    .flags_o          ( flags_uloop                ),
    .uloop_code_i     ( code_uloop                 ),
    .registers_read_i ( ro_reg                     )
  );

  assign ro_reg[NE16_ULOOP_RO_WEIGHTS_KOM_ITER]       = config_i.uloop_iter.weights_kom_iter;
  assign ro_reg[NE16_ULOOP_RO_WEIGHTS_KIM_ITER]       = config_i.uloop_iter.weights_kim_iter;
  assign ro_reg[NE16_ULOOP_RO_WEIGHTS_KOM_RESET_ITER] = config_i.uloop_iter.weights_kom_reset_iter;
  assign ro_reg[NE16_ULOOP_RO_WEIGHTS_KIM_RESET_ITER] = config_i.uloop_iter.weights_kim_reset_iter;
  assign ro_reg[NE16_ULOOP_RO_INFEAT_KIM_ITER]        = config_i.uloop_iter.infeat_kim_iter;
  assign ro_reg[NE16_ULOOP_RO_INFEAT_WOM_ITER]        = config_i.uloop_iter.infeat_wom_iter;
  assign ro_reg[NE16_ULOOP_RO_INFEAT_HOM_ITER]        = config_i.uloop_iter.infeat_hom_iter;
  assign ro_reg[NE16_ULOOP_RO_INFEAT_KIM_RESET_ITER]  = config_i.uloop_iter.infeat_kim_reset_iter;
  assign ro_reg[NE16_ULOOP_RO_INFEAT_WOM_RESET_ITER]  = config_i.uloop_iter.infeat_wom_reset_iter;
  assign ro_reg[NE16_ULOOP_RO_INFEAT_HOM_RESET_ITER]  = config_i.uloop_iter.infeat_hom_reset_iter;
  assign ro_reg[NE16_ULOOP_RO_OUTFEAT_WOM_ITER]       = config_i.uloop_iter.outfeat_wom_iter;
  assign ro_reg[NE16_ULOOP_RO_OUTFEAT_HOM_ITER]       = config_i.uloop_iter.outfeat_hom_iter;
  assign ro_reg[NE16_ULOOP_RO_OUTFEAT_KOM_ITER]       = config_i.uloop_iter.outfeat_kom_iter;
  assign ro_reg[NE16_ULOOP_RO_OUTFEAT_WOM_RESET_ITER] = config_i.uloop_iter.outfeat_wom_reset_iter;
  assign ro_reg[NE16_ULOOP_RO_OUTFEAT_HOM_RESET_ITER] = config_i.uloop_iter.outfeat_hom_reset_iter;
  assign ro_reg[NE16_ULOOP_RO_OUTFEAT_KOM_RESET_ITER] = config_i.uloop_iter.outfeat_kom_reset_iter;
  assign ro_reg[NE16_ULOOP_RO_SCALE_KOM_ITER]         = config_i.uloop_iter.scale_kom_iter;
  assign ro_reg[NE16_ULOOP_RO_ZERO]                   = '0;

  /* index registers */
  logic index_sample_en;
  assign index_sample_en = ((state_d == WEIGHTOFFS & config_i.filter_mode==NE16_FILTER_MODE_3X3_DW) || state_d == LOAD || state_d == STREAMOUT_DONE) & state_change_d;
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      index_q        <= '0;
      index_update_q <= '0;
      base_addr_q    <= '0;
    end
    else if(clear_i) begin
      index_q        <= '0;
      index_update_q <= '0;
      base_addr_q    <= '0;
    end
    else if(index_sample_en) begin // commit indeces when loading
      index_q        <= index_d;
      index_update_q <= index_update_d;
      base_addr_q    <= base_addr_d;
    end
  end

  /* FSM output binding */
  assign state_o        = state_d;
  assign state_change_o = state_change_d;

  assign index_d.k_out_major = config_i.filter_mode==NE16_FILTER_MODE_3X3_DW ? flags_uloop.idx[2] : flags_uloop.idx[3];
  assign index_d.i_major     = config_i.filter_mode==NE16_FILTER_MODE_3X3_DW ? flags_uloop.idx[1] : flags_uloop.idx[2];
  assign index_d.j_major     = config_i.filter_mode==NE16_FILTER_MODE_3X3_DW ? flags_uloop.idx[0] : flags_uloop.idx[1];
  assign index_d.k_in_major  = config_i.filter_mode==NE16_FILTER_MODE_3X3_DW ? flags_uloop.idx[2] : flags_uloop.idx[0];

  assign index_update_d.k_out_major = config_i.filter_mode==NE16_FILTER_MODE_3X3_DW ? flags_uloop.idx_update[2] : flags_uloop.idx_update[3];
  assign index_update_d.i_major     = config_i.filter_mode==NE16_FILTER_MODE_3X3_DW ? flags_uloop.idx_update[1] : flags_uloop.idx_update[2];
  assign index_update_d.j_major     = config_i.filter_mode==NE16_FILTER_MODE_3X3_DW ? flags_uloop.idx_update[0] : flags_uloop.idx_update[1];
  assign index_update_d.k_in_major  = config_i.filter_mode==NE16_FILTER_MODE_3X3_DW ? flags_uloop.idx_update[2] : flags_uloop.idx_update[0];

  assign base_addr_d.weights = flags_uloop.offs[NE16_ULOOP_BASE_ADDR_W];
  assign base_addr_d.infeat  = flags_uloop.offs[NE16_ULOOP_BASE_ADDR_X];
  assign base_addr_d.outfeat = flags_uloop.offs[NE16_ULOOP_BASE_ADDR_Y];
  assign base_addr_d.scale   = flags_uloop.offs[NE16_ULOOP_BASE_ADDR_S];

  assign index_o     = index_sample_en ? index_d     : index_q;
  assign base_addr_o = index_sample_en ? base_addr_d : base_addr_q;

  assign streamin_en = config_i.streamin & ((index_update_d.k_out_major | index_update_d.i_major | index_update_d.j_major) | (index_q.k_out_major=='0 & index_q.k_in_major=='0 & index_q.i_major=='0 & index_q.j_major=='0));

endmodule // ne16_ctrl_fsm
