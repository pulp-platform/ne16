/*
 * ne16_ctrl.sv
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

module ne16_ctrl #(
  parameter int unsigned N_CORES = NR_CORES,
  parameter int unsigned ID      = ID_WIDTH
) (
  // global signals
  input  logic                                  clk_i,
  input  logic                                  rst_ni,
  input  logic                                  test_mode_i,
  output logic                                  clear_o,
  // events
  output logic [N_CORES-1:0][REGFILE_N_EVT-1:0] evt_o,
  output logic                                  busy_o,
  // ctrl & flags
  output ctrl_streamer_t                        ctrl_streamer_o,
  input  flags_streamer_t                       flags_streamer_i,
  output ctrl_engine_t                          ctrl_engine_o,
  input  flags_engine_t                         flags_engine_i,
  // periph slave port
  hwpe_ctrl_intf_periph.slave                   periph
);

  logic start;
  config_ne16_t config_;
  state_ne16_t  state;
  logic         state_change;
  logic uloop_ready_d, uloop_ready_q;
  index_ne16_t  index;
  base_addr_ne16_t base_addr;

  ctrl_slave_t   slave_ctrl;
  flags_slave_t  slave_flags;
  ctrl_regfile_t reg_file;

  ctrl_engine_t   ctrl_engine_d, ctrl_engine_q;
  ctrl_streamer_t ctrl_streamer_d, ctrl_streamer_q;

  /* HWPE controller slave port + register file */
  hwpe_ctrl_slave #(
    .N_CORES        ( N_CORES ),
    .N_CONTEXT      ( 2       ),
    .N_IO_REGS      ( 24      ),
    .N_GENERIC_REGS ( 0       ),
    .ID_WIDTH       ( ID      )
  ) i_slave (
    .clk_i    ( clk_i       ),
    .rst_ni   ( rst_ni      ),
    .clear_o  ( clear_o     ),
    .cfg      ( periph      ),
    .ctrl_i   ( slave_ctrl  ),
    .flags_o  ( slave_flags ),
    .reg_file ( reg_file    )
  );
  assign evt_o = slave_flags.evt;
  always_comb
  begin
    slave_ctrl = '0;
    slave_ctrl.done = (state==DONE) & state_change;
  end
  assign busy_o = slave_flags.is_working;

  /* Main FSM driving the NE16 */
  ne16_ctrl_fsm i_ctrl_fsm (
    .clk_i            ( clk_i            ),
    .rst_ni           ( rst_ni           ),
    .test_mode_i      ( test_mode_i      ),
    .clear_i          ( clear_o          ),
    .start_i          ( start            ),
    .flags_engine_i   ( flags_engine_i   ),
    .flags_streamer_i ( flags_streamer_i ),
    .config_i         ( config_          ),
    .state_o          ( state            ),
    .state_change_o   ( state_change     ),
    .uloop_ready_i    ( uloop_ready_q    ),
    .index_o          ( index            ),
    .base_addr_o      ( base_addr        )
  );

  /* Binding register file <-> configuration */
  assign config_.weights_ptr         = reg_file.hwpe_params[NE16_REG_WEIGHTS_PTR];
  assign config_.infeat_ptr          = reg_file.hwpe_params[NE16_REG_INFEAT_PTR];
  assign config_.outfeat_ptr         = reg_file.hwpe_params[NE16_REG_OUTFEAT_PTR];
  assign config_.scale_ptr           = reg_file.hwpe_params[NE16_REG_SCALE_PTR];
  assign config_.scale_shift_ptr     = reg_file.hwpe_params[NE16_REG_SCALE_SHIFT_PTR];
  assign config_.scale_bias_ptr      = reg_file.hwpe_params[NE16_REG_SCALE_BIAS_PTR];
  assign config_.subtile_nb_ko       = reg_file.hwpe_params[NE16_REG_SUBTILE_NB0] [31:16];
  assign config_.subtile_rem_ko      = reg_file.hwpe_params[NE16_REG_SUBTILE_REM0][31:16];
  assign config_.subtile_nb_ki       = reg_file.hwpe_params[NE16_REG_SUBTILE_NB0] [15: 0];
  assign config_.subtile_rem_ki      = reg_file.hwpe_params[NE16_REG_SUBTILE_REM0][15: 0];
  assign config_.subtile_nb_ho       = reg_file.hwpe_params[NE16_REG_SUBTILE_NB1] [31:16];
  assign config_.subtile_rem_ho      = reg_file.hwpe_params[NE16_REG_SUBTILE_REM1][31:16];
  assign config_.subtile_nb_wo       = reg_file.hwpe_params[NE16_REG_SUBTILE_NB1] [15: 0];
  assign config_.subtile_rem_wo      = reg_file.hwpe_params[NE16_REG_SUBTILE_REM1][15: 0];
  assign config_.subtile_rem_hi      = reg_file.hwpe_params[NE16_REG_SUBTILE_REM2][31:16];
  assign config_.subtile_rem_wi      = reg_file.hwpe_params[NE16_REG_SUBTILE_REM2][15: 0];
  assign config_.infeat_d0_stride    = reg_file.hwpe_params[NE16_REG_INFEAT_D0_STRIDE];
  assign config_.infeat_d1_stride    = reg_file.hwpe_params[NE16_REG_INFEAT_D1_STRIDE];
  assign config_.infeat_d2_stride    = reg_file.hwpe_params[NE16_REG_INFEAT_D2_STRIDE];
  assign config_.weights_d0_stride   = reg_file.hwpe_params[NE16_REG_WEIGHTS_D0_STRIDE];
  assign config_.weights_d1_stride   = reg_file.hwpe_params[NE16_REG_WEIGHTS_D1_STRIDE];
  assign config_.weights_d2_stride   = reg_file.hwpe_params[NE16_REG_WEIGHTS_D2_STRIDE];
  assign config_.outfeat_d0_stride   = reg_file.hwpe_params[NE16_REG_OUTFEAT_D0_STRIDE];
  assign config_.outfeat_d1_stride   = reg_file.hwpe_params[NE16_REG_OUTFEAT_D1_STRIDE];
  assign config_.outfeat_d2_stride   = reg_file.hwpe_params[NE16_REG_OUTFEAT_D2_STRIDE];
  assign config_.padding_top         = reg_file.hwpe_params[NE16_REG_PADDING][31:28];
  assign config_.padding_right       = reg_file.hwpe_params[NE16_REG_PADDING][27:24];
  assign config_.padding_bottom      = reg_file.hwpe_params[NE16_REG_PADDING][23:20];
  assign config_.padding_left        = reg_file.hwpe_params[NE16_REG_PADDING][19:16];
  assign config_.padding_value       = reg_file.hwpe_params[NE16_REG_PADDING][15:0];
  assign config_.weight_offset_scale = reg_file.hwpe_params[NE16_REG_WEIGHT_OFFSET];
  assign config_.filter_mask_top     = reg_file.hwpe_params[NE16_REG_FILTER_MASK][31:24];
  assign config_.filter_mask_right   = reg_file.hwpe_params[NE16_REG_FILTER_MASK][23:16];
  assign config_.filter_mask_bottom  = reg_file.hwpe_params[NE16_REG_FILTER_MASK][15: 8];
  assign config_.filter_mask_left    = reg_file.hwpe_params[NE16_REG_FILTER_MASK][ 7: 0];
  assign config_.norm_option_bias    = reg_file.hwpe_params[NE16_REG_CONFIG0][25];
  assign config_.norm_option_shift   = reg_file.hwpe_params[NE16_REG_CONFIG0][24];
  assign config_.shift_reqnt         = reg_file.hwpe_params[NE16_REG_CONFIG0][20:16];
  assign config_.filter_mode         = reg_file.hwpe_params[NE16_REG_CONFIG0][6:5];
  assign config_.quant_mode          = reg_file.hwpe_params[NE16_REG_CONFIG0][22:21];
  assign config_.norm_mode           = reg_file.hwpe_params[NE16_REG_CONFIG0][13:12];
  assign config_.streamin            = reg_file.hwpe_params[NE16_REG_CONFIG0][14];
  assign config_.streamout_quant     = reg_file.hwpe_params[NE16_REG_CONFIG0][4];
  assign config_.mode_16             = reg_file.hwpe_params[NE16_REG_CONFIG0][3];
  assign config_.mode_linear         = reg_file.hwpe_params[NE16_REG_CONFIG0][7]; // FIXME ne16.cpp
  assign config_.mode_strided        = reg_file.hwpe_params[NE16_REG_CONFIG0][8]; // FIXME ne16.cpp
  assign config_.weight_bits         = {1'b0, reg_file.hwpe_params[NE16_REG_CONFIG0][2:0]} + 1;
  assign config_.use_rounding        = ~reg_file.hwpe_params[NE16_REG_CONFIG0][11];
  assign config_.relu                = ~reg_file.hwpe_params[NE16_REG_CONFIG0][23];
  assign start = slave_flags.start;

  /* norm variables */
  logic [15:0] norm_len;
  logic [31:0] norm_stride;
  logic [15:0] norm_shift_len;
  logic [31:0] norm_shift_stride;
  logic [15:0] norm_bias_len;
  logic [31:0] norm_bias_stride;

  /* runtime parameters */
  logic [15:0] h_size_in;
  logic [15:0] w_size_in;
  logic [15:0] h_size_out;
  logic [15:0] w_size_out;
  logic [15:0] k_in_lim;
  logic [5:0]  k_out_lim;
  logic [31:0] h_size_in_X_w_size_in;
  logic [31:0] h_size_out_X_w_size_out;
  logic [31:0] tot_len_weights;
  logic [15:0] d0_len_weights;
  logic [2:0]  dim_enable_1h_weights;
  logic [15:0] qw_lim;
  logic [31:0] qw_k_out_lim;

  /* 
    These assignments are used to calculate online runtime parameters.
    Some simplification can / should be performed here.
   */
  assign h_size_in  = (index.i_major < config_.subtile_nb_ho-1) || (config_.subtile_rem_hi==0) ? (config_.filter_mode == NE16_FILTER_MODE_1X1 ? 3 : 5) : config_.subtile_rem_hi;
  assign w_size_in  = (index.j_major < config_.subtile_nb_wo-1) || (config_.subtile_rem_wi==0) ? (config_.filter_mode == NE16_FILTER_MODE_1X1 ? 3 : 5) : config_.subtile_rem_wi;
  assign h_size_out = (index.i_major < config_.subtile_nb_ho-1) || (config_.subtile_rem_ho==0) ? 3 : config_.subtile_rem_ho;
  assign w_size_out = (index.j_major < config_.subtile_nb_wo-1) || (config_.subtile_rem_wo==0) ? 3 : config_.subtile_rem_wo;
  assign k_in_lim   = (index.k_in_major < config_.subtile_nb_ki-1) || (config_.subtile_rem_ki==0) ? NE16_TP_IN : (config_.mode_linear && config_.mode_16) ? config_.subtile_rem_ki/2 : config_.subtile_rem_ki;
  assign k_out_lim  = config_.filter_mode == NE16_FILTER_MODE_3X3_DW ? (index.k_out_major == config_.subtile_nb_ko-1) && (config_.subtile_rem_ko != NE16_TP_IN)  && (config_.subtile_rem_ko != 0) ? config_.subtile_rem_ko : NE16_TP_IN :
                                                                       (index.k_out_major == config_.subtile_nb_ko-1) && (config_.subtile_rem_ko != NE16_TP_OUT) && (config_.subtile_rem_ko != 0) ? config_.subtile_rem_ko : NE16_TP_OUT;

  assign h_size_in_X_w_size_in = (((index.i_major <  config_.subtile_nb_ho-1) || (config_.subtile_rem_hi==0)) && ((index.j_major <  config_.subtile_nb_wo-1) || (config_.subtile_rem_wi==0))) ? (config_.filter_mode == NE16_FILTER_MODE_1X1 ? 9                        : 25                       ) :
                                 (((index.i_major <  config_.subtile_nb_ho-1) || (config_.subtile_rem_hi==0)) &&  (index.j_major >= config_.subtile_nb_wo-1)                                ) ? (config_.filter_mode == NE16_FILTER_MODE_1X1 ? 3*config_.subtile_rem_wi : 5*config_.subtile_rem_wi ) :
                                 ( (index.i_major >= config_.subtile_nb_ho-1)                                 && ((index.j_major <  config_.subtile_nb_wo-1) || (config_.subtile_rem_wi==0))) ? (config_.filter_mode == NE16_FILTER_MODE_1X1 ? config_.subtile_rem_hi*3 : config_.subtile_rem_hi*5 ) :
                                                                                                                                                                                                config_.subtile_rem_hi*config_.subtile_rem_wi; // FIXME BEWARE MULTIPLIER!!! --> can it be made static? probably not

  assign h_size_out_X_w_size_out = (((index.i_major <  config_.subtile_nb_ho-1) || (config_.subtile_rem_ho==0)) && ((index.j_major <  config_.subtile_nb_wo-1) || (config_.subtile_rem_wo==0)))  ? 9 :
                                   (((index.i_major <  config_.subtile_nb_ho-1) || (config_.subtile_rem_ho==0)) &&  (index.j_major >= config_.subtile_nb_wo-1)                                )  ? 3*config_.subtile_rem_wo :
                                   ( (index.i_major >= config_.subtile_nb_ho-1)                                 && ((index.j_major <  config_.subtile_nb_wo-1) || (config_.subtile_rem_wo==0)))  ? config_.subtile_rem_ho*3 :
                                                                                                                                                                                                   config_.subtile_rem_ho*config_.subtile_rem_wo; // FIXME BEWARE MULTIPLIER!!! --> make static

  assign qw_lim = ~config_.mode_linear & config_.filter_mode == NE16_FILTER_MODE_1X1 ? 1 : config_.weight_bits;
  assign qw_k_out_lim = qw_lim * k_out_lim; // this uses a non-static multiplier, consider moving it to an own register

  /* 
    uloop iterators: these serial multipliers are used to compute subtiling spatial iteration parameters at runtime:
      input vertical iterator    infeat_hom_iter  <- h_size_out * infeat_d1_stride
      input horizontal iterator  infeat_wom_iter  <- w_size_out * infeat_d0_stride
      output vertical iterator   outfeat_hom_iter <- h_size_out * outfeat_d2_stride
      output horizontal iterator outfeat_wom_iter <- w_size_out * outfeat_d1_stride
    Output iterators rely on d1,d2 strides due to the fact that the d0 stride is used to discriminate 32/8b outputs.
    Input iterators rely on d0,d1 strides instead.
    The products take 16 cycles each (the size of h/w_size_out), at the very start of the NE16 operation.
    The iterators are used by uloop to compute new indeces inside a loop when using automatic subtiling.
   */
  logic [47:0] infeat_hom_prod;
  logic        infeat_hom_valid, infeat_hom_ready;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_infeat_hom_iter_mult (
    .clk_i    ( clk_i                    ),
    .rst_ni   ( rst_ni                   ),
    .clear_i  ( clear_o | (state==IDLE)  ),
    .start_i  ( start                    ),
    .a_i      ( h_size_out               ),
    .b_i      ( config_.infeat_d1_stride ),
    .invert_i ( 1'b0                     ),
    .valid_o  ( infeat_hom_valid         ),
    .ready_o  ( infeat_hom_ready         ),
    .prod_o   ( infeat_hom_prod          )
  );
  assign config_.uloop_iter.infeat_hom_iter = infeat_hom_prod[31:0];

  logic [47:0] infeat_wom_prod;
  logic        infeat_wom_valid, infeat_wom_ready;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_infeat_wom_iter_mult (
    .clk_i   ( clk_i                    ),
    .rst_ni  ( rst_ni                   ),
    .clear_i ( clear_o | (state==IDLE)  ),
    .start_i ( start                    ),
    .a_i     ( w_size_out               ),
    .b_i     ( config_.infeat_d0_stride ),
    .invert_i ( 1'b0                     ),
    .valid_o ( infeat_wom_valid         ),
    .ready_o ( infeat_wom_ready         ),
    .prod_o  ( infeat_wom_prod          )
  );
  assign config_.uloop_iter.infeat_wom_iter = infeat_wom_prod[31:0];

  assign config_.uloop_iter.infeat_kim_iter = config_.mode_linear ? config_.mode_16 ? 512 : 256 : NE16_TP_IN;

  logic [47:0] outfeat_hom_prod;
  logic        outfeat_hom_valid, outfeat_hom_ready;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_outfeat_hom_iter_mult (
    .clk_i   ( clk_i                     ),
    .rst_ni  ( rst_ni                    ),
    .clear_i ( clear_o | (state==IDLE)   ),
    .start_i ( start                     ),
    .a_i     ( h_size_out                ),
    .b_i     ( config_.outfeat_d2_stride ),
    .invert_i ( 1'b0                     ),
    .valid_o ( outfeat_hom_valid         ),
    .ready_o ( outfeat_hom_ready         ),
    .prod_o  ( outfeat_hom_prod          )
  );
  assign config_.uloop_iter.outfeat_hom_iter = outfeat_hom_prod[31:0];

  logic [47:0] outfeat_wom_prod;
  logic        outfeat_wom_valid, outfeat_wom_ready;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_outfeat_wom_iter_mult (
    .clk_i   ( clk_i                     ),
    .rst_ni  ( rst_ni                    ),
    .clear_i ( clear_o | (state==IDLE)   ),
    .start_i ( start                     ),
    .a_i     ( w_size_out                ),
    .b_i     ( config_.outfeat_d1_stride ),
    .invert_i ( 1'b0                     ),
    .valid_o ( outfeat_wom_valid         ),
    .ready_o ( outfeat_wom_ready         ),
    .prod_o  ( outfeat_wom_prod          )
  );
  assign config_.uloop_iter.outfeat_wom_iter = outfeat_wom_prod[31:0];

  /* 
    uloop iterators: subtiling channel iteration parameters are computed depending on the various operating filter_modes/linear operation, mode 16/mode 8.
    The iterators are used by uloop to compute new indeces inside a loop when using automatic subtiling.
   */
  assign config_.uloop_iter.outfeat_kom_iter = config_.filter_mode == NE16_FILTER_MODE_3X3_DW ? (config_.quant_mode == NE16_MODE_32B ? NE16_TP_IN  << 2 : NE16_TP_IN) :
                                                                                                (config_.quant_mode == NE16_MODE_32B ? NE16_TP_OUT << 2 : NE16_TP_OUT);

  assign config_.uloop_iter.weights_kom_iter = config_.mode_linear                            ? config_.weights_d2_stride :
                                               config_.filter_mode == NE16_FILTER_MODE_3X3_DW ? config_.uloop_iter.weights_kim_iter :
                                               config_.filter_mode == NE16_FILTER_MODE_3X3    ? NE16_TP_OUT*config_.subtile_nb_ki*config_.weight_bits * 3*3 * (config_.mode_16 ? 1 : 2) : // this uses a non-static multiplier, consider moving it to an own register
                                                                                                NE16_TP_OUT*config_.subtile_nb_ki*config_.weight_bits * (config_.mode_16 ? 1 : 2);        // this uses a non-static multiplier, consider moving it to an own register

  assign config_.uloop_iter.weights_kim_iter = config_.mode_linear                         ? 32 :
                                               config_.filter_mode == NE16_FILTER_MODE_1X1 ? config_.weight_bits * (config_.mode_16 ? 1 : 2) :
                                                                                             config_.weight_bits * 3*3 * (config_.mode_16 ? 1 : 2);

  /* 
    uloop reset iterators: these serial multipliers are used to compute subtiling reset iteration parameters at runtime:
      weights_kom_reset_iter = (subtile_nb_ko-1) * weights_kom_iter
      weights_kim_reset_iter = (subtile_nb_ki-1) * weights_kim_iter
      infeat_kim_reset_iter  = (subtile_nb_ki-1) * infeat_kim_iter
      infeat_wom_reset_iter  = (subtile_nb_wo-1) * infeat_wom_iter
      infeat_hom_reset_iter  = (subtile_nb_ho-1) * infeat_hom_iter
      outfeat_kom_reset_iter = (subtile_nb_ko-1) * outfeat_kom_iter
      outfeat_wom_reset_iter = (subtile_nb_wo-1) * outfeat_wom_iter
      outfeat_hom_reset_iter = (subtile_nb_ho-1) * outfeat_hom_iter
    The products take 16 cycles each (the size of h/w_size_out), at the very start of the NE16 operation (for weights) or
    immediately after the in/outfeat iterator computation has finished (infeat/outfeat iterators).
    The reset iterators are used by uloop at the end of a loop cycle in order to reset state before switching to an outer
    loop.
   */
  logic [15:0] subtile_nb_ko_neg;
  logic [15:0] subtile_nb_ho_neg;
  logic [15:0] subtile_nb_wo_neg;
  logic [15:0] subtile_nb_ki_neg;
  assign subtile_nb_ko_neg = (config_.subtile_nb_ko-1);
  assign subtile_nb_ho_neg = (config_.subtile_nb_ho-1);
  assign subtile_nb_wo_neg = (config_.subtile_nb_wo-1);
  assign subtile_nb_ki_neg = (config_.subtile_nb_ki-1);

  logic [47:0] weights_kom_reset_prod;
  logic        weights_kom_reset_valid;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_weights_kom_reset_iter_mult (
    .clk_i   ( clk_i                               ),
    .rst_ni  ( rst_ni                              ),
    .clear_i ( clear_o | (state==IDLE)             ),
    .start_i ( start                               ),
    .a_i     ( subtile_nb_ko_neg                   ),
    .b_i     ( config_.uloop_iter.weights_kom_iter ),
    .invert_i ( 1'b1                               ),
    .valid_o ( weights_kom_reset_valid             ),
    .ready_o ( ),
    .prod_o  ( weights_kom_reset_prod              )
  );
  assign config_.uloop_iter.weights_kom_reset_iter = weights_kom_reset_prod[31:0];

  logic [47:0] weights_kim_reset_prod;
  logic        weights_kim_reset_valid;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_weights_kim_reset_iter_mult (
    .clk_i   ( clk_i                               ),
    .rst_ni  ( rst_ni                              ),
    .clear_i ( clear_o | (state==IDLE)             ),
    .start_i ( start                               ),
    .a_i     ( subtile_nb_ki_neg                   ),
    .b_i     ( config_.uloop_iter.weights_kim_iter ),
    .invert_i ( 1'b1                               ),
    .valid_o ( weights_kim_reset_valid             ),
    .ready_o ( ),
    .prod_o  ( weights_kim_reset_prod              )
  );
  assign config_.uloop_iter.weights_kim_reset_iter = weights_kim_reset_prod[31:0];

  logic [47:0] infeat_kim_reset_prod;
  logic        infeat_kim_reset_valid;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_infeat_kim_reset_iter_mult (
    .clk_i   ( clk_i                              ),
    .rst_ni  ( rst_ni                             ),
    .clear_i ( clear_o | (state==IDLE)            ),
    .start_i ( start                              ),
    .a_i     ( subtile_nb_ki_neg                  ),
    .b_i     ( config_.uloop_iter.infeat_kim_iter ),
    .invert_i ( 1'b1                              ),
    .valid_o ( infeat_kim_reset_valid             ),
    .ready_o ( ),
    .prod_o  ( infeat_kim_reset_prod              )
  );
  assign config_.uloop_iter.infeat_kim_reset_iter = infeat_kim_reset_prod[31:0];

  logic [47:0] infeat_wom_reset_prod;
  logic        infeat_wom_reset_valid, infeat_wom_reset_ready;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_infeat_wom_reset_iter_mult (
    .clk_i   ( clk_i                                      ),
    .rst_ni  ( rst_ni                                     ),
    .clear_i ( clear_o | (state==IDLE)                    ),
    .start_i ( infeat_wom_valid & ~infeat_wom_reset_valid & infeat_wom_reset_ready ),
    .a_i     ( subtile_nb_wo_neg                          ),
    .b_i     ( config_.uloop_iter.infeat_wom_iter         ),
    .invert_i ( 1'b1                                      ),
    .valid_o ( infeat_wom_reset_valid                     ),
    .ready_o ( infeat_wom_reset_ready                     ),
    .prod_o  ( infeat_wom_reset_prod                      )
  );
  assign config_.uloop_iter.infeat_wom_reset_iter = infeat_wom_reset_prod[31:0];

  logic [47:0] infeat_hom_reset_prod;
  logic        infeat_hom_reset_valid, infeat_hom_reset_ready;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_infeat_hom_reset_iter_mult (
    .clk_i   ( clk_i                                      ),
    .rst_ni  ( rst_ni                                     ),
    .clear_i ( clear_o | (state==IDLE)                    ),
    .start_i ( infeat_hom_valid & ~infeat_hom_reset_valid & infeat_hom_reset_ready ),
    .a_i     ( subtile_nb_ho_neg                          ),
    .b_i     ( config_.uloop_iter.infeat_hom_iter         ),
    .invert_i ( 1'b1                                      ),
    .valid_o ( infeat_hom_reset_valid                     ),
    .ready_o ( infeat_hom_reset_ready                      ),
    .prod_o  ( infeat_hom_reset_prod                      )
  );
  assign config_.uloop_iter.infeat_hom_reset_iter = infeat_hom_reset_prod[31:0];

  logic [47:0] outfeat_kom_reset_prod;
  logic        outfeat_kom_reset_valid;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_outfeat_kom_reset_iter_mult (
    .clk_i   ( clk_i                               ),
    .rst_ni  ( rst_ni                              ),
    .clear_i ( clear_o | (state==IDLE)             ),
    .start_i ( start                               ),
    .a_i     ( subtile_nb_ko_neg                   ),
    .b_i     ( config_.uloop_iter.outfeat_kom_iter ),
    .invert_i ( 1'b1                               ),
    .valid_o ( outfeat_kom_reset_valid             ),
    .ready_o ( ),
    .prod_o  ( outfeat_kom_reset_prod              )
  );
  assign config_.uloop_iter.outfeat_kom_reset_iter = outfeat_kom_reset_prod[31:0];

  logic [47:0] outfeat_wom_reset_prod;
  logic        outfeat_wom_reset_valid, outfeat_wom_reset_ready;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_outfeat_wom_reset_iter_mult (
    .clk_i   ( clk_i                                        ),
    .rst_ni  ( rst_ni                                       ),
    .clear_i ( clear_o | (state==IDLE)                      ),
    .start_i ( outfeat_wom_valid & ~outfeat_wom_reset_valid & outfeat_wom_reset_ready ),
    .a_i     ( subtile_nb_wo_neg                            ),
    .b_i     ( config_.uloop_iter.outfeat_wom_iter          ),
    .invert_i ( 1'b1                                        ),
    .valid_o ( outfeat_wom_reset_valid                      ),
    .ready_o ( outfeat_wom_reset_ready                      ),
    .prod_o  ( outfeat_wom_reset_prod                       )
  );
  assign config_.uloop_iter.outfeat_wom_reset_iter = outfeat_wom_reset_prod[31:0];

  logic [47:0] outfeat_hom_reset_prod;
  logic        outfeat_hom_reset_valid, outfeat_hom_reset_ready;
  hwpe_ctrl_seq_mult #( .AW(16), .BW(32) ) i_outfeat_hom_reset_iter_mult (
    .clk_i   ( clk_i                                        ),
    .rst_ni  ( rst_ni                                       ),
    .clear_i ( clear_o | (state==IDLE)                      ),
    .start_i ( outfeat_hom_valid & ~outfeat_hom_reset_valid & outfeat_hom_reset_ready ),
    .a_i     ( subtile_nb_ho_neg                            ),
    .b_i     ( config_.uloop_iter.outfeat_hom_iter          ),
    .invert_i ( 1'b1                                        ),
    .valid_o ( outfeat_hom_reset_valid                      ),
    .ready_o ( outfeat_hom_reset_ready                      ),
    .prod_o  ( outfeat_hom_reset_prod                       )
  );
  assign config_.uloop_iter.outfeat_hom_reset_iter = outfeat_hom_reset_prod[31:0];

  assign config_.uloop_iter.scale_kom_iter = 1;

  /* 
    uloop_ready_q is set whenever the uloop parameters have been calculated.
   */
  assign uloop_ready_d = infeat_wom_reset_valid & infeat_hom_reset_valid & infeat_kim_reset_valid & outfeat_wom_reset_valid & outfeat_hom_reset_valid & outfeat_kom_reset_valid; // the others are always computed earlier
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni)
      uloop_ready_q <= '0;
    else if(clear_o)
      uloop_ready_q <= '0;
    else
      uloop_ready_q <= uloop_ready_d;
  end

  /* Streamers binding:
    feat_source   -> infeat
    weight_source -> weights
    conv_sink     -> outfeat
   */

  /*
    infeat source base address is given by the pointer in regfile + the subtiling offset (base_addr) calculated by uloop.
    In linear mode, infeat loads 16x 128-bit words in 8-bit mode, 32x 128-bit words in 16-bit mode. In 1x1 mode, it loads 9x 128-bit words; in 3x3 mode (DW or regular), 25x 128-bit words.
    These are divided in two dimensions:
     - d0: 3 elements in 1x1 mode, 5 in 3x3 mode, 16/32 in linear mode (for 8/16 bit respectively)
     - d1: 3 elements in 1x1 mode, 5 in 3x3 mode, disabled in linear mode
    Strides for d0, d1 are directly propagated from register file.
    Stride for d2 is currently unused.
  */
  assign ctrl_streamer_d.feat_source_ctrl.addressgen_ctrl.base_addr     = config_.infeat_ptr + base_addr.infeat;
  assign ctrl_streamer_d.feat_source_ctrl.addressgen_ctrl.tot_len       = config_.mode_linear ? (config_.mode_16 ? 32 : 16) : config_.filter_mode == NE16_FILTER_MODE_1X1 ? 9 : 25;
  assign ctrl_streamer_d.feat_source_ctrl.addressgen_ctrl.d0_stride     = config_.infeat_d0_stride;
  assign ctrl_streamer_d.feat_source_ctrl.addressgen_ctrl.d0_len        = config_.mode_linear ? (config_.mode_16 ? 32 : 16) : config_.filter_mode == NE16_FILTER_MODE_1X1 ? 3 : 5;
  assign ctrl_streamer_d.feat_source_ctrl.addressgen_ctrl.d1_stride     = config_.infeat_d1_stride;
  assign ctrl_streamer_d.feat_source_ctrl.addressgen_ctrl.d1_len        = config_.mode_linear ? '1 : config_.filter_mode == NE16_FILTER_MODE_1X1 ? 3 : 5;
  assign ctrl_streamer_d.feat_source_ctrl.addressgen_ctrl.d2_stride     = config_.infeat_d2_stride; // currently unused
  assign ctrl_streamer_d.feat_source_ctrl.addressgen_ctrl.dim_enable_1h = config_.mode_linear ? 3'b001 : '1;

  /*
    weight source base address is given by the pointer in regfile + the subtiling offset (base_addr) calculated by uloop.
    In 1x1 mode, weight loads k_out_lim (up to 32x) 144-bit words. In all other modes, it loads weight_bits * k_out_lim (up to 8 * 32x) 144-bit words.
    These are divided in two dimensions:
     - d0: 1 element in 1x1 mode, weight_bits (up to 8x) in other modes
     - d1: disabled in 1x1 mode, k_out_lim (up to 32x) in other modes
    Strides for d0, d1 are directly propagated from register file.
    Stride for d2 is technically propagated to the streamer but not actually used there directly, because d0,d1 exhaust the full iteration.
    However, it is used to calculate weight subtiling iterators.
  */

  assign tot_len_weights       = ~config_.mode_linear & config_.filter_mode == NE16_FILTER_MODE_1X1 ? k_out_lim :
                                                                               config_.weight_bits * k_out_lim; // repeated config_.subtile_nb_ki times
  assign d0_len_weights        = ~config_.mode_linear & config_.filter_mode == NE16_FILTER_MODE_1X1 ? 1      : config_.weight_bits;
  assign dim_enable_1h_weights = ~config_.mode_linear & config_.filter_mode == NE16_FILTER_MODE_1X1 ? 3'b001 : 3'b011;

  assign ctrl_streamer_d.weight_source_ctrl.addressgen_ctrl.base_addr     = config_.weights_ptr + base_addr.weights;
  assign ctrl_streamer_d.weight_source_ctrl.addressgen_ctrl.tot_len       = tot_len_weights;
  assign ctrl_streamer_d.weight_source_ctrl.addressgen_ctrl.d0_stride     = config_.weights_d0_stride;
  assign ctrl_streamer_d.weight_source_ctrl.addressgen_ctrl.d0_len        = d0_len_weights;
  assign ctrl_streamer_d.weight_source_ctrl.addressgen_ctrl.d1_stride     = config_.weights_d1_stride;
  assign ctrl_streamer_d.weight_source_ctrl.addressgen_ctrl.d1_len        = ~config_.mode_linear & config_.filter_mode == NE16_FILTER_MODE_1X1 ? 1 : '1;
  assign ctrl_streamer_d.weight_source_ctrl.addressgen_ctrl.d2_stride     = config_.weights_d2_stride;
  assign ctrl_streamer_d.weight_source_ctrl.addressgen_ctrl.dim_enable_1h = dim_enable_1h_weights;

  /*
    outfeat sink base address is given by the pointer in regfile + the subtiling offset (base_addr) calculated by uloop.
    The total length is given by the output quantization mode and by the number of output channels in the current subtile, 
    according to the following rule:
     - 8-bit  quant                       --> length = 9
     - 32-bit quant, k_out_lim in [25,32] --> length = 36
     - 32-bit quant, k_out_lim in [17,24] --> length = 27
     - 32-bit quant, k_out_lim in [9,16]  --> length = 18
     - 32-bit quant, k_out_lim in [1,8]   --> length = 9
    Notice that even if the subtile is not spatially full due to remainders or padding, all column accumulators are streamed
    out none the less -- disabling the write enables for columns that are not actually used. This also applies to linear mode,
    which accumulates outputs only on the first column.
    These are divided in three dimensions:
     - d0: total length divided by 9
     - d1: 3 elements
     - d2: 3 elements
    Strides for d0, d1, d2 are directly propagated from register file.
    Notice that these parameters are also used for streamin, not only streamout.
  */

  logic [15:0] w_size_out_with_strb;              // currently using a nil'd strobe to remove outputs --> constant w_size_out
  logic [31:0] h_size_out_X_w_size_out_with_strb; // currently using a nil'd strobe to remove outputs --> constant h_size_out / w_size_out
  assign w_size_out_with_strb = 3;
  assign h_size_out_X_w_size_out_with_strb = (config_.quant_mode == NE16_MODE_8B) || (k_out_lim <= 8)  ? 9  :
                                                                                     (k_out_lim <= 16) ? 18 :
                                                                                     (k_out_lim <= 24) ? 27 : 36;
  assign ctrl_streamer_d.conv_sink_ctrl.addressgen_ctrl.base_addr     = config_.outfeat_ptr + base_addr.outfeat;
  assign ctrl_streamer_d.conv_sink_ctrl.addressgen_ctrl.tot_len       = h_size_out_X_w_size_out_with_strb;
  assign ctrl_streamer_d.conv_sink_ctrl.addressgen_ctrl.d0_stride     = config_.outfeat_d0_stride;
  assign ctrl_streamer_d.conv_sink_ctrl.addressgen_ctrl.d0_len        = (config_.quant_mode == NE16_MODE_32B) ? (k_out_lim/8 > 0 ? k_out_lim/8 + (k_out_lim%8==0 ? 0 : 1) : 1) : 1;
  assign ctrl_streamer_d.conv_sink_ctrl.addressgen_ctrl.d1_stride     = config_.outfeat_d1_stride;
  assign ctrl_streamer_d.conv_sink_ctrl.addressgen_ctrl.d1_len        = w_size_out_with_strb;
  assign ctrl_streamer_d.conv_sink_ctrl.addressgen_ctrl.d2_stride     = config_.outfeat_d2_stride;
  assign ctrl_streamer_d.conv_sink_ctrl.addressgen_ctrl.dim_enable_1h = '1;
  assign ctrl_streamer_d.streamin_source_ctrl.addressgen_ctrl = ctrl_streamer_d.conv_sink_ctrl; // streamin is tied to streamout

  /*
    scale/bias source base address is given by the pointer in regfile + the subtiling offset (base_addr) calculated by uloop.
    It is used for three distinct parameters: scale, bias, and shift.

    Scale:
    The total length of scale is given by the output scaling mode and by the number of output channels in the current subtile, 
    according to the following rule:
     - 8-bit  scaling                       --> length = 1
     - 16-bit scaling, k_out_lim in [17,32] --> length = 2
     - 16-bit scaling, k_out_lim in [1,16]  --> length = 1
     - 32-bit scaling, k_out_lim in [25,32] --> length = 4
     - 32-bit scaling, k_out_lim in [17,24] --> length = 3
     - 32-bit scaling, k_out_lim in [9,16]  --> length = 2
     - 32-bit scaling, k_out_lim in [1,8]   --> length = 1
    These are single-dimension, with d0 stride statically set to 32.
    
    Shift:
    The total length of shift is 1 (if norm_option_shift is set).

    Bias:
    The total length of bias is given by the number of output channels in the current subtile (if norm_option_bias is set):
     - k_out_lim in [25,32] --> length = 4
     - k_out_lim in [17,24] --> length = 3
     - k_out_lim in [9,16]  --> length = 2
     - k_out_lim in [1,8]   --> length = 1
    The access pattern is single-dimension, with d0 stride statically set to 32.
  */

  assign norm_len    = (config_.norm_mode == NE16_MODE_8B)                     ? 1 :
                       (config_.norm_mode == NE16_MODE_16B && k_out_lim <= 16) ? 1 :
                       (config_.norm_mode == NE16_MODE_16B)                    ? 2 :
                       (config_.norm_mode == NE16_MODE_32B && k_out_lim <= 8)  ? 1 :
                       (config_.norm_mode == NE16_MODE_32B && k_out_lim <= 16) ? 2 :
                       (config_.norm_mode == NE16_MODE_32B && k_out_lim <= 24) ? 3 : 4;
  assign norm_stride = 32;
  assign norm_shift_len    = config_.norm_option_shift ? 1 : 0;
  assign norm_shift_stride = config_.norm_option_shift ? (config_.filter_mode == NE16_FILTER_MODE_3X3_DW ? NE16_TP_IN/4 : NE16_TP_OUT/4) : 0;
  assign norm_bias_len    = config_.norm_option_bias ? (k_out_lim <= 8  ? 1 :
                                                        k_out_lim <= 16 ? 2 :
                                                        k_out_lim <= 24 ? 3 : 4) : 0;
  assign norm_bias_stride = config_.norm_option_bias ? 32 : 0;

  logic [7:0] scale_base_mul;
  logic [7:0] scale_bias_base_mul;
  logic [7:0] scale_shift_base_mul;
  assign scale_base_mul = config_.filter_mode == NE16_FILTER_MODE_3X3_DW ? (config_.norm_mode==NE16_MODE_32B ? 64 :  config_.norm_mode==NE16_MODE_16B ? 32 : 16) :
                                                                           (config_.norm_mode==NE16_MODE_32B ? 128 : config_.norm_mode==NE16_MODE_16B ? 64 : 32);
  assign scale_bias_base_mul = config_.filter_mode == NE16_FILTER_MODE_3X3_DW ? 64 :
                                                                                128;
  assign scale_shift_base_mul = config_.filter_mode == NE16_FILTER_MODE_3X3_DW ? 16 :
                                                                                 32;
  assign ctrl_streamer_d.norm_source_ctrl.addressgen_ctrl.base_addr = state==NORMQUANT       ? config_.scale_ptr       + base_addr.scale * scale_base_mul :
                                                                      state==NORMQUANT_BIAS  ? config_.scale_bias_ptr  + base_addr.scale * scale_bias_base_mul :
                                                                      state==NORMQUANT_SHIFT ? config_.scale_shift_ptr + base_addr.scale * scale_shift_base_mul : '0;
  assign ctrl_streamer_d.norm_source_ctrl.addressgen_ctrl.tot_len   = state==NORMQUANT       ? norm_len :
                                                                      state==NORMQUANT_BIAS  ? norm_bias_len :
                                                                      state==NORMQUANT_SHIFT ? norm_shift_len : '0;
  assign ctrl_streamer_d.norm_source_ctrl.addressgen_ctrl.d0_stride = state==NORMQUANT       ? norm_stride :
                                                                      state==NORMQUANT_BIAS  ? norm_bias_stride :
                                                                      state==NORMQUANT_SHIFT ? norm_shift_stride : '0;
  assign ctrl_streamer_d.norm_source_ctrl.addressgen_ctrl.d0_len        = '0;
  assign ctrl_streamer_d.norm_source_ctrl.addressgen_ctrl.d1_stride     = '0;
  assign ctrl_streamer_d.norm_source_ctrl.addressgen_ctrl.d1_len        = '0;
  assign ctrl_streamer_d.norm_source_ctrl.addressgen_ctrl.d2_stride     = '0;
  assign ctrl_streamer_d.norm_source_ctrl.addressgen_ctrl.dim_enable_1h = 2;

  /*
    Streamers are kick-started in the appropriate state when the state_change bit is 1, indicating
    a state transition (like in a Mealy FSM).
  */
  assign ctrl_streamer_d.feat_source_ctrl.req_start     = (state==LOAD)       & state_change;
  assign ctrl_streamer_d.weight_source_ctrl.req_start   = config_.streamin ? (state==MATRIXVEC & state_change) : (state==WEIGHTOFFS & state_change);
  assign ctrl_streamer_d.norm_source_ctrl.req_start     = (state==NORMQUANT || state==NORMQUANT_BIAS || state==NORMQUANT_SHIFT)  & state_change;
  assign ctrl_streamer_d.conv_sink_ctrl.req_start       = (state==STREAMOUT)  & state_change;
  assign ctrl_streamer_d.streamin_source_ctrl.req_start = (state==STREAMIN)   & state_change;

  /*
    Set the streamer muxes / demuxes according to the current operating state.
  */
  assign ctrl_streamer_d.ld_which_mux_sel = (state==LOAD)                                                         ? LD_FEAT_SEL :
                                            (state==MATRIXVEC || state==WEIGHTOFFS)                               ? LD_WEIGHT_SEL :
                                            (state==NORMQUANT || state==NORMQUANT_BIAS || state==NORMQUANT_SHIFT) ? LD_NORM_SEL :
                                            (state==STREAMIN)                                                     ? LD_STREAMIN_SEL :
                                                                                                                    LD_FEAT_SEL;
  assign ctrl_streamer_d.ld_st_mux_sel    = (state==STREAMOUT || state==STREAMOUT_DONE) ? ST_SEL : LD_SEL;

  /*
    Clear FIFO contents when moving in/out of certain states, to disable spurious transactions that might
    have creeped in.
  */
  assign ctrl_streamer_d.clear_fifo       = (state==LOAD) & state_change;
  assign ctrl_streamer_d.clear_source     = (state==STREAMOUT || state==STREAMOUT_DONE);
  assign ctrl_streamer_d.clear_sink       = (state==LOAD) & state_change;

  /*
    Implicit padding indicates the output padding that is "required" by spatial subtiling residuals.
    Explicit padding is that imposed externally by means of padding configuration registers.
  */
  logic [NE16_INPUT_BUFFER_SIZE-1:0] implicit_padding_map;
  logic [NE16_INPUT_BUFFER_SIZE-1:0] explicit_padding_map;
  logic [9-1:0] filter_mask_map;

  logic [4:0] h_size_in_map;
  logic [4:0] w_size_in_map;

  /*
    implicit_padding_map encodes which of the 5x5 elements in the array are valid (1) and which ones are unused (0).
  */
  assign h_size_in_map = (1 << h_size_in) - 1;
  assign w_size_in_map = (1 << w_size_in) - 1;
  always_comb
  begin
    implicit_padding_map = '1;
    // padding from incomplete output
    implicit_padding_map[24:0] &= {5{w_size_in_map}};
    implicit_padding_map[24:0] &= {{5{h_size_in_map[4]}}, {5{h_size_in_map[3]}}, {5{h_size_in_map[2]}}, {5{h_size_in_map[1]}}, {5{h_size_in_map[0]}}};
  end

  logic [4:0] t_explicit_padding_map;
  logic [4:0] r_explicit_padding_map_r, r_explicit_padding_map;
  logic [4:0] b_explicit_padding_map_r, b_explicit_padding_map;
  logic [4:0] l_explicit_padding_map;

  assign t_explicit_padding_map   = (1 << config_.padding_top) - 1;
  assign r_explicit_padding_map_r = (1 << config_.padding_right) - 1;
  assign r_explicit_padding_map   = {<<{r_explicit_padding_map_r}};
  assign b_explicit_padding_map_r = (1 << config_.padding_bottom) - 1;
  assign b_explicit_padding_map   = {<<{b_explicit_padding_map_r}};
  assign l_explicit_padding_map   = (1 << config_.padding_left) - 1;

  /*
    explicit_padding_map encodes which of the 5x5 elements in the array are padded (0) and which ones are not (1).
  */
  always_comb
  begin
    explicit_padding_map = '0;
    // padding from registers
    if(index.j_major == 0)
      explicit_padding_map[24:0] |= {5{l_explicit_padding_map}};
    if(index.j_major == config_.subtile_nb_wo-1)
      explicit_padding_map[24:0] |= {5{r_explicit_padding_map}};
    if(index.i_major == 0)
      explicit_padding_map[24:0] |= {{5{t_explicit_padding_map[4]}}, {5{t_explicit_padding_map[3]}}, {5{t_explicit_padding_map[2]}}, {5{t_explicit_padding_map[1]}}, {5{t_explicit_padding_map[0]}}};
    if(index.i_major == config_.subtile_nb_ho-1)
      explicit_padding_map[24:0] |= {{5{b_explicit_padding_map[4]}}, {5{b_explicit_padding_map[3]}}, {5{b_explicit_padding_map[2]}}, {5{b_explicit_padding_map[1]}}, {5{b_explicit_padding_map[0]}}};
  end

  /*
    Filter masking in the current implementation works like padding.
    Currently, there is no logic to scatter contiguous filters into a filter-masked configuration --> this has to be fixed in further revisions.
  */
  logic [2:0] t_filter_mask_map;
  logic [2:0] r_filter_mask_map_r, r_filter_mask_map;
  logic [2:0] b_filter_mask_map_r, b_filter_mask_map;
  logic [2:0] l_filter_mask_map;

  assign t_filter_mask_map   = (1 << config_.filter_mask_top) - 1;
  assign r_filter_mask_map_r = (1 << config_.filter_mask_right) - 1;
  assign r_filter_mask_map   = {<<{r_filter_mask_map_r}};
  assign b_filter_mask_map_r = (1 << config_.filter_mask_bottom) - 1;
  assign b_filter_mask_map   = {<<{b_filter_mask_map_r}};
  assign l_filter_mask_map   = (1 << config_.filter_mask_left) - 1;

  /*
    filter_mask_map encodes which of the 3x3 filters in the array are filtered (1) and which ones are not (0).
  */
  always_comb
  begin
    filter_mask_map = '0;
    // padding from registers
    filter_mask_map |= {3{l_filter_mask_map}};
    filter_mask_map |= {3{r_filter_mask_map}};
    filter_mask_map |= {{3{t_filter_mask_map[2]}}, {3{t_filter_mask_map[1]}}, {3{t_filter_mask_map[0]}}};
    filter_mask_map |= {{3{b_filter_mask_map[2]}}, {3{b_filter_mask_map[1]}}, {3{b_filter_mask_map[0]}}};
  end

  /* 
    binding of datapath control signals
   */

  // input buffer goes to its LOAD state together with the ctrl FSM LOAD state
  assign ctrl_engine_d.ctrl_input_buffer.goto_load    = (state==LOAD) & state_change; 
  // input buffer goes to its EXTRACT state autonomously, so goto_extract is bound to 0
  assign ctrl_engine_d.ctrl_input_buffer.goto_extract = '0;
  // input buffer goes to its IDLE state when the main FSM enters a state that is not LOAD, WEIGHTOFFS, MATRIXVEC, STREAMIN (or UPDATEIDX, when the 3x3 depthwise mode is used)
  assign ctrl_engine_d.ctrl_input_buffer.goto_idle    = config_.filter_mode == NE16_FILTER_MODE_3X3_DW ? (state!=LOAD && state!=WEIGHTOFFS && state!=MATRIXVEC && state!=STREAMIN && state!=UPDATEIDX) & state_change :
                                                                                                         (state!=LOAD && state!=WEIGHTOFFS && state!=MATRIXVEC && state!=STREAMIN) & state_change;
  // the input buffer load length is essentially aligned to the tot_length of the infeat source, with some overhead for what concerns 1x1 mode (FIXME: should be possible to optimize this).
  assign ctrl_engine_d.ctrl_input_buffer.load_len     = config_.mode_linear ? (config_.mode_16 ? 32 : 16) : config_.filter_mode == NE16_FILTER_MODE_1X1 ? 13 : 25;

  // propagate implicit padding, except for linear mode, and explicit padding
  assign ctrl_engine_d.ctrl_input_buffer.enable_implicit_padding = config_.mode_linear ? '0 : ~implicit_padding_map;
  assign ctrl_engine_d.ctrl_input_buffer.enable_explicit_padding = explicit_padding_map;
  assign ctrl_engine_d.ctrl_input_buffer.explicit_padding_value_lo = config_.padding_value[7:0];
  assign ctrl_engine_d.ctrl_input_buffer.explicit_padding_value_hi = config_.mode_16 ? config_.padding_value[15:8] : config_.padding_value[7:0];

  // in linear mode, the input buffer works similarly to 3x3 mode
  assign ctrl_engine_d.ctrl_input_buffer.filter_mode = config_.mode_linear ? NE16_FILTER_MODE_3X3 : config_.filter_mode;

  // the NE array has 9 columns -- one per each spatial pixel in the output space that it can support (3x3)
  logic [2:0] enable_column_vert, enable_column_horiz;
  logic [8:0] enable_column_strided;
  logic [8:0] enable_column;

  // enable columns depending on the subtile size considering residuals in the horizontal & vertical directions
  assign enable_column_vert  = (1 << h_size_out) - 1;
  assign enable_column_horiz = (1 << w_size_out) - 1;

  // in 2x2 strided mode, apply an explicit mask of pattern
  //   +---+---+---+ 
  //   | 1 | 0 | 1 |
  //   +---+---+---+
  //   | 0 | 0 | 0 |
  //   +---+---+---+
  //   | 1 | 0 | 1 |
  //   +---+---+---+
  assign enable_column_strided = config_.mode_strided ? 9'b101000101 : '1;

  // overall column enable takes into account both horizontal and vertical enables, as well as strided mode
  assign enable_column = {3 {enable_column_horiz}} & {{3{enable_column_vert[2]}}, {3{enable_column_vert[1]}}, {3{enable_column_vert[0]}}} & enable_column_strided;

  // propagate config to NE16 binconv array
  assign ctrl_engine_d.ctrl_binconv_array.weight_offset = state==LOAD | state==WEIGHTOFFS;
  assign ctrl_engine_d.ctrl_binconv_array.filter_mode = config_.filter_mode;
  assign ctrl_engine_d.ctrl_binconv_array.depthwise_len = k_in_lim[4:0];
  assign ctrl_engine_d.ctrl_binconv_array.mode_linear = config_.mode_linear;
  assign ctrl_engine_d.ctrl_binconv_array.mode_16 = config_.mode_16;
  assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.padding_value = config_.padding_value;
  assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.ctrl_block.qw = config_.weight_bits;
  assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.ctrl_block.filter_mode = config_.filter_mode;
  assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.ctrl_block.weight_offset = state==LOAD | state==WEIGHTOFFS;
  assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.ctrl_block.mode_16 = config_.mode_16;
  assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.ctrl_block.mode_linear = config_.mode_linear;
  
  // in linear mode, the first 2 columns (8-bit mode) or the first 4 columns (16-bit mode) are used, ignoring the enable_column calculated above
  assign ctrl_engine_d.ctrl_binconv_array.enable_column = config_.mode_linear ? (config_.mode_16 ? 9'b01111 : 9'b011 ) : enable_column;

  // block-level enables depend on the operating mode -- during WEIGHTOFFS, only 1; in 1x1 mode, as many as the weight_bits are; and in 3x3, all 9 according to the filter_mask_map
  assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.enable_block  = config_.mode_linear ? 8'hff :
                                                                      ((state==LOAD || state==WEIGHTOFFS) && config_.filter_mode == NE16_FILTER_MODE_1X1) ? 1 :
                                                                      (config_.filter_mode == NE16_FILTER_MODE_1X1)                                       ? (1 << config_.weight_bits) - 1 :
                                                                                                                                                           ~filter_mask_map;

  // in linear mode, 8 blocks per column are used, one for each k_in supported -- the following maps these according to the configured k_in_lim
  generate
    for(genvar rr=0; rr<NE16_NR_COLUMN; rr++) begin : enable_block_linear_row_gen
      for(genvar cc=0; cc<NE16_COLUMN_SIZE; cc++) begin : enable_block_linear_col_gen
        localparam i_kin_8bit  = (cc<8 && rr<2) ? rr*8 + cc : -1;
        localparam i_kin_16bit = (cc<8 && rr<4) ? (rr/2)*8 + cc : -1;
        assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.enable_block_linear[rr][cc] = config_.mode_16 ? ((i_kin_16bit != -1 && i_kin_16bit < k_in_lim) ? 1'b1 : 1'b0) :  ((i_kin_8bit != -1 && i_kin_8bit < k_in_lim) ? 1'b1 : 1'b0);
      end
    end
  endgenerate

  // clear array state when entering LOAD or MATRIXVEC
  assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.ctrl_block.clear = (state==LOAD || state==MATRIXVEC) & state_change;

  // inside each block, enable by default all 1x8b MACs in linear mode, or k_in_lim ones otherwise
  assign ctrl_engine_d.ctrl_binconv_array.ctrl_column.ctrl_block.enable_mac = config_.mode_linear ? '1 : (1 << k_in_lim) - 1;

  // enable accumulator ICG cells
  assign ctrl_engine_d.enable_accumulator = config_.mode_linear ? 9'b01 : enable_column;

  // control the accumulator's state or the norm/quant unit's state
  assign ctrl_engine_d.ctrl_accumulator.clear          = (state==IDLE) | (state==STREAMOUT_DONE && state_change==1'b1);
  assign ctrl_engine_d.ctrl_accumulator.clear_offset   = (state==IDLE) | (state==WEIGHTOFFS && state_change==1'b1);
  assign ctrl_engine_d.ctrl_accumulator.goto_normquant = ((state==NORMQUANT & ~config_.norm_option_shift) | (state==NORMQUANT_SHIFT)) & state_change;
  assign ctrl_engine_d.ctrl_accumulator.goto_accum     = (state==MATRIXVEC || state==WEIGHTOFFS) & state_change;
  assign ctrl_engine_d.ctrl_accumulator.goto_streamin  = (state==STREAMIN)  & state_change;
  assign ctrl_engine_d.ctrl_accumulator.goto_streamout = (state==STREAMOUT) & state_change;
  assign ctrl_engine_d.ctrl_accumulator.goto_idle      = (state==DONE)      & state_change;
  assign ctrl_engine_d.ctrl_accumulator.quant_mode     = config_.quant_mode;
  assign ctrl_engine_d.ctrl_accumulator.norm_mode      = config_.norm_mode;
  assign ctrl_engine_d.ctrl_accumulator.sample_shift   = ((state==NORMQUANT) | (state==NORMQUANT_BIAS)) & state_change;
  assign ctrl_engine_d.ctrl_accumulator.depthwise      = config_.filter_mode == NE16_FILTER_MODE_3X3_DW;
  assign ctrl_engine_d.ctrl_accumulator.ctrl_normquant.start             = (state==WEIGHTOFFS || state==NORMQUANT) & state_change;
  assign ctrl_engine_d.ctrl_accumulator.ctrl_normquant.relu              = (state==WEIGHTOFFS)                                  ? 1'b0 :
                                                                           (state==NORMQUANT && config_.norm_option_bias==1'b1) ? 1'b0 :
                                                                                                                                  config_.relu;
  assign ctrl_engine_d.ctrl_accumulator.ctrl_normquant.right_shift       = (state==WEIGHTOFFS)                                  ? 0 :
                                                                           (state==NORMQUANT && config_.norm_option_bias==1'b1) ? '0 :
                                                                                                                                  config_.shift_reqnt;
  assign ctrl_engine_d.ctrl_accumulator.ctrl_normquant.norm_mode         = (state==WEIGHTOFFS)                                  ? NE16_MODE_8B :
                                                                           (state==NORMQUANT && config_.norm_option_bias==1'b1) ? ctrl_engine_d.ctrl_accumulator.norm_mode :
                                                                                                                                  ctrl_engine_d.ctrl_accumulator.norm_mode;
  assign ctrl_engine_d.ctrl_accumulator.ctrl_normquant.quant_mode        = (state==WEIGHTOFFS)                                  ? NE16_MODE_32B :
                                                                           (state==NORMQUANT && config_.norm_option_bias==1'b1) ? NE16_MODE_32B :
                                                                                                                                  ctrl_engine_d.ctrl_accumulator.quant_mode;
  assign ctrl_engine_d.ctrl_accumulator.ctrl_normquant.norm_signed       = (state==WEIGHTOFFS)                                  ? 1'b1 :
                                                                           (state==NORMQUANT && config_.norm_option_bias==1'b1) ? 1'b0 :
                                                                                                                                  1'b0;
  assign ctrl_engine_d.ctrl_accumulator.ctrl_normquant.use_rounding      = (state==WEIGHTOFFS)                                  ? 1'b0 :
                                                                           (state==NORMQUANT && config_.norm_option_bias==1'b1) ? 1'b0 :
                                                                                                                                  config_.use_rounding;
  assign ctrl_engine_d.ctrl_accumulator.ctrl_normquant.use_shifting      = (state==WEIGHTOFFS)                                  ? 1'b0 :
                                                                           (state==NORMQUANT && config_.norm_option_bias==1'b1) ? 1'b0 :
                                                                                                                                  1'b1;
  assign ctrl_engine_d.ctrl_accumulator.full_accumulation_len = config_.filter_mode == NE16_FILTER_MODE_3X3_DW ? state==WEIGHTOFFS ? k_out_lim : qw_k_out_lim :
                                                                                                                 state==WEIGHTOFFS ? 1         : qw_k_out_lim;
  assign ctrl_engine_d.ctrl_accumulator.streamout_len = k_out_lim;
  assign ctrl_engine_d.ctrl_accumulator.scale_len = config_.norm_mode == NE16_MODE_8B  ? k_out_lim/4+(k_out_lim%4==0 ? 0 : 1) :
                                                    config_.norm_mode == NE16_MODE_16B ? k_out_lim/2+(k_out_lim%2==0 ? 0 : 1) : k_out_lim;
  assign ctrl_engine_d.ctrl_accumulator.bias_len = k_out_lim;
  assign ctrl_engine_d.ctrl_accumulator.weight_offset = (state==WEIGHTOFFS) ? 1'b1 : 1'b0;
  assign ctrl_engine_d.ctrl_accumulator.qw = ~config_.mode_linear & config_.filter_mode == NE16_FILTER_MODE_1X1 ? '0 : config_.weight_bits;
  assign ctrl_engine_d.ctrl_accumulator.enable_streamout = config_.mode_linear ? 9'b01 : enable_column;
  assign ctrl_engine_d.ctrl_accumulator.weight_offset_scale = config_.weight_offset_scale;
  assign ctrl_engine_d.ctrl_accumulator.norm_option_bias  = config_.norm_option_bias;
  assign ctrl_engine_d.ctrl_accumulator.norm_option_shift = config_.norm_option_shift;

  // the serializer is used to combine data from multiple columns
  assign ctrl_engine_d.ctrl_serialize.first_stream       = '0;
  assign ctrl_engine_d.ctrl_serialize.clear_serdes_state = '0;
  assign ctrl_engine_d.ctrl_serialize.nb_contig_m1       = (config_.quant_mode == NE16_MODE_32B) ? (k_out_lim/(NE16_MEM_BANDWIDTH/NE16_ACCUM_SIZE) + (k_out_lim%(NE16_MEM_BANDWIDTH/NE16_ACCUM_SIZE)==0 ? 0 : 1) )-1 : 0;
  assign ctrl_engine_d.clear_des = (state != STREAMIN) ? 1'b1 : 1'b0;

  assign ctrl_engine_d.mode_16 = config_.mode_16;
  assign ctrl_engine_d.mode_linear  = config_.mode_linear;

  // engine and streamer configuration is propagated with one cycle of delay
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      ctrl_engine_q <= '0;
    end
    else if(clear_o) begin
      ctrl_engine_q <= '0;
    end
    else begin
      ctrl_engine_q <= ctrl_engine_d;
    end
  end
  assign ctrl_engine_o = ctrl_engine_q;

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      ctrl_streamer_q <= '0;
    end
    else if(clear_o) begin
      ctrl_streamer_q <= '0;
    end
    else begin
      ctrl_streamer_q <= ctrl_streamer_d;
    end
  end
  assign ctrl_streamer_o = ctrl_streamer_q;

endmodule // ne16_ctrl
