/*
 * ne16_normquant_bias.sv
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

module ne16_normquant_bias #(
  parameter int unsigned NADD = 8,
  parameter int unsigned ACC = ne16_package::NE16_ACCUM_SIZE,
  parameter int unsigned QNT = 32,
  parameter int unsigned OUTPUT_REGISTER = 0
) (
  // global signals
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  logic                         test_mode_i,
  // local clear
  input  logic                         clear_i,
  // normalization parameters
  input  logic unsigned [NADD*ACC-1:0] norm_bias_i,
  input  logic unsigned [NADD*8-1:0]   shift_i,
  // accumulation
  input  logic signed   [NADD*ACC-1:0] accumulator_i,
  output logic signed   [NADD*ACC-1:0] accumulator_o,
  // control channel
  input  ne16_package::ctrl_normquant_t  ctrl_i
);

  generate

    logic        [NADD-1:0][ACC-1:0] biased_data;

    for(genvar ii=0; ii<NADD; ii++) begin : biased_data_gen
      assign biased_data[ii] = norm_bias_i[(ii+1)*ACC-1:ii*ACC] + accumulator_i[(ii+1)*ACC-1:ii*ACC];
    end

    for(genvar ii=0; ii<NADD; ii++) begin : shift_sat_gen

      logic [31:0] accumulator_loc;
      ne16_normquant_shifter #(
        .ACC             ( ACC             ),
        .INT             ( ACC             ),
        .OUTPUT_REGISTER ( OUTPUT_REGISTER )
      ) i_shifter (
        .clk_i         ( clk_i                    ),
        .rst_ni        ( rst_ni                   ),
        .test_mode_i   ( test_mode_i              ),
        .clear_i       ( clear_i                  ),
        .data_i        ( biased_data[ii]          ),
        .shift_i       ( shift_i[(ii+1)*8-1:ii*8] ),
        .accumulator_o ( accumulator_loc          ),
        .ctrl_i        ( ctrl_i                   )
      );
      assign accumulator_o[(ii+1)*32-1:ii*32] = accumulator_loc;

    end
  endgenerate

endmodule // ne16_normquant_bias
