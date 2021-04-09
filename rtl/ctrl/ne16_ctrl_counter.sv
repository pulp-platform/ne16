/*
 * ne16_ctrl_counter.sv
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

module ne16_ctrl_counter #(
  parameter int unsigned BITS = 16
) (
  // global signals
  input  logic            clk_i,
  input  logic            rst_ni,
  input  logic            test_mode_i,
  input  logic            clear_i,
  input  logic            enable_i,
  input  logic [BITS-1:0] limit_i,
  output logic [BITS-1:0] count_o,
  output logic            wrap_o
);

  logic [BITS-1:0] cnt_d, cnt_q;
  logic            wrap_d, wrap_q;

  always_ff @(posedge clk_i or negedge rst_ni)
  begin : cnt_ff
    if(~rst_ni) begin
      cnt_q <= '0;
    end
    else if(clear_i) begin
      cnt_q <= '0;
    end
    else if(enable_i) begin
      cnt_q <= cnt_d;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni)
  begin : wrap_ff
    if(~rst_ni) begin
      wrap_q <= '0;
    end
    else if(clear_i) begin
      wrap_q <= '0;
    end
    else if(enable_i) begin
      wrap_q <= wrap_d;
    end
  end

  assign cnt_d  = (cnt_q < limit_i-1) ? cnt_q + 1 : '0;
  assign wrap_d = enable_i & (cnt_q == limit_i-1);
  assign wrap_o = wrap_d; // FIXME warning!!! may introduce a LONG path!

endmodule // ne16_ctrl_counter
