# Neural Engine 16-channels
The Neural Engine 16-channels (NE16) is a Deep Neural Network accelerator which uses Hardware Processing Engine (HWPE) concepts [1] and is designed to be integrated in a PULPOpen cluster configuration in combination with the Heterogeneous Cluster Interconnect (HCI). It makes use of the open-source IPs 'hci', 'hwpe-ctrl', and 'hwpe-stream'.

In general the NE16 has built-in HW supports the following features:

- Filters: 1x1, 3x3, depthwise, linear
- Batch normalization
- ReLU
- Activation input bits: 8,16
- Weight bits: 2,3,4,5,6,7,8
- Activation output bits: 8,16,32
- Nr of input channels: arbitrary
- Nr of output channels: arbitrary

The NE16 is a direct derivative of the Reconfigurable Binary Engine (RBE) design https://github.com/pulp-platform/rbe by Gianna Paulin (ETH Zürich) and Francesco Conti (University of Bologna).

## Contributors
 - Francesco Conti, University of Bologna and GreenWaves Technologies (*f.conti@unibo.it*)

## Acknowledgement
The development of NE16 has been funded by GreenWaves Technologies, SAS.

# License
This repository makes use of two licenses:
- for all *software*: Apache License Version 2.0
- for all *hardware*: Solderpad Hardware License Version 0.51

For further information have a look at the license files: `LICENSE.hw`, `LICENSE.sw`

# References
[1] F. Conti, P. Schiavone, and L Benini. "XNOR neural engine: A hardware accelerator IP for 21.6-fJ/op binary neural network inference." IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems 37.11 (2018): 2940-2951.
