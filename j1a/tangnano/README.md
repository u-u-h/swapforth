# This directory contains a port to the Tang Nano device. #

The onboard chip is GW1N-LV1QN48C6/I5, equipped with 1152 LUT4 logic
resources, 1 PLL and 4 Block total 72Kbit SRAM, packaged as
QFN48. There is an additional 64Mbits PSRAM storage.

The board has an onboard 24MHz crystal, 3 RGB LEDs, and a USB
interface (using a CH552 serial-to-SUB chip).

(we'll ignore the 40-pin LCD interface for the moment)

[https://tangnano.sipeed.com/en/](Sipeed web page for the board),
including
[https://tangnano.sipeed.com/assets/tang_nano_pinout_v1.0.0_w5676_h4000_large.png](the
schematic) with port names.


It should synthesize with a recent version of [https://github.com/YosysHQ/yosys](yosys), 
Yosys 0.9+3845 (git sha1 4762cc0) is what was used to develop it.


