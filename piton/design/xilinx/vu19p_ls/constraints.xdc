# Copyright (c) 2016 Princeton University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Princeton University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# some constraints from the example design
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 8 [current_design]
set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN div-1 [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]
set_property BITSTREAM.CONFIG.SPI_OPCODE 8'h6B [current_design]
set_property CONFIG_MODE SPIx8 [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pulldown [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]

# Clock signals
# set_property -dict {PACKAGE_PIN F31 IOSTANDARD DIFF_SSTL12}        [get_ports "chipset_clk_osc_n"] ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L13N_T2L_N1_GC_QBC_47
# set_property -dict {PACKAGE_PIN G31 IOSTANDARD DIFF_SSTL12}        [get_ports "chipset_clk_osc_p"] ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L13P_T2L_N0_GC_QBC_47

set_property -dict {PACKAGE_PIN CB18 IOSTANDARD DIFF_SSTL12} [get_ports chipset_clk_osc_n];#Programmable clock pair2
set_property -dict {PACKAGE_PIN CB19 IOSTANDARD DIFF_SSTL12} [get_ports chipset_clk_osc_p];#Programmable clock pair2


# ref clock for MIG

# set_property PACKAGE_PIN E12 [ get_ports "mc_clk_p" ]
# set_property IOSTANDARD DIFF_SSTL12 [ get_ports "mc_clk_p" ]
# set_property PACKAGE_PIN D12 [ get_ports "mc_clk_n" ]
# set_property IOSTANDARD DIFF_SSTL12 [ get_ports "mc_clk_n" ]

set_property IOSTANDARD DIFF_SSTL12 [get_ports mc_clk_p];#Programmable clock pair7
set_property PACKAGE_PIN Y52 [get_ports mc_clk_p];#Programmable clock pair7
set_property PACKAGE_PIN Y53 [get_ports mc_clk_n];#Programmable clock pair7
set_property IOSTANDARD DIFF_SSTL12 [get_ports mc_clk_n];#Programmable clock pair7


set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets chipset/clk_mmcm/inst/clkin1_ibufds/O]

# Reset, note that this is active high on this board!!
#set_property -dict {PACKAGE_PIN L19  IOSTANDARD LVCMOS12} [get_ports "sys_rst_n"] ;# Bank  73 VCCO - VCC1V2_FPGA - IO_T1U_N12_73

# set_property -dict {PACKAGE_PIN BC27  IOSTANDARD LVCMOS12} [get_ports "sys_rst_n"] ;# Bank  73 VCCO - VCC1V2_FPGA - IO_T1U_N12_73

# False paths
set_false_path -to [get_cells -hierarchical *afifo_ui_rst_r*]
set_false_path -to [get_cells -hierarchical *ui_clk_sync_rst_r*]
set_false_path -to [get_cells -hierarchical *ui_clk_syn_rst_delayed*]
set_false_path -to [get_cells -hierarchical *init_calib_complete_f*]
set_false_path -to [get_cells -hierarchical *chipset_rst_n*]
# net not instantiated yet
#set_false_path -from [get_clocks chipset_clk_clk_mmcm] -to [get_clocks net_axi_clk_clk_mmcm]
#set_false_path -from [get_clocks net_axi_clk_clk_mmcm] -to [get_clocks chipset_clk_clk_mmcm]

# JTAG is connected to internal JTAG chain via the bscane2 primitive
# Male PMOD1 Header J53
# set_property -dict {PACKAGE_PIN G23 IOSTANDARD LVCMOS12} [get_ports tck_i]   ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L5P_T0U_N8_AD14P_47
# set_property -dict {PACKAGE_PIN J27 IOSTANDARD LVCMOS12} [get_ports td_i]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L4N_T0U_N7_DBC_AD7N_47
# set_property -dict {PACKAGE_PIN K22 IOSTANDARD LVCMOS12} [get_ports td_o]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L4P_T0U_N6_DBC_AD7P_47
# set_property -dict {PACKAGE_PIN J26 IOSTANDARD LVCMOS12} [get_ports tms_i]   ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L3N_T0L_N5_AD15N_47
# set_property -dict {PACKAGE_PIN J29 IOSTANDARD LVCMOS12} [get_ports trst_ni] ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L3P_T0L_N4_AD15P_47
# unused
#set_property -dict {PACKAGE_PIN L31 IOSTANDARD LVCMOS12} [get_ports ]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L2N_T0L_N3_47
#set_property -dict {PACKAGE_PIN M31 IOSTANDARD LVCMOS12} [get_ports ]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L2P_T0L_N2_47
#set_property -dict {PACKAGE_PIN R29 IOSTANDARD LVCMOS12} [get_ports ]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L1N_T0L_N1_DBC_47

set_property -dict {PACKAGE_PIN BJ27 IOSTANDARD LVCMOS12} [get_ports tck_i]   ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L5P_T0U_N8_AD14P_47
set_property -dict {PACKAGE_PIN BE27 IOSTANDARD LVCMOS12} [get_ports td_i]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L4N_T0U_N7_DBC_AD7N_47
set_property -dict {PACKAGE_PIN BF29 IOSTANDARD LVCMOS12} [get_ports td_o]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L4P_T0U_N6_DBC_AD7P_47
set_property -dict {PACKAGE_PIN BE28 IOSTANDARD LVCMOS12} [get_ports tms_i]   ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L3N_T0L_N5_AD15N_47
set_property -dict {PACKAGE_PIN BF26 IOSTANDARD LVCMOS12} [get_ports trst_ni] ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L3P_T0L_N4_AD15P_47
# unused
#set_property -dict {PACKAGE_PIN L31 IOSTANDARD LVCMOS12} [get_ports ]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L2N_T0L_N3_47
#set_property -dict {PACKAGE_PIN M31 IOSTANDARD LVCMOS12} [get_ports ]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L2P_T0L_N2_47
#set_property -dict {PACKAGE_PIN R29 IOSTANDARD LVCMOS12} [get_ports ]    ;# Bank  47 VCCO - VCC1V2_FPGA - IO_L1N_T0L_N1_DBC_47


## To use FTDI FT2232 JTAG
## Add some additional constraints for JTAG signals, set to 10MHz to be on the safe side
create_clock -period 100.000 -name tck_i -waveform {0.000 50.000} [get_ports tck_i]

set_input_delay  -clock tck_i -clock_fall 5 [get_ports td_i    ]
set_input_delay  -clock tck_i -clock_fall 5 [get_ports tms_i   ]
set_output_delay -clock tck_i             5 [get_ports td_o    ]
set_false_path   -from                      [get_ports trst_ni ]

# constrain clock domain crossing
set_max_delay -datapath_only -from [get_clocks -include_generated_clocks chipset_clk_clk_mmcm] -to [get_clocks tck_i] 8.0
set_max_delay -datapath_only -from [get_clocks tck_i] -to [get_clocks -include_generated_clocks chipset_clk_clk_mmcm] 8.0

# accept sub-optimal placement
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets tck_i_IBUF_inst/O]

# SD on female PMOD0 header (Digilent PMOD SD adapter)
# set_property -dict {PACKAGE_PIN AY14 IOSTANDARD LVCMOS18} [get_ports "sd_dat[3]"]  ;# Bank  67 VCCO - VADJ_1V8_FPGA - IO_L10N_T1U_N7_QBC_AD4N_67
# set_property -dict {PACKAGE_PIN AY15 IOSTANDARD LVCMOS18} [get_ports "sd_cmd"]     ;# Bank  67 VCCO - VADJ_1V8_FPGA - IO_L10P_T1U_N6_QBC_AD4P_67
# set_property -dict {PACKAGE_PIN AW15 IOSTANDARD LVCMOS18} [get_ports "sd_dat[0]"]  ;# Bank  67 VCCO - VADJ_1V8_FPGA - IO_L9N_T1L_N5_AD12N_67
# set_property -dict {PACKAGE_PIN AV15 IOSTANDARD LVCMOS18} [get_ports "sd_clk_out"] ;# Bank  67 VCCO - VADJ_1V8_FPGA - IO_L9P_T1L_N4_AD12P_67
# set_property -dict {PACKAGE_PIN AV16 IOSTANDARD LVCMOS18} [get_ports "sd_dat[1]"]  ;# Bank  67 VCCO - VADJ_1V8_FPGA - IO_L8N_T1L_N3_AD5N_67
# set_property -dict {PACKAGE_PIN AU16 IOSTANDARD LVCMOS18} [get_ports "sd_dat[2]"]  ;# Bank  67 VCCO - VADJ_1V8_FPGA - IO_L8P_T1L_N2_AD5P_67
# set_property -dict {PACKAGE_PIN AT15 IOSTANDARD LVCMOS18} [get_ports "sd_cd"]      ;# Bank  67 VCCO - VADJ_1V8_FPGA - IO_L7N_T1L_N1_QBC_AD13N_67
# no reset on this board. this is the write-protect signal.
#set_property -dict {PACKAGE_PIN AT16 IOSTANDARD LVCMOS18} [get_ports "sd_reset"]   ;# Bank  67 VCCO - VADJ_1V8_FPGA - IO_L7P_T1L_N0_QBC_AD13P_67

set_property -dict {PACKAGE_PIN BR27 IOSTANDARD LVCMOS18} [get_ports {sd_dat[3]}]
set_property -dict {PACKAGE_PIN BL27 IOSTANDARD LVCMOS18} [get_ports sd_cmd]
set_property -dict {PACKAGE_PIN BN29 IOSTANDARD LVCMOS18} [get_ports {sd_dat[0]}]
set_property -dict {PACKAGE_PIN BM27 IOSTANDARD LVCMOS18} [get_ports sd_clk_out]
set_property -dict {PACKAGE_PIN BM29 IOSTANDARD LVCMOS18} [get_ports {sd_dat[1]}]
set_property -dict {PACKAGE_PIN BR28 IOSTANDARD LVCMOS18} [get_ports {sd_dat[2]}]
set_property -dict {PACKAGE_PIN BM26 IOSTANDARD LVCMOS18} [get_ports sd_cd]
# no reset on this board. this is the write-protect signal.
#set_property -dict {PACKAGE_PIN BN47 IOSTANDARD LVCMOS18} [get_ports "sd_reset"]   ;# Bank  67 VCCO - VADJ_1V8_FPGA - IO_L7P_T1L_N0_QBC_AD13P_67

# UART
#IO_L11N_T1_SRCC_35 Sch=uart_rxd_out
# set_property -dict {PACKAGE_PIN AW25 IOSTANDARD LVCMOS18} [get_ports "uart_rx"] ;# Bank  64 VCCO - VCC1V8_FPGA - IO_L9P_T1L_N4_AD12P_64
# set_property -dict {PACKAGE_PIN BB21 IOSTANDARD LVCMOS18} [get_ports "uart_tx"] ;# Bank  64 VCCO - VCC1V8_FPGA - IO_L8N_T1L_N3_AD5N_64
# unused
# set_property -dict {PACKAGE_PIN AY25 IOSTANDARD LVCMOS18} [get_ports "uart_cts"] ;# Bank  64 VCCO - VCC1V8_FPGA - IO_L9N_T1L_N5_AD12N_64
# set_property -dict {PACKAGE_PIN BB22 IOSTANDARD LVCMOS18} [get_ports "uart_rts"] ;# Bank  64 VCCO - VCC1V8_FPGA - IO_L8P_T1L_N2_AD5P_64


set_property -dict {PACKAGE_PIN BN14 IOSTANDARD LVCMOS18} [get_ports uart_rx]
set_property -dict {PACKAGE_PIN BN13 IOSTANDARD LVCMOS18} [get_ports uart_tx]
# unused
set_property -dict {PACKAGE_PIN BL14 IOSTANDARD LVCMOS18} [get_ports uart_cts]
set_property -dict {PACKAGE_PIN BL15 IOSTANDARD LVCMOS18} [get_ports uart_rts]

# Switches.
# set_property -dict {PACKAGE_PIN B17 IOSTANDARD LVCMOS12} [get_ports "sw[0]"] ;# Bank  73 VCCO - VCC1V2_FPGA - IO_L19N_T3L_N1_DBC_AD9N_73
# set_property -dict {PACKAGE_PIN G16 IOSTANDARD LVCMOS12} [get_ports "sw[1]"] ;# Bank  73 VCCO - VCC1V2_FPGA - IO_T2U_N12_73
# set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS12} [get_ports "sw[2]"] ;# Bank  73 VCCO - VCC1V2_FPGA - IO_L7N_T1L_N1_QBC_AD13N_73
# set_property -dict {PACKAGE_PIN D21 IOSTANDARD LVCMOS12} [get_ports "sw[3]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_T3U_N12_72


set_property -dict {PACKAGE_PIN U13 IOSTANDARD LVCMOS18} [get_ports {sw[0]}];#S2.1
set_property -dict {PACKAGE_PIN U14 IOSTANDARD LVCMOS18} [get_ports {sw[1]}];#S2.2
# set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS12} [get_ports "sw[2]"] ;# Bank  73 VCCO - VCC1V2_FPGA - IO_L7N_T1L_N1_QBC_AD13N_73
# set_property -dict {PACKAGE_PIN D21 IOSTANDARD LVCMOS12} [get_ports "sw[3]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_T3U_N12_72

set_property IOSTANDARD LVCMOS18 [get_ports {sw[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sw[2]}]

set_property CFGBVS GND [current_design]

set_property IOSTANDARD LVCMOS18 [get_ports sys_rst_n]
set_property PACKAGE_PIN AC13 [get_ports sys_rst_n];#SW2

# LEDs
# set_property -dict {PACKAGE_PIN AT32 IOSTANDARD LVCMOS12} [get_ports "leds[0]"] ;# Bank  40 VCCO - VCC1V2_FPGA - IO_L19N_T3L_N1_DBC_AD9N_40
# set_property -dict {PACKAGE_PIN AV34 IOSTANDARD LVCMOS12} [get_ports "leds[1]"] ;# Bank  40 VCCO - VCC1V2_FPGA - IO_T2U_N12_40
# set_property -dict {PACKAGE_PIN AY30 IOSTANDARD LVCMOS12} [get_ports "leds[2]"] ;# Bank  40 VCCO - VCC1V2_FPGA - IO_L7N_T1L_N1_QBC_AD13N_40
# set_property -dict {PACKAGE_PIN BB32 IOSTANDARD LVCMOS12} [get_ports "leds[3]"] ;# Bank  40 VCCO - VCC1V2_FPGA - IO_T1U_N12_40
# set_property -dict {PACKAGE_PIN BF32 IOSTANDARD LVCMOS12} [get_ports "leds[4]"] ;# Bank  40 VCCO - VCC1V2_FPGA - IO_L1N_T0L_N1_DBC_40
# set_property -dict {PACKAGE_PIN AU37 IOSTANDARD LVCMOS12} [get_ports "leds[5]"] ;# Bank  42 VCCO - VCC1V2_FPGA - IO_T3U_N12_42
# set_property -dict {PACKAGE_PIN AV36 IOSTANDARD LVCMOS12} [get_ports "leds[6]"] ;# Bank  42 VCCO - VCC1V2_FPGA - IO_L19N_T3L_N1_DBC_AD9N_42
# set_property -dict {PACKAGE_PIN BA37 IOSTANDARD LVCMOS12} [get_ports "leds[7]"] ;# Bank  42 VCCO - VCC1V2_FPGA - IO_L13N_T2L_N1_GC_QBC_42

set_property -dict {PACKAGE_PIN AD15 IOSTANDARD LVCMOS18} [get_ports {leds[0]}]; #led35
set_property -dict {PACKAGE_PIN AD16 IOSTANDARD LVCMOS18} [get_ports {leds[1]}]; #led36
set_property -dict {PACKAGE_PIN AD17 IOSTANDARD LVCMOS18} [get_ports {leds[2]}]; #led37
set_property -dict {PACKAGE_PIN V16 IOSTANDARD LVCMOS18} [get_ports {leds[3]}];  #J16.1
set_property -dict {PACKAGE_PIN W15 IOSTANDARD LVCMOS18} [get_ports {leds[4]}];  #J16.2
set_property -dict {PACKAGE_PIN W16 IOSTANDARD LVCMOS18} [get_ports {leds[5]}];  #J16.3
set_property -dict {PACKAGE_PIN Y14 IOSTANDARD LVCMOS18} [get_ports {leds[6]}];  #J16.4
set_property -dict {PACKAGE_PIN Y15 IOSTANDARD LVCMOS18} [get_ports {leds[7]}];  #J16.6

#Buttons
# set_property -dict {PACKAGE_PIN BB24 IOSTANDARD LVCMOS18} [get_ports "btnu"] ;# Bank  64 VCCO - VCC1V8_FPGA - IO_L5P_T0U_N8_AD14P_64
# set_property -dict {PACKAGE_PIN BF22 IOSTANDARD LVCMOS18} [get_ports "btnl"] ;# Bank  64 VCCO - VCC1V8_FPGA - IO_L4N_T0U_N7_DBC_AD7N_64
# set_property -dict {PACKAGE_PIN BE22 IOSTANDARD LVCMOS18} [get_ports "btnd"] ;# Bank  64 VCCO - VCC1V8_FPGA - IO_L4P_T0U_N6_DBC_AD7P_64
# set_property -dict {PACKAGE_PIN BE23 IOSTANDARD LVCMOS18} [get_ports "btnr"] ;# Bank  64 VCCO - VCC1V8_FPGA - IO_L3N_T0L_N5_AD15N_64
# set_property -dict {PACKAGE_PIN BD23 IOSTANDARD LVCMOS18} [get_ports "btnc"] ;# Bank  64 VCCO - VCC1V8_FPGA - IO_L3P_T0L_N4_AD15P_64

set_property -dict {PACKAGE_PIN AA14 IOSTANDARD LVCMOS18} [get_ports "btnu"] ;# Bank  98 J16.7
set_property -dict {PACKAGE_PIN V14 IOSTANDARD LVCMOS18} [get_ports "btnl"] ;# Bank  98 S2.4
set_property -dict {PACKAGE_PIN AC14  IOSTANDARD LVCMOS18} [get_ports "btnd"] ;#SW3
set_property -dict {PACKAGE_PIN AA15 IOSTANDARD LVCMOS18} [get_ports "btnr"] ;# Bank  98 J16.8
set_property -dict {PACKAGE_PIN V13 IOSTANDARD LVCMOS18} [get_ports "btnc"] ;# Bank  98 S2.3

## Ethernet

## not wired up yet...
## NOTUSED? set_property PACKAGE_PIN AK16 [get_ports net_ip2intc_irpt]
## NOTUSED? set_property IOSTANDARD LVCMOS18 [get_ports net_ip2intc_irpt]
## NOTUSED? set_property PULLUP true [get_ports net_ip2intc_irpt]
#set_property PACKAGE_PIN AF12 [get_ports net_phy_mdc]
#set_property IOSTANDARD LVCMOS15 [get_ports net_phy_mdc]
#set_property PACKAGE_PIN AG12 [get_ports net_phy_mdio_io]
#set_property IOSTANDARD LVCMOS15 [get_ports net_phy_mdio_io]
#set_property PACKAGE_PIN AH24 [get_ports net_phy_rst_n]
#set_property IOSTANDARD LVCMOS33 [get_ports net_phy_rst_n]
##set_property -dict { PACKAGE_PIN AK15  IOSTANDARD LVCMOS18 } [get_ports { ETH_PMEB }]; #IO_L1N_T0_32 Sch=eth_pmeb
#set_property PACKAGE_PIN AG10 [get_ports net_phy_rxc]
#set_property IOSTANDARD LVCMOS15 [get_ports net_phy_rxc]
#set_property PACKAGE_PIN AH11 [get_ports net_phy_rxctl]
#set_property IOSTANDARD LVCMOS15 [get_ports net_phy_rxctl]
#set_property PACKAGE_PIN AJ14 [get_ports {net_phy_rxd[0]}]
#set_property IOSTANDARD LVCMOS15 [get_ports {net_phy_rxd[0]}]
#set_property PACKAGE_PIN AH14 [get_ports {net_phy_rxd[1]}]
#set_property IOSTANDARD LVCMOS15 [get_ports {net_phy_rxd[1]}]
#set_property PACKAGE_PIN AK13 [get_ports {net_phy_rxd[2]}]
#set_property IOSTANDARD LVCMOS15 [get_ports {net_phy_rxd[2]}]
#set_property PACKAGE_PIN AJ13 [get_ports {net_phy_rxd[3]}]
#set_property IOSTANDARD LVCMOS15 [get_ports {net_phy_rxd[3]}]
#set_property PACKAGE_PIN AE10 [get_ports net_phy_txc]
#set_property IOSTANDARD LVCMOS15 [get_ports net_phy_txc]
#set_property PACKAGE_PIN AJ12 [get_ports {net_phy_txd[0]}]
#set_property IOSTANDARD LVCMOS15 [get_ports {net_phy_txd[0]}]
#set_property PACKAGE_PIN AK11 [get_ports {net_phy_txd[1]}]
#set_property IOSTANDARD LVCMOS15 [get_ports {net_phy_txd[1]}]
#set_property PACKAGE_PIN AJ11 [get_ports {net_phy_txd[2]}]
#set_property IOSTANDARD LVCMOS15 [get_ports {net_phy_txd[2]}]
#set_property PACKAGE_PIN AK10 [get_ports {net_phy_txd[3]}]
#set_property IOSTANDARD LVCMOS15 [get_ports {net_phy_txd[3]}]
#set_property PACKAGE_PIN AK14 [get_ports net_phy_txctl]
#set_property IOSTANDARD LVCMOS15 [get_ports net_phy_txctl]


## DDR4

# we only use 64 bit out of the 80bit available

# set_property PACKAGE_PIN N20      [get_ports "ddr_reset_n"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L1N_T0L_N1_DBC_72
# set_property PACKAGE_PIN E14      [get_ports "ddr_ck_n"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L16N_T2U_N7_QBC_AD3N_71
# set_property PACKAGE_PIN F14      [get_ports "ddr_ck_p"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L16P_T2U_N6_QBC_AD3P_71
# set_property PACKAGE_PIN H13      [get_ports "ddr_bg"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L15P_T2L_N4_AD11P_71
# set_property PACKAGE_PIN E13      [get_ports "ddr_act_n"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L14N_T2L_N3_GC_71
# set_property PACKAGE_PIN F13      [get_ports "ddr_cs_n"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L14P_T2L_N2_GC_71
# set_property PACKAGE_PIN C8       [get_ports "ddr_odt"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L7N_T1L_N1_QBC_AD13N_71
# set_property PACKAGE_PIN A10      [get_ports "ddr_cke"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_T1U_N12_71

# set_property PACKAGE_PIN G15      [get_ports "ddr_ba[0]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L17P_T2U_N8_AD10P_71
# set_property PACKAGE_PIN G13      [get_ports "ddr_ba[1]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L15N_T2L_N5_AD11N_71

# set_property PACKAGE_PIN G15      [get_ports "ddr_bg[0]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L17P_T2U_N8_AD10P_71
# set_property PACKAGE_PIN G13      [get_ports "ddr_bg[1]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L15N_T2L_N5_AD11N_71

# set_property PACKAGE_PIN D14      [get_ports "ddr_addr[0]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_T3U_N12_71
# set_property PACKAGE_PIN B15      [get_ports "ddr_addr[1]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L24N_T3U_N11_71
# set_property PACKAGE_PIN B16      [get_ports "ddr_addr[2]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L24P_T3U_N10_71
# set_property PACKAGE_PIN C14      [get_ports "ddr_addr[3]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L23N_T3U_N9_71
# set_property PACKAGE_PIN C15      [get_ports "ddr_addr[4]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L23P_T3U_N8_71
# set_property PACKAGE_PIN A13      [get_ports "ddr_addr[5]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L22N_T3U_N7_DBC_AD0N_71
# set_property PACKAGE_PIN A14      [get_ports "ddr_addr[6]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L22P_T3U_N6_DBC_AD0P_71
# set_property PACKAGE_PIN A15      [get_ports "ddr_addr[7]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L21N_T3L_N5_AD8N_71
# set_property PACKAGE_PIN A16      [get_ports "ddr_addr[8]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L21P_T3L_N4_AD8P_71
# set_property PACKAGE_PIN B12      [get_ports "ddr_addr[9]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L20N_T3L_N3_AD1N_71
# set_property PACKAGE_PIN C12      [get_ports "ddr_addr[10]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L20P_T3L_N2_AD1P_71
# set_property PACKAGE_PIN B13      [get_ports "ddr_addr[11]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L19N_T3L_N1_DBC_AD9N_71
# set_property PACKAGE_PIN C13      [get_ports "ddr_addr[12]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L19P_T3L_N0_DBC_AD9P_71
# set_property PACKAGE_PIN D15      [get_ports "ddr_addr[13]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_T2U_N12_71
# set_property PACKAGE_PIN H14      [get_ports "ddr_addr[14]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L18N_T2U_N11_AD2N_71
# set_property PACKAGE_PIN H15      [get_ports "ddr_addr[15]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L18P_T2U_N10_AD2P_71
# set_property PACKAGE_PIN F15      [get_ports "ddr_addr[16]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L17N_T2U_N9_AD10N_71



set_property PACKAGE_PIN Y48 [get_ports ddr_reset_n]
set_property PACKAGE_PIN AB51 [get_ports {ddr_ck_p[0]}]
set_property PACKAGE_PIN AB52 [get_ports {ddr_ck_n[0]}]
set_property PACKAGE_PIN AB49 [get_ports ddr_act_n]
set_property PACKAGE_PIN W56 [get_ports {ddr_cs_n[0]}]
set_property PACKAGE_PIN V56 [get_ports {ddr_odt[0]}]
set_property PACKAGE_PIN V53 [get_ports {ddr_cke[0]}]

set_property PACKAGE_PIN AB47 [get_ports {ddr_ba[0]}]
set_property PACKAGE_PIN Y54 [get_ports {ddr_ba[1]}]

set_property PACKAGE_PIN W51 [get_ports {ddr_bg[0]}]
set_property PACKAGE_PIN W55 [get_ports {ddr_bg[1]}]

set_property PACKAGE_PIN AC46 [get_ports {ddr_addr[0]}]
set_property PACKAGE_PIN Y47 [get_ports {ddr_addr[1]}]
set_property PACKAGE_PIN Y45  [get_ports {ddr_addr[2]}]
set_property PACKAGE_PIN AA46 [get_ports {ddr_addr[3]}]
set_property PACKAGE_PIN W53 [get_ports {ddr_addr[4]}]
set_property PACKAGE_PIN W52 [get_ports {ddr_addr[5]}]
set_property PACKAGE_PIN Y49 [get_ports {ddr_addr[6]}]
set_property PACKAGE_PIN AA49 [get_ports {ddr_addr[7]}]
set_property PACKAGE_PIN Y50  [get_ports {ddr_addr[8]}]
set_property PACKAGE_PIN AA47 [get_ports {ddr_addr[9]}]
set_property PACKAGE_PIN AC48 [get_ports {ddr_addr[10]}]
set_property PACKAGE_PIN AA45 [get_ports {ddr_addr[11]}]
set_property PACKAGE_PIN V51  [get_ports {ddr_addr[12]}]
set_property PACKAGE_PIN AC51 [get_ports {ddr_addr[13]}]
set_property PACKAGE_PIN AB46 [get_ports {ddr_addr[14]}]
set_property PACKAGE_PIN AC49 [get_ports {ddr_addr[15]}]
set_property PACKAGE_PIN AC50 [get_ports {ddr_addr[16]}]




#set_property PACKAGE_PIN C9       [get_ports "ddr_dm[9]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L7P_T1L_N0_QBC_AD13P_71
#set_property PACKAGE_PIN E24      [get_ports "ddr_dm[8]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L19P_T3L_N0_DBC_AD9P_72
set_property PACKAGE_PIN R52 [get_ports {ddr_dm[7]}]
set_property PACKAGE_PIN M53 [get_ports {ddr_dm[6]}]
set_property PACKAGE_PIN P57 [get_ports {ddr_dm[5]}]
set_property PACKAGE_PIN W60 [get_ports {ddr_dm[4]}]
set_property PACKAGE_PIN U56 [get_ports {ddr_dm[3]}]
set_property PACKAGE_PIN AB61 [get_ports {ddr_dm[2]}]
set_property PACKAGE_PIN AC53 [get_ports {ddr_dm[1]}]
set_property PACKAGE_PIN AH58 [get_ports {ddr_dm[0]}]

#set_property PACKAGE_PIN A8       [get_ports "ddr_dqs_n[9]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L10N_T1U_N7_QBC_AD4N_71
#set_property PACKAGE_PIN C22      [get_ports "ddr_dqs_n[8]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L22N_T3U_N7_DBC_AD0N_72


#set_property PACKAGE_PIN A9       [get_ports "ddr_dqs_p[9]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L10P_T1U_N6_QBC_AD4P_71
#set_property PACKAGE_PIN D22      [get_ports "ddr_dqs_p[8]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L22P_T3U_N6_DBC_AD0P_72
set_property PACKAGE_PIN N53 [get_ports {ddr_dqs_p[7]}]
set_property PACKAGE_PIN N54 [get_ports {ddr_dqs_n[7]}]
set_property PACKAGE_PIN K52 [get_ports {ddr_dqs_p[6]}]
set_property PACKAGE_PIN K53 [get_ports {ddr_dqs_n[6]}]
set_property PACKAGE_PIN M56 [get_ports {ddr_dqs_p[5]}]
set_property PACKAGE_PIN L56 [get_ports {ddr_dqs_n[5]}]
set_property PACKAGE_PIN V60 [get_ports {ddr_dqs_p[4]}]
set_property PACKAGE_PIN V61 [get_ports {ddr_dqs_n[4]}]
set_property PACKAGE_PIN T54 [get_ports {ddr_dqs_p[3]}]
set_property PACKAGE_PIN T55 [get_ports {ddr_dqs_n[3]}]
set_property PACKAGE_PIN Y59 [get_ports {ddr_dqs_p[2]}]
set_property PACKAGE_PIN Y60 [get_ports {ddr_dqs_n[2]}]
set_property PACKAGE_PIN AA54 [get_ports {ddr_dqs_p[1]}]
set_property PACKAGE_PIN AA55 [get_ports {ddr_dqs_n[1]}]
set_property PACKAGE_PIN AG62 [get_ports {ddr_dqs_p[0]}]
set_property PACKAGE_PIN AG63 [get_ports {ddr_dqs_n[0]}]

#set_property PACKAGE_PIN A11      [get_ports "ddr_dq[79]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L12N_T1U_N11_GC_71
#set_property PACKAGE_PIN B11      [get_ports "ddr_dq[78]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L12P_T1U_N10_GC_71
#set_property PACKAGE_PIN B10      [get_ports "ddr_dq[77]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L11N_T1U_N9_GC_71
#set_property PACKAGE_PIN C10      [get_ports "ddr_dq[76]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L11P_T1U_N8_GC_71
#set_property PACKAGE_PIN B7       [get_ports "ddr_dq[75]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L9N_T1L_N5_AD12N_71
#set_property PACKAGE_PIN B8       [get_ports "ddr_dq[74]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L9P_T1L_N4_AD12P_71
#set_property PACKAGE_PIN C7       [get_ports "ddr_dq[73]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L8N_T1L_N3_AD5N_71
#set_property PACKAGE_PIN D7       [get_ports "ddr_dq[72]"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L8P_T1L_N2_AD5P_71
#set_property PACKAGE_PIN A21      [get_ports "ddr_dq[71]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L24N_T3U_N11_72
#set_property PACKAGE_PIN B21      [get_ports "ddr_dq[70]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L24P_T3U_N10_72
#set_property PACKAGE_PIN B22      [get_ports "ddr_dq[69]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L23N_T3U_N9_72
#set_property PACKAGE_PIN B23      [get_ports "ddr_dq[68]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L23P_T3U_N8_72
#set_property PACKAGE_PIN C23      [get_ports "ddr_dq[67]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L21N_T3L_N5_AD8N_72
#set_property PACKAGE_PIN C24      [get_ports "ddr_dq[66]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L21P_T3L_N4_AD8P_72
#set_property PACKAGE_PIN A23      [get_ports "ddr_dq[65]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L20N_T3L_N3_AD1N_72
#set_property PACKAGE_PIN A24      [get_ports "ddr_dq[64]"] ;# Bank  72 VCCO - VCC1V2_FPGA - IO_L20P_T3L_N2_AD1P_72
set_property PACKAGE_PIN T52  [get_ports {ddr_dq[63]}]
set_property PACKAGE_PIN N51  [get_ports {ddr_dq[62]}]
set_property PACKAGE_PIN P53  [get_ports {ddr_dq[61]}]
set_property PACKAGE_PIN P51  [get_ports {ddr_dq[60]}]
set_property PACKAGE_PIN U52  [get_ports {ddr_dq[59]}]
set_property PACKAGE_PIN U53  [get_ports {ddr_dq[58]}]
set_property PACKAGE_PIN T51  [get_ports {ddr_dq[57]}]
set_property PACKAGE_PIN P52  [get_ports {ddr_dq[56]}]
set_property PACKAGE_PIN L51  [get_ports {ddr_dq[55]}]
set_property PACKAGE_PIN L52  [get_ports {ddr_dq[54]}]
set_property PACKAGE_PIN L54  [get_ports {ddr_dq[53]}]
set_property PACKAGE_PIN M52  [get_ports {ddr_dq[52]}]
set_property PACKAGE_PIN M51  [get_ports {ddr_dq[51]}]
set_property PACKAGE_PIN J51  [get_ports {ddr_dq[50]}]
set_property PACKAGE_PIN J52  [get_ports {ddr_dq[49]}]
set_property PACKAGE_PIN K54  [get_ports {ddr_dq[48]}]
set_property PACKAGE_PIN K58  [get_ports {ddr_dq[47]}]
set_property PACKAGE_PIN N55  [get_ports {ddr_dq[46]}]
set_property PACKAGE_PIN M58  [get_ports {ddr_dq[45]}]
set_property PACKAGE_PIN M57  [get_ports {ddr_dq[44]}]
set_property PACKAGE_PIN L57  [get_ports {ddr_dq[43]}]
set_property PACKAGE_PIN K57  [get_ports {ddr_dq[42]}]
set_property PACKAGE_PIN N58  [get_ports {ddr_dq[41]}]
set_property PACKAGE_PIN N56  [get_ports {ddr_dq[40]}]
set_property PACKAGE_PIN U62  [get_ports {ddr_dq[39]}]
set_property PACKAGE_PIN U61  [get_ports {ddr_dq[38]}]
set_property PACKAGE_PIN V58  [get_ports {ddr_dq[37]}]
set_property PACKAGE_PIN V63  [get_ports {ddr_dq[36]}]
set_property PACKAGE_PIN V59  [get_ports {ddr_dq[35]}]
set_property PACKAGE_PIN W63  [get_ports {ddr_dq[34]}]
set_property PACKAGE_PIN W62  [get_ports {ddr_dq[33]}]
set_property PACKAGE_PIN U63  [get_ports {ddr_dq[32]}]
set_property PACKAGE_PIN R57  [get_ports {ddr_dq[31]}]
set_property PACKAGE_PIN R54  [get_ports {ddr_dq[30]}]
set_property PACKAGE_PIN P56  [get_ports {ddr_dq[29]}]
set_property PACKAGE_PIN T56  [get_ports {ddr_dq[28]}]
set_property PACKAGE_PIN R58  [get_ports {ddr_dq[27]}]
set_property PACKAGE_PIN P55  [get_ports {ddr_dq[26]}]
set_property PACKAGE_PIN R55  [get_ports {ddr_dq[25]}]
set_property PACKAGE_PIN T57  [get_ports {ddr_dq[24]}]
set_property PACKAGE_PIN Y63  [get_ports {ddr_dq[23]}]
set_property PACKAGE_PIN AA61 [get_ports {ddr_dq[22]}]
set_property PACKAGE_PIN AB58 [get_ports {ddr_dq[21]}]
set_property PACKAGE_PIN AA60 [get_ports {ddr_dq[20]}]
set_property PACKAGE_PIN Y62  [get_ports {ddr_dq[19]}]
set_property PACKAGE_PIN AA62 [get_ports {ddr_dq[18]}]
set_property PACKAGE_PIN AA59 [get_ports {ddr_dq[17]}]
set_property PACKAGE_PIN AB59 [get_ports {ddr_dq[16]}]
set_property PACKAGE_PIN AB57 [get_ports {ddr_dq[15]}]
set_property PACKAGE_PIN AC56 [get_ports {ddr_dq[14]}]
set_property PACKAGE_PIN AA56 [get_ports {ddr_dq[13]}]
set_property PACKAGE_PIN AB54 [get_ports {ddr_dq[12]}]
set_property PACKAGE_PIN AB56 [get_ports {ddr_dq[11]}]
set_property PACKAGE_PIN AA57 [get_ports {ddr_dq[10]}]
set_property PACKAGE_PIN AB53 [get_ports {ddr_dq[9]}]
set_property PACKAGE_PIN AC55 [get_ports {ddr_dq[8]}]
set_property PACKAGE_PIN AE62 [get_ports {ddr_dq[7]}]
set_property PACKAGE_PIN AF60 [get_ports {ddr_dq[6]}]
set_property PACKAGE_PIN AF61 [get_ports {ddr_dq[5]}]
set_property PACKAGE_PIN AG58 [get_ports {ddr_dq[4]}]
set_property PACKAGE_PIN AF59 [get_ports {ddr_dq[3]}]
set_property PACKAGE_PIN AG61 [get_ports {ddr_dq[2]}]
set_property PACKAGE_PIN AF62 [get_ports {ddr_dq[1]}]
set_property PACKAGE_PIN AG59 [get_ports {ddr_dq[0]}]

# unused
#set_property PACKAGE_PIN R17      [get_ports "ddr_addr[LERT_B"] ;# Bank  73 VCCO - VCC1V2_FPGA - IO_L1N_T0L_N1_DBC_73
#set_property PACKAGE_PIN G10      [get_ports "DDR4_C1_PAR"] ;# Bank  71 VCCO - VCC1V2_FPGA - IO_L1N_T0L_N1_DBC_71
#set_property PACKAGE_PIN A20      [get_ports "DDR4_C1_TEN"] ;# Bank  73 VCCO - VCC1V2_FPGA - IO_T3U_N12_73

## False paths
#set_clock_groups -name sync_gr1 -logically_exclusive -group [get_clocks chipset_clk_clk_mmcm] -group [get_clocks -include_generated_clocks mc_sys_clk_clk_mmcm]

## Ethernet Constraints for 100 Mb/s

########## Input constraints
## hint from here: https://forums.xilinx.com/t5/Timing-Analysis/XDC-constraints-Source-Synchronous-ADC-DDR/td-p/292807
#create_clock -period 40.000 -name net_phy_rxc_virt
## conservatively assuming +/- 2ns skew of rxd/rxctl
#create_clock -period 40.000 -name net_phy_rxc -waveform {2.000 22.000} [get_ports net_phy_rxc]
#set_clock_groups -asynchronous -group [get_clocks chipset_clk_clk_mmcm] -group [get_clocks net_phy_rxc]
#set_input_delay -clock [get_clocks net_phy_rxc_virt] -min -add_delay 0.000 [get_ports {net_phy_rxd[*]}]
#set_input_delay -clock [get_clocks net_phy_rxc_virt] -max -add_delay 4.000 [get_ports {net_phy_rxd[*]}]
#set_input_delay -clock [get_clocks net_phy_rxc_virt] -clock_fall -min -add_delay 0.000 [get_ports net_phy_rxctl]
#set_input_delay -clock [get_clocks net_phy_rxc_virt] -clock_fall -max -add_delay 4.000 [get_ports net_phy_rxctl]
#set_input_delay -clock [get_clocks net_phy_rxc_virt] -min -add_delay 0.000 [get_ports net_phy_rxctl]
#set_input_delay -clock [get_clocks net_phy_rxc_virt] -max -add_delay 4.000 [get_ports net_phy_rxctl]
#
########### Output Constraints
#create_generated_clock -name net_phy_txc -source [get_pins chipset/net_phy_txc_oddr/C] -divide_by 1 -invert [get_ports net_phy_txc]


#############################################
# SD Card Constraints for 25MHz
#############################################
create_generated_clock -name sd_fast_clk -source [get_pins chipset/clk_mmcm/sd_sys_clk] -divide_by 2 [get_pins chipset/chipset_impl/piton_sd_top/sdc_controller/clock_divider0/fast_clk_reg/Q]
create_generated_clock -name sd_slow_clk -source [get_pins chipset/clk_mmcm/sd_sys_clk] -divide_by 200 [get_pins chipset/chipset_impl/piton_sd_top/sdc_controller/clock_divider0/slow_clk_reg/Q]
create_generated_clock -name sd_clk_out -source [get_pins chipset/sd_clk_oddr/C] -divide_by 1 -add -master_clock sd_fast_clk [get_ports sd_clk_out]
create_generated_clock -name sd_clk_out_1 -source [get_pins chipset/sd_clk_oddr/C] -divide_by 1 -add -master_clock sd_slow_clk [get_ports sd_clk_out]

# compensate for board trace and level shifter uncertainty
set_clock_uncertainty 2.000 [get_clocks sd_clk_out]
set_clock_uncertainty 2.000 [get_clocks sd_clk_out_1]

#################
# FPGA out / card in
# data is aligned with clock (source synchronous)

# hold fast (spec requires minimum 2ns), note that data is launched on falling edge, so 0.0 is ok here
set_output_delay -clock [get_clocks sd_clk_out] -min -add_delay 0.000 [get_ports {sd_dat[*]}]
set_output_delay -clock [get_clocks sd_clk_out] -min -add_delay 0.000 [get_ports sd_cmd]

# setup fast (spec requires minimum 6ns)
set_output_delay -clock [get_clocks sd_clk_out] -max -add_delay 8.000 [get_ports {sd_dat[*]}]
set_output_delay -clock [get_clocks sd_clk_out] -max -add_delay 8.000 [get_ports sd_cmd]

# hold slow (spec requires minimum 5ns), note that data is launched on falling edge, so 0.0 is ok here
set_output_delay -clock [get_clocks sd_clk_out_1] -min -add_delay 0.000 [get_ports {sd_dat[*]}]
set_output_delay -clock [get_clocks sd_clk_out_1] -min -add_delay 0.000 [get_ports sd_cmd]

# setup slow (spec requires minimum 5ns)
set_output_delay -clock [get_clocks sd_clk_out_1] -max -add_delay 8.000 [get_ports {sd_dat[*]}]
set_output_delay -clock [get_clocks sd_clk_out_1] -max -add_delay 8.000 [get_ports sd_cmd]

#################
# card out / FPGA in
# assume ~15cm/ns propagation time
# 14ns pd from card + 2 x 1ns trace + 2 x 2ns level shifter
# data is launched on negative clock edge here

# propdelay fast
set_input_delay -clock [get_clocks sd_clk_out] -clock_fall -max -add_delay 20.000 [get_ports {sd_dat[*]}]
set_input_delay -clock [get_clocks sd_clk_out] -clock_fall -max -add_delay 20.000 [get_ports sd_cmd]

# contamination delay fast
set_input_delay -clock [get_clocks sd_clk_out] -clock_fall -min -add_delay -1.000 [get_ports {sd_dat[*]}]
set_input_delay -clock [get_clocks sd_clk_out] -clock_fall -min -add_delay -1.000 [get_ports sd_cmd]

# propdelay slow
set_input_delay -clock [get_clocks sd_clk_out_1] -clock_fall -max -add_delay 20.000 [get_ports {sd_dat[*]}]
set_input_delay -clock [get_clocks sd_clk_out_1] -clock_fall -max -add_delay 20.000 [get_ports sd_cmd]

# contamination  slow
set_input_delay -clock [get_clocks sd_clk_out_1] -clock_fall -min -add_delay -1.000 [get_ports {sd_dat[*]}]
set_input_delay -clock [get_clocks sd_clk_out_1] -clock_fall -min -add_delay -1.000 [get_ports sd_cmd]

#################
# clock groups

set_clock_groups -physically_exclusive -group [get_clocks -include_generated_clocks sd_clk_out] -group [get_clocks -include_generated_clocks sd_clk_out_1]
set_clock_groups -logically_exclusive -group [get_clocks -include_generated_clocks sd_fast_clk] -group [get_clocks -include_generated_clocks sd_slow_clk]
set_clock_groups -asynchronous -group [get_clocks -include_generated_clocks chipset_clk_clk_mmcm] -group [get_clocks -filter { NAME =~  "*sd*" }]


create_debug_core u_ila_0 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 4 [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER true [get_debug_cores u_ila_0]
set_property C_DATA_DEPTH 4096 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL true [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets [list chipset/clk_mmcm/inst/chipset_clk]]
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
set_property port_width 9 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list {chipset/chipset_impl/piton_sd_top/bram_addr[0]} {chipset/chipset_impl/piton_sd_top/bram_addr[1]} {chipset/chipset_impl/piton_sd_top/bram_addr[2]} {chipset/chipset_impl/piton_sd_top/bram_addr[3]} {chipset/chipset_impl/piton_sd_top/bram_addr[4]} {chipset/chipset_impl/piton_sd_top/bram_addr[5]} {chipset/chipset_impl/piton_sd_top/bram_addr[6]} {chipset/chipset_impl/piton_sd_top/bram_addr[7]} {chipset/chipset_impl/piton_sd_top/bram_addr[8]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
set_property port_width 1 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list chipset/chipset_impl/piton_sd_top/init_done]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
set_property port_width 1 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list chipset/chipset_impl/piton_sd_top/sd_int_cmd]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
set_property port_width 1 [get_debug_ports u_ila_0/probe3]
connect_debug_port u_ila_0/probe3 [get_nets [list chipset/chipset_impl/piton_sd_top/sd_int_data]]
create_debug_core u_ila_1 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_1]
set_property ALL_PROBE_SAME_MU_CNT 4 [get_debug_cores u_ila_1]
set_property C_ADV_TRIGGER true [get_debug_cores u_ila_1]
set_property C_DATA_DEPTH 4096 [get_debug_cores u_ila_1]
set_property C_EN_STRG_QUAL true [get_debug_cores u_ila_1]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_1]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_1]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_1]
set_property port_width 1 [get_debug_ports u_ila_1/clk]
connect_debug_port u_ila_1/clk [get_nets [list chipset/chipset_impl/piton_sd_top/sdc_controller/clock_divider0/slow_clk_reg_0]]
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_1/probe0]
set_property port_width 5 [get_debug_ports u_ila_1/probe0]
connect_debug_port u_ila_1/probe0 [get_nets [list {chipset/chipset_impl/piton_sd_top/sdc_controller/int_status_o[0]} {chipset/chipset_impl/piton_sd_top/sdc_controller/int_status_o[1]} {chipset/chipset_impl/piton_sd_top/sdc_controller/int_status_o[2]} {chipset/chipset_impl/piton_sd_top/sdc_controller/int_status_o[3]} {chipset/chipset_impl/piton_sd_top/sdc_controller/int_status_o[4]}]]
create_debug_port u_ila_1 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_1/probe1]
set_property port_width 4 [get_debug_ports u_ila_1/probe1]
connect_debug_port u_ila_1/probe1 [get_nets [list {chipset/chipset_impl/piton_sd_top/sd_dat_dat_i[0]} {chipset/chipset_impl/piton_sd_top/sd_dat_dat_i[1]} {chipset/chipset_impl/piton_sd_top/sd_dat_dat_i[2]} {chipset/chipset_impl/piton_sd_top/sd_dat_dat_i[3]}]]
create_debug_port u_ila_1 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_1/probe2]
set_property port_width 4 [get_debug_ports u_ila_1/probe2]
connect_debug_port u_ila_1/probe2 [get_nets [list {chipset/chipset_impl/piton_sd_top/sd_dat_out_o[0]} {chipset/chipset_impl/piton_sd_top/sd_dat_out_o[1]} {chipset/chipset_impl/piton_sd_top/sd_dat_out_o[2]} {chipset/chipset_impl/piton_sd_top/sd_dat_out_o[3]}]]
create_debug_port u_ila_1 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_1/probe3]
set_property port_width 1 [get_debug_ports u_ila_1/probe3]
connect_debug_port u_ila_1/probe3 [get_nets [list chipset/chipset_impl/piton_sd_top/sd_dat_oe_o]]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk]
