import os

from migen.build.generic_platform import *
from migen.build.xilinx import XilinxPlatform, VivadoProgrammer

# All IO pins/ports on zcu111 device.
# Above each section is listed the format for how to Ctrl-F to that 
# specfic pin set in the ZCU111.xdc file. Swappable parts are surrounded
# by ().
_io = [
    # CPU_RESET
    ("cpu_reset", 0, Pins("AF15"), IOStandard("LVCMOS18")),

	# GPIO_LED_(#)_LS
	("user_led", 0, Pins("AR13"), IOStandard("LVCMOS18")),
    ("user_led", 1, Pins("AP13"), IOStandard("LVCMOS18")),
    ("user_led", 2, Pins("AR16"), IOStandard("LVCMOS18")),
    ("user_led", 3, Pins("AP16"), IOStandard("LVCMOS18")),
    ("user_led", 4, Pins("AP15"), IOStandard("LVCMOS18")),
    ("user_led", 5, Pins("AN16"), IOStandard("LVCMOS18")),
    ("user_led", 6, Pins("AN17"), IOStandard("LVCMOS18")),
    ("user_led", 7, Pins("AV15"), IOStandard("LVCMOS18")),

    # GPIO_SW_(direction)
    ("user_sw_c", 0, Pins("AW5"), IOStandard("LVCMOS18")),
    ("user_sw_n", 0, Pins("AW3"), IOStandard("LVCMOS18")),
    ("user_sw_s", 0, Pins("E8"), IOStandard("LVCMOS18")),
    ("user_sw_w", 0, Pins("AW6"), IOStandard("LVCMOS18")),
    ("user_sw_e", 0, Pins("AW4"), IOStandard("LVCMOS18")),

    # GPIO_DIP_SW(#)
    ("user_dip_sw", 0, Pins("AF16"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 1, Pins("AF17"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 2, Pins("AH15"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 3, Pins("AH16"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 4, Pins("AH17"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 5, Pins("AG17"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 6, Pins("AJ15"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 7, Pins("AJ16"), IOStandard("LVCMOS18")),

    # USER_SMA_MGT_CLOCK_C_(N/P)
    ("user_sma_clock", 0,
        Subsignal("p", Pins("T32"), IOStandard("LVDS")), # Swap LVDS w/ MGTREFCLK ?
        Subsignal("n", Pins("T31"), IOStandard("LVDS"))
    ),

    # CLK_(#)_(N/P)
    ("clk100", 0,
        Subsignal("p", Pins("AM15"), IOStandard("LVDS")),
        Subsignal("n", Pins("AN15"), IOStandard("LVDS"))
    ),
    ("clk125", 0,
        Subsignal("p", Pins("AL17"), IOStandard("LVDS")),
        Subsignal("n", Pins("AM17"), IOStandard("LVDS"))
    ),

    # PL_I2C(#)_(SCL/SDC)_LS
    ("i2c", 0,
        Subsignal("scl", Pins("AT16")),
        Subsignal("sda", Pins("AW16")),
        IOStandard("LVCMOS18")
    ),
    ("i2c", 1,
        Subsignal("scl", Pins("AV16")),
        Subsignal("sda", Pins("AV13")),
        IOStandard("LVCMOS18")
    ),

    # UART2_(CTS/RTS)_B
    # UART2_(RXD/TXD)_FPGA_(RXD/TXD)
    # DOUBLE CHECK RX/TX DIRECTION
    ("serial", 0,
        Subsignal("cts", Pins("AT14")),
        Subsignal("rts", Pins("AU14")),
        Subsignal("tx", Pins("AU15")),
        Subsignal("rx", Pins("AT15")),
        IOStandard("LVCMOS18")
	),

    # ADCIO_(##)
    ("adc", 0, Pins("AP5"), IOStandard("LVCMOS18")),
    ("adc", 1, Pins("AP6"), IOStandard("LVCMOS18")),
    ("adc", 2, Pins("AR6"), IOStandard("LVCMOS18")),
    ("adc", 3, Pins("AR7"), IOStandard("LVCMOS18")),
    ("adc", 4, Pins("AV7"), IOStandard("LVCMOS18")),
    ("adc", 5, Pins("AU7"), IOStandard("LVCMOS18")),
    ("adc", 6, Pins("AV8"), IOStandard("LVCMOS18")),
    ("adc", 7, Pins("AU8"), IOStandard("LVCMOS18")),
    ("adc", 8, Pins("AT6"), IOStandard("LVCMOS18")),
    ("adc", 9, Pins("AT7"), IOStandard("LVCMOS18")),
    ("adc", 10, Pins("AU5"), IOStandard("LVCMOS18")),
    ("adc", 11, Pins("AT5"), IOStandard("LVCMOS18")),
    ("adc", 12, Pins("AU3"), IOStandard("LVCMOS18")),
    ("adc", 13, Pins("AU4"), IOStandard("LVCMOS18")),
    ("adc", 14, Pins("AV5"), IOStandard("LVCMOS18")),
    ("adc", 15, Pins("AV6"), IOStandard("LVCMOS18")),
    ("adc", 16, Pins("AU1"), IOStandard("LVCMOS18")),
    ("adc", 17, Pins("AU2"), IOStandard("LVCMOS18")),
    ("adc", 18, Pins("AV2"), IOStandard("LVCMOS18")),
    ("adc", 19, Pins("AV3"), IOStandard("LVCMOS18")),

    # DACIO_(##)
    ("dac", 0, Pins("A9"), IOStandard("LVCMOS18")),
    ("dac", 1, Pins("A10"), IOStandard("LVCMOS18")),
    ("dac", 2, Pins("A6"), IOStandard("LVCMOS18")),
    ("dac", 3, Pins("A7"), IOStandard("LVCMOS18")),
    ("dac", 4, Pins("A5"), IOStandard("LVCMOS18")),
    ("dac", 5, Pins("B5"), IOStandard("LVCMOS18")),
    ("dac", 6, Pins("C5"), IOStandard("LVCMOS18")),
    ("dac", 7, Pins("C6"), IOStandard("LVCMOS18")),
    ("dac", 8, Pins("B9"), IOStandard("LVCMOS18")),
    ("dac", 9, Pins("B10"), IOStandard("LVCMOS18")),
    ("dac", 10, Pins("B7"), IOStandard("LVCMOS18")),
    ("dac", 11, Pins("B8"), IOStandard("LVCMOS18")),
    ("dac", 12, Pins("D8"), IOStandard("LVCMOS18")),
    ("dac", 13, Pins("D9"), IOStandard("LVCMOS18")),
    ("dac", 14, Pins("C7"), IOStandard("LVCMOS18")),
    ("dac", 15, Pins("C8"), IOStandard("LVCMOS18")),
    ("dac", 16, Pins("C10"), IOStandard("LVCMOS18")),
    ("dac", 17, Pins("D10"), IOStandard("LVCMOS18")),
    ("dac", 18, Pins("D6"), IOStandard("LVCMOS18")),
    ("dac", 19, Pins("E7"), IOStandard("LVCMOS18")),

    # PL_DDR4_
    ("ddram", 0,
    	# PL_DDR4_A(#)
        Subsignal("a", Pins(
            "D18 E19 E17 E18 E16 F16 F19 G19",
            "F15 G15 G18 H18 K17 L17"), IOStandard("SSTL12")),
        # PL_DDR4_CK0_C
        Subsignal("ck_c", Pins("F17"), IOStandard("DIFF_POD12")),
        # PL_DDR4_CK0_T
        Subsignal("ck_t", Pins("G17"), IOStandard("DIFF_POD12")),
        # PL_DDR4_BA(#)
        Subsignal("ba", Pins("K18 K19"), IOStandard("SSTL12")),
        # PL_DDR4_BG0
        Subsignal("bg", Pins("C16"), IOStandard("SSTL12")),
        # PL_DDR4_RAS_B
        Subsignal("ras_n", Pins("C18"), IOStandard("SSTL12")),
        # PL_DDR4_CAS_B
        Subsignal("cas_n", Pins("D15"), IOStandard("SSTL12")),
        # PL_DDR4_WE_B
        Subsignal("we_n", Pins("B17"), IOStandard("SSTL12")),
        # PL_DDR4_CS_B
        Subsignal("cs_n", Pins("D16"), IOStandard("SSTL12")),
        # PL_DDR4_ACT_B
        Subsignal("act_n", Pins("A19"), IOStandard("SSTL12")),
        # PL_DDR4_ALERT_B
        Subsignal("alert_n", Pins("B18"), IOStandard("POD12")),
        # PL_DDR4_PARITY
        Subsignal("par", Pins("D19"), IOStandard("POD12")),
        # PL_DDR4_DM(#)_B
        Subsignal("dm", Pins("G13 C12 K13 J8 C23 F21 J23 N20"), IOStandard("POD12")),
        # PL_DDR4_DQ(#)
        Subsignal("dq", Pins(
            "D14 A14 F14 F12 E14 H12 G14 H13", # 0 -> 7
            "B13 A15 A12 A14 D13 B14 A11 C13", # 8 -> 15
    
            "K11 J11 H10 F11 K10 F10 J10 H11", # 16 -> 23
            "G9  G7  F9  G6  H6  H7  J9  K9", # 24 -> 31
    
            "C22 A20 A21 C21 A24 B20 B24 C20", # 32 -> 39
            "E24 E22 E23 G20 F24 E21 F20 D21", # 40 -> 47
    
            "H23 G23 K24 G22 J21 H22 L24 H21", # 48 -> 55
            "L23 L20 L22 L21 M20 L19 M19 N19", # 56 -> 63
           ), IOStandard("POD12")),
        # PL_DDR4_DQS(#)_T
        Subsignal("dqs_t", Pins("E13 C15 J14 H8 B22 D23 J20 K21 K22"), IOStandard("DIFF_POD12")),
        # PL_DDR4_DQS(#)_C
        Subsignal("dqs_c", Pins("E12 B15 J13 G8 A22 D24 H20"), IOStandard("DIFF_POD12")),
        # PL_DDR4_CKE
        Subsignal("cke", Pins("A16"), IOStandard("SSTL12")),
        # PL_DDR4_ODT
        Subsignal("odt", Pins("B19"), IOStandard("SSTL12")),
        # PL_DDR4_RESET_B
        Subsignal("reset_n", Pins("A17"), IOStandard("LVCMOS12")),
    ),

    # USER_SMA_MGT_CLOCK_(N/P)
    ("user_sma_mgt_clk", 0,
        Subsignal("p", Pins("T31")),
        Subsignal("n", Pins("T32"))
    ),

    # SFP(#)_RX_(N/P)
    ("sfp_rx", 0,
        Subsignal("p", Pins("AA38")),
        Subsignal("n", Pins("AA39"))
    ),
    ("sfp_rx", 1,
        Subsignal("p", Pins("W38")),
        Subsignal("n", Pins("W39"))
    ),
    ("sfp_rx", 2,
        Subsignal("p", Pins("U38")),
        Subsignal("n", Pins("U39"))
    ),
    ("sfp_rx", 3,
        Subsignal("p", Pins("R38")),
        Subsignal("n", Pins("R39"))
    ),

    # SFP(#)_TX_(N/P)
    ("sfp_tx", 0,
        Subsignal("p", Pins("Y35")),
        Subsignal("n", Pins("Y36"))
    ),
    ("sfp_tx", 1,
        Subsignal("p", Pins("V35")),
        Subsignal("n", Pins("V36"))
    ),
    ("sfp_tx", 2,
        Subsignal("p", Pins("T35")),
        Subsignal("n", Pins("T36"))
    ),
    ("sfp_tx", 3,
        Subsignal("p", Pins("R33")),
        Subsignal("n", Pins("R34"))
    ),

    # SFP(#)_TX_DISABLE_B
    ("sfp_tx_disable_n", 0, Pins("G12"), IOStandard("LVCMOS18")),
    ("sfp_tx_disable_n", 1, Pins("G10"), IOStandard("LVCMOS18")),
    ("sfp_tx_disable_n", 2, Pins("K12"), IOStandard("LVCMOS18")),
    ("sfp_tx_disable_n", 3, Pins("J7"), IOStandard("LVCMOS18")),
    
    # SFP_REC_CLOCK_C_(N/P)
    ("sfp_rec_clock_c", 0,
    	Subsignal("p", Pins("AW14")),
    	Subsignal("n", Pins("AW13")),
    	IOStandard("LVDS")
	),

	# SFP_SI5382_IN1_C_(N/P)
    ("sfp_si5382_in1_c", 0,
    	Subsignal("p", Pins("AA33")),
    	Subsignal("n", Pins("AA34"))
	),

	# SFP_SI5382_OUT_C_(N/P)
    ("sfp_si5382_out_c", 0,
    	Subsignal("p", Pins("Y31")),
    	Subsignal("n", Pins("Y32"))
	),

	# SFP_SI5382_CLK_IN_SEL
    ("sfp_si5382_in1_c", 0, Pins("E39"), IOStandard("LVCMOS18")),

    # SFP_SI5382_CLK_IN_SEL
    ("sfp_si5382_clk_in_sel", 0, Pins("E9"), IOStandard("LVCMOS18")),

    # PMOD0_(#)_LS
    ("pmod0", 0, Pins("C17"), IOStandard("LVCMOS12")),
    ("pmod0", 1, Pins("M18"), IOStandard("LVCMOS12")),
    ("pmod0", 2, Pins("H16"), IOStandard("LVCMOS12")),
    ("pmod0", 3, Pins("H17"), IOStandard("LVCMOS12")),
    ("pmod0", 4, Pins("J16"), IOStandard("LVCMOS12")),
    ("pmod0", 5, Pins("K16"), IOStandard("LVCMOS12")),
    ("pmod0", 6, Pins("H15"), IOStandard("LVCMOS12")),
    ("pmod0", 7, Pins("J15"), IOStandard("LVCMOS12")),

    # PMOD1_(#)_LS
    ("pmod1", 0, Pins("L14"), IOStandard("LVCMOS12")),
    ("pmod1", 1, Pins("L15"), IOStandard("LVCMOS12")),
    ("pmod1", 2, Pins("M13"), IOStandard("LVCMOS12")),
    ("pmod1", 3, Pins("N13"), IOStandard("LVCMOS12")),
    ("pmod1", 4, Pins("M15"), IOStandard("LVCMOS12")),
    ("pmod1", 5, Pins("N15"), IOStandard("LVCMOS12")),
    ("pmod1", 6, Pins("M14"), IOStandard("LVCMOS12")),
    ("pmod1", 7, Pins("N14"), IOStandard("LVCMOS12")),

    # MSP430_GPIO_PL_(#)_LS
    ("msp430_gpio", 0, Pins("M17"), IOStandard("LVCMOS12")),
    ("msp430_gpio", 1, Pins("N17"), IOStandard("LVCMOS12")),
    ("msp430_gpio", 2, Pins("L12"), IOStandard("LVCMOS12")),
    ("msp430_gpio", 3, Pins("M12"), IOStandard("LVCMOS12")),

    # MSP430_UCA1_(TXD/RXD)_LS
    ("msp430_uca_tx", 0, Pins("M17"), IOStandard("LVCMOS18")),
    ("msp430_uca_rx", 0, Pins("L12"), IOStandard("LVCMOS18")),

    # USER_SI570_(N/P)
    ("user_si570", 0,
    	Subsignal("p", Pins("J19")),
    	Subsignal("n", Pins("J18"))
	),

	# VRP_(##)
	("vrp_65", 0, Pins("AU9"), IOStandard("LVCMOSxx")),
	("vrp_66", 0, Pins("AU22"), IOStandard("LVCMOSxx")),
	("vrp_67", 0, Pins("K23"), IOStandard("LVCMOSxx")),
	("vrp_68", 0, Pins("F7"), IOStandard("LVCMOSxx")),
	("vrp_69", 0, Pins("L16"), IOStandard("LVCMOSxx")),

	# SYSMON_(SCL/SDA)_LS
	("sysmon_scl", 0, Pins("B12"), IOStandard("LVCMOS12")),
	("sysmon_sda", 0, Pins("B11"), IOStandard("LVCMOS12")),

	# SYSREF_FPGA_C_(N/P)
	("sysref_fpga_c", 0,
    	Subsignal("p", Pins("AK17")),
    	Subsignal("n", Pins("AK16"))
	),

	# FPGA_REFCLK_OUT_C_(N/P)
	("fpga_refclk_out_c", 0,
    	Subsignal("p", Pins("AL16")),
    	Subsignal("n", Pins("AL15"))
	),

	# AMS_FPGA_REF_CLK
	("ams_fpga_ref_clk", 0, Pins("AP14"), IOStandard("LVCMOS18")),

	# USER_MGT_SI570_CLOCK_C_(N/P)
	("user_mgt_si570_clock_c", 0,
    	Subsignal("p", Pins("V31")),
    	Subsignal("n", Pins("V32"))
	),

]

# Above each section is listed the format for how to Ctrl-F to that 
# specfic pin set in the ZCU111.xdc file. Swappable parts are surrounded
# by ().
_connectors = [
    ("FMCP_HSPC", {

    	# FMCP_HSPC_DP(#)_M2C_(N/P)
        "DP0_M2C_P": "N38",
        "DP0_M2C_N": "N39",
        "DP1_M2C_P": "M36",
        "DP1_M2C_N": "M37",
        "DP2_M2C_P": "L38",
        "DP2_M2C_N": "L39",
        "DP3_M2C_P": "A4",
        "DP3_M2C_N": "A3",
        "DP4_M2C_P": "M2",
        "DP4_M2C_N": "M1",
        "DP5_M2C_P": "H2",
        "DP5_M2C_N": "H1",
        "DP6_M2C_P": "K2",
        "DP6_M2C_N": "K1",
        "DP7_M2C_P": "F2",
        "DP7_M2C_N": "F1",
        "DP8_M2C_P": "E38",
        "DP8_M2C_N": "E39",
        "DP9_M2C_P": "D36",
        "DP9_M2C_N": "D37",
        "DP10_M2C_P": "C38",
        "DP10_M2C_N": "C39",
        "DP11_M2C_P": "B36",
        "DP11_M2C_N": "B37",

        # FMCP_HSPC_DP(#)_C2M_(N/P)
        "DP0_C2M_P": "P35",
        "DP0_C2M_N": "P36",
        "DP1_C2M_P": "N33",
        "DP1_C2M_N": "N34",
        "DP2_C2M_P": "L33",
        "DP2_C2M_N": "L34",
        "DP3_C2M_P": "B6",
        "DP3_C2M_N": "B5",
        "DP4_C2M_P": "N4",
        "DP4_C2M_N": "N3",
        "DP5_C2M_P": "J4",
        "DP5_C2M_N": "J3",
        "DP6_C2M_P": "L4",
        "DP6_C2M_N": "L3",
        "DP7_C2M_P": "G4",
        "DP7_C2M_N": "G3",
        "DP8_C2M_P": "D31",
        "DP8_C2M_N": "D32",
        "DP9_C2M_P": "C33",
        "DP9_C2M_N": "C34",
        "DP10_C2M_P": "B31",
        "DP10_C2M_N": "B32",
        "DP11_C2M_P": "A33",
        "DP11_C2M_N": "A34",

        # FMCP_HSPC_GBTCLK0_M2C_(N/P)
        "GBTCLK0_M2C_C_N": "W34",
        "GBTCLK0_M2C_C_P": "W33",
        "GBTCLK1_M2C_C_N": "U34",
        "GBTCLK1_M2C_C_P": "U33",
        "GBTCLK2_M2C_C_N": "P32",
        "GBTCLK2_M2C_C_P": "P31",

        # FMCP_HSPC_SYNC_C2M_(N/P)
        "SYNC_C2M_N": "AT11",
        "SYNC_C2M_P": "AT12",

        # FMCP_HSPC_SYNC_M2C_(N/P)     IOSTANDARD : LVDS
        "SYNC_M2C_N": "AV12",
        "SYNC_M2C_P": "AU12",

        # FMCP_HSPC_CLK(#)_M2LEDNDARD : LVDS
        "CLK0_M2C_P": "AN10",
        "CLK0_M2C_N": "AP10",
        "CLK1_M2C_P": "AP20",
        "CLK1_M2C_N": "AP19",

        # FMCP_HSPC_LA(##)_(N/P)     IOSTANDARD : LVCMOS18
        "LA00_CC_P": "AP9",
        "LA00_CC_N": "AR9",
        "LA01_CC_P": "AP8",
        "LA01_CC_N": "AR8",
		"LA01_P": "AP8",
        "LA01_N": "AR8",
        "LA02_P": "AH13",
        "LA02_N": "AJ13",
        "LA03_P": "AJ12",
        "LA03_N": "AK12",
        "LA04_P": "AG12",
        "LA04_N": "AH12",
        "LA05_P": "AM8",
        "LA05_N": "AM7",
        "LA06_P": "AL8",
        "LA06_N": "AL7",
        "LA07_P": "AK13",
        "LA07_N": "AL12",
        "LA08_P": "AL9",
        "LA08_N": "AM9",
        "LA09_P": "AN8",
        "LA09_N": "AN7",
        "LA10_P": "AM12",
        "LA10_N": "AN12",
        "LA11_P": "AT10",
        "LA11_N": "AU10",
        "LA12_P": "AL10",
        "LA12_N": "AM10",
        "LA13_P": "AM13",
        "LA13_N": "AN13",
        "LA14_P": "AL14",
        "LA14_N": "AM14",
        "LA15_P": "AJ14",
        "LA15_N": "AK14",
        "LA16_P": "AR12",
        "LA16_N": "AR11",
        "LA17_P": "AN21",
        "LA17_N": "AP21",
        "LA18_P": "AM20",
        "LA18_N": "AN20",
		"LA19_P": "AU20",
        "LA19_N": "AU19",
		"LA20_P": "AR17",
        "LA20_N": "AT17",
        "LA21_P": "AL19",
        "LA21_N": "AM19",
        "LA22_P": "AR19",
        "LA22_N": "AT19",
        "LA23_P": "AM18",
        "LA23_N": "AN18",
        "LA24_P": "AL22",
        "LA24_N": "AM22",
        "LA25_P": "AL21",
        "LA25_N": "AL20",
        "LA26_P": "AR22",
        "LA26_N": "AT22",
        "LA27_P": "AR21",
        "LA27_N": "AT21",
        "LA28_P": "AJ18",
        "LA28_N": "AK18",
        "LA29_P": "AK22",
        "LA29_N": "AK21",
        "LA30_P": "AG20",
        "LA30_N": "AH20",
        "LA31_P": "AJ20",
        "LA31_N": "AJ19",
        "LA32_P": "AF20",
        "LA32_N": "AF19",
        "LA33_P": "AG18",
        "LA33_N": "AH18",

        # FMCP_HSPC_REFCLK_M2C_(N/P)     IOSTANDARD : LVDS
        "REFCLK_M2C_N": "AP11",
        "REFCLK_M2C_P": "AN11",

        # FMCP_HSPC_REFCLK_C2M_(N/P)     IOSTANDARD : LVDS
        "REFCLK_M2C_N": "AW11",
        "REFCLK_M2C_P": "AV11",

        }
    ),
]


class Platform(XilinxPlatform):
    default_clk_name = "clk100"
    default_clk_period = 10.0

    def __init__(self):
        XilinxPlatform.__init__(self, "xczu28dr-ffvg1517-2-e", _io, _connectors, toolchain="vivado")
        # crowbar project command to add BOARD_PART in order to avoid IP warnings/errors upon import
        self.toolchain.project_commands = ["set_property BOARD_PART xilinx.com:zcu111:part0:1.1 [current_project]"]
    
    # Traverse path and add all ip files
    def add_ip_dir(self, path):
        dir_files = []
        for root, dirs, files in os.walk(path):
            for filename in files:
                dir_files.append(os.path.join(root, filename))
        for filename in dir_files:
            ext = filename.split('.')[-1]
            if ext == "xci":
                self.add_ip(filename)

    def add_base_tcl(self, tcl_path, build_dir="build", build_name="top"):
        self.toolchain.project_commands += ["source {}".format(tcl_path)]
        self.toolchain.project_commands += ["generate_target all [get_files \"./{}/{}.srcs/sources_1/bd/base/base.bd\"]".format(build_dir, build_name)]
        self.toolchain.project_commands += ["make_wrapper -files [get_files ./{}.srcs/sources_1/bd/base/base.bd] -top".format(build_name)]
        self.toolchain.project_commands += ["add_files -norecurse ./{}.srcs/sources_1/bd/base/hdl/base_wrapper.v".format(build_name)]
        self.toolchain.project_commands += ["update_compile_order -fileset sources_1"]
        # self.toolchain.pre_synthesis_commands += ["update_compile_order -fileset sources_1"]
        # self.toolchain.pre_synthesis_commands += ["close_project"]
        # self.toolchain.pre_synthesis_commands += ["open_project {}.xpr".format(build_name)]
        # self.toolchain.pre_synthesis_commands += ["update_compile_order -fileset sources_1"]
        # self.toolchain.pre_synthesis_commands += ["set_property top {} [current_fileset]".format(build_name)]

    def create_programmer(self):
        return VivadoProgrammer()