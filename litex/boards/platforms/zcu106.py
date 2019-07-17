from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform, VivadoProgrammer

# All IO pins/ports on zcu106 device.
# Above each section is listed the format for how to Ctrl-F to that 
# specfic pin set in the ZCU106.xdc file. Swappable parts are surrounded
# by ().
_io = [
    # GPIO_LED_(#)_LS
    ("user_led", 0, Pins("AL11"), IOStandard("LVCMOS12")),
    ("user_led", 1, Pins("AL13"), IOStandard("LVCMOS12")),
    ("user_led", 2, Pins("AK13"), IOStandard("LVCMOS12")),
    ("user_led", 3, Pins("AE15"), IOStandard("LVCMOS12")),
    ("user_led", 4, Pins("AM8"), IOStandard("LVCMOS12")),
    ("user_led", 5, Pins("AM9"), IOStandard("LVCMOS12")),
    ("user_led", 6, Pins("AM10"), IOStandard("LVCMOS12")),
    ("user_led", 7, Pins("AM11"), IOStandard("LVCMOS12")),

    # CPU_RESET
    ("cpu_reset", 0, Pins("G13"), IOStandard("LVCMOS18")),

    # GPIO_SW_(direction)
    ("user_btn_c", 0, Pins("AL10"), IOStandard("LVCMOS12")),
    ("user_btn_n", 0, Pins("AG13"), IOStandard("LVCMOS12")),
    ("user_btn_s", 0, Pins("AP20"), IOStandard("LVCMOS12")),
    ("user_btn_w", 0, Pins("AK12"), IOStandard("LVCMOS12")),
    ("user_btn_e", 0, Pins("AC14"), IOStandard("LVCMOS12")),

    # GPIO_DIP_SW(#)
    ("user_dip_sw", 0, Pins("A17"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 1, Pins("A16"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 2, Pins("B16"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 3, Pins("B15"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 4, Pins("A15"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 5, Pins("A14"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 6, Pins("B14"), IOStandard("LVCMOS18")),
    ("user_dip_sw", 7, Pins("B13"), IOStandard("LVCMOS18")),
   
    # CLK_125_(N/P) 125 Mhz
    ("clk125", 0,
        Subsignal("p", Pins("H9")),
        Subsignal("n", Pins("G9")),
        IOStandard("LVDS")
    ),

    # CLK_74_25_(N/P) 74.25 Mhz
    ("clk_74_25", 0,
        Subsignal("p", Pins("D15")),
        Subsignal("n", Pins("D14")),
        IOStandard("LVDS")
    ),

    # USER_SI570_(N/P)
    ("user_si570", 0,
        Subsignal("p", Pins("AH12")),
        Subsignal("n", Pins("AJ12")),
        IOStandard("DIFF_SSTL12")
    ),

    # SYS_MON
    ("sysmon", 0,
        Subsignal("scl", Pins("A22")),
        Subsignal("sda", Pins("B20")),
        IOStandard("LVCMOS18")
    ),

    # TRACE
    ("trace", 0,
        Subsignal("data", Pins(
            "H6 G6 H7 E1 D1 C1 B1 A3"
            "D2 C2 C3 B3 C4 B4 E4 D4"
            )),
        Subsignal("dbgrq", Pins("E5")),
        Subsignal("srst_b", Pins("F6")),
        Subsignal("tdo", Pins("D5")),
        Subsignal("rtck", Pins("D6")),
        Subsignal("tck", Pins("A5")),
        Subsignal("tms", Pins("B5")),
        Subsignal("tdi", Pins("F4")),
        Subsignal("trst_b", Pins("F5")),
        Subsignal("clka", Pins("E2")),
        Subsignal("dbgack", Pins("E3")),
        Subsignal("exttrig", Pins("A2")),
        Subsignal("ctl", Pins("K8")),
        IOStandard("LVCMOS33")
    ),

    # PL_I2C0_(SCL/SDC)_LS
    ("i2c", 0,
        Subsignal("scl", Pins("AE19")),
        Subsignal("sda", Pins("AH23")),
        IOStandard("LVCMOS12")
    ),

    # PL_I2C1_(SCL/SDC)_LS
    ("i2c", 1,
        Subsignal("scl", Pins("AH19")),
        Subsignal("sda", Pins("AL21")),
        IOStandard("LVCMOS12")
    ),

    # UART2_(CTS/RTS)_B
    # UART2_(RXD/TXD)_FPGA_(RXD/TXD)
    # Double-check serial TX/RX directions!!
    ("serial", 0,
        Subsignal("cts", Pins("AP17")),
        Subsignal("rts", Pins("AM15")),
        Subsignal("tx", Pins("AH17")),
        Subsignal("rx", Pins("AL17")),
        IOStandard("LVCMOS12")
	),

    # MSP430
    ("msp430", 0,
        Subsignal("gpio", Pins("J6 J7 J9 K9"), IOStandard("LVCMOS33")),
        Subsignal("uca_rx", Pins("AA17"), IOStandard("LVCMOS12")),
        Subsignal("uca_tx", Pins("AH16"), IOStandard("LVCMOS12"))
	),

    # PCIE
    ("pcie", 0,
        Subsignal("perst_b", Pins("L8"), IOStandard("LVCMOS33")),
        Subsignal("wake_b", Pins("L10"), IOStandard("LVCMOS33")),
        Subsignal("rx_p", Pins("AE2 AF4 AG2 AJ2")),
        Subsignal("rx_n", Pins("AE1 AF3 AG1 AJ1")),
        Subsignal("tx_p", Pins("AD4 AE6 AG6 AH4")),
        Subsignal("tx_n", Pins("AD3 AE5 AG5 AH3")),
        Subsignal("clk_p", Pins("AB8")),
        Subsignal("clk_n", Pins("AB7"))
    ),

    # HDMI
    ("hdmi", 0,
        Subsignal("tx_lvds_out_p", Pins("G21"), IOStandard("LVDS")),
        Subsignal("tx_lvds_out_n", Pins("F21"), IOStandard("LVDS")),
        Subsignal("tx_en", Pins("N11"), IOStandard("LVCMOS33")),
        Subsignal("tx_cec", Pins("M12"), IOStandard("LVCMOS33")),
        Subsignal("tx_hpd", Pins("N13"), IOStandard("LVCMOS33")),
        Subsignal("tx_src_scl", Pins("N8"), IOStandard("LVCMOS33")),
        Subsignal("tx_src_sda", Pins("N9"), IOStandard("LVCMOS33")),
        Subsignal("tx_p", Pins("AN6 AM4 AL6")),
        Subsignal("tx_n", Pins("AN5 AM3 AL5")),
        Subsignal("rx_pwr_det", Pins("M8"), IOStandard("LVCMOS33")),
        Subsignal("rx_hpd", Pins("M10"), IOStandard("LVCMOS33")),
        Subsignal("rx_snk_scl", Pins("M9"), IOStandard("LVCMOS33")),
        Subsignal("rx_snk_sda", Pins("M11"), IOStandard("LVCMOS33")),
        Subsignal("rx_p", Pins("AP4 AN2 AL2")),
        Subsignal("rx_n", Pins("AP3 AN1 AL1")),
        Subsignal("rx_clk_p", Pins("AC10")),
        Subsignal("rx_clk_n", Pins("AC9")),
        Subsignal("ctl_scl", Pins("N12"), IOStandard("LVCMOS33")),
        Subsignal("ctl_sda", Pins("P12"), IOStandard("LVCMOS33")),
        Subsignal("rec_clock_p", Pins("G14"), IOStandard("LVDS")),
        Subsignal("rec_clock_n", Pins("F13"), IOStandard("LVDS")),
        Subsignal("si5324_lol", Pins("G8"), IOStandard("LVCMOS33")),
        Subsignal("si5324_rst", Pins("H8"), IOStandard("LVCMOS33")),
        Subsignal("si5324_out_p", Pins("AD8")),
        Subsignal("si5324_out_n", Pins("AD7"))
    ),

    # DDR4
    ("ddram", 0,
        Subsignal("a", Pins(
            "AG11 AJ10 AL8 AK10 AH8 AJ9 AG8 AH9 AG10",
            "AH13 AG9 AM13 AF8"),
            IOStandard("SSTL12")),
        Subsignal("ba", Pins("AK8 AL12"), IOStandard("SSTL12")),
        Subsignal("bg", Pins("AE14"), IOStandard("SSTL12")),
        Subsignal("ras_b", Pins("AF11"), IOStandard("SSTL12")), # A16
        Subsignal("cas_b", Pins("AE12 "), IOStandard("SSTL12")), # A15
        Subsignal("we_b", Pins("AC12"), IOStandard("SSTL12")), # A14
        Subsignal("cs_b", Pins("AD12"), IOStandard("SSTL12")),
        Subsignal("act_b", Pins("AD14"), IOStandard("SSTL12")),
        Subsignal("par", Pins("AC13"), IOStandard("SSTL12")),
        Subsignal("dm", Pins("AH18 AD15 AM16 AP18 AE18 AH22 AL20 AP19"),
            IOStandard("POD12_DCI")),
        Subsignal("dq", Pins(
            "AF16 AF18 AG15 AF17 AF15 AG18 AG14 AE17",
            "AA14 AC16 AB15 AD16 AB16 AC17 AB14 AD17",

            "AJ16 AJ17 AL15 AK17 AJ15 AK18 AL16 AL18",
            "AP13 AP16 AP15 AN16 AN13 AM18 AN17 AN18",

            "AB19 AD19 AC18 AC19 AA20 AE20 AA19 AD20",
            "AF22 AH21 AG19 AG21 AE24 AG20 AE23 AF21",

            "AL22 AJ22 AL23 AJ21 AK20 AJ19 AK19 AJ20",
            "AP22 AN22 AP21 AP23 AM19 AM23 AN19 AN23",
            ),
            IOStandard("POD12_DCI")),
        Subsignal("dqs_t", Pins("AH14 AA16 AK15 AM14 AA18 AF23 AK22 AM21"),
            IOStandard("DIFF_POD12")),
        Subsignal("dqs_c", Pins("AJ14 AA15 AK14 AN14 AB18 AG23 AK23 AN21"),
            IOStandard("DIFF_POD12")),
        Subsignal("ck_t", Pins("AH11"), IOStandard("DIFF_POD12")),
        Subsignal("ck_c", Pins("AJ11"), IOStandard("DIFF_POD12")),
        Subsignal("cke", Pins("AB13"), IOStandard("SSTL12")),
        Subsignal("odt", Pins("AF10"), IOStandard("SSTL12")),
        Subsignal("reset_b", Pins("AF12"), IOStandard("LVCMOS12"))
    ),

    # SDI
    ("sdi", 0,
        Subsignal("sclk", Pins("B21"), IOStandard("LVCMOS18")),
        Subsignal("miso", Pins("H23"), IOStandard("LVCMOS18")),
        Subsignal("mosi", Pins("L21"), IOStandard("LVCMOS18")),
        Subsignal("cs_rclkr", Pins("A9"), IOStandard("LVCMOS18")),
        Subsignal("cs_rcvr", Pins("J20"), IOStandard("LVCMOS18")),
        Subsignal("cs_drvr", Pins("J19"), IOStandard("LVCMOS18")),
        Subsignal("xalarm_rx", Pins("E13"), IOStandard("LVCMOS18")),
        Subsignal("xalarm_tx", Pins("C14"), IOStandard("LVCMOS18")),
        Subsignal("mgt_rx_p", Pins("AC2")),
        Subsignal("mgt_rx_n", Pins("AC1")),
        Subsignal("mgt_tx_p", Pins("AC6")),
        Subsignal("mgt_tx_n", Pins("AC5"))
    ),

    # AES
    ("aes", 0,
        Subsignal("in", Pins("G7"), IOStandard("LVCMOS33")),
        Subsignal("out_p", Pins("AE13"), IOStandard("DIFF_SSTL12")),
        Subsignal("out_n", Pins("AF13"), IOStandard("DIFF_SSTL12"))
    ),

    # USER_MGT_SI570_CLOCK(#)_C_(N/P)
    ("user_mgt_si570_clock_c", 1,
        Subsignal("p", Pins("U10")),
        Subsignal("n", Pins("U9"))
    ),
    ("user_mgt_si570_clock_c", 2,
        Subsignal("p", Pins("R10")),
        Subsignal("n", Pins("R9"))
    ),

    # USER_SMA_MGT_CLOCK_C_(N/P)
    ("user_sma_mgt_clock_c", 0,
        Subsignal("p", Pins("AA10")),
        Subsignal("n", Pins("AA9"))
    ),

    # SMA_MGT_TX_(N/P)
    ("user_sma_mgt_tx", 0,
        Subsignal("p", Pins("AA6")),
        Subsignal("n", Pins("AA5"))
    ),

    # SMA_MGT_RX_C_(N/P)
    ("user_sma_mgt_rx", 0,
        Subsignal("p", Pins("AB4")),
        Subsignal("n", Pins("AB3"))
    ),

    # SFP(#)_TX_(N/P)
    ("sfp_tx", 0,
        Subsignal("p", Pins("Y4")),
        Subsignal("n", Pins("Y3"))
    ),
    ("sfp_tx", 1,
        Subsignal("p", Pins("W6")),
        Subsignal("n", Pins("W5"))
    ),

    # SFP(#)_RX_(N/P)
    ("sfp_rx", 0,
        Subsignal("p", Pins("AA2")),
        Subsignal("n", Pins("AA1"))
    ),
    ("sfp_rx", 1,
        Subsignal("p", Pins("W2")),
        Subsignal("n", Pins("W1"))
    ),

    # SFP(#)_TX_DISABLE_N
    ("sfp_tx_disable_n", 0, Pins("AE22"), IOStandard("LVCMOS12")),
    ("sfp_tx_disable_n", 1, Pins("AF20"), IOStandard("LVCMOS12")),

    # SFP_REC_CLOCK_C_(N/P)
    ("sfp_rec_clock_c", 0,
        Subsignal("p", Pins("H11")),
        Subsignal("n", Pins("G11")),
        IOStandard("LVDS")
    ),

    # SFP_SI5328_OUT_C_(N/P)
    ("sfp_si5328_out_c", 0,
        Subsignal("p", Pins("W10")),
        Subsignal("n", Pins("W9"))
    ),

    # PMOD0_(#)_LS
    ("pmod0", 0, Pins("B23"), IOStandard("LVCMOS18")),
    ("pmod0", 1, Pins("A23"), IOStandard("LVCMOS18")),
    ("pmod0", 2, Pins("F25"), IOStandard("LVCMOS18")),
    ("pmod0", 3, Pins("E20"), IOStandard("LVCMOS18")),
    ("pmod0", 4, Pins("K24"), IOStandard("LVCMOS18")),
    ("pmod0", 5, Pins("L23"), IOStandard("LVCMOS18")),
    ("pmod0", 6, Pins("L22"), IOStandard("LVCMOS18")),
    ("pmod0", 7, Pins("D7"), IOStandard("LVCMOS18")),

    # PMOD1_(#)_LS
    ("pmod1", 0, Pins("AN8"), IOStandard("LVCMOS12")),
    ("pmod1", 1, Pins("AN9"), IOStandard("LVCMOS12")),
    ("pmod1", 2, Pins("AP11"), IOStandard("LVCMOS12")),
    ("pmod1", 3, Pins("AN11"), IOStandard("LVCMOS12")),
    ("pmod1", 4, Pins("AP9"), IOStandard("LVCMOS12")),
    ("pmod1", 5, Pins("AP10"), IOStandard("LVCMOS12")),
    ("pmod1", 6, Pins("AP12"), IOStandard("LVCMOS12")),
    ("pmod1", 7, Pins("AN12"), IOStandard("LVCMOS12")),
]

_connectors = [
    ("FMC_HPC0", {
        "DP0_M2C_P": "R2",
        "DP0_M2C_N": "R1",
        "DP0_C2M_P": "R6",
        "DP0_C2M_N": "R5",
        "DP1_M2C_P": "U2",
        "DP1_M2C_N": "U1",
        "DP1_C2M_P": "T4",
        "DP1_C2M_N": "T3",
        "DP2_M2C_P": "P4",
        "DP2_M2C_N": "P3",
        "DP2_C2M_P": "N6",
        "DP2_C2M_N": "N5",
        "DP3_M2C_P": "V4",
        "DP3_M2C_N": "V3",
        "DP3_C2M_P": "U6",
        "DP3_C2M_N": "U5",
        "DP4_M2C_P": "G2",
        "DP4_M2C_N": "G1",
        "DP4_C2M_P": "H4",
        "DP4_C2M_N": "H3",
        "DP5_M2C_P": "L2",
        "DP5_M2C_N": "L1",
        "DP5_C2M_P": "L6",
        "DP5_C2M_N": "L5",
        "DP6_M2C_P": "N2",
        "DP6_M2C_N": "N1",
        "DP6_C2M_P": "M4",
        "DP6_C2M_N": "M3",
        "DP7_M2C_P": "J2",
        "DP7_M2C_N": "J1",
        "DP7_C2M_P": "K4",
        "DP7_C2M_N": "K3",
        "LA00_CC_P": "F17",
        "LA00_CC_N": "F16",
        "LA01_CC_P": "H18",
        "LA01_CC_N": "H17",
        "LA02_P": "L20",
        "LA02_N": "K20",
        "LA03_P": "K19",
        "LA03_N": "K18",
        "LA04_P": "L17",
        "LA04_N": "L16",
        "LA05_P": "K17",
        "LA05_N": "J17",
        "LA06_P": "H19",
        "LA06_N": "G19",
        "LA07_P": "J16",
        "LA07_N": "J15",
        "LA08_P": "E18",
        "LA08_N": "E7",
        "LA09_P": "H16",
        "LA09_N": "G16",
        "LA10_P": "L15",
        "LA10_N": "K15",
        "LA11_P": "A13",
        "LA11_N": "A12",
        "LA12_P": "G18",
        "LA12_N": "F18",
        "LA13_P": "G15",
        "LA13_N": "F15",
        "LA14_P": "C13",
        "LA14_N": "C12",
        "LA15_P": "D16",
        "LA15_N": "C16",
        "LA16_P": "D17",
        "LA16_N": "C17",
        "LA17_CC_P": "F11",
        "LA17_CC_N": "E10",
        "LA18_CC_P": "D11",
        "LA18_CC_N": "D10",
        "LA19_P": "D12",
        "LA19_N": "C11",
        "LA20_P": "F12",
        "LA20_N": "E12",
        "LA21_P": "B10",
        "LA21_N": "A10",
        "LA22_P": "H13",
        "LA22_N": "H12", 
        "LA23_P": "B11",
        "LA23_N": "A11",
        "LA24_P": "B6",
        "LA24_N": "A6",
        "LA25_P": "C7",
        "LA25_N": "C6",
        "LA26_P": "B9",
        "LA26_N": "B8",
        "LA27_P": "A8",
        "LA27_N": "A7",
        "LA28_P": "M13",
        "LA28_N": "L13",
        "LA29_P": "K10",
        "LA29_N": "J10",
        "LA30_P": "E9",
        "LA30_N": "D9",
        "LA31_P": "F7",
        "LA31_N": "E7",
        "LA32_P": "F8",
        "LA32_N": "E8",
        "LA33_P": "C9",
        "LA33_N": "C8",
        "CLK0_M2C_P": "E15",
        "CLK0_M2C_N": "E14",
        "CLK1_M2C_P": "G10",
        "CLK1_M2C_N": "F10",        
        "GBTCLK0_M2C_C_P": "V8",
        "GBTCLK0_M2C_C_N": "V7",
        "GBTCLK1_M2C_C_P": "T8",
        "GBTCLK1_M2C_C_N": "T7",
        }
    ),
    ("FMC_HPC1", {
        "DP0_M2C_P": "AK4",
        "DP0_M2C_N": "AK3",
        "DP0_C2M_P": "AJ6",
        "DP0_C2M_N": "AJ5",
        "LA00_CC_P": "B18",
        "LA00_CC_N": "B19",
        "LA01_CC_P": "E24",
        "LA01_CC_N": "D24",
        "LA02_P": "K22",
        "LA02_N": "K23",
        "LA03_P": "J21",
        "LA03_N": "J22",
        "LA04_P": "J24",
        "LA04_N": "H24",
        "LA05_P": "G25",
        "LA05_N": "G26",
        "LA06_P": "H21",
        "LA06_N": "H22",
        "LA07_P": "D22",
        "LA07_N": "C23",
        "LA08_P": "J25",
        "LA08_N": "H26",
        "LA09_P": "G20",
        "LA09_N": "F20",
        "LA10_P": "F22",
        "LA10_N": "E22",
        "LA11_P": "A20",
        "LA11_N": "A21",
        "LA12_P": "E19",
        "LA12_N": "D19",
        "LA13_P": "C21",
        "LA13_N": "C22",
        "LA14_P": "D20",
        "LA14_N": "D21",
        "LA15_P": "A18",
        "LA15_N": "A19",
        "LA16_P": "C18",
        "LA16_N": "C19",
        "CLK0_M2C_P": "F23",
        "CLK0_M2C_N": "E23",    
        "GBTCLK0_M2C_C_P": "Y8",
        "GBTCLK0_M2C_C_N": "Y7",
        }
    )
]

class Platform(XilinxPlatform):
    default_clk_name = "clk125"
    default_clk_period = 8.0

    def __init__(self):
        XilinxPlatform.__init__(self, "xczu7ev-ffvc1156-2-e", _io, _connectors, toolchain="vivado")

    def create_programmer(self):
        return VivadoProgrammer()