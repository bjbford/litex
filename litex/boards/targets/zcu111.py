#!/usr/bin/env python3

import os
import argparse
import importlib

from migen import *
from litex.build.generic_platform import tools
from litex.soc.integration.soc_core import *
from litex.soc.integration.cpu_interface import get_csr_header
from litex.soc.interconnect import wishbone
from litex.soc.interconnect import axi_lite, axi
from litex.soc.cores.clock import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import csr_bus

from litex.boards.platforms import zcu111


# class _CRG(Module):
#     def __init__(self, platform):
#         self.clock_domains.cd_sys = ClockDomain()
#         self.clock_domains.cd_sys4x = ClockDomain(reset_less=True)
#         self.clock_domains.cd_clk200 = ClockDomain()
#         self.clock_domains.cd_ic = ClockDomain()

#         clk125 = platform.request("clk125")
#         clk125_ibufds = Signal()
#         clk125_buffered = Signal()
#         pll_locked = Signal()
#         pll_fb = Signal()
#         pll_sys4x = Signal()
#         pll_clk200 = Signal()
#         self.specials += [
#             Instance("IBUFDS", i_I=clk125.p, i_IB=clk125.n, o_O=clk125_ibufds),
#             Instance("BUFG", i_I=clk125_ibufds, o_O=clk125_buffered),
#             Instance("PLLE2_BASE", name="crg_main_mmcm",
#                 i_RST=platform.request("cpu_reset"),
#                 p_STARTUP_WAIT="FALSE", o_LOCKED=pll_locked,

#                 # VCO @ 1GHz
#                 p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=8.0,
#                 p_CLKFBOUT_MULT=8, p_DIVCLK_DIVIDE=1,
#                 i_CLKIN1=clk125_buffered, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,

#                 # 500MHz
#                 p_CLKOUT0_DIVIDE=2, p_CLKOUT0_PHASE=0.0, o_CLKOUT0=pll_sys4x,

#                 # 200MHz
#                 p_CLKOUT1_DIVIDE=5, p_CLKOUT1_PHASE=0.0, o_CLKOUT1=pll_clk200,
#             ),
#             Instance("BUFGCE_DIV", name="main_bufgce_div",
#                 p_BUFGCE_DIVIDE=4,
#                 i_CE=1, i_I=pll_sys4x, o_O=self.cd_sys.clk),
#             Instance("BUFGCE", name="main_bufgce",
#                 i_CE=1, i_I=pll_sys4x, o_O=self.cd_sys4x.clk),
#             Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk),
#             AsyncResetSynchronizer(self.cd_clk200, ~pll_locked),
#         ]

#         ic_reset_counter = Signal(max=64, reset=63)
#         ic_reset = Signal(reset=1)
#         self.sync.clk200 += \
#             If(ic_reset_counter != 0,
#                 ic_reset_counter.eq(ic_reset_counter - 1)
#             ).Else(
#                 ic_reset.eq(0)
#             )
#         ic_rdy = Signal()
#         ic_rdy_counter = Signal(max=64, reset=63)
#         self.cd_sys.rst.reset = 1
#         self.comb += self.cd_ic.clk.eq(self.cd_sys.clk)
#         self.sync.ic += [
#             If(ic_rdy,
#                 If(ic_rdy_counter != 0,
#                     ic_rdy_counter.eq(ic_rdy_counter - 1)
#                 ).Else(
#                     self.cd_sys.rst.eq(0)
#                 )
#             )
#         ]
#         self.specials += [
#             Instance("IDELAYCTRL", p_SIM_DEVICE="ULTRASCALE",
#                      i_REFCLK=ClockSignal("clk200"), i_RST=ic_reset,
#                      o_RDY=ic_rdy),
#             AsyncResetSynchronizer(self.cd_ic, ic_reset)
#         ]

# class BaseSoC(SoCCore):
#     def __init__(self, platform, **kwargs):
#         sys_clk_freq = int(1e9/platform.default_clk_period)
#         SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
#             integrated_rom_size=0x8000,
#             integrated_main_ram_size=16*1024,
#             **kwargs)
        # self.submodules.crg = _CRG(platform)

class SoCZynqUPlus(SoCCore):
    SoCCore.mem_map["csr"] = 0x00000000
    def __init__(self, platform, clk_freq, reset, **kwargs):
        SoCCore.__init__(self, platform, clk_freq, cpu_type=None, shadow_base=0x00000000, **kwargs)

        # PS
        self.axi_0 = axi_lite.Interface(data_width=32, address_width=40)
        # ps_ddram_pads = platform.request("ps_ddram")
        self.btns = Signal(5)
        self.btns[0].eq(platform.request("user_sw_c"))
        self.btns[1].eq(platform.request("user_sw_n"))
        self.btns[2].eq(platform.request("user_sw_s"))
        self.btns[3].eq(platform.request("user_sw_w"))
        self.btns[4].eq(platform.request("user_sw_e"))
        self.sws = Signal(8)
        for i in range(8):
            self.sws[i].eq(platform.request("user_dip_sw"))
        self.i2c = platform.request("i2c")

        # Instantiate the base overlay wrapper provided by Vivado
        self.specials.base = Instance("base_wrapper",
                # o_aclk_100 = ClockSignal(),
                # o_aclk_100 = self.aclk_100.clk,
                i_reset = reset,
                # o_peripheral_aresetn = ResetSignal(),
                # o_peripheral_aresetn = self.aclk_100.rst,
                io_fmch_iic_scl_io = self.i2c.scl,
                io_fmch_iic_sda_io = self.i2c.sda,
                i_gpio_btns_tri_i = self.btns,
                # o_gpio_leds_tri_o = self.leds,
                i_gpio_sws_tri_i = self.sws,
                o_m_axi_config_0_araddr = self.axi_0.ar.addr,
                o_m_axi_config_0_arprot = self.axi_0.ar.prot,
                i_m_axi_config_0_arready = self.axi_0.ar.ready,
                o_m_axi_config_0_arvalid = self.axi_0.ar.valid,
                o_m_axi_config_0_awaddr = self.axi_0.aw.addr,
                o_m_axi_config_0_awprot = self.axi_0.aw.prot,
                i_m_axi_config_0_awready = self.axi_0.aw.ready,
                o_m_axi_config_0_awvalid = self.axi_0.aw.valid,
                o_m_axi_config_0_bready = self.axi_0.b.ready,
                i_m_axi_config_0_bresp = self.axi_0.b.resp,
                i_m_axi_config_0_bvalid = self.axi_0.b.valid,
                i_m_axi_config_0_rdata = self.axi_0.r.data,
                o_m_axi_config_0_rready = self.axi_0.r.ready,
                i_m_axi_config_0_rresp = self.axi_0.r.resp,
                i_m_axi_config_0_rvalid = self.axi_0.r.valid,
                o_m_axi_config_0_wdata = self.axi_0.w.data,
                i_m_axi_config_0_wready = self.axi_0.w.ready,
                o_m_axi_config_0_wstrb = self.axi_0.w.strb,
                o_m_axi_config_0_wvalid = self.axi_0.w.valid,
                # m_axi_config_1_araddr = 0,
                # m_axi_config_1_arprot = 0,
                # m_axi_config_1_arready = 0,
                # m_axi_config_1_arvalid = 0,
                # m_axi_config_1_awaddr = 0,
                # m_axi_config_1_awprot = 0,
                # m_axi_config_1_awready = 0,
                # m_axi_config_1_awvalid = 0,
                # m_axi_config_1_bready = 0,
                # m_axi_config_1_bresp = 0,
                # m_axi_config_1_bvalid = 0,
                # m_axi_config_1_rdata = 0,
                # m_axi_config_1_rready = 0,
                # m_axi_config_1_rresp = 0,
                # m_axi_config_1_rvalid = 0,
                # m_axi_config_1_wdata = 0,
                # m_axi_config_1_wready = 0,
                # m_axi_config_1_wstrb = 0,
                # m_axi_config_1_wvalid = 0,
                # m_axi_config_2_araddr = 0,
                # m_axi_config_2_arprot = 0,
                # m_axi_config_2_arready = 0,
                # m_axi_config_2_arvalid = 0,
                # m_axi_config_2_awaddr = 0,
                # m_axi_config_2_awprot = 0,
                # m_axi_config_2_awready = 0,
                # m_axi_config_2_awvalid = 0,
                # m_axi_config_2_bready = 0,
                # m_axi_config_2_bresp = 0,
                # m_axi_config_2_bvalid = 0,
                # m_axi_config_2_rdata = 0,
                # m_axi_config_2_rready = 0,
                # m_axi_config_2_rresp = 0,
                # m_axi_config_2_rvalid = 0,
                # m_axi_config_2_wdata = 0,
                # m_axi_config_2_wready = 0,
                # m_axi_config_2_wstrb = 0,
                # m_axi_config_2_wvalid = 0,
        )
        # platform.add_ip(os.path.join("ip", ps_name + ".xci"))

        # CSR-2 bus
        self.csr = csr_bus.Interface(data_width=32, address_width=38)
        # AXI-to-CSR Bridge
        self.submodules.bridge = axi_lite.AXILite2CSR(self.axi_0, self.csr)
        self.submodules.csrbank = csr_bus.CSRBankArray(self, self.map_csr, data_width=32, address_width=38)
        self.submodules.csrcon = csr_bus.Interconnect(self.csr, self.csrbank.get_buses())

    def map_csr(self, name, memory):
        return {"csrmodule": 0}[name]

        # AXI to Wishbone
        # self.wb_0 = wb_0 = wishbone.Interface()
        # # m_axi_config_0
        # axi2wishbone = axi.AXI2Wishbone(self.axi_0, wb_0, base_address=0x80010000)
        # self.submodules += axi2wishbone
        # self.add_wb_master(wb_0)

    def generate_software_header(self, filename):
        csr_header = get_csr_header(self.get_csr_regions(),
                                    self.get_constants(),
                                    with_access_functions=False)
        tools.write_to_file(filename, csr_header)        


def main():
    parser = argparse.ArgumentParser(description="LiteX SoC port to ZCU111")
    builder_args(parser)
    args = parser.parse_args()

    platform = zcu111.Platform()
    platform.add_source("/home/casper/sdmay19-41/hdl/base_overlay/base_wrapper.v")
    reset = platform.request("cpu_reset")
    sys_clk_freq = int(1e9/platform.default_clk_period)
    soc = SoCZynqUPlus(platform, sys_clk_freq, reset)
    # builder = Builder(soc, **builder_argdict(args))
    builder = Builder(soc, output_dir="blah", csr_csv="blah/csr.csv")
    builder.build()


if __name__ == "__main__":
    main()