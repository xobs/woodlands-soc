#!/usr/bin/env python3
# This variable defines all the external programs that this module
# relies on.  lxbuildenv reads this variable in order to ensure
# the build will finish without exiting due to missing third-party
# programs.
LX_DEPENDENCIES = ["riscv", "vivado"]

# Import lxbuildenv to integrate the deps/ directory
import lxbuildenv

from migen import Module, Signal, ClockDomain, Instance, If, ClockSignal, ResetSignal
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.fhdl.decorators import ClockDomainsRenamer

from litex.build.generic_platform import Pins, IOStandard, Subsignal, Misc
from litex.soc.integration.builder import Builder
from litex.soc.integration.soc_sdram import SoCSDRAM
from litex.soc.integration.soc_core import mem_decoder
from litex.build.xilinx import XilinxPlatform, VivadoProgrammer
from litex.build.xilinx.vivado import XilinxVivadoToolchain

from litedram.modules import MT41K128M16
from litedram.phy import a7ddrphy
from litedram.core import ControllerSettings

from liteeth.phy.mii import LiteEthPHYMII
from liteeth.core.mac import LiteEthMAC
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone
from liteeth.common import convert_ip

_io = [
    ("clk100", 0, Pins("E3"), IOStandard("LVCMOS33")),

    ("cpu_reset", 0, Pins("C2"), IOStandard("LVCMOS33")),
    
    ("serial", 1,
        Subsignal("tx", Pins("D10")), # hax 7
        Subsignal("rx", Pins("A9")), # hax 8
        IOStandard("LVCMOS33")
    ),

    ("ddram", 0,
        Subsignal("a", Pins(
            "R2 M6 N4 T1 N6 R7 V6 U7",
            "R8 V7 R6 U6 T6 T8"),
            IOStandard("SSTL135")),
        Subsignal("ba", Pins("R1 P4 P2"), IOStandard("SSTL135")),
        Subsignal("ras_n", Pins("P3"), IOStandard("SSTL135")),
        Subsignal("cas_n", Pins("M4"), IOStandard("SSTL135")),
        Subsignal("we_n", Pins("P5"), IOStandard("SSTL135")),
        Subsignal("cs_n", Pins("U8"), IOStandard("SSTL135")),
        Subsignal("dm", Pins("L1 U1"), IOStandard("SSTL135")),
        Subsignal("dq", Pins(
            "K5 L3 K3 L6 M3 M1 L4 M2",
            "V4 T5 U4 V5 V1 T3 U3 R3"),
            IOStandard("SSTL135"),
            Misc("IN_TERM=UNTUNED_SPLIT_40")),
        Subsignal("dqs_p", Pins("N2 U2"), IOStandard("DIFF_SSTL135")),
        Subsignal("dqs_n", Pins("N1 V2"), IOStandard("DIFF_SSTL135")),
        Subsignal("clk_p", Pins("U9"), IOStandard("DIFF_SSTL135")),
        Subsignal("clk_n", Pins("V9"), IOStandard("DIFF_SSTL135")),
        Subsignal("cke", Pins("N5"), IOStandard("SSTL135")),
        Subsignal("odt", Pins("R5"), IOStandard("SSTL135")),
        Subsignal("reset_n", Pins("K6"), IOStandard("SSTL135")),
        Misc("SLEW=FAST"),
    ),

    ("eth_ref_clk", 0, Pins("G18"), IOStandard("LVCMOS33")),
    ("eth_clocks", 0,
        Subsignal("tx", Pins("H16")),
        Subsignal("rx", Pins("F15")),
        IOStandard("LVCMOS33"),
    ),
    ("eth", 0,
        Subsignal("rst_n", Pins("C16")),
        Subsignal("mdio", Pins("K13")),
        Subsignal("mdc", Pins("F16")),
        Subsignal("rx_dv", Pins("G16")),
        Subsignal("rx_er", Pins("C17")),
        Subsignal("rx_data", Pins("D18 E17 E18 G17")),
        Subsignal("tx_en", Pins("H15")),
        Subsignal("tx_data", Pins("H14 J14 J13 H17")),
        Subsignal("col", Pins("D17")),
        Subsignal("crs", Pins("G14")),
        IOStandard("LVCMOS33"),
    ),
]

class Platform(XilinxPlatform):
    default_clk_name = "clk100"
    default_clk_period = 10.0

    def __init__(self, toolchain="vivado", programmer="vivado"):
        part = "xc7a35ticsg324-1L"
        XilinxPlatform.__init__(self, part, _io,
                                toolchain=toolchain)
        self.programer = programmer
        self.add_platform_command("set_property INTERNAL_VREF 0.675 [get_iobanks 34]")

    def create_programmer(self):
        if self.programmer == "vivado":
            return VivadoProgrammer(flash_part="n25q128-3.3v-spi-x1_x2_x4")
        else:
            raise ValueError("{} programmer is not supported"
                             .format(self.programmer))

    def do_finalize(self, fragment):
        XilinxPlatform.do_finalize(self, fragment)

class WoodlandsSoC(SoCSDRAM):
    csr_map = {
        "ddrphy": 16,
        # "phy": 17,
        # "mac": 18,
        # "core": 19,
        "ethphy": 17,
        "ethmac": 18,
    }
    csr_map.update(SoCSDRAM.csr_map)

    interrupt_map = {
        "ethmac": 3,
        # "mac": 3,
    }
    interrupt_map.update(SoCSDRAM.interrupt_map)

    mem_map = {
        # "mac": 0x30000000,  # (shadow @0xb0000000)
        "ethmac": 0x30000000,  # (shadow @0xb0000000)
        "vexriscv_debug": 0xf00f0000,
    }
    mem_map.update(SoCSDRAM.mem_map)
    
    class _CRG(Module):
        def __init__(self, platform):
            self.clock_domains.cd_sys = ClockDomain()
            self.clock_domains.cd_sys4x = ClockDomain(reset_less=True)
            self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
            self.clock_domains.cd_clk200 = ClockDomain()
            self.clock_domains.cd_clk50 = ClockDomain()

            clk100 = platform.request("clk100")
            rst = ~platform.request("cpu_reset")

            pll_locked = Signal()
            pll_fb = Signal()
            self.pll_sys = Signal()
            pll_sys4x = Signal()
            pll_sys4x_dqs = Signal()
            pll_clk200 = Signal()
            pll_clk50 = Signal()
            self.specials += [
                Instance("PLLE2_BASE",
                        p_STARTUP_WAIT="FALSE", o_LOCKED=pll_locked,

                        # VCO @ 1600 MHz
                        p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=10.0,
                        p_CLKFBOUT_MULT=16, p_DIVCLK_DIVIDE=1,
                        i_CLKIN1=clk100, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,

                        # 100 MHz
                        p_CLKOUT0_DIVIDE=16, p_CLKOUT0_PHASE=0.0,
                        o_CLKOUT0=self.pll_sys,

                        # 400 MHz
                        p_CLKOUT1_DIVIDE=4, p_CLKOUT1_PHASE=0.0,
                        o_CLKOUT1=pll_sys4x,

                        # 400 MHz dqs
                        p_CLKOUT2_DIVIDE=4, p_CLKOUT2_PHASE=90.0,
                        o_CLKOUT2=pll_sys4x_dqs,

                        # 200 MHz
                        p_CLKOUT3_DIVIDE=8, p_CLKOUT3_PHASE=0.0,
                        o_CLKOUT3=pll_clk200,

                        # 50MHz
                        p_CLKOUT4_DIVIDE=32, p_CLKOUT4_PHASE=0.0,
                        o_CLKOUT4=pll_clk50
                ),
                Instance("BUFG", i_I=self.pll_sys, o_O=self.cd_sys.clk),
                Instance("BUFG", i_I=pll_sys4x, o_O=self.cd_sys4x.clk),
                Instance("BUFG", i_I=pll_sys4x_dqs, o_O=self.cd_sys4x_dqs.clk),
                Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk),
                Instance("BUFG", i_I=pll_clk50, o_O=self.cd_clk50.clk),
                AsyncResetSynchronizer(self.cd_sys, ~pll_locked | rst),
                AsyncResetSynchronizer(self.cd_clk200, ~pll_locked | rst),
                AsyncResetSynchronizer(self.cd_clk50, ~pll_locked | rst),
            ]

            reset_counter = Signal(4, reset=15)
            ic_reset = Signal(reset=1)
            self.sync.clk200 += \
                If(reset_counter != 0,
                    reset_counter.eq(reset_counter - 1)
                ).Else(
                ic_reset.eq(0)
                )
            self.specials += Instance("IDELAYCTRL", i_REFCLK=ClockSignal("clk200"), i_RST=ic_reset)


    def __init__(self, platform, mac_address=0x1337320dbabe, ip_address="10.0.11.2", **kwargs):
        clk_freq = int(100e6)
        SoCSDRAM.__init__(self, platform, clk_freq,
            integrated_rom_size=0x8000,
            integrated_sram_size=0x8000,
            #shadow_base=0x00000000,
            ident="NeTV2 LiteX Base SoC",
            reserve_nmi_interrupt=False,
            cpu_type="vexriscv",
            cpu_variant="debug",
            **kwargs)

        self.submodules.crg = WoodlandsSoC._CRG(platform)

        self.submodules.ddrphy = a7ddrphy.A7DDRPHY(platform.request("ddram"))
        sdram_module = MT41K128M16(self.clk_freq, "1:4")
        self.add_constant("READ_LEVELING_BITSLIP", 3)
        self.add_constant("READ_LEVELING_DELAY", 14)
        self.register_sdram(self.ddrphy,
                            sdram_module.geom_settings,
                            sdram_module.timing_settings,
                            controller_settings=ControllerSettings(with_bandwidth=True,
                                                                   cmd_buffer_depth=8,
                                                                   with_refresh=True))


#         self.submodules.phy = LiteEthPHYMII(self.platform.request("eth_clocks"),
#                                             self.platform.request("eth"))
#         self.submodules.core = LiteEthUDPIPCore(self.phy, mac_address, convert_ip(ip_address), clk_freq)
#         if isinstance(platform.toolchain, XilinxVivadoToolchain):
#             self.crg.cd_sys.clk.attr.add("keep")
#             self.phy.crg.cd_eth_rx.clk.attr.add("keep")
#             self.phy.crg.cd_eth_tx.clk.attr.add("keep")
#             platform.add_platform_command("""
# create_clock -name sys_clk -period 6.0 [get_nets sys_clk]
# create_clock -name eth_rx_clk -period 8.0 [get_nets eth_rx_clk]
# create_clock -name eth_tx_clk -period 8.0 [get_nets eth_tx_clk]
# set_false_path -from [get_clocks sys_clk] -to [get_clocks eth_rx_clk]
# set_false_path -from [get_clocks eth_rx_clk] -to [get_clocks sys_clk]
# set_false_path -from [get_clocks sys_clk] -to [get_clocks eth_tx_clk]
# set_false_path -from [get_clocks eth_tx_clk] -to [get_clocks sys_clk]
# """)
#         self.submodules.etherbone = LiteEthEtherbone(self.core.udp, 1234, mode="master")
#         self.add_wb_master(self.etherbone.wishbone.bus)







        self.submodules.ethphy = LiteEthPHYMII(self.platform.request("eth_clocks"),
                                               self.platform.request("eth"))
        self.submodules.ethmac = LiteEthMAC(phy=self.ethphy, dw=32, interface="wishbone")
        self.add_wb_slave(mem_decoder(self.mem_map["ethmac"]), self.ethmac.bus)
        self.add_memory_region("ethmac", self.mem_map["ethmac"] | self.shadow_base, 0x2000)

        self.crg.cd_sys.clk.attr.add("keep")
        self.ethphy.crg.cd_eth_rx.clk.attr.add("keep")
        self.ethphy.crg.cd_eth_tx.clk.attr.add("keep")
        self.platform.add_period_constraint(self.crg.cd_sys.clk, 10.0)
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_rx.clk, 80.0)
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_tx.clk, 80.0)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.crg.cd_eth_rx.clk,
            self.ethphy.crg.cd_eth_tx.clk)




        # # # Ethernet PHY setup
        # phy = LiteEthPHYMII(platform.request("eth_clocks"),
        #                        platform.request("eth"))
        # phy = ClockDomainsRenamer("clk50")(phy)
        
        # # This acts as a MAC as well as an IP and UDP core.
        # core = LiteEthUDPIPCore(phy, mac_address, convert_ip(ip_address), int(50e6), with_icmp=True)
        # core = ClockDomainsRenamer("clk50")(core)
        # self.submodules += phy, core
        # self.submodules.mac = core.mac
        # self.add_wb_slave(mem_decoder(self.mem_map["mac"]), core.mac.bus)
        # self.add_memory_region("mac", self.mem_map["mac"] | self.shadow_base, 0x2000)

        # # Interface the Wishbone bus with Ethernet
        # etherbone_cd = ClockDomain("etherbone")
        # self.clock_domains += etherbone_cd
        # self.comb += [
        #     etherbone_cd.clk.eq(ClockSignal("sys")),
        #     etherbone_cd.rst.eq(ResetSignal("sys"))
        # ]
        # self.submodules.etherbone = LiteEthEtherbone(core.udp, 1234, mode="master", cd="etherbone")
        # self.add_wb_master(self.etherbone.wishbone.bus)

        # self.platform.add_false_path_constraints(
        #    self.crg.cd_sys.clk,
        #    self.crg.cd_clk50.clk
        # )

        self.register_mem("vexriscv_debug", 0xf00f0000, self.cpu_or_bridge.debug_bus, 0x10)

def main():
    platform = Platform()
    soc = WoodlandsSoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="test/csr.csv")
    for (k,v) in soc.csr_map.items():
        print("CSR[{}]: {}".format(k, v))
    vns = builder.build()
    soc.do_exit(vns)

if __name__ == "__main__":
    main()
