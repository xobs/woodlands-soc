#!/usr/bin/env python3
import lxbuildenv
LX_DEPENDENCIES = "vivado"

import argparse
import os

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.boards.platforms import arty, arty_s7

from litex.soc.integration.soc_core import mem_decoder
from litex.soc.integration.soc_sdram import *
from litex.soc.cores import spi_flash
from litex.soc.integration.builder import *
from litex.soc.cores.uart import *
from litex.soc.interconnect.stream import *

from litedram.modules import MT41K128M16
from litedram.phy import a7ddrphy
from litedram.core.controller import ControllerSettings
from litedram.frontend.bist import LiteDRAMBISTGenerator
from litedram.frontend.bist import LiteDRAMBISTChecker

from liteeth.phy import LiteEthPHY
from liteeth.core.mac import LiteEthMAC

from litex.soc.cores import dna, xadc
#from gateware import led


def csr_map_update(csr_map, csr_peripherals):
    csr_map.update(dict((n, v)
        for v, n in enumerate(csr_peripherals, start=max(csr_map.values()) + 1)))

def period_ns(freq):
    return 1e9/freq


class CRG(Module):
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

        if platform.device[:4] == "xc7a":
            eth_clk = Signal()
            self.specials += [
                Instance("BUFR", p_BUFR_DIVIDE="4", i_CE=1, i_CLR=0, i_I=clk100, o_O=eth_clk),
                Instance("BUFG", i_I=eth_clk, o_O=platform.request("eth_ref_clk")),
            ]


class BaseSoC(SoCSDRAM):
    csr_peripherals = [
        "spiflash",
        "ddrphy",
        "dna",
        "xadc",
        "leds",
        "rgb_led0",
        "rgb_led1",
        "rgb_led2",
        "rgb_led3",
        "generator",
        "checker",
    ]
    csr_map_update(SoCSDRAM.csr_map, csr_peripherals)

    mem_map = {
        "spiflash": 0x20000000,  # (default shadow @0xa0000000)
    }
    mem_map.update(SoCSDRAM.mem_map)

    def __init__(self, platform,
                 with_sdram_bist=True, bist_async=True,
                 spiflash="spiflash",
                 **kwargs):
        clk_freq = int(100e6)
        SoCSDRAM.__init__(self, platform, clk_freq,
            integrated_rom_size=0x8000,
            integrated_sram_size=0x8000,
            ident="Arty LiteX Test SoC", ident_version=True,
            with_uart=False,
            **kwargs)

        self.submodules.crg = CRG(platform)
        self.submodules.dna = dna.DNA()
        self.submodules.xadc = xadc.XADC()

        uart_interfaces = [RS232PHYInterface() for i in range(2)]
        self.submodules.uart = UART(uart_interfaces[0])
        self.submodules.bridge = WishboneStreamingBridge(uart_interfaces[1], self.clk_freq)
        self.add_wb_master(self.bridge.wishbone)

        self.submodules.uart_phy = RS232PHY(platform.request("serial"), self.clk_freq, 115200)
        self.submodules.uart_multiplexer = RS232PHYMultiplexer(uart_interfaces, self.uart_phy)
        self.comb += self.uart_multiplexer.sel.eq(platform.request("user_sw", 0))

        self.crg.cd_sys.clk.attr.add("keep")
        self.platform.add_period_constraint(self.crg.cd_sys.clk, period_ns(100e6))

        # self.submodules.leds = led.ClassicLed(Cat(platform.request("user_led", i)
        #     for i in range(4)))
        # self.submodules.rgb_led0 = led.RGBLed(platform.request("rgb_led", 0))
        # self.submodules.rgb_led1 = led.RGBLed(platform.request("rgb_led", 1))
        # self.submodules.rgb_led2 = led.RGBLed(platform.request("rgb_led", 2))
        # self.submodules.rgb_led3 = led.RGBLed(platform.request("rgb_led", 3))

        # sdram
        self.submodules.ddrphy = a7ddrphy.A7DDRPHY(platform.request("ddram"))
        sdram_module = MT41K128M16(self.clk_freq, "1:4")
        self.register_sdram(self.ddrphy,
                            sdram_module.geom_settings,
                            sdram_module.timing_settings,
                            controller_settings=ControllerSettings(cmd_buffer_depth=8))

        # sdram bist
        if with_sdram_bist:
            generator_user_port = self.sdram.crossbar.get_port(mode="write", clock_domain="clk50" if bist_async else "sys")
            self.submodules.generator = LiteDRAMBISTGenerator(generator_user_port)

            checker_user_port = self.sdram.crossbar.get_port(mode="read", clock_domain="clk50" if bist_async else "sys")
            self.submodules.checker = LiteDRAMBISTChecker(checker_user_port)

        # spi flash
        spiflash_pads = platform.request(spiflash)
        spiflash_pads.clk = Signal()
        self.specials += Instance("STARTUPE2",
                                  i_CLK=0, i_GSR=0, i_GTS=0, i_KEYCLEARB=0, i_PACK=0,
                                  i_USRCCLKO=spiflash_pads.clk, i_USRCCLKTS=0, i_USRDONEO=1, i_USRDONETS=1)
        spiflash_dummy = {
            "spiflash": 9,
            "spiflash4x": 11,
        }
        self.submodules.spiflash = spi_flash.SpiFlash(spiflash_pads, dummy=spiflash_dummy[spiflash], div=2)
        self.add_constant("SPIFLASH_PAGE_SIZE", 256)
        self.add_constant("SPIFLASH_SECTOR_SIZE", 0x10000)
        self.add_wb_slave(mem_decoder(self.mem_map["spiflash"]), self.spiflash.bus)
        self.add_memory_region("spiflash",
         	self.mem_map["spiflash"] | self.shadow_base, 16*1024*1024)


class MiniSoC(BaseSoC):
    csr_peripherals = {
        "ethphy",
        "ethmac",
    }
    csr_map_update(BaseSoC.csr_map, csr_peripherals)

    interrupt_map = {
        "ethmac": 3,
    }
    interrupt_map.update(BaseSoC.interrupt_map)

    mem_map = {
        "ethmac": 0x30000000,  # (shadow @0xb0000000)
    }
    mem_map.update(BaseSoC.mem_map)

    def __init__(self, *args, **kwargs):
        BaseSoC.__init__(self, *args, **kwargs)

        self.add_constant("ETHPHY_MDIO_ADDR", 1)
        self.submodules.ethphy = LiteEthPHY(self.platform.request("eth_clocks"),
                                            self.platform.request("eth"))
        self.submodules.ethmac = LiteEthMAC(phy=self.ethphy, dw=32, interface="wishbone")
        self.add_wb_slave(mem_decoder(self.mem_map["ethmac"]), self.ethmac.bus)
        self.add_memory_region("ethmac", self.mem_map["ethmac"] | self.shadow_base, 0x2000)


        self.ethphy.crg.cd_eth_rx.clk.attr.add("keep")
        self.ethphy.crg.cd_eth_tx.clk.attr.add("keep")
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_rx.clk, period_ns(25e6))
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_tx.clk, period_ns(25e6))
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.crg.cd_eth_rx.clk,
            self.ethphy.crg.cd_eth_tx.clk)

    def configure_ip(self, ip_type, ip):
        for i, e in enumerate(ip):
            s = ip_type + str(i + 1)
            s = s.upper()
            self.add_constant(s, e)


# def main():
#     parser = argparse.ArgumentParser(description="Arty LiteX SoC")
#     builder_args(parser)
#     soc_sdram_args(parser)
#     parser.add_argument("--s7", action="store_true",
#                         help="Create SoC for Arty S7 (No Ethernet)")
#     parser.add_argument("--with-ethernet", action="store_true",
#                         help="enable Ethernet support")
#     parser.add_argument("--nocompile-gateware", action="store_true")
#     args = parser.parse_args()

#     platform = arty_s7.Platform() if args.s7 else arty.Platform()
#     cls = MiniSoC if (args.with_ethernet and not args.s7) else BaseSoC
#     soc = cls(platform, **soc_sdram_argdict(args))
#     builder = Builder(soc, output_dir="build",
#                       compile_gateware=not args.nocompile_gateware,
#                       csr_csv="test/csr.csv")
#     vns = builder.build()

# if __name__ == "__main__":
#     main()

# #!/usr/bin/env python3

# from arty_base import *

from litex.soc.integration.soc_core import *

from liteeth.common import convert_ip
from liteeth.phy.mii import LiteEthPHYMII
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone


class EtherboneSoC(BaseSoC):
    csr_peripherals = [
        "ethphy",
        "ethcore",
    ]
    csr_map_update(BaseSoC.csr_map, csr_peripherals)

    def __init__(self, platform, mac_address=0x10e2d5000000, ip_address="192.168.1.50"):
        BaseSoC.__init__(self, platform, cpu_type=None, csr_data_width=8, l2_size=32)

        # ethernet mac/udp/ip stack
        self.submodules.ethphy = LiteEthPHYMII(self.platform.request("eth_clocks"),
                                               self.platform.request("eth"))
        self.submodules.ethcore = LiteEthUDPIPCore(self.ethphy,
                                                   mac_address,
                                                   convert_ip(ip_address),
                                                   self.clk_freq,
                                                   with_icmp=True)

        # etherbone bridge
        self.add_cpu_or_bridge(LiteEthEtherbone(self.ethcore.udp, 1234))
        self.add_wb_master(self.cpu_or_bridge.wishbone.bus)

        self.ethphy.crg.cd_eth_rx.clk.attr.add("keep")
        self.ethphy.crg.cd_eth_tx.clk.attr.add("keep")
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_rx.clk, period_ns(25e6))
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_tx.clk, period_ns(25e6))

        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.crg.cd_eth_rx.clk,
            self.ethphy.crg.cd_eth_tx.clk)


def main():
    parser = argparse.ArgumentParser(description="Arty LiteX SoC")
    builder_args(parser)
    parser.add_argument("--nocompile-gateware", action="store_true")
    args = parser.parse_args()

    platform = arty.Platform()
    soc = EtherboneSoC(platform)
    builder = Builder(soc, output_dir="build",
                      compile_gateware=not args.nocompile_gateware,
                      csr_csv="test/csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()