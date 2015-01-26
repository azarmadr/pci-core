-- libraries and packages
-- ieee
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- wishbone/gsi/cern
library work;
use work.wishbone_pkg.all;
use work.genram_pkg.all;

-- entity
entity wb_pmc_host_bridge is
  port (
    -- FPGA signals
    clk_sys_i     : in  std_logic;
    rst_n_i       : in  std_logic;
    -- Commands from PMC to FPGA
    master_clk_i  : in    std_logic := '0';
    master_rstn_i : in    std_logic := '1';
    master_o      : out   t_wishbone_master_out;
    master_i      : in    t_wishbone_master_in;
    -- Command to PMC from FPGA
    slave_clk_i   : in    std_logic := '0';
    slave_rstn_i  : in    std_logic := '1';
    slave_i       : in    t_wishbone_slave_in;
    slave_o       : out   t_wishbone_slave_out;
    -- PCI signals - generic
    pci_clk_i     : in    std_logic := '0';
    pci_rst_i     : in    std_logic := '0';
    buf_oe_o      : out   std_logic;
    busmode_io    : inout std_logic_vector(3 downto 0);
    -- PCI signals (required) - address and data
    ad_io         : inout std_logic_vector(31 downto 0);
    c_be_io       : inout std_logic_vector(3 downto 0);
    par_io        : inout std_logic;
    -- PCI signals (required) - interface control pins
    frame_io      : inout std_logic;
    trdy_io       : inout std_logic;
    irdy_io       : inout std_logic;
    stop_io       : inout std_logic;
    devsel_io     : inout std_logic;
    idsel_i       : in    std_logic;
    -- PCI signals (required) - error reporting
    perr_io       : inout std_logic;
    serr_io       : inout std_logic;
    -- PCI signals (required) - arbitration
    req_o         : out   std_logic;
    gnt_i         : in    std_logic;
    -- PCI signals (optional) - interrupts pins
    inta_o        : out   std_logic;
    intb_o        : out   std_logic;
    intc_o        : out   std_logic;
    intd_o        : out   std_logic
);    
end wb_pmc_host_bridge;

architecture rtl of wb_pmc_host_bridge is
  
begin
  
end rtl;

