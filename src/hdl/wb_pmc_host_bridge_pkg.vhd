-- libraries and packages
-- ieee
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- wishbone/gsi/cern
library work;
use work.wishbone_pkg.all;
use work.genram_pkg.all;

-- package declaration
package wb_pmc_host_bridge_pkg is


  constant c_pmc_msi : t_sdb_msi := (
--    abi_class     => x"0000", -- undocumented device
--    abi_ver_major => x"01",
--    abi_ver_minor => x"01",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"4", -- 32-bit port granularity
    sdb_component => (
    addr_first    => x"0000000000000000",
    addr_last     => x"000000000000ffff",
    product => (
    vendor_id     => x"0000000000000651", -- GSI
    device_id     => x"94ECf80C",
    version       => x"00000001",
    date          => x"20150115", 
    name          => "PCI-Bridge-MSI-Tgt "))); 


  component wb_pmc_host_bridge
    generic(
      g_family   : string := "Arria V";
      g_sdb_addr : t_wishbone_address);
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
    buf_oe_o      : out   std_logic := '0';
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
    req_o         : out   std_logic;
    gnt_i         : in    std_logic;
      -- PCI signals (required) - error reporting
      perr_io       : inout std_logic;
      serr_io       : inout std_logic;
      -- PCI signals (optional) - interrupts pins
      inta_o        : out   std_logic;

    debug_i       : in  std_logic_vector(7 downto 0);
    debug_o       : out std_logic_vector(7 downto 0)

    );
  end component;
  


end wb_pmc_host_bridge_pkg;
