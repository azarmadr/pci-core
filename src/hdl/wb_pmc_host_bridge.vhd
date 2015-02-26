-- libraries and packages
-- ieee
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


-- wishbone/gsi/cern
library work;
use work.wishbone_pkg.all;
use work.genram_pkg.all;
use work.wb_pci_pkg.all;
--use work.wb_pmc_host_bridge_pkg.all;

-- XWB control BAR is mapped to BAR1
-- XWB devices BAR is mapped to BAR2

-- entity
entity wb_pmc_host_bridge is
  generic(
    g_family    : string := "Arria V";
    g_sdb_addr  : t_wishbone_address);
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
  
--------------------------------------------------------------------------------
-- CONSTANTS DECLERATION
--------------------------------------------------------------------------------

-- active low
constant BUF_OE : std_logic := '0';

--------------------------------------------------------------------------------
-- DATA TYPE DECLARATIONS
--------------------------------------------------------------------------------	

signal wb_clk       : std_logic;
signal wb_rst_in    : std_logic;
signal wb_rst_out   : std_logic;
 
signal wb_int_in    : std_logic;
signal wb_int_out   : std_logic;

-- Wishbone Slave Interface !!! Not Used
signal wbs_adr      : std_logic_vector(31 downto 0);
signal wbs_dat_in   : std_logic_vector(31 downto 0);
signal wbs_dat_out  : std_logic_vector(31 downto 0);
signal wbs_sel      : std_logic_vector( 3 downto 0);
signal wbs_cyc      : std_logic;
signal wbs_stb      : std_logic;
signal wbs_we       : std_logic;
signal wbs_ack_in   : std_logic;
signal wbs_ack_out  : std_logic;
signal wbs_rty      : std_logic;
signal wbs_err      : std_logic;
signal wbs_cti      : std_logic_vector(2 downto 0);
signal wbs_bte      : std_logic_vector(1 downto 0);

-- Wishbone Master Interface
signal wbm_adr      : std_logic_vector(31 downto 0);
signal wbm_dat_in   : std_logic_vector(31 downto 0);
signal wbm_dat_out  : std_logic_vector(31 downto 0);
signal wbm_sel      : std_logic_vector( 3 downto 0);
signal wbm_cyc      : std_logic;
signal wbm_stb      : std_logic;
signal wbm_we       : std_logic;
signal wbm_ack      : std_logic := wbm_stb;
signal wbm_rty      : std_logic := '0';
signal wbm_err      : std_logic := '0';
signal wbm_cti      : std_logic_vector(2 downto 0);
signal wbm_bte      : std_logic_vector(1 downto 0);

-- PCI Core Interface
signal AD_out       : std_logic_vector(31 downto 0);
signal AD_en        : std_logic_vector(31 downto 0);
signal CBE_out      : std_logic_vector(3 downto 0);
signal CBE_en       : std_logic_vector(3 downto 0);
signal INTA_en      : std_logic;
signal INTA_out     : std_logic;
signal REQ_en       : std_logic;
signal REQ_out      : std_logic;
signal FRAME_out    : std_logic;
signal FRAME_en     : std_logic;
signal IRDY_out     : std_logic;
signal IRDY_en      : std_logic;
signal DEVSEL_out   : std_logic;
signal DEVSEL_en    : std_logic;
signal TRDY_out     : std_logic;
signal TRDY_en      : std_logic;
signal STOP_out     : std_logic;
signal STOP_en      : std_logic;
signal PAR_out      : std_logic;
signal PAR_en       : std_logic;
signal PERR_out     : std_logic;
signal PERR_en      : std_logic;
signal SERR_out     : std_logic;
signal SERR_en      : std_logic;


--#####################################################
  signal internal_wb_clk, internal_wb_rstn, stall : std_logic; 
  signal internal_wb_rstn_sync : std_logic_vector(3 downto 0) := (others => '0');
  
  signal tx_alloc_mask : std_logic := '1'; -- Only pass every even tx32_alloc to tx64_alloc.
  
  signal wb_stb   : std_logic;
  signal wb_cyc   : std_logic;
  signal wb_ack   : std_logic;
  signal wb_stall : std_logic;
  signal wb_adr   : std_logic_vector(63 downto 0);
  signal wb_bar   : std_logic_vector( 5 downto 0);
  signal wb_dat   : std_logic_vector(31 downto 0);
  
  
  -- Internal WB clock, PC->FPGA
  signal int_slave_i : t_wishbone_slave_in;
  signal int_slave_o : t_wishbone_slave_out;
  -- Internal WB clock: FPGA->PC
  signal int_master_o : t_wishbone_master_out;
  signal int_master_i : t_wishbone_master_in;
  
  -- control registers
  signal r_cyc   : std_logic;
  signal r_int   : std_logic := '0'; -- interrupt mask, starts=0
  signal r_addr  : std_logic_vector(31 downto 16);
  signal r_error : std_logic_vector(63 downto  0);
  
  -- interrupt signals
  signal fifo_full, r_fifo_full, app_int_sts, app_msi_req : std_logic;
  
  
  type t_bus_state is (st_idle, st_wait4ack);
  
  signal bus_state    : t_bus_state;
  signal stb_prev     : std_logic;
  signal ack_prev     : std_logic;
  signal stb_asserted : std_logic;
  signal ack_asserted : std_logic; 

  signal pci_bar_hit      : std_logic_vector(5 downto 0);
  signal r_pci_bar_hit    : std_logic_vector(5 downto 0);
  

begin


--------------------------------------------
pci_core_inst : pci_bridge32 -- instance of Verilog core
port map (

 -- PCI INTERFACE
 -- system pins
 pci_clk_i      => pci_clk_i,

 pci_rst_i      => pci_rst_i,
 pci_rst_o      => open, -- not used in GUEST
 pci_rst_oe_o   => open, -- not used in GUEST

 pci_inta_i     => '1', -- not used in GUEST
 pci_inta_o     => INTA_out,
 pci_inta_oe_o  => INTA_en,

 -- arbitration pins
 pci_req_o      => REQ_out, -- not used in GUEST
 pci_req_oe_o   => REQ_en,  -- not used in GUEST

 pci_gnt_i      => '1',     -- not used in GUEST

 -- protocol pins
 pci_frame_i      => frame_io,
 pci_frame_o      => FRAME_out,
 pci_frame_oe_o   => FRAME_en,
 
 pci_irdy_i       => irdy_io,
 pci_irdy_o       => IRDY_out,
 pci_irdy_oe_o    => IRDY_en,
 
 pci_idsel_i      => idsel_i,
 
 pci_devsel_i     => devsel_io,
 pci_devsel_o     => DEVSEL_out,
 pci_devsel_oe_o  => DEVSEL_en,
 
 pci_trdy_i       => trdy_io,
 pci_trdy_o       => TRDY_out,
 pci_trdy_oe_o    => TRDY_en,
 
 pci_stop_i       => stop_io,
 pci_stop_o       => STOP_out,
 pci_stop_oe_o    => STOP_en,

 -- data transfer pins
 pci_ad_i         => ad_io,
 pci_ad_o         => AD_out,
 pci_ad_oe_o      => AD_en,
 
 pci_cbe_i        => c_be_io,
 pci_cbe_o        => CBE_out,
 pci_cbe_oe_o     => CBE_en,

 -- parity generation and checking pins
 pci_par_i        => par_io,
 pci_par_o        => PAR_out,
 pci_par_oe_o     => PAR_en,
 
 pci_perr_i       => perr_io,
 pci_perr_o       => PERR_out,
 pci_perr_oe_o    => PERR_en,

 -- system error pin
 pci_serr_o       => SERR_out,
 pci_serr_oe_o    => SERR_en, 

 pci_bar_hit_o    => pci_bar_hit,

 -- WISHBONE system signals
 wb_clk_i   => wb_clk,
 wb_rst_i   => wb_rst_in,
 wb_rst_o   => wb_rst_out,
 
 wb_int_i   => '0', -- wb_int_in,
 wb_int_o   => open,  -- not used in GUEST

 -- WISHBONE slave interface
 wbs_adr_i  => (others => '0'),
 wbs_dat_i  => (others => '0'),
 wbs_dat_o  => open,
 wbs_sel_i  => (others => '0'),
 wbs_cyc_i  => '0',
 wbs_stb_i  => '0',
 wbs_we_i   => '0',
 wbs_ack_o  => open,
 wbs_rty_o  => open,
 wbs_err_o  => open,
 
--`ifdef PCI_WB_REV_B3
 wbs_cti_i  => (others => '0'),
 wbs_bte_i  => (others => '0'),
--`else
-- wbs_cab_i  => wbs_cab,
--`endif

 -- WISHBONE master interface
 wbm_stb_o  => wbm_stb,
 wbm_adr_o  => wbm_adr,
 
 wbm_we_o   => wbm_we,
 wbm_dat_o  => wbm_dat_out,
 wbm_sel_o  => wbm_sel,

 wbm_cyc_o  => wbm_cyc,
 
 wbm_ack_i  => wbm_ack,
 wbm_rty_i  => wbm_rty,
 wbm_err_i  => wbm_err,
 wbm_dat_i  => wbm_dat_in,
 
 wbm_cti_o  => wbm_cti,
 wbm_bte_o  => wbm_bte

 
);


--******************************************************
-- PCI IO BUFFERS INSTANTIATION
--******************************************************	

gen_al_ad: for i in 0 to 31 generate
  ad_io(i) <= AD_out(i) when AD_en(i) = BUF_OE else 'Z';
end generate;

gen_al_cbe: for j in 0 to 3 generate
  c_be_io(j)     <= CBE_out(j) when CBE_en(j) = BUF_OE else 'Z';
end generate;

frame_io  <=  FRAME_out   when FRAME_en   = BUF_OE else 'Z';
irdy_io   <=  IRDY_out    when IRDY_en    = BUF_OE else 'Z';
devsel_io <=  DEVSEL_out  when DEVSEL_en  = BUF_OE else 'Z';
trdy_io   <=  TRDY_out    when TRDY_en    = BUF_OE else 'Z';
stop_io   <=  STOP_out    when STOP_en    = BUF_OE else 'Z';
inta_o    <=  INTA_out    when INTA_en    = BUF_OE else 'Z';
par_io    <=  PAR_out     when PAR_en     = BUF_OE else 'Z';
perr_io   <=  PERR_out    when PERR_en    = BUF_OE else 'Z';
serr_io   <=  SERR_out    when SERR_en    = BUF_OE else 'Z';

req_o     <= 'Z'; -- not used in GUEST


busmode_io(3 downto 1)  <= (others => 'Z'); -- only used as inputs

-- drive BUSMODE1# according to BUSMODE[4:2]# signals
busmode_io(0) <= '0' when (busmode_io(3 downto 1) = "000" or -- card present test
                           busmode_io(3 downto 1) = "001")   -- PCI capable test
                  else '1';

-- disconnect PMC from the bus if host tries unsuported protocol
buf_oe_o  <= not busmode_io(0);


--******************************************************
-- Wishbone <> XWishbone connections
--******************************************************

wb_clk          <= master_clk_i;
internal_wb_clk <= master_clk_i;

wb_rst_in <= not internal_wb_rstn;

--------------------------------------------------------------------
-- strobe and ack modification to connection non pipelined 
-- master with pipelined slave interface
-- see Wishbone B4 specification document (2010)
-- page 83, chapter 5.1 "Standard master connected to pipelined slave"
p_bus_activitiy_fsm: process(internal_wb_clk)
begin
  if rising_edge(internal_wb_clk) then
    if internal_wb_rstn = '0' then
      bus_state <= st_idle;
      stb_prev  <= '0';
      ack_prev  <= '0';
    else
      stb_prev  <= wbm_stb;
      ack_prev  <= wb_ack;
    
      if stb_asserted = '1' then
        bus_state <= st_wait4ack;
      elsif ack_asserted = '1' then
        bus_state <= st_idle;
      else -- hold
        bus_state <= bus_state;
      end if;
    end if;
  end if;
end process p_bus_activitiy_fsm;

-- rising edge detectors
stb_asserted <= '1' when (stb_prev = '0' and wbm_stb = '1') else '0';
ack_asserted <= '1' when (ack_prev = '0' and  wb_ack = '1') else '0';

wb_stb  <= wbm_stb when bus_state = st_idle else '0';


--------------------------------------------------------------------
-- bar hit register - store bar hit vector from PCI core when it changes
p_bar_hit_reg: process(internal_wb_clk)
begin
  if rising_edge(internal_wb_clk) then
    if internal_wb_rstn = '0' then
      r_pci_bar_hit <= (others => '0');
    else 
      if pci_bar_hit /= "000000" and pci_bar_hit /= r_pci_bar_hit then
        r_pci_bar_hit <= pci_bar_hit;
      else 
        r_pci_bar_hit <= r_pci_bar_hit;
      end if;
    end if; -- reset
  end if; -- clk
end process p_bar_hit_reg;

wb_bar  <= r_pci_bar_hit;

wb_adr          <= x"00000000" & wbm_adr;

int_slave_i.we  <= wbm_we;
int_slave_i.dat <= wbm_dat_out;
int_slave_i.sel <= wbm_sel;

wb_cyc          <= wbm_cyc;

-- wb_stall;
wbm_ack     <= wb_ack;
wbm_err     <= int_slave_o.err;
wbm_rty     <= int_slave_o.rty;
wbm_dat_in  <= wb_dat;


  
  internal_wb_rstn <= internal_wb_rstn_sync(0);

  reset : process(internal_wb_clk)
  begin
    if rising_edge(internal_wb_clk) then
      internal_wb_rstn_sync <= (master_rstn_i and slave_rstn_i) & internal_wb_rstn_sync(internal_wb_rstn_sync'length-1 downto 1);
      
    end if;
  end process;
  
  PC_to_FPGA_clock_crossing : xwb_clock_crossing port map(
    slave_clk_i    => internal_wb_clk,
    slave_rst_n_i  => internal_wb_rstn,
    slave_i        => int_slave_i,
    slave_o        => int_slave_o,
    master_clk_i   => master_clk_i, 
    master_rst_n_i => master_rstn_i,
    master_i       => master_i,
    master_o       => master_o);
  
  int_slave_i.stb <= wb_stb            when wb_bar = "000100" else '0';
  wb_stall        <= int_slave_o.stall when wb_bar = "000100" else '0';
  
  int_slave_i.cyc <= r_cyc;
  int_slave_i.adr(r_addr'range) <= r_addr;
  int_slave_i.adr(r_addr'right-1 downto 0)  <= wb_adr(r_addr'right-1 downto 0);
  
  FPGA_to_PC_clock_crossing : xwb_clock_crossing port map(
    slave_clk_i    => slave_clk_i,
    slave_rst_n_i  => slave_rstn_i,
    slave_i        => slave_i,
    slave_o        => slave_o,
    master_clk_i   => internal_wb_clk, 
    master_rst_n_i => internal_wb_rstn,
    master_i       => int_master_i,
    master_o       => int_master_o );

  -- Notify the system when the FIFO is non-empty
  fifo_full <= int_master_o.cyc and int_master_o.stb;
  app_int_sts <= fifo_full and r_int; -- Classic interrupt until FIFO drained
  app_msi_req <= fifo_full and not r_fifo_full; -- Edge-triggered MSI
  
  int_master_i.rty <= '0';
  
  control : process(internal_wb_clk)
  begin
    if rising_edge(internal_wb_clk) then
      r_fifo_full <= fifo_full;
      
      -- Shift in the error register
      if int_slave_o.ack = '1' or int_slave_o.err = '1' or int_slave_o.rty = '1' then
        r_error <= r_error(r_error'length-2 downto 0) & (int_slave_o.err or int_slave_o.rty);
      end if;
      
      if wb_bar = "000100" then
        wb_ack <= int_slave_o.ack;
        wb_dat <= int_slave_o.dat;
      elsif wb_bar = "000010" then -- The control BAR is targetted
        -- Feedback acks one cycle after strobe
        wb_ack <= wb_stb;
	
        -- Always output read result (even w/o stb or we)
        case wb_adr(6 downto 2) is
          when "00000" => -- Control register high
            wb_dat(31) <= r_cyc;
            wb_dat(30) <= '0';
            wb_dat(29) <= r_int;
            wb_dat(28 downto 0) <= (others => '0');
          when "00010" => -- Error flag high x08
            wb_dat <= r_error(63 downto 32);
          when "00011" => -- Error flag low  x0c
            wb_dat <= r_error(31 downto 0);
          when "00101" => -- Window offset low
            wb_dat(r_addr'range) <= r_addr;
            wb_dat(r_addr'right-1 downto 0) <= (others => '0');
          when "00111" => -- SDWB address low
            wb_dat <= g_sdb_addr;
          when "10000" => -- Master FIFO status & flags
            wb_dat(31) <= fifo_full;
            wb_dat(30) <= int_master_o.we;
            wb_dat(29 downto 4) <= (others => '0');
            wb_dat( 3 downto 0) <= int_master_o.sel;
          when "10011" => -- Master FIFO adr low
            wb_dat <= int_master_o.adr;
          when "10101" => -- Master FIFO dat low
            wb_dat <= int_master_o.dat;
          when others =>
            wb_dat <= (others => '0');
        end case;
	
	-- Unless requested to by the PC, don't deque the FPGA->PC FIFO
        int_master_i.stall  <= '1';
        int_master_i.ack    <= '0';
        int_master_i.err    <= '0';
        
        -- Is this a write to the register space?
        if wb_stb = '1' and int_slave_i.we = '1' then
          case wb_adr(6 downto 2) is
            when "00000" => -- Control register high
              if int_slave_i.sel(3) = '1' then
                if int_slave_i.dat(30) = '1' then
                  r_cyc <= int_slave_i.dat(31);
                end if;
                if int_slave_i.dat(28) = '1' then
                  r_int <= int_slave_i.dat(29);
                end if;
              end if;
            when "00101" => -- Window offset low x14
              if int_slave_i.sel(3) = '1' then
                r_addr(31 downto 24) <= int_slave_i.dat(31 downto 24);
              end if;
              if int_slave_i.sel(2) = '1' then
                r_addr(24 downto 16) <= int_slave_i.dat(24 downto 16);
              end if;
            when "10000" => -- Master FIFO status & flags
              if int_slave_i.sel(0) = '1' then
                case int_slave_i.dat(1 downto 0) is
                  when "00" => null;
                  when "01" => int_master_i.stall <= '0';
                  when "10" => int_master_i.ack <= '1';
                  when "11" => int_master_i.err <= '1';
                end case;
              end if;
            when "10101" => -- Master FIFO data low
              int_master_i.dat <= int_slave_i.dat;
            when others => null;
          end case; -- wb_adr(6 downto 2) is
        end if; -- wb_stb = '1' and int_slave_i.we = '1' then
      end if; --  if wb_bar = "000100" else
    end if; -- rising_edge(internal_wb_clk)
  end process; -- control  
end rtl;
