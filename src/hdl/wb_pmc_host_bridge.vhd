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
use work.wb_pmc_host_bridge_pkg.all;
use work.gencores_pkg.all;
use work.aux_functions_pkg.all;

-- XWB control BAR is mapped to BAR1
-- XWB devices BAR is mapped to BAR2

-- entity
entity wb_pmc_host_bridge is
  generic(
    g_family    : string  := "Arria V";
    g_fast_ack  : boolean := true;
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
    req_o         : out   std_logic;
    gnt_i         : in    std_logic;
    
    -- PCI signals (required) - error reporting
    perr_io       : inout std_logic;
    serr_io       : inout std_logic;
    
    -- PCI signals (optional) - interrupts pins
    inta_o        : out   std_logic;

    debug_i       : in  std_logic_vector(15 downto 0);
    debug_o       : out std_logic_vector(15 downto 0)
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
signal wbs_stall    : std_logic;

signal msi_control  : std_logic_vector(31 downto 0);
signal msi_address  : std_logic_vector(31 downto 0);
signal msi_msg_data : std_logic_vector(15 downto 0);

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
  
  
  signal wb_stb   : std_logic;
  signal wb_cyc   : std_logic;
  signal wb_ack   : std_logic;
  signal wb_stall : std_logic;
  signal wb_adr   : std_logic_vector(31 downto 0);
  signal wb_bar   : std_logic_vector( 5 downto 0);
  signal wb_dat   : std_logic_vector(31 downto 0);
  
  
  -- Internal WB clock, PC->FPGA
  signal int_slave_i : t_wishbone_slave_in;
  signal int_slave_o : t_wishbone_slave_out;
  -- Internal WB clock: FPGA->PC
  signal int_master_o : t_wishbone_master_out;
  signal int_master_i : t_wishbone_master_in;
  signal ext_slave_o  : t_wishbone_slave_out;
  
  -- control registers
  signal r_cyc   : std_logic;
  signal r_int   : std_logic := '0'; -- interrupt mask, starts=0
  signal r_addr  : std_logic_vector(31 downto 16);
  signal r_error : std_logic_vector(63 downto  0);
  
  -- interrupt signals
  signal msi_fifo_not_empty   : std_logic;
  signal r_msi_fifo_not_empty : std_logic;
  signal app_int_sts : std_logic;
  signal app_msi_req : std_logic;
  signal r_intx_irq_en : std_logic;
  signal r_msi_irq_en  : std_logic;
  
  
  type t_bus_state is (st_idle, st_wait4ack);
  
  signal bus_state    : t_bus_state;
  signal stb_prev     : std_logic;
  signal ack_prev     : std_logic;
  signal stb_asserted : std_logic;
  signal ack_asserted : std_logic; 





  -- debug signals 
  signal irq_button_intx : std_logic;
  signal irq_button_msi  : std_logic;

  signal s_msi_irq_button_reg : std_logic_vector(1 downto 0);
  signal s_msi_irq_button_red : std_logic;

  signal IRDY_out_delay_reg	: std_logic_vector(1 downto 0);
  

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
   pci_req_o      => REQ_out,
   pci_req_oe_o   => REQ_en,

   pci_gnt_i      => gnt_i,

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

   msi_control_o   => open,
   msi_address_o   => msi_address,
   msi_msg_data_o  => msi_msg_data,

   -- WISHBONE system signals
   wb_clk_i   => wb_clk,
   wb_rst_i   => wb_rst_in,
   wb_rst_o   => wb_rst_out,
   
   wb_int_i   => wb_int_in,
   wb_int_o   => open,  -- not used in GUEST

   -- WISHBONE slave interface
   wbs_adr_i  => wbs_adr,
   wbs_dat_i  => wbs_dat_in,
   wbs_dat_o  => wbs_dat_out,
   wbs_sel_i  => wbs_sel,
   wbs_cyc_i  => wbs_cyc,
   wbs_stb_i  => wbs_stb,
   wbs_we_i   => wbs_we,
   wbs_ack_o  => wbs_ack_out,
   wbs_rty_o  => wbs_rty,
   wbs_err_o  => wbs_err,
   
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

  --irdy_delay : process(pci_clk_i)
  --begin
  --  if rising_edge(pci_clk_i) then
  --    if pci_rst_i = '0' then
  --	   IRDY_out_delay_reg <= "00";
  --	 else	
  --		IRDY_out_delay_reg <= IRDY_out_delay_reg(0) & IRDY_out;
  --    end if;
  --  end if;
  --end process irdy_delay;
		
  frame_io  <=  FRAME_out   when FRAME_en   = BUF_OE else 'Z';
  irdy_io   <=  IRDY_out    when IRDY_en    = BUF_OE else 'Z';
  devsel_io <=  DEVSEL_out  when DEVSEL_en  = BUF_OE else 'Z';
  trdy_io   <=  TRDY_out    when TRDY_en    = BUF_OE else 'Z';
  stop_io   <=  STOP_out    when STOP_en    = BUF_OE else 'Z';
  inta_o    <=  INTA_out    when INTA_en    = BUF_OE else 'Z';
  req_o     <=  REQ_out     when REQ_en     = BUF_OE else 'Z';
  par_io    <=  PAR_out     when PAR_en     = BUF_OE else 'Z';
  perr_io   <=  PERR_out    when PERR_en    = BUF_OE else 'Z';
  serr_io   <=  SERR_out    when SERR_en    = BUF_OE else 'Z';



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
      else
      
        if wb_ack = '1' then
          bus_state <= st_idle;
        elsif wbm_stb = '1' then
          bus_state <= st_wait4ack;
        else -- hold
          bus_state <= bus_state;
        end if;
      end if;
    end if;
  end process p_bus_activitiy_fsm;

  wb_stb  <= wbm_stb when bus_state = st_idle else '0';


  -- Using ADDRESS TRANSLATION feature of the PCI core to get bar_hit
  -- see ./verilog/pci_user_constants.v : 
  -- `define PCI_TA1 24'h0100_00
  -- `define PCI_TA2 24'h0200_00
  -- BAR1 on WB address bus is translated from 0xPPxxxxxx on PCI > 0x01xxxxxx on WB
  -- BAR2 on WB address bus is translated from 0xPPxxxxxx on PCI > 0x02xxxxxx on WB
  wb_bar  <= wbm_adr(28 downto 24) & '0';

  wb_adr          <= wbm_adr;
  wb_cyc          <= wbm_cyc;

  int_slave_i.we  <= wbm_we;
  int_slave_i.dat <= wbm_dat_out;
  int_slave_i.sel <= wbm_sel;

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

  -------------------------------------------------------------------------------------
  -- interface between WB master and slave FIFO to monster
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
-- stall not used, because WB master is B3
--  wb_stall      <= int_slave_o.stall when wb_bar = "000100" else '0';
  
  int_slave_i.cyc <= r_cyc;
  int_slave_i.adr(r_addr'range) <= r_addr;
  int_slave_i.adr(r_addr'right-1 downto 0)  <= wb_adr(r_addr'right-1 downto 0);
  
  -------------------------------------------------------------------------------------
  -- interface between WB master and MSI FIFO from monster
  FPGA_to_PC_clock_crossing : xwb_clock_crossing port map(
    slave_clk_i    => slave_clk_i,
    slave_rst_n_i  => slave_rstn_i,
    slave_i        => slave_i,
    slave_o        => ext_slave_o,
    master_clk_i   => internal_wb_clk, 
    master_rst_n_i => internal_wb_rstn,
    master_i       => int_master_i,
    master_o       => int_master_o );

  -- Do not wait for software acknowledgement
  fask_ack : if g_fast_ack generate
    slave_o.stall <= ext_slave_o.stall;
    slave_o.rty <= '0';
    slave_o.err <= '0';
    slave_o.dat <= (others => '0');
    
    fast_ack : process(slave_clk_i)
    begin
      if rising_edge(slave_clk_i) then
        slave_o.ack <= slave_i.cyc and slave_i.stb and not ext_slave_o.stall;
      end if;
    end process;
  end generate;
  
  -- Uses ack/err and dat from software
  slow_ack : if not g_fast_ack generate
    slave_o <= ext_slave_o;
  end generate;



-------------------------------------------------------------------------------------
-- Interrupt generation
-------------------------------------------------------------------------------------

  -- Notify the system when the FIFO is non-empty
  msi_fifo_not_empty <= int_master_o.cyc and int_master_o.stb;

  app_int_sts <= msi_fifo_not_empty and r_int and r_intx_irq_en; -- Classic interrupt until FIFO drained
  app_msi_req <= msi_fifo_not_empty and r_int and not r_msi_fifo_not_empty and r_msi_irq_en; -- Edge-triggered MSI
 

  -- INTx 
  wb_int_in <= app_int_sts; -- connect generated IRQ signal to PCI core signal
  int_master_i.rty <= '0';

  -- send MSI IRQs message over WB slave/PCI master
  wb_irq_master : process(internal_wb_clk)
  begin
   if rising_edge(internal_wb_clk) then
      if(internal_wb_rstn = '0') then
         wbs_cyc <= '0';
         wbs_stb <= '0';
      else
         if wbs_cyc = '1' then
           if wbs_stall = '0' then
             wbs_stb <= '0';
           end if;
           if (wbs_ack_out = '1' or wbs_err = '1') then
             wbs_cyc <= '0';
           end if;
         else
           wbs_cyc <= app_msi_req;
           wbs_stb <= app_msi_req;
         end if;
      end if;
    end if;
  end process;  
  
  -- Wishbone Slave Interface !!! Used for posting MSI
  wbs_stall <= '0' when wbs_cyc = '0' else not wbs_ack_out;
  wbs_adr    <= msi_address; 
  wbs_dat_in <= x"0000" & msi_msg_data;
  wbs_sel      <= "1111"; 
  wbs_we       <= '1';
  wbs_rty      <= '0';
  wbs_bte      <= "00";  



  
  control : process(internal_wb_clk)
  begin
    if rising_edge(internal_wb_clk) then
      r_msi_fifo_not_empty <= msi_fifo_not_empty and r_int; -- delay for edge detection
      
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
          when "00101" => -- Window offset low x14
            wb_dat(r_addr'range) <= r_addr;
            wb_dat(r_addr'right-1 downto 0) <= (others => '0');
          when "00111" => -- SDWB address low x1C
            wb_dat <= g_sdb_addr;
          when "10000" => -- Master FIFO status & flags
            wb_dat(31) <= msi_fifo_not_empty;
            wb_dat(30) <= int_master_o.we;
            wb_dat(29 downto 4) <= (others => '0');
            wb_dat( 3 downto 0) <= int_master_o.sel;
          when "10011" => -- Master FIFO adr low
            wb_dat <= int_master_o.adr and c_pmc_msi.sdb_component.addr_last(31 downto 0);
          when "10101" => -- Master FIFO dat low
            wb_dat <= int_master_o.dat;
          when "01000" => -- IRQ control register (PMC specific)
            wb_dat(0)          <= r_msi_irq_en;
            wb_dat(3 downto 2) <= (others => '0');
            wb_dat(4)          <= r_intx_irq_en;
            wb_dat(31 downto 5)<= (others => '0');
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
            when "01000" => -- IRQ control register (PMC specific)
              r_msi_irq_en  <= int_slave_i.dat(0) and not int_slave_i.dat(4); -- do not enable both
              r_intx_irq_en <= int_slave_i.dat(4) and not int_slave_i.dat(0); -- do not enable both
            when others => null;
          end case; -- wb_adr(6 downto 2) is
        end if; -- wb_stb = '1' and int_slave_i.we = '1' then
      end if; --  if wb_bar = "000100" else
    end if; -- rising_edge(internal_wb_clk)
  end process; -- control  




-- debug/test signals
debug_o( 0) <= int_slave_i.cyc;
debug_o( 1) <= int_slave_i.stb;
debug_o( 2) <= int_slave_i.we;
debug_o( 3) <= int_slave_o.ack;
debug_o( 4) <= int_slave_o.err;
debug_o( 5) <= int_slave_o.rty;
debug_o( 6) <= int_slave_o.stall;
debug_o( 7) <= int_slave_o.int;

debug_o( 8) <= wbm_cyc;
debug_o( 9) <= wbm_stb;
debug_o(10) <= wbm_we;
debug_o(11) <= wbm_ack;
debug_o(12) <= wbm_err;
debug_o(13) <= wbm_rty;
debug_o(14) <= wb_stall;
debug_o(15) <= '0';

------------------------------------------------------------------------------------
-- IRQ test via buttons
-- FPGA button triggers INTx IRQ
-- CPLD button triggers MSI IRQ on release


--intx_irq_btn_debounce : debounce
--generic map
--    ( DB_Cnt => 6250000) -- 50ms
--port map(
--    Reset   => not internal_wb_rstn, 
--    Clk     => internal_wb_clk,
--    DB_In   => not debug_i(0),
--    DB_Out  => irq_button_intx
--    );
--
--msi_irq_btn_debounce : debounce
--generic map
--    ( DB_Cnt => 6250000) -- 50ms
--port map(
--    Reset   => not internal_wb_rstn, 
--    Clk     => internal_wb_clk,
--    DB_In   => not debug_i(1),
--    DB_Out  => irq_button_msi
--    );
--
---- rising edge detection for buttons, on button release
--p_button_red: process(internal_wb_clk)
--begin
--	if rising_edge(internal_wb_clk) then
--    if(internal_wb_rstn = '0') then
--      s_msi_irq_button_reg <= "00";
--      s_msi_irq_button_red <= '0';
--    else
--      s_msi_irq_button_reg  <= irq_button_msi & s_msi_irq_button_reg(1); -- shift right
--
--      if s_msi_irq_button_reg(0) = '0' and s_msi_irq_button_reg(1)= '1' then
--        s_msi_irq_button_red <= '1';
--      else
--        s_msi_irq_button_red <= '0';
--      end if;
--    end if;  
--  end if; -- clk
--end process p_button_red;
--
--  -- trigger MSI IRQ when CPLD button released
--  app_msi_req <= s_msi_irq_button_red;
--
--
--  -- trigger INTx IRQ when FPGA button pressed and IRQs enabled in CONTROL_REGISTER_HIGH (r_int)
--  app_int_sts <= irq_button_intx and r_int;



end rtl;

