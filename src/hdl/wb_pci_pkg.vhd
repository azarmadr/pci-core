----------------------------------------------------------------------
----                                                              ----
----  File name "pci_bridge32.v"                                  ----
----                                                              ----
----  This file is part of the "PCI bridge" project               ----
----  http:--www.opencores.org/cores/pci/                         ----
----                                                              ----
----  Author(s):                                                  ----
----      - Miha Dolenc (mihad@opencores.org)                     ----
----      - Tadej Markovic (tadej@opencores.org)                  ----
----                                                              ----
----  All additional information is avaliable in the README       ----
----  file.                                                       ----
----                                                              ----
----                                                              ----
----------------------------------------------------------------------
----                                                              ----
---- Copyright (C) 2001 Miha Dolenc, mihad@opencores.org          ----
----                                                              ----
---- This source file may be used and distributed without         ----
---- restriction provided that this copyright statement is not    ----
---- removed from the file and that any derivative work contains  ----
---- the original copyright notice and the associated disclaimer. ----
----                                                              ----
---- This source file is free software; you can redistribute it   ----
---- and/or modify it under the terms of the GNU Lesser General   ----
---- Public License as published by the Free Software Foundation; ----
---- either version 2.1 of the License, or (at your option) any   ----
---- later version.                                               ----
----                                                              ----
---- This source is distributed in the hope that it will be       ----
---- useful, but WITHOUT ANY WARRANTY; without even the implied   ----
---- warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ----
---- PURPOSE.  See the GNU Lesser General Public License for more ----
---- details.                                                     ----
----                                                              ----
---- You should have received a copy of the GNU Lesser General    ----
---- Public License along with this source; if not, download it   ----
---- from http:--www.opencores.org/lgpl.shtml                     ----
----                                                              ----
----------------------------------------------------------------------
--
-- CVS Revision History
--
-- $Log: pci_bridge32.v,v $
-- Revision 1.19  2004/09/23 13:48:53  mihad
-- The control inputs from PCI are now muxed with control outputs
-- using output enable state for given signal.
--
-- Revision 1.18  2004/08/19 15:27:34  mihad
-- Changed minimum pci image size to 256 bytes because
-- of some PC system problems with size of IO images.
--
-- Revision 1.17  2004/01/24 11:54:18  mihad
-- Update! SPOCI Implemented!
--
-- Revision 1.16  2003/12/19 11:11:30  mihad
-- Compact PCI Hot Swap support added.
-- New testcases added.
-- Specification updated.
-- Test application changed to support WB B3 cycles.
--
-- Revision 1.15  2003/12/10 12:02:54  mihad
-- The wbs B3 to B2 translation logic had wrong reset wire connected!
--
-- Revision 1.14  2003/12/09 09:33:57  simons
-- Some warning cleanup.
--
-- Revision 1.13  2003/10/17 09:11:52  markom
-- mbist signals updated according to newest convention
--
-- Revision 1.12  2003/08/21 20:49:03  tadejm
-- Added signals for WB Master B3.
--
-- Revision 1.11  2003/08/08 16:36:33  tadejm
-- Added 'three_left_out' to pci_pciw_fifo signaling three locations before full. Added comparison between current registered cbe and next unregistered cbe to signal wb_master whether it is allowed to performe burst or not. Due to this, I needed 'three_left_out' so that writing to pci_pciw_fifo can be registered, otherwise timing problems would occure.
--
-- Revision 1.10  2003/08/03 18:05:06  mihad
-- Added limited WISHBONE B3 support for WISHBONE Slave Unit.
-- Doesn't support full speed bursts yet.
--
-- Revision 1.9  2003/01/27 16:49:31  mihad
-- Changed module and file names. Updated scripts accordingly. FIFO synchronizations changed.
--
-- Revision 1.8  2002/10/21 13:04:33  mihad
-- Changed BIST signal names etc..
--
-- Revision 1.7  2002/10/18 03:36:37  tadejm
-- Changed wrong signal name mbist_sen into mbist_ctrl_i.
--
-- Revision 1.6  2002/10/17 22:51:50  tadejm
-- Changed BIST signals for RAMs.
--
-- Revision 1.5  2002/10/11 10:09:01  mihad
-- Added additional testcase and changed rst name in BIST to trst
--
-- Revision 1.4  2002/10/08 17:17:05  mihad
-- Added BIST signals for RAMs.
--
-- Revision 1.3  2002/02/01 15:25:12  mihad
-- Repaired a few bugs, updated specification, added test bench files and design document
--
-- Revision 1.2  2001/10/05 08:14:28  mihad
-- Updated all files with inclusion of timescale file for simulation purposes.
--
-- Revision 1.1.1.1  2001/10/02 15:33:46  mihad
-- New project directory structure
--
--

--`include "pci_constants.v"

-- synopsys translate_off
--`include "timescale.v"
-- synopsys translate_on

-- this is top level module of pci bridge core
-- it instantiates and connects other lower level modules
-- check polarity of PCI output enables in file out_reg.v and change it according to IO interface specification

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


package wb_pci_pkg is

component pci_bridge32
port (
    -- WISHBONE system signals
    wb_clk_i             : in      std_logic;
    wb_rst_i             : in      std_logic;
    wb_rst_o             : out     std_logic;
    wb_int_i             : in      std_logic;
    wb_int_o             : out     std_logic;

    -- WISHBONE slave interface
    wbs_adr_i            : in      std_logic_vector(31 downto 0);
    wbs_dat_i            : in      std_logic_vector(31 downto 0);
    wbs_dat_o            : out     std_logic_vector(31 downto 0);
    wbs_sel_i            : in      std_logic_vector( 3 downto 0);
    wbs_cyc_i            : in      std_logic;
    wbs_stb_i            : in      std_logic;
    wbs_we_i             : in      std_logic;


--`ifdef PCI_WB_REV_B3
    wbs_cti_i            : in      std_logic_vector(2 downto 0);
    wbs_bte_i            : in      std_logic_vector(1 downto 0);
--    
--    wbs_cab_i            : in      std_logic;
--
    wbs_ack_o            : out     std_logic;
    wbs_rty_o            : out     std_logic;
    wbs_err_o            : out     std_logic;

-- WISHBONE master interface
    wbm_adr_o            : out     std_logic_vector(31 downto 0);
    wbm_dat_i            : in      std_logic_vector(31 downto 0);
    wbm_dat_o            : out     std_logic_vector(31 downto 0);
    wbm_sel_o            : out     std_logic_vector( 3 downto 0);
    wbm_cyc_o            : out     std_logic;
    wbm_stb_o            : out     std_logic;
    wbm_we_o             : out     std_logic;
    wbm_cti_o            : out     std_logic_vector(2 downto 0);
    wbm_bte_o            : out     std_logic_vector(1 downto 0);
    wbm_ack_i            : in      std_logic;
    wbm_rty_i            : in      std_logic;
    wbm_err_i            : in      std_logic;

   	msi_control_o        : out     std_logic_vector(31 downto 0);
    msi_address_o        : out     std_logic_vector(31 downto 0);
    msi_msg_data_o       : out     std_logic_vector(15 downto 0);
    
-- pci interface - system pins
    pci_clk_i            : in      std_logic;
    pci_rst_i            : in      std_logic;
    pci_rst_o            : out     std_logic;
    pci_rst_oe_o         : out     std_logic;

    pci_inta_i           : in      std_logic;
    pci_inta_o           : out     std_logic;
    pci_inta_oe_o        : out     std_logic;

-- arbitration pins
    pci_req_o            : out     std_logic;
    pci_req_oe_o         : out     std_logic;

    pci_gnt_i            : in      std_logic;


-- protocol pins
    pci_frame_i          : in      std_logic;
    pci_frame_o          : out     std_logic;
    pci_frame_oe_o       : out     std_logic;
    pci_irdy_oe_o        : out     std_logic;
    pci_devsel_oe_o      : out     std_logic;
    pci_trdy_oe_o        : out     std_logic;
    pci_stop_oe_o        : out     std_logic;
    pci_ad_oe_o          : out     std_logic_vector(31 downto 0);
    pci_cbe_oe_o         : out     std_logic_vector( 3 downto 0);

    pci_irdy_i           : in      std_logic;
    pci_irdy_o           : out     std_logic;

    pci_idsel_i          : in      std_logic;

    pci_devsel_i         : in      std_logic;
    pci_devsel_o         : out     std_logic;

    pci_trdy_i           : in      std_logic;
    pci_trdy_o           : out     std_logic;

    pci_stop_i           : in      std_logic;
    pci_stop_o           : out     std_logic;


-- data transfer pins
    pci_ad_i             : in      std_logic_vector(31 downto 0);
    pci_ad_o             : out     std_logic_vector(31 downto 0);

    pci_cbe_i            : in      std_logic_vector(3 downto 0);
    pci_cbe_o            : out     std_logic_vector(3 downto 0);

--parity    
    pci_par_i            : in      std_logic;
    pci_par_o            : out     std_logic;
    pci_par_oe_o         : out     std_logic;

    pci_perr_i           : in      std_logic;
    pci_perr_o           : out     std_logic;
    pci_perr_oe_o        : out     std_logic;
                         
-- system
    pci_serr_o           : out     std_logic;
    pci_serr_oe_o        : out     std_logic
    
    );
    
end component; --  pci_bridge32

end package wb_pci_pkg;    
    


