--
-- A simulation model of VIC20 hardware
-- Copyright (c) MikeJ - March 2003
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS CODE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- You are responsible for any legal issues arising from your use of this code.
--
-- The latest version of this file can be found at: www.fpgaarcade.com
--
-- Email vic20@fpgaarcade.com
--
--
-- Revision list
--
-- 200x-xx-xx	version 001 initial release
-- 2008-xx-xx	version 002 spartan3e release
-- 2009-02-15	changed the clocksignal generator code. The original DCM required a 50MHz input, this was not really accurate to be used as a base for the PAL and VGA signals, but addequate enough. Unfortunately on the spartan3 the DCM used to generate the 8.66 MHz signal is too low for the DCM, the DCM's output cannot function at such a low freq. On a spartan 3E this is not a problem but the minimig uses a spartan3, so this needed to be changed
--					the new code uses the input clock freq of 4.433MHz and multiplies that x8 and devides it then outside the DCM to the required 8 and 4 MHz clockfreqs that are needed for the core
-- 2009-02-16	added an automatic reset sequence, the system resets for 0.1 second after configuration. This elliminates the need for an external reset signal.
--			if the user wants to reset the core, then reconfigure the FPGA (by pressing the minimig's reset) (which takes approx. 4 seconds).
-- 2009-02-17	added the IEC-bus (and it really works)	
--	2009-02-20	joystick is now correctly mapped (in UCF), this now works

library ieee ;
  use ieee.std_logic_1164.all ;
  use ieee.std_logic_unsigned.all;
  use ieee.numeric_std.all;

library UNISIM;
  use UNISIM.Vcomponents.all;

entity VIC20 is port 
	(
	I_SCANDOUBLER				:in	std_logic;
   --
   I_PS2_CLK             	:in	std_logic;
   I_PS2_DATA            	:in	std_logic;
   --
   O_VIDEO_R             	:out	std_logic_vector(3 downto 0);
   O_VIDEO_G             	:out	std_logic_vector(3 downto 0);
   O_VIDEO_B             	:out	std_logic_vector(3 downto 0);
   O_HSYNC               	:out	std_logic;
   O_VSYNC               	:out	std_logic;
   --
   O_AUDIO_L             	:out	std_logic;
   O_AUDIO_R             	:out	std_logic;
   --
   I_CLK_REF             	:in	std_logic;
	--
	IEC_ATN						:out		std_logic;
	IEC_CLOCK					:inout	std_logic;
	IEC_DATA						:inout	std_logic;
	
--	auto_reset					:in std_logic;
	
	pwrled						:out std_logic;
	
	joy1_up						:in	std_logic;
	joy1_down					:in	std_logic;
	joy1_left					:in	std_logic;
	joy1_right					:in	std_logic;
	joy1_fire					:in	std_logic;
	
	
	--also the unused circuitry that is on the board must be defined, otherwise certain logic may be in a state where it draws excessive power or even oscillates
	joy1_button2				:in	std_logic;
------------------------	n_joy2						:out	std_logic_vector(5 downto 0);
	
------------------------	mclk							:out	std_logic;
	msclk							:out	std_logic;
	msdat							:out	std_logic;

	spiclk						:out	std_logic;
	spidin						:out	std_logic;
	spidout						:out	std_logic;

	cts							:out	std_logic;
	rts							:out	std_logic;
	rxd							:out	std_logic;
	txd							:out	std_logic;

	ramaddress					:out	std_logic_vector(18 downto 0);
	ramdata						:inout 	std_logic_vector(15 downto 0);
	n_ramsel0        			:out	std_logic;
	n_ramsel1        			:out	std_logic;
	n_lb        				:out	std_logic;
	n_oe        				:out	std_logic;
	n_ub        				:out	std_logic;
	n_we        				:out	std_logic;

	r_w							:out	std_logic;
	n_uds							:out	std_logic;
	n_as             			:out	std_logic;
	n_cpureset             	:out	std_logic;
	n_dtack             		:out	std_logic;
	n_ipl							:out	std_logic_vector(2 downto 0);
	n_lds            			:out	std_logic;
	cpuaddress					:out	std_logic_vector(23 downto 1);
	cpuclk						:out	std_logic;
	cpudata						:out	std_logic_vector(15 downto 0)
	);
end;

architecture RTL of VIC20 is

  	component clock_gen port(
    CLKIN_IN 			: in std_logic; 
    CLKFX_OUT 			: out std_logic;
    CLKIN_IBUFG_OUT 	: out std_logic
    );
	end component;
	
	-- default
	constant K_OFFSET : std_logic_vector (4 downto 0) := "10000"; -- h position of screen to centre on your telly
	-- lunar lander is WAY off to the left
	--constant K_OFFSET : std_logic_vector (4 downto 0) := "11100"; -- h position of screen to centre on your telly
	
	signal clock_35MHz			: std_logic;
	signal clock_div				: std_logic_vector(2 downto 0);
	--signal auto_reset        	: std_logic;
	--signal auto_reset_counter	: std_logic_vector(20 downto 0);

    signal clock_8MHz         : std_logic;
    signal ena_4              : std_logic;
    -- cpu
    signal c_ena              : std_logic;
    signal c_addr             : std_logic_vector(23 downto 0);
    signal c_din              : std_logic_vector(7 downto 0);
    signal c_dout             : std_logic_vector(7 downto 0);
    signal c_rw_l             : std_logic;
    signal c_irq_l            : std_logic;
    signal c_nmi_l            : std_logic;
    --
    signal io_sel_l           : std_logic_vector(3 downto 0);
    signal blk_sel_l          : std_logic_vector(7 downto 0);
    signal ram_sel_l          : std_logic_vector(7 downto 0);

    -- vic
    signal vic_addr           : std_logic_vector(13 downto 0);
    signal vic_oe_l           : std_logic;
    signal vic_dout           : std_logic_vector( 7 downto 0);
    signal vic_din            : std_logic_vector(11 downto 0);
    signal p2_h               : std_logic;
    signal ena_1mhz           : std_logic;
    signal vic_audio          : std_logic_vector( 3 downto 0);
    signal audio_pwm          : std_logic;
    signal via1_dout          : std_logic_vector( 7 downto 0);
    signal via2_dout          : std_logic_vector( 7 downto 0);
    -- video system
    signal v_addr             : std_logic_vector(13 downto 0);
    signal v_data             : std_logic_vector( 7 downto 0);
    signal v_data_oe_l        : std_logic;
    signal v_data_read_mux    : std_logic_vector( 7 downto 0);
    signal v_data_read_muxr   : std_logic_vector( 7 downto 0);
    signal v_rw_l             : std_logic;
    signal col_ram_sel_l      : std_logic;

    -- ram
    signal ram0_dout          : std_logic_vector(7 downto 0);
	 signal ram12_dout          : std_logic_vector(7 downto 0);
	 signal ram3_dout         : std_logic_vector(7 downto 0);
    signal ram45_dout         : std_logic_vector(7 downto 0);
    signal ram67_dout         : std_logic_vector(7 downto 0);
    --
    signal col_ram_dout       : std_logic_vector(7 downto 0);

    signal char_rom_dout      : std_logic_vector(7 downto 0);
    signal basic_rom_dout     : std_logic_vector(7 downto 0);
    signal kernal_rom_dout    : std_logic_vector(7 downto 0);

    signal exp_ram_din        : std_logic_vector(7 downto 0);
	 signal	exp_ram_cs_n	   : std_logic;
    signal expansion_din      : std_logic_vector(7 downto 0);
    signal expansion_nmi_l    : std_logic;
    signal expansion_irq_l    : std_logic;

    -- VIAs
    signal via1_nmi_l         : std_logic;
    signal via1_pa_in         : std_logic_vector(7 downto 0);
    signal via1_pa_out        : std_logic_vector(7 downto 0);

    signal via2_irq_l         : std_logic;

    signal cass_write         : std_logic;
    signal cass_read          : std_logic;
    signal cass_motor         : std_logic;
    signal cass_sw            : std_logic;

    signal keybd_col_out      : std_logic_vector(7 downto 0);
    signal keybd_col_in       : std_logic_vector(7 downto 0);
    signal keybd_col_oe_l     : std_logic_vector(7 downto 0);
    signal keybd_row_in       : std_logic_vector(7 downto 0);
    signal keybd_restore      : std_logic;

    --signal joy                : std_logic_vector(3 downto 0);
    --signal light_pen          : std_logic;

    signal serial_srq_in      : std_logic;
    signal serial_atn_out     : std_logic; -- the vic does not listen to atn_in
    signal serial_clk_out     : std_logic;
    signal serial_clk_in      : std_logic;
    signal serial_data_out    : std_logic;
    signal serial_data_in     : std_logic;

    -- user port
    signal user_port_cb1_in   : std_logic;
    signal user_port_cb1_out  : std_logic;
    signal user_port_cb1_oe_l : std_logic;
    signal user_port_cb2_in   : std_logic;
    signal user_port_cb2_out  : std_logic;
    signal user_port_cb2_oe_l : std_logic;
    signal user_port_in       : std_logic_vector(7 downto 0);
    signal user_port_out      : std_logic_vector(7 downto 0);
    signal user_port_oe_l     : std_logic_vector(7 downto 0);
    -- misc

    signal video_r            : std_logic_vector(3 downto 0);
    signal video_g            : std_logic_vector(3 downto 0);
    signal video_b            : std_logic_vector(3 downto 0);
    signal hsync              : std_logic;
    signal vsync              : std_logic;
    signal csync              : std_logic;
    signal video_r_x2         : std_logic_vector(3 downto 0);
    signal video_g_x2         : std_logic_vector(3 downto 0);
    signal video_b_x2         : std_logic_vector(3 downto 0);
    signal hsync_x2           : std_logic;
    signal vsync_x2           : std_logic;

--	 signal cart_data          : std_logic_vector(7 downto 0);
	 
	 signal  we           : std_logic;
	 signal we0           : std_logic; 
	 signal we1           : std_logic; 
	 signal we2           : std_logic; 
	 signal we3           : std_logic; 
	 signal we4           : std_logic; 
	 signal we5           : std_logic; 
	 
	 signal 	u_dout : unsigned(7 downto 0);
	 signal	u_addr : unsigned(15 downto 0);
	 
	 signal ram31_dout         : std_logic_vector(7 downto 0);
	 signal por_n_s : std_logic;
	 signal reset_n_s : std_logic;
	
	 
begin

 -----------------------------------------------------------------------------
  -- Reset generation
  -----------------------------------------------------------------------------
  por_b : entity work.vic_por
    port map (
      clk_i   => clock_8MHz,
      por_n_o => por_n_s
    );
 
  reset_n_s <= por_n_s and joy1_button2;
  pwrled <= reset_n_s;



--	Automatic_reset : process		--this will activate the reset line for a short moment after configuration of the FPGA
--	begin
--		wait until rising_edge(clock_8MHz);
--		if (auto_reset_counter(20) = '0') then
--			auto_reset_counter <= auto_reset_counter + 1;
--			pwrled <= '0';
--			auto_reset <= '0';			--the LED is OFF
--		else
--			pwrled <= '1';					--the LED is ON (iow: it now emits lights)
--			auto_reset <= '1';
--		end if;
--	end process;

	clock_divider : process		--this will divide the DCM's output signal into a useable set of clocks
	begin
		wait until rising_edge(clock_35MHz);
		clock_div <= clock_div + 1;
	end process;
	
	BUFG0 : BUFG port map (I=> clock_div(1), O => clock_8MHz);	--definition of clocksignal tied to a buffer
	BUFG1 : BUFG port map (I=> clock_div(2), O => ena_4);	--definition of clocksignal tied to a buffer

	IECbusclk : process					 	-- IEC-bus clock signal (aka: serial bus)
	begin
		wait until rising_edge(clock_8MHz);
		if (serial_clk_out = '0') then	-- keep in mind that the signals are inverted by the open-collector buffer in the real VIC20
			IEC_CLOCK <= 'Z';					-- so that's what we do here as well. 'Z' simply means making the output high-impedance (floating), and the pull-up (as defined in the UCF) will make it a logic '1'
		else
			IEC_CLOCK <= '0';
		end if;
	end process;
	
	IECbusdat : process					 	-- IEC-bus data signal (aka: serial bus)
	begin
		wait until rising_edge(clock_8MHz);
		if (serial_data_out = '0') then	-- keep in mind that the signals are inverted by the open-collector buffer in the real VIC20
			IEC_DATA	<= 'Z';					-- so that's what we do here as well. 'Z' simply means making the output high-impedance (floating), and the pull-up (as defined in the UCF) will make it a logic '1'
		else
			IEC_DATA	<= '0';
		end if;
	end process;

	IEC_ATN <= not serial_atn_out;		-- the ATN signal is NOT a bidirectional signal, so this is easy programming. The inverting is required because in the real VIC20, this signal is buffered by an inverting buffer
	serial_clk_in <= IEC_CLOCK;
	serial_data_in <= IEC_DATA;
	serial_srq_in <= '0';	

	--also the unused circuitry that is on the board must be defined
--	joy1_button2				<= 'Z';			--the VIC20's joystick port has only one "fire" button
---------	n_joy2(5 downto 0)		<= "ZZZZZZ";	--the VIC20 has only one joystick port, so we disable the second one
	
---------	mclk							<= 'Z';	--this core does not communicate with the MMC/SD-card, so these lines must be disabled so that they can't "hurt" the card's IO-pins
	msclk							<= 'Z';
	msdat							<= 'Z';

	spiclk						<= 'Z';	--this core does not communicate with the PIC and does not communicate with the MMC/SD-card using the SPI-lines, so these lines must be disabled so that they can't "hurt" the PIC's IO-pins and that it does not interfere with the spi communication between the PIC and the MMC/SD-card
	spidin						<= 'Z';
	spidout						<= 'Z';
	
	cts							<= 'Z';
	rts							<= 'Z';
	rxd							<= 'Z';
	txd							<= 'Z';

--	ramaddress(19 downto 1)	<= "0000000000000000000";
--	ramdata(15 downto 0)		<= "ZZZZZZZZZZZZZZZZ"; 
--	n_ramsel0        			<= '1';
	n_ramsel1        			<= '1';
--	n_lb        				<= '1';
--	n_oe        				<= '1';
--	n_ub        				<= '1';
--	n_we        				<= '1';

	r_w							<= 'Z';
	n_uds							<= 'Z';
	n_as        				<= 'Z';
	n_cpureset  				<= '0';
	n_dtack     				<= '1';
	n_ipl(2 downto 0)			<= "111";
	n_lds       				<= 'Z';
	cpuaddress(23 downto 1)	<= "00000000000000000000000";
	cpuclk						<= '0';
	cpudata(15 downto 0)		<= "ZZZZZZZZZZZZZZZZ"; 
  
  
n_lb <= c_addr(0);
n_ub <= not c_addr(0);
n_oe <= '0';
n_ramsel0 <= exp_ram_cs_n;
n_we <= c_rw_l;

ramdata( 7 downto 0) <= c_dout when (c_rw_l='0' and exp_ram_cs_n='0' and c_addr(0)='0')
                      else "ZZZZZZZZ";

ramdata( 15 downto 8) <= c_dout when (c_rw_l='0' and exp_ram_cs_n='0' and c_addr(0)='1')
                      else "ZZZZZZZZ";
	
exp_ram_din <= ramdata(7 downto 0)    when (exp_ram_cs_n='0' and c_addr(0)='0')
            else ramdata(15 downto 8) when (exp_ram_cs_n='0' and c_addr(0)='1')
            else "ZZZZZZZZ";	

exp_ram_cs_n <= blk_sel_l(1) and blk_sel_l(2) and blk_sel_l(3) and blk_sel_l(5);  
 

expansion_din <= exp_ram_din;

 ramaddress(18 downto 15) <= "0000";  
 ramaddress(14 downto 0) <= c_addr(15 downto 1);

--  ramaddress(18 downto 14) <= "00000";  
--  ramaddress(13 downto 0) <= c_addr(14 downto 1) when (blk_sel_l(1)='0' and blk_sel_l(2)='0' and blk_sel_l(3)='0')
--									else "000" & c_addr(11 downto 1) when blk_sel_l(5)='0'
--									else (others=>'Z');
  --
  -- IO connect these to the outside world if you wish ...
  --
--p_expansion : process(exp_ram_cs_n, exp_ram_din)
--  begin
--    expansion_din <= x"FF";
--    if (exp_ram_cs_n= '0') then
--      expansion_din <= exp_ram_din;
--    end if;
--  end process;
  
  -- expansion port
 -- ramaddress <= "000"+ c_addr;
  -- <= c_rw_l;
  -- <= v_rw_l;
  expansion_nmi_l <= '1';
  expansion_irq_l <= '1';
  -- <= ram_sel_l;
  -- <= io_sel_l;
  -- <= reset_l_sampled;

  -- user port
  user_port_cb1_in <= '0';
  user_port_cb2_in <= '0';
  user_port_in <= x"00";
  -- <= user_port_out
  -- <= user_port_out_oe_l

  -- tape
  cass_read <= '0';
  --<= cass_write;
  --<= cass_motor
  cass_sw <= '1'; -- motor off

  clock_generator : clock_gen
	port map (
		CLKIN_IN    		=> I_CLK_REF,
		CLKFX_OUT			=> clock_35MHz,
		CLKIN_IBUFG_OUT	=> open
	);
	 

  c_ena <= ena_1mhz and ena_4; -- clk ena
--========================================================
  cpu : entity work.T65
      port map (
          Mode    => "00",
          Res_n   => reset_n_s,
          Enable  => c_ena,
          Clk     => clock_8MHz,
          Rdy     => '1',
          Abort_n => '1',
          IRQ_n   => c_irq_l,
          NMI_n   => c_nmi_l,
          SO_n    => '1',
          R_W_n   => c_rw_l,
          Sync    => open,
          EF      => open,
          MF      => open,
          XF      => open,
          ML_n    => open,
          VP_n    => open,
          VDA     => open,
          VPA     => open,
          A       => c_addr,
          DI      => c_din,
          DO      => c_dout
      );

--===========================================================
--cpu :  entity work.cpu65xx
--      generic map(
--              pipelineOpcode => true,
--              pipelineAluMux => true,
--              pipelineAluOut => true
--      )
--      port map (
--              clk       => clock_8MHz,
--              enable => c_ena,
--              reset  =>  not  reset_n_s,
--              nmi_n  =>  c_nmi_l,
--              irq_n => c_irq_l,
--              so_n  => '1',
-- 
--              di =>   unsigned(c_din),
--              do =>   u_dout,
--              addr => u_addr,
--              we => we,
--
--              debugOpcode => open,
--              debugPc => open,
--              debugA => open,
--              debugX => open,
--              debugY => open,
--              debugS => open
--      );
--
--
--      c_dout <=  std_logic_vector(resize(unsigned(u_dout),8)) ;
--      c_addr <=  std_logic_vector(resize(unsigned(u_addr),16));
--	   c_rw_l <= not we;
--=================================================================


  vic : entity work.VIC20_VIC
    generic map (
      K_OFFSET        => K_OFFSET
      )
    port map (
      I_RW_L          => v_rw_l,

      I_ADDR          => v_addr(13 downto 0),
      O_ADDR          => vic_addr(13 downto 0),

      I_DATA          => vic_din,
      O_DATA          => vic_dout,
      O_DATA_OE_L     => vic_oe_l,
      --
      O_AUDIO         => vic_audio,

      O_VIDEO_R       => video_r,
      O_VIDEO_G       => video_g,
      O_VIDEO_B       => video_b,

      O_HSYNC         => hsync,
      O_VSYNC         => vsync,
      O_COMP_SYNC_L   => csync,
      --
      --
      I_LIGHT_PEN     => joy1_fire,	--light_pen,
      I_POTX          => '0',
      I_POTY          => '0',

      O_ENA_1MHZ      => ena_1mhz,
      O_P2_H          => p2_h,
      ENA_4           => ena_4,
      CLK             => clock_8MHz
      );

  via1 : entity work.M6522
    port map (
      I_RS            => c_addr(3 downto 0),
      I_DATA          => v_data(7 downto 0),
      O_DATA          => via1_dout,
      O_DATA_OE_L     => open,

      I_RW_L          => c_rw_l,
      I_CS1           => c_addr(4),
      I_CS2_L         => io_sel_l(0),

      O_IRQ_L         => via1_nmi_l, -- note, not open drain

      I_CA1           => keybd_restore,
      I_CA2           => cass_motor,
      O_CA2           => cass_motor,
      O_CA2_OE_L      => open,

      I_PA            => via1_pa_in,
      O_PA            => via1_pa_out,
      O_PA_OE_L       => open,

      -- port b
      I_CB1           => user_port_cb1_in,
      O_CB1           => user_port_cb1_out,
      O_CB1_OE_L      => user_port_cb1_oe_l,

      I_CB2           => user_port_cb2_in,
      O_CB2           => user_port_cb2_out,
      O_CB2_OE_L      => user_port_cb2_oe_l,

      I_PB            => user_port_in,
      O_PB            => user_port_out,
      O_PB_OE_L       => user_port_oe_l,

      I_P2_H          => p2_h,
      RESET_L         => reset_n_s,
      ENA_4           => ena_4,
      CLK             => clock_8MHz
      );

  serial_atn_out <= via1_pa_out(7);
  via1_pa_in(7) <= via1_pa_out(7);
  via1_pa_in(6) <= cass_sw;
  via1_pa_in(5) <= joy1_fire;		--light_pen;
  via1_pa_in(4) <= joy1_down;
  via1_pa_in(3) <= joy1_left;
  via1_pa_in(2) <= joy1_right;
  via1_pa_in(1) <= serial_data_in;
  via1_pa_in(0) <= serial_clk_in;

  via2 : entity work.M6522
    port map (
      I_RS            => c_addr(3 downto 0),
      I_DATA          => v_data(7 downto 0),
      O_DATA          => via2_dout,
      O_DATA_OE_L     => open,

      I_RW_L          => c_rw_l,
      I_CS1           => c_addr(5),
      I_CS2_L         => io_sel_l(0),

      O_IRQ_L         => via2_irq_l, -- note, not open drain

      I_CA1           => cass_read,
      I_CA2           => serial_clk_out,
      O_CA2           => serial_clk_out,
      O_CA2_OE_L      => open,

      I_PA            => keybd_row_in,
      O_PA            => open,
      O_PA_OE_L       => open,

      -- port b
      I_CB1           => serial_srq_in,
      O_CB1           => open,
      O_CB1_OE_L      => open,

      I_CB2           => serial_data_out,
      O_CB2           => serial_data_out,
      O_CB2_OE_L      => open,

      I_PB            => keybd_col_in,
      O_PB            => keybd_col_out,
      O_PB_OE_L       => keybd_col_oe_l,

      I_P2_H          => p2_h,
      RESET_L         => reset_n_s,
      ENA_4           => ena_4,
      CLK             => clock_8MHz
      );

  --p_keybd_col_in : process(keybd_col_out, keybd_col_oe_l, joy)
  p_keybd_col_in : process(keybd_col_out, keybd_col_oe_l, joy1_up)
  begin
    for i in 0 to 6 loop
      keybd_col_in(i) <= keybd_col_out(i);
    end loop;

    if (keybd_col_oe_l(7) = '0') then
      keybd_col_in(7) <= keybd_col_out(7);
    else
      keybd_col_in(7) <= joy1_up;
    end if;
  end process;
  cass_write <= keybd_col_out(3);

  keybd : entity work.VIC20_PS2_IF
    port map (

      I_PS2_CLK       => I_PS2_CLK,
      I_PS2_DATA      => I_PS2_DATA,

      I_COL           => keybd_col_out,
      O_ROW           => keybd_row_in,
      O_RESTORE       => keybd_restore,

      I_ENA_1MHZ      => ena_1mhz,
      I_P2_H          => p2_h,
      RESET_L         => reset_n_s,
      ENA_4           => ena_4,
      CLK             => clock_8MHz
      );

  p_irq_resolve : process(expansion_irq_l, expansion_nmi_l,via2_irq_l, via1_nmi_l)
  begin
    c_irq_l <= '1';
    if (expansion_irq_l = '0') or (via2_irq_l = '0') then
      c_irq_l <= '0';
    end if;

    c_nmi_l <= '1';
    if (expansion_nmi_l = '0') or (via1_nmi_l = '0') then
      c_nmi_l <= '0';
    end if;
  end process;

  --
  -- decode
  --
  p_io_addr_decode : process(c_addr)
  begin

    io_sel_l <= "1111";
    if (c_addr(15 downto 13) = "100") then -- blk4
      case c_addr(12 downto 10) is
        when "000" => io_sel_l <= "1111";
        when "001" => io_sel_l <= "1111";
        when "010" => io_sel_l <= "1111";
        when "011" => io_sel_l <= "1111";
        when "100" => io_sel_l <= "1110";
        when "101" => io_sel_l <= "1101"; -- col
        when "110" => io_sel_l <= "1011";
        when "111" => io_sel_l <= "0111";
        when others => null;
      end case;
    end if;
  end process;

  p_blk_addr_decode : process(c_addr)
  begin
    blk_sel_l <= "11111111";
    case c_addr(15 downto 13) is
      when "000" => blk_sel_l <= "11111110";
      when "001" => blk_sel_l <= "11111101";
      when "010" => blk_sel_l <= "11111011";
      when "011" => blk_sel_l <= "11110111";
      when "100" => blk_sel_l <= "11101111";
      when "101" => blk_sel_l <= "11011111"; -- Cart
      when "110" => blk_sel_l <= "10111111"; -- basic
      when "111" => blk_sel_l <= "01111111"; -- kernal
      when others => null;
    end case;
  end process;

  p_v_mux : process(c_addr, c_dout, c_rw_l, p2_h, vic_addr, v_data_read_mux,blk_sel_l, io_sel_l)
  begin
    -- simplified data source mux
    if (p2_h = '0') then
      v_addr(13 downto 0) <= vic_addr(13 downto 0);
      v_data <= v_data_read_mux(7 downto 0);
      v_rw_l <= '1';
      col_ram_sel_l <= '1'; -- colour ram has dedicated mux for vic, so disable
    else -- cpu
      v_addr(13 downto 0) <= blk_sel_l(4) & c_addr(12 downto 0);
      v_data <= c_dout;
      v_rw_l <= c_rw_l;
      col_ram_sel_l <= io_sel_l(1);
    end if;

  end process;

  p_ram_addr_decode : process(v_addr, blk_sel_l, p2_h)
  begin
    ram_sel_l <= "11111111";
    if ((p2_h = '1') and (blk_sel_l(0) = '0')) or -- cpu
       ((p2_h = '0') and (v_addr(13) = '1')) then
      case v_addr(12 downto 10) is
        when "000" => ram_sel_l <= "11111110";
        when "001" => ram_sel_l <= "11111101";
        when "010" => ram_sel_l <= "11111011";
        when "011" => ram_sel_l <= "11110111";
        when "100" => ram_sel_l <= "11101111";
        when "101" => ram_sel_l <= "11011111";
        when "110" => ram_sel_l <= "10111111";
        when "111" => ram_sel_l <= "01111111";
        when others => null;
      end case;
    end if;
  end process;

  p_vic_din_mux : process(p2_h, col_ram_dout, v_data)
  begin
    if (p2_h = '0') then
      vic_din(11 downto 8) <= col_ram_dout(3 downto 0);
    else
      vic_din(11 downto 8) <= v_data(3 downto 0);
    end if;

    vic_din(7 downto 0) <= v_data(7 downto 0);
  end process;

  p_v_read_mux : process(col_ram_sel_l, ram_sel_l, vic_oe_l, v_addr,col_ram_dout, ram0_dout, ram45_dout, ram67_dout,vic_dout, char_rom_dout, v_data_read_muxr)
  begin
    -- simplified data read mux
    -- nasty if statement but being lazy
    -- these are exclusive, but the tools may not spot this.

    v_data_oe_l <= '1';
    if (col_ram_sel_l = '0') then
      v_data_read_mux <= "0000" & col_ram_dout(3 downto 0);
      v_data_oe_l     <= '0';
    elsif (vic_oe_l = '0') then
      v_data_read_mux <= vic_dout;
      v_data_oe_l     <= '0';
    elsif (ram_sel_l(0) = '0') then
      v_data_read_mux <= ram0_dout;
      v_data_oe_l     <= '0';
--=============================================	3k expansion not working
--		elsif (ram_sel_l(1) = '0') then
--      v_data_read_mux <= ram12_dout;
--      v_data_oe_l     <= '0';
--	
--	   elsif (ram_sel_l(2) = '0') then
--      v_data_read_mux <= ram12_dout;
--      v_data_oe_l     <= '0';
--	
--		elsif (ram_sel_l(3) = '0') then
--      v_data_read_mux <= ram3_dout;
--      v_data_oe_l     <= '0';
--==============================================		
    elsif (ram_sel_l(4) = '0') then
      v_data_read_mux <= ram45_dout;
      v_data_oe_l     <= '0';
    elsif (ram_sel_l(5) = '0') then
      v_data_read_mux <= ram45_dout;
      v_data_oe_l     <= '0';
    elsif (ram_sel_l(6) = '0') then
      v_data_read_mux <= ram67_dout;
      v_data_oe_l     <= '0';
    elsif (ram_sel_l(7) = '0') then
      v_data_read_mux <= ram67_dout;
      v_data_oe_l     <= '0';
    elsif (v_addr(13 downto 12) = "00") then
      v_data_read_mux <= char_rom_dout;
      v_data_oe_l     <= '0';
		
	-- ===============================================lak 8k ram 0x2000	
--	 elsif (blk_sel_l(1))='0' then
--      v_data_read_mux <= ram8k_dout;
--      v_data_oe_l     <= '0';	
		
    else
      -- emulate floating bus
      --v_data_read_mux <= "XXXXXXXX";
      v_data_read_mux <= v_data_read_muxr;
    end if;

  end process;

  p_v_bus_hold : process
  begin
    wait until rising_edge(clock_8MHz);
    if (ena_4 = '1') then
      v_data_read_muxr <= v_data_read_mux;
    end if;
  end process;

  p_cpu_read_mux : process(p2_h, c_addr, io_sel_l, ram_sel_l, blk_sel_l, v_data_read_mux, via1_dout, via2_dout, v_data_oe_l, basic_rom_dout, kernal_rom_dout, expansion_din)
  begin
    if (p2_h = '0') then -- vic is on the bus
      --c_din <= "XXXXXXXX";
      c_din <= "00000000";
    elsif (io_sel_l(0) = '0') and (c_addr(4) = '1') then -- blk4
      c_din <= via1_dout;
    elsif (io_sel_l(0) = '0') and (c_addr(5) = '1') then -- blk4
      c_din <= via2_dout;
	 elsif (blk_sel_l(1) = '0') then
      c_din <= expansion_din;		
    elsif (blk_sel_l(2) = '0') then
      c_din <= expansion_din;
	 elsif (blk_sel_l(3) = '0') then
      c_din <= expansion_din;
	 elsif (blk_sel_l(5) = '0') then
      c_din <= expansion_din;
    elsif (blk_sel_l(6) = '0') then
      c_din <= basic_rom_dout;
    elsif (blk_sel_l(7) = '0') then
      c_din <= kernal_rom_dout;
    elsif (v_data_oe_l = '0') then
      c_din <= v_data_read_mux;
    else
      c_din <= "11111111";
    end if;
  end process;

  -- main memory
  rams0 : entity work.VIC20_RAMS
    port map (
      V_ADDR => v_addr(9 downto 0),
      DIN    => v_data,
      DOUT   => ram0_dout,
      V_RW_L => v_rw_l,
      CS1_L  => ram_sel_l(0),
      CS2_L  => '1',
      ENA    => ena_4,
      CLK    => clock_8MHz
      );

  rams45 : entity work.VIC20_RAMS
    port map (
      V_ADDR => v_addr(9 downto 0),
      DIN    => v_data,
      DOUT   => ram45_dout,
      V_RW_L => v_rw_l,
      CS1_L  => ram_sel_l(4),
      CS2_L  => ram_sel_l(5),
      ENA    => ena_4,
      CLK    => clock_8MHz
      );

  rams67 : entity work.VIC20_RAMS
    port map (
      V_ADDR => v_addr(9 downto 0),
      DIN    => v_data,
      DOUT   => ram67_dout,
      V_RW_L => v_rw_l,
      CS1_L  => ram_sel_l(6),
      CS2_L  => ram_sel_l(7),
      ENA    => ena_4,
      CLK    => clock_8MHz
      );

-- ============================================================================= for 3K expansion - not working
------------------------------------------------------------ 1k ram 
--  we5 <= (not ram_sel_l(3)) and (not v_rw_l );
-- 
--  rams31 : entity work.generic_ram_en_dist
--	 generic map (
--    addr_width_g => 10,
--    data_width_g => 8
--    )
--    port map (
--	   clk_i    => clock_8Mhz,
--		ena	=> ena_4,
--      a_i => v_addr(9 downto 0), 
--		we_i => we5,
--      d_i    => v_data,
--      d_o   => ram3_dout
--      );
-- 
-----------------------------------------------------------------2k ram
--  we4 <= ((not ram_sel_l(1)) or (not ram_sel_l(2))) and (not c_rw_l );
-- 
--  rams12 : entity work.generic_ram_en  --2k
--	 generic map (
--    addr_width_g => 11,
--    data_width_g => 8
--    )
--    port map (
--	   clk_i    => clock_8Mhz,
--		ena	=> ena_4,
--      a_i => c_addr(10 downto 0), 
--		we_i => we4,
--      d_i    => c_dout,
--      d_o   => ram12_dout
--      );
 
--===================================================================================== 
 
--   we0 <= (not ram_sel_l(0)) and (not v_rw_l );
-- 
--  rams0 : entity work.generic_ram_en_dist
--	 generic map (
--    addr_width_g => 10,
--    data_width_g => 8
--    )
--    port map (
--	   clk_i    => clock_8Mhz,
--		ena	=> ena_4,
--      a_i => v_addr(9 downto 0), 
--		we_i => we0,
--      d_i    => v_data,
--      d_o   => ram0_dout
--      );
--
--  we1 <= ((not ram_sel_l(4)) or (not ram_sel_l(5))) and (not v_rw_l );
--  
--  rams45 : entity work.generic_ram_en_dist
--	 generic map (
--    addr_width_g => 11,
--    data_width_g => 8
--    )
--    port map (
--	   clk_i    => clock_8Mhz,
--		ena	=> ena_4,
--      a_i => v_addr(10 downto 0), 
--		we_i => we1,
--      d_i    => v_data,
--      d_o   => ram45_dout
--      );
--
--  we2 <= ((not ram_sel_l(6)) or (not ram_sel_l(7))) and (not v_rw_l );
--  
--  rams67 : entity work.generic_ram_en_dist
--	 generic map (
--    addr_width_g => 11,
--    data_width_g => 8
--    )
--    port map (
--	   clk_i    => clock_8Mhz,
--		ena	=> ena_4,
--      a_i => v_addr(10 downto 0), 
--		we_i => we2,
--      d_i    => v_data,
--      d_o   => ram67_dout
--      );
--
--  we3 <= (not col_ram_sel_l) and (not v_rw_l );
   
--  col_ram : entity work.generic_ram_en_dist
--	 generic map (
--    addr_width_g => 10,
--    data_width_g => 8
--    )
--    port map (
--	   clk_i    => clock_8Mhz,
--		ena	=> ena_4,
--      a_i => v_addr(9 downto 0), 
--		we_i => we3,
--      d_i    => v_data(7 downto 0),
--      d_o   => col_ram_dout(7 downto 0)
--      );
--==========================================================================
   col_ram : entity work.VIC20_RAM
    port map (
      V_ADDR => v_addr(9 downto 0),
      DIN    => v_data,
      DOUT   => col_ram_dout,
      V_RW_L => v_rw_l,
      CS_L   => col_ram_sel_l,
      ENA    => ena_4,
      CLK    => clock_8MHz
      );

  char_rom : entity work.VIC20_CHAR_ROM		  -- VIC20's character ROM
    port map (
      CLK         => clock_8MHz,
      ENA         => ena_4,
      ADDR        => v_addr(11 downto 0),
      DATA        => char_rom_dout
      );

  basic_rom : entity work.VIC20_BASIC_ROM		  -- VIC20's basic ROM
    port map (
      CLK         => clock_8MHz,
      ENA         => ena_4,
      ADDR        => c_addr(12 downto 0),
      DATA        => basic_rom_dout
      );

  kernal_rom : entity work.VIC20_KERNAL_ROM		-- VIC20's kernal ROM
    port map (
      CLK         => clock_8MHz,
      ENA         => ena_4,
      ADDR        => c_addr(12 downto 0),
      DATA        => kernal_rom_dout
      );

  u_dblscan : entity work.VIC20_DBLSCAN			-- scan doubler, this is the logic that makes the VGA output possible
    port map (
      I_R               => video_r,
      I_G               => video_g,
      I_B               => video_b,
      I_HSYNC           => hsync,
      I_VSYNC           => vsync,
      --
      O_R               => video_r_x2,
      O_G               => video_g_x2,
      O_B               => video_b_x2,
      O_HSYNC           => hsync_x2,
      O_VSYNC           => vsync_x2,
      --
      ENA_X2            => '1',
      ENA               => ena_4,
      CLK               => clock_8MHz
    );
  --

  p_video_ouput : process						--the user can select the PAL or VGA mode, so this logic handles that selection
  begin
    wait until rising_edge(clock_8MHz);
    if (I_SCANDOUBLER = '1') then
      O_VIDEO_R <= video_r_x2;
      O_VIDEO_G <= video_g_x2;
      O_VIDEO_B <= video_b_x2;
      O_HSYNC   <= hSync_X2;
      O_VSYNC   <= vSync_X2;
    else
      O_VIDEO_R <= video_r;
      O_VIDEO_G <= video_g;
      O_VIDEO_B <= video_b;
      --O_HSYNC   <= hSync;
      --O_VSYNC   <= vSync;
      O_HSYNC   <= cSync;
      O_VSYNC   <= '1';
    end if;
  end process;

  u_dac : entity work.dac		  -- Audio, this is the 1-bit digital to analog converter, do not be fooled by the name 1-bit. This is the most common DAC today. It's easy, it's cheap, it's simple and most of all... it's accurate (enough)
    generic map(
      msbi_g => 3
    )
    port  map(
      clk_i   => clock_35MHz,
      res_n_i => reset_n_s,
      dac_i   => vic_audio,
      dac_o   => audio_pwm
    );

  O_AUDIO_L <= audio_pwm;
  O_AUDIO_R <= audio_pwm;
  
--  rams2000h : entity work.generic_ram_en_dist
--	 generic map (
--    addr_width_g => 12,--13
--    data_width_g => 8
--    )
--    port map (
--	   clk_i    => clock_8Mhz,
--		ena	=> not blk_sel_l(1),
--      a_i => v_addr(11 downto 0), --12
--		we_i => not v_rw_l,
--      d_i    => v_data,
--      d_o   => ram8k_dout
--      );

  --
  -- cart slot 0xA000-0xBFFF (8K)
  --
--cart : entity work.cart_rom
--    port map (
--      CLK         => clock_8Mhz,
--      ENA         => ena_4,--not blk_sel_l(5) 
--      ADDR        => c_addr(12 downto 0),
--      DATA        => cart_data
--      );


--  cart: entity work.generic_ram_en  --8k
--	 generic map (
--    addr_width_g => 13,
--    data_width_g => 8
--    )
--    port map (
--	   clk_i    => clock_8Mhz,
--		ena	=> not blk_sel_l(5),
--      a_i => c_addr(12 downto 0), 
--		we_i => not v_rw_l,
--      d_i    => v_data,
--      d_o   =>  cart_data
--      );
--		
end RTL;
