
library ieee;
use ieee.std_logic_1164.all;

entity vic_por is

  port (
    clk_i   : in  std_logic;
    por_n_o : out std_logic
  );

end vic_por;


library ieee;
use ieee.numeric_std.all;

architecture spartan3 of vic_por is

  -----------------------------------------------------------------------------
  -- According to
  -- "XST User Guide", Chapter 6 "VHDL Language Support", "Initial Values"
  -- XST honors the initial value assigned to a flip-flop. Simple :-)
  --
  signal por_cnt_q : unsigned(3 downto 0) := "0000";
  signal por_n_q   : std_logic := '0';
  --
  -----------------------------------------------------------------------------

begin

  -----------------------------------------------------------------------------
  -- Process por_cnt
  --
  -- Purpose:
  --   Generate a power-on reset for 4 clock cycles.
  --
  por_cnt: process (clk_i)
  begin
    if clk_i'event and clk_i = '1' then
      if por_cnt_q = "11" then
        por_n_q   <= '1';
      else
        por_cnt_q <= por_cnt_q + 1;
      end if;
    end if;
  end process por_cnt;
  --
  -----------------------------------------------------------------------------

  por_n_o <= por_n_q;

end spartan3;
