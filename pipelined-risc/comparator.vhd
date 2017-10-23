library std;
use std.standard.all;
library ieee;
use ieee.std_logic_1164.all;

entity comparator is
  port ( X, Y : in std_logic_vector(15 downto 0);
  		eq : out std_logic);
end entity ;

architecture compare of comparator is
begin
process (X, Y)
begin
	if ((X xor Y) = "0000000000000000") then
		eq <= '1';
	else
		eq <= '0';
	end if;
end process;
end compare;