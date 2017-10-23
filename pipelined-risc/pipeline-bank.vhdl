library std;
use std.standard.all;
library ieee;
use ieee.std_logic_1164.all;

entity pipe is
	port ( ir_in, npc_in, t1_in, t2_in, t3_in, memd_in : in std_logic_vector(15 downto 0);
		ir_out, npc_out, t1_out, t2_out, t3_out, memd_out : out std_logic_vector(15 downto 0);
		contr_in : in std_logic_vector(18 downto 0);
		contr_out : out std_logic_vector(18 downto 0);
		pipe_en, c_in, z_in : in std_logic;
		c_out, z_out : out std_logic;
		clk : in std_logic);
end entity;

architecture pipe_behave of pipe is
	component dregister is
	  port (
	    din  : in  std_logic_vector(15 downto 0);
	    dout : out std_logic_vector(15 downto 0);
	    enable: in std_logic;
	    clk     : in  std_logic);
	end component;

	component dregister_19 is
	  port (
	    din  : in  std_logic_vector(18 downto 0);
	    dout : out std_logic_vector(18 downto 0);
	    enable: in std_logic;
	    clk     : in  std_logic);
	end component;

	component dregister_1 is                 -- no. of bits
	  port (
	    din  : in  std_logic;
	    dout : out std_logic;
	    enable: in std_logic;
	    clk     : in  std_logic);
	end component;


begin

	ir: dregister port map (ir_in, ir_out, pipe_en, clk);
	npc: dregister port map (npc_in, npc_out, pipe_en, clk);
	t1: dregister port map (t1_in, t1_out, pipe_en, clk);
	t2: dregister port map (t2_in, t2_out, pipe_en, clk);
	t3: dregister port map (t3_in, t3_out, pipe_en, clk);
	memd: dregister port map (memd_in, memd_out, pipe_en, clk);
	contr: dregister_19 port map (contr_in, contr_out, pipe_en, clk);
	car: dregister_1 port map (c_in, c_out, pipe_en, clk);
	zer: dregister_1 port map (z_in, z_out, pipe_en, clk);
	
end pipe_behave;