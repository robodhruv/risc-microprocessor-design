library std;
use std.textio.all;

library std;
use std.standard.all;

library ieee;
use ieee.std_logic_1164.all;

entity risc is
	port (	
			X : in std_logic_vector(33 downto 0);
			clock : in std_logic;
			Y : out std_logic_vector(3 downto 0)); -- Stores the current state
end entity;

architecture risc_proc of risc is

component alu is 
	port( X,Y : in std_logic_vector(15 downto 0);
		x0,x1 : in std_logic ;
		C_in: in std_logic;
		C_out, Z_out: out std_logic;
		S : out std_logic_vector(15 downto 0));
end component; 

component se7 is 
	port( se_in : in std_logic_vector(8 downto 0);
		  se_type : in std_logic;
		  se_out: out std_logic_vector(15 downto 0));
end component;

component se10 is 
	port( se_in : in std_logic_vector(5 downto 0);
		  se_out: out std_logic_vector(15 downto 0));
end component;

component rf is 
	port( A1,A2,A3 : in std_logic_vector(2 downto 0);
		  D3: in std_logic_vector(15 downto 0);
		  
		clk,wr, pc_wr,pc_read: in std_logic ;
		D1, D2: out std_logic_vector(15 downto 0));
end component;

component dregister is                 -- no. of bits
  port (
    din  : in  std_logic_vector(15 downto 0);
    dout : out std_logic_vector(15 downto 0);
    enable: in std_logic;
    clk     : in  std_logic);
end component;

component memory is 
	port ( wr,rd : in std_logic; 
			Add_in, D_in: in std_logic_vector(15 downto 0);
			Y_out: out std_logic_vector(15 downto 0)); 
end component; 

component pr_encoder is
	port( x_in : in std_logic_vector (7 downto 0);
		  x_out : out std_logic_vector(2 downto 0));
end component;

t1: dregister port map (t1_in, t1_out, t1_en, clock);
t2: dregister port map (t2_in, t2_out, t2_en, clock);
t3: dregister port map (t3_in, t3_out, t3_en, clock);

a: dregister port map (a_in, a_out, a_en, clock);
c: dregister port map (c_in, c_out, c_en, clock);
ir: dregister port map (ir_in, ir_out, ir_en, clock);

rf_main : rf port map (rA1, rA2, rA3, rD3, clock, rwr, pc_wr, pc_read, rD1, rD2);
alu_main : alu port map (alu_x, alu_y, alu_x0, alu_x1, car_in, car_out, z_out, alu_out);




begin

end risc_proc;