library std;
use std.standard.all;
library ieee;
use ieee.std_logic_1164.all;
library ieee;
use ieee.numeric_std.all; 

entity prisc is
  port ( clk, rst : in std_logic);
end entity;

architecture pipelined of prisc is

	component decoder is
		port(X: in std_logic_vector(3 downto 0);
			 Y: out std_logic_vector(18 downto 0));
	end component;

	component pipe is
		port ( ir_in, npc_in, t1_in, t2_in, t3_in, memd_in : in std_logic_vector(15 downto 0);
			ir_out, npc_out, t1_out, t2_out, t3_out, memd_out : out std_logic_vector(15 downto 0);
			contr_in : in std_logic_vector(18 downto 0);
			contr_out : out std_logic_vector(18 downto 0);
			pipe_en, c_in, z_in : in std_logic;
			c_out, z_out : out std_logic;
			clk : in std_logic);
	end component;

	component alu is 
		port( X,Y : in std_logic_vector(15 downto 0);
			x0,x1 : in std_logic ;
			C_in: in std_logic;
			C_out, Z_out: out std_logic;
			S : out std_logic_vector(15 downto 0));
	end component;

	component memory is 
		port ( wr,rd, init : in std_logic; 
				Add_in, D_in: in std_logic_vector(15 downto 0);
				Y_out: out std_logic_vector(15 downto 0)); 
	end component; 

	component rf is 
		port( A1,A2,A3 : in std_logic_vector(2 downto 0);
			  D3: in std_logic_vector(15 downto 0);
			  
			clk,wr, reset: in std_logic ; -- No separate control for PC required; simply drive 111 to A_
			D1, D2: out std_logic_vector(15 downto 0));
	end component;

	component dregister_1 is                 -- no. of bits
	  port (
	    din  : in  std_logic;
	    dout : out std_logic;
	    enable: in std_logic;
	    clk     : in  std_logic);
	end component;

	component dregister is                 -- no. of bits
	  port (
	    din  : in  std_logic_vector(15 downto 0);
	    dout : out std_logic_vector(15 downto 0);
	    enable: in std_logic;
	    clk     : in  std_logic);
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

	component pr_encoder is
		port( x_in : in std_logic_vector (7 downto 0);
			  x_out: out std_logic_vector(2 downto 0);
			  x_none: out std_logic);
	end component;

signal zeros, prc_in, prc_out, palu_out, malu_out, codemem_out, ir_out_p0, ir_out_pa, ir_out_pb, ir_out_pc, ir_out_pd, npc_out_p0, npc_out_pb,
npc_out_pa, rf_D1, rf_D2, rf_D3, npc_out_pd, memd_out_pd, t3_out_pd, datamem_a, datamem_out, datamem_din , t1_in, t2_out_pb, t2_out_pc, t2_out_pd,
t1_out_pb, t3_out_pb, npc_out_pc, t1_out_pc, t1_out_pd, t3_out_pc, alu_1, alu_2, ir_out_pb_50, ir_out_pb_80, ir_out_pd_80, ir : std_logic_vector(15 downto 0) := (others => '0');
signal one : std_logic_vector(15 downto 0) := (0 => '1', others => '0');
signal pc_en, prc_en, codemem_init, p0_en, pa_en, pb_en, pd_en, rf_wr, rf_rst, cen, zen, datamem_init, datamem_rd,
datamem_wr, zin, zout, cin, cout, alu_op1, iter_in, iter_out, iter_en : std_logic := '0';
signal decoded_contr, contr_in_pa, contr_in_pb, contr_in_pc, contr_in_pd, contr_p0_out, contr_pa_out, contr_pb_out, contr_pc_out, contr_pd_out : std_logic_vector(18 downto 0) := (others => '0');
signal pe_out,rf_A1,rf_A2,rf_A3 : std_logic_vector(2 downto 0) := "000";

begin

-- Instruction Fetch
pc: dregister port map (prc_in, prc_out, prc_en, clk);
codemem: memory port map ('0', '1', codemem_init, prc_out, zeros, codemem_out);
palu: alu port map (X => prc_out, Y => one, x0 => '1', x1 => '1', C_in => '0', S => palu_out);
-- Instruction Decode
dec: decoder port map (ir_out_p0(15 downto 12), decoded_contr);
pipe0: pipe port map (ir_in => codemem_out, npc_in => palu_out, t1_in => zeros, t2_in => zeros, t3_in => zeros, memd_in => zeros, contr_in => "0000000000000000000",
						pipe_en => p0_en, clk => clk, ir_out => ir_out_p0, npc_out => npc_out_p0, contr_out => contr_p0_out, t1_out => t1_out_p0, t2_out => t2_out_p0,
						t3_out => t3_out_p0, memd_out => memd_out_p0, c_in => '0', z_in => '0', z_out => z_out_p0, c_out => c_out_p0);

-- Register Read
rf_main: rf port map (rf_A1, rf_A2, rf_A3, rf_D3, clk, rf_wr, rf_rst, rf_D1, rf_D2);
-- pipeA: pipe port map (ir_in => ir_out_p0, npc_in => npc_out_p0, t1_in => zeros, t2_in => zeros, t3_in => zeros, memd_in => zeros, contr_in => contr_pa_in, pipe_en => pa_en,
--						clk => clk, ir_out => ir_out_pa, npc_out => npc_out_pa, contr_out => contr_pa_out);

pipeA: pipe port map (ir_in => ir_in_pa, npc_in => npc_in_pa, t1_in => t1_in_pa, t2_in => t2_in_pa, t3_in => t3_in_pa, memd_in => memd_in_pa, contr_in => contr_in_pa,
						pipe_en => pa_en, clk => clk, ir_out => ir_out_pa, npc_out => npc_out_pa, contr_out => contr_pa_out, t1_out => t1_out_pa, t2_out => t2_out_pa,
						t3_out => t3_out_pa, memd_out => memd_out_pa, c_in => c_in_pa, z_in => z_in_pa, c_out => c_out_pa, z_out => z_out_pa);
-- Execute
malu: alu port map (X => alu_1, Y => alu_2, x0 => alu_op1, x1 => '1', C_in => cout, C_out => cin, Z_out => zin, S => malu_out);
pipeB: pipe port map (ir_in => ir_in_pb, npc_in => npc_in_pb, t1_in => t1_in_pb, t2_in => t2_in_pb, t3_in => t3_in_pb, memd_in => memd_in_pb, contr_in => contr_in_pb,
						pipe_en => pa_en, clk => clk, ir_out => ir_out_pb, npc_out => npc_out_pb, contr_out => contr_pb_out, t1_out => t1_out_pb, t2_out => t2_out_pb,
						t3_out => t3_out_pb, memd_out => memd_out_pb, c_in => c_in_pb, z_in => z_in_pb, c_out => c_out_pb, z_out => z_out_pb);
-- Memory
datamem: memory port map (datamem_wr, datamem_rd, datamem_init, datamem_a, datamem_din, datamem_out);
pipeC: pipe port map (ir_in => ir_in_pc, npc_in => npc_in_pc, t1_in => t1_in_pc, t2_in => t2_in_pc, t3_in => t3_in_pc, memd_in => memd_in_pc, contr_in => contr_in_pc,
						pipe_en => pc_en, clk => clk, ir_out => ir_out_pc, npc_out => npc_out_pc, contr_out => contr_pc_out, t1_out => t1_out_pc, t2_out => t2_out_pc,
						t3_out => t3_out_pc, memd_out => memd_out_pc, c_in => c_in_pc, z_in => z_in_pc, c_out => c_out_pc, z_out => z_out_pc);
-- RF Write-Back
pipeD: pipe port map (ir_in => ir_in_pd, npc_in => npc_in_pd, t1_in => t1_in_pd, t2_in => t2_in_pd, t3_in => t3_in_pd, memd_in => memd_in_pd, contr_in => contr_in_pd,
						pipe_en => pd_en, clk => clk, ir_out => ir_out_pd, npc_out => npc_out_pd, contr_out => contr_pd_out, t1_out => t1_out_pd, t2_out => t2_out_pd,
						t3_out => t3_out_pd, memd_out => memd_out_pd, c_in => c_in_pd, z_in => z_in_pd, c_out => c_out_pd, z_out => z_out_pd);


alu_se10: se10 port map (ir_out_pb(5 downto 0), ir_out_pb_50); -- sign extended for alu_1
alu_se7: se7 port map (ir_out_pb(8 downto 0), '1', ir_out_pb_80);
pd_se7: se7 port map (ir_out_pd(8 downto 0), '0', ir_out_pd_80);
car: dregister_1 port map (cin, cout, cen, clk);
zer: dregister_1 port map (zin, zout, zen, clk);
iter: dregister_1 port map (iter_in, iter_out, iter_en, clk);

sm_pe: pr_encoder port map (ir_out_pc(7 downto 0), sm_index, sm_fin);
lm_pe: pr_encoder port map (ir_out_pd(7 downto 0), lm_index, lm_fin);

process(clk, rst)
	begin
		pc_in <= palu_out;
		if (contr_pa_out(18) = '1') then
			rf_A1 <= sm_index;
		else
			rf_A1 <= ir_out_pa(12 downto 10);
		end if;

		-- RF signals dep on contr_pa_out
		if (contr_pa_out(17) = '0') then
			rf_A2 <= ir_out_pa(9 downto 7);
		else
			rf_A2 <= pe_out;
		end if;

		-- ALU signals dep on contr_pb_out
		if (contr_pb_out(15 downto 14) = "00") then
			alu_1 <= t1_out_pb;
		elsif (contr_pb_out(15 downto 14) = "01") then
			alu_1 <= ir_out_pb_50;
		-- elsif (contr_pb_out(15 downto 14) = "10") then -- replacing with else to prevent latch possibility / NA
		else
			alu_1 <= ir_out_pb_80;
		end if;

		alu_op1 <= not contr_pb_out(11); -- Op Code logic
		cen <= contr_pb_out(9);
		zen <= contr_pb_out(8);

		-- MEM signals dep on contr_pc_out
		datamem_rd <= contr_pc_out(7);
		datamem_wr <= contr_pc_out(6);
		datamem_din <= t1_out_pc;
		if (contr_pc_out(5) = '1') then
			datamem_a <= t3_out_pc;
		else
			datamem_a <= zeros;
		end if;

		-- RF_WB signals dep on contr_pd_out
		if (contr_pd_out(4 downto 3) = "01") then
			rf_A3 <= ir_out_pd(6 downto 4);
		elsif (contr_pd_out(4 downto 3) = "10") then
			rf_A3 <= ir_out_pd(12 downto 10);
		else -- Including 
			rf_A3 <= pe_out;
		end if;
		
		rf_wr <= contr_pd_out(0);

		if (contr_pd_out(2 downto 1) = "00") then
			rf_D3 <= t3_out_pd;
		elsif (contr_pd_out(2 downto 1) = "01") then
			rf_D3 <= npc_out_pd;
		elsif (contr_pd_out(2 downto 1) = "10") then
			rf_D3 <= memd_out_pd;
		else
			rf_D3 <= ir_out_pd_80;
		end if;

		-- Op Codes
		op_a <= ir_out_pa(15 downto 12);
		op_b <= ir_out_pb(15 downto 12);
		op_c <= ir_out_pc(15 downto 12);
		op_d <= ir_out_pd(15 downto 12);
			
		-- Mapping of Pipe A
		if (op_a = "0111") then
			-- Flush
			ir_in_pa <= (others => '1');
			contr_in_pa <= (others => '0');

			c_in_pa <= '0';
			z_in_pa <= '0';
			npc_in_pa <= (others => '0');
			t1_in_pa <= (others => '0');
			t2_in_pa <= (others => '0');
			t3_in_pa <= (others => '0');
			memd_in_pa <= (others => '0');
		elsif ((op_c = "0111") and (not sm_fin)) then
			-- pc_out goes to pa_in
			ir_in_pa <= ir_out_pc;
			npc_in_pa <= npc_out_pc;
			t1_in_pa <= t1_out_pc;
			t2_in_pa <= t2_out_pc;
			t3_in_pa <= t3_out_pc;
			memd_in_pa <= memd_out_pc;
			contr_in_pa <= contr_pc_out;
			c_in_pa <= c_out_pc;
			z_in_pa <= z_out_pc;
		else
			-- directly from p0
			contr_in_pa <= decoded_contr;
			ir_in_pa <= ir_out_p0;
			npc_in_pa <= npc_out_p0;
			t1_in_pa <= t1_out_p0;
			t2_in_pa <= t2_out_p0;
			t3_in_pa <= t3_out_p0;
			memd_in_pa <= memd_out_p0;		
			c_in_pa <= c_out_p0;
			z_in_pa <= z_out_p0;
		end if;
			

		-- Mapping of Pipe B
		if (op_b = "0110") then
			-- Flush
			ir_in_pb <= (others => '1');
			contr_in_pb <= (others => '0');

			c_in_pb <= '0';
			z_in_pb <= '0';
			npc_in_pb <= (others => '0');
			t1_in_pb <= (others => '0');
			t2_in_pb <= (others => '0');
			t3_in_pb <= (others => '0');
			memd_in_pb <= (others => '0');

		elsif ((op_d = "0110") and (not lm_fin)) then
			-- pb_in <= pd_out
			ir_in_pb <= ir_out_pd;
			npc_in_pb <= npc_out_pd;
			t1_in_pb <= t1_out_pd;
			t2_in_pb <= t2_out_pd;
			t3_in_pb <= t3_out_pd;
			memd_in_pb <= memd_out_pd;
			contr_in_pb <= contr_pd_out;
			c_in_pb <= c_out_pd;
			z_in_pb <= z_out_pd;
		else
			-- pb_in <= pa_out
			t1_in_pb <= rf_D1;
			t2_in_pb <= rf_D2;

			ir_in_pb <= ir_out_pa;
			npc_in_pb <= npc_out_pa;
			t3_in_pb <= t3_out_pa;
			memd_in_pb <= memd_out_pa;
			contr_in_pb <= contr_pa_out;
			c_in_pb <= c_out_pa;
			z_in_pb <= z_out_pa;
		end if;

		-- Mapping of Pipe C
		t3_in_pc <= malu_out;

		ir_in_pc <= ir_out_pb;
		npc_in_pc <= npc_out_pb;
		t1_in_pc <= t1_out_pb;
		t2_in_pc <= t2_out_pb;
		memd_in_pc <= memd_out_pb;
		contr_in_pc <= contr_pb_out;
		c_in_pc <= c_out_pb;
		z_in_pc <= z_out_pb;

		-- Mapping of Pipe D
		memd_in_pd <= datamem_out;

		ir_in_pd <= ir_out_pc;
		npc_in_pd <= npc_out_pc;
		t1_in_pd <= t1_out_pc;
		t2_in_pd <= t2_out_pc;
		t3_in_pd <= t3_out_pc;
		contr_in_pd <= contr_pc_out;
		c_in_pd <= c_out_pc;
		z_in_pd <= z_out_pc;		

	end process;
end pipelined;