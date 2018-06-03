library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity period is
	generic (
		CNT_LEN: natural := 16);
	port (
		CLK:     in  std_logic;
		SIG_IN:  in  std_logic;
		CNT_OUT: out std_logic_vector(CNT_LEN-1 downto 0));
end entity period;

architecture rtl of period is
	signal cnt: unsigned(CNT_LEN-1 downto 0);
	signal edge_buf: std_logic_vector(2 downto 0) := "111";
begin
	
	cnt_proc: process (CLK)
	begin
		if rising_edge(CLK) then
			edge_buf <= edge_buf(1 downto 0) & SIG_IN;
			if SIG_IN = '1' and edge_buf = "011" then -- rising edge
				CNT_OUT <= std_logic_vector(cnt);
				cnt <= (others => '0');
			elsif cnt /= (cnt'LENGTH => '1') then
				cnt <= cnt + 1;
			end if;
		end if;
	
	end process;
	
end architecture rtl;
