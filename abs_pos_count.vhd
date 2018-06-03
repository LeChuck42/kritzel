library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity abs_pos_cnt is
	generic (
		C_CNT_LEN: natural := 24);
	port (
		CLK         :  in std_logic; -- min 100kHz
		RESET       :  in std_logic;
		
		LB_A        :  in std_logic;
		LB_B        :  in std_logic;
		
		COUNT       : out std_logic_vector(C_CNT_LEN - 1 downto 0);
		SET_VAL     :  in std_logic_vector(C_CNT_LEN - 1 downto 0);
		SET         :  in std_logic);
end entity abs_pos_cnt;


-- positive direction:
--            ____      ____
-- LB_A    __/    \____/
--              ____      __
-- LB_B    ____/    \____/

architecture rtl of abs_pos_cnt is

	signal pos_cnt:      unsigned(C_CNT_LEN-1 downto 0) := (others => '0');
	signal last_state:   std_logic_vector(1 downto 0);
	signal cur_state:    std_logic_vector(1 downto 0);
begin
	
	cur_state <= LB_A & LB_B;
	
	step_fsm: process(CLK)
	begin
		if rising_edge(CLK) then
			last_state <= cur_state;
			
			if SET = '1' then
				pos_cnt <= unsigned(SET_VAL);
			else
				case last_state is
					when "00" =>
						case cur_state is
							when "10" =>
								pos_cnt <= pos_cnt + 1;
							when "01" =>
								pos_cnt <= pos_cnt - 1;
							when others =>
						end case;
					when "10" =>
						case cur_state is
							when "11" =>
								pos_cnt <= pos_cnt + 1;
							when "00" =>
								pos_cnt <= pos_cnt - 1;
							when others =>
						end case;
					when "11" =>
						case cur_state is
							when "01" =>
								pos_cnt <= pos_cnt + 1;
							when "10" =>
								pos_cnt <= pos_cnt - 1;
							when others =>
						end case;
					when "01" =>
						case cur_state is
							when "00" =>
								pos_cnt <= pos_cnt + 1;
							when "11" =>
								pos_cnt <= pos_cnt - 1;
							when others =>
						end case;
				end case;
			end if;
		end if;
	end process step_fsm;
	
end architecture rtl;
