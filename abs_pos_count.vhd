library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity abs_pos_cnt is
	generic (
		C_CNT_LEN: natural := 24;
		C_PRD_LEN: natural := 16);
	port (
		CLK         :  in std_logic; -- min 100kHz
		RESET       :  in std_logic;

		LB_A        :  in std_logic;
		LB_B        :  in std_logic;

		ABS_POS     : out signed(C_CNT_LEN - 1 downto 0);
		INIT_VAL    :  in signed(C_CNT_LEN - 1 downto 0);
		INIT_EN     :  in std_logic;

		PERIOD      : out unsigned(C_PRD_LEN - 1 downto 0);
		DIRECTION   : out std_logic -- 1=positive
		);
end entity abs_pos_cnt;


-- positive direction:
--            ____      ____
-- LB_A    __/    \____/
--              ____      __
-- LB_B    ____/    \____/

architecture rtl of abs_pos_cnt is

	signal pos_cnt:      signed(C_CNT_LEN-1 downto 0) := (others => '0');
	signal last_state:   std_logic_vector(1 downto 0);
	signal cur_state:    std_logic_vector(1 downto 0);


	signal speed_mon:    std_logic;
	signal cnt:          unsigned(C_PRD_LEN-1 downto 0);
	signal edge_buf:     std_logic_vector(2 downto 0) := "111";

begin

	cur_state <= LB_A & LB_B;

	step_fsm: process(CLK)
	begin
		if rising_edge(CLK) then
			last_state <= cur_state;

			if INIT_EN = '1' then
				pos_cnt <= INIT_VAL;
			else
				case last_state is
					when "00" =>
						case cur_state is
							when "10" =>
								pos_cnt <= pos_cnt + 1;
								DIRECTION <= '1';
							when "01" =>
								pos_cnt <= pos_cnt - 1;
								DIRECTION <= '0';
							when others =>
						end case;
					when "10" =>
						case cur_state is
							when "11" =>
								pos_cnt <= pos_cnt + 1;
								DIRECTION <= '1';
							when "00" =>
								pos_cnt <= pos_cnt - 1;
								DIRECTION <= '0';
							when others =>
						end case;
					when "11" =>
						case cur_state is
							when "01" =>
								pos_cnt <= pos_cnt + 1;
								DIRECTION <= '1';
							when "10" =>
								pos_cnt <= pos_cnt - 1;
								DIRECTION <= '0';
							when others =>
						end case;
					when "01" =>
						case cur_state is
							when "00" =>
								pos_cnt <= pos_cnt + 1;
								DIRECTION <= '1';
							when "11" =>
								pos_cnt <= pos_cnt - 1;
								DIRECTION <= '0';
							when others =>
						end case;
				end case;
			end if;
		end if;
	end process step_fsm;

	speed_mon <= LB_A;-- xor LB_B;

	cnt_proc: process (CLK)
	begin
		if rising_edge(CLK) then
			edge_buf <= edge_buf(1 downto 0) & speed_mon;
			if speed_mon = '1' and edge_buf = "011" then -- rising edge
				PERIOD <= cnt;
				cnt <= (others => '0');
			elsif cnt /= (cnt'RANGE => '1') then
				cnt <= cnt + 1;
			end if;
		end if;

	end process;

end architecture rtl;
